
using System;
using System.Collections.Generic;

namespace Glaze
{
	public static class Config
	{
		public const double
			areaMassRatio  = 0.01,
			resolveSlop    = 0.1,
			resolveBias    = 0.1;
		
		public static Material defaultMaterial =
			new Material {restitution=0.2, friction=0.9};
	}
	
	
	
	public abstract class Space
	{
		public LinkedList<Body>    bodies;
		public LinkedList<Shape>   shapes;
		public LinkedList<Arbiter> arbiters;
		
		public int iterations = 5;
		
		internal uint stamp = 0;
		
		internal Space ()
		{
			bodies   = new LinkedList<Body>     ();
			arbiters = new LinkedList<Arbiter>  ();
			shapes   = new LinkedList<Shape>    ();
		}
		
		#region CONTROLS
		public void AddBody (Body body)
		{
			foreach (Shape s in body.shapes) s.Attach (shapes);
			body.Attach (bodies);
		}
		
		public void RemoveBody (Body body)
		{
			foreach (Shape s in body.shapes) s.Remove ();
			body.Remove ();
		}
		
		public virtual IEnumerable<Shape> Query (AABB area)
		{
			foreach (Shape s in shapes)
				if (s.aabb.Intersect (area))
					yield return s;
		}
		
		public void RunPhysics (double dt)
		{
			foreach (Body b in bodies)
			{
				b.UpdateVelocity (dt);
				foreach (Shape s in b.shapes) s.UpdateShape ();
			}
			
			BroadPhase ();
			
			for (var n = arbiters.First; n != null;)
			{
				Arbiter arb = n.Value; n = n.Next;
				if (stamp - arb.stamp > 3) arb.Remove (); else arb.Prestep (dt);
			}
			
			// generally 70% of time spent here
			for (int i=0; i<iterations; i++)
				foreach (Arbiter arb in arbiters)
					arb.Perform ();
			
			foreach (Body b in bodies) b.UpdatePosition (dt);
			
			stamp++;
		}
		#endregion
		
		#region COLLISION DETECTION
		protected abstract void BroadPhase ();
		
		protected void NarrowPhase (Shape sa, Shape sb)
		{
			if (sa.shapeType > sb.shapeType) { var t=sa; sa=sb; sb=t; }
			
			Body a = sa.body, b = sb.body;
			if (a == b || (a.group != 0 && a.group == b.group)) return;
			
			Arbiter arb = null;
			foreach (Arbiter x in a.arbiters) if (x.Belong (sa,sb)) { arb = x; break; }
			
			bool first = arb == null;
			
			if (!first)
				if (arb.stamp == stamp) return;
				else { arb.sa = sa; arb.sb = sb; }
			
			if (Calc.Check (sa, sb, ref arb))
			{
				if (first)
				{
					if (arb.sa == null) { arb.sa = sa; arb.sb = sb;}
					a.arbiters.AddFirst (arb);
					b.arbiters.AddFirst (arb);
					arb.Attach (arbiters);
				}
				
				arb.stamp = stamp;
			}
		}
		#endregion
	}
	
	
	
	public class Entry<T> where T : Entry<T>
	{
		internal LinkedListNode<T> node;
		internal virtual void Attach (LinkedList<T> list) { node = list.AddFirst (this as T); }
		internal virtual void Remove () { node.List.Remove (node); }
	}
}
