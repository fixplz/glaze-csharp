
using System;
using System.Collections.Generic;

namespace Glaze
{
	public abstract class Space
	{
		public LinkedList<Body>    bodies;
		public LinkedList<Shape>   shapes;
		public LinkedList<Arbiter> arbiters;
		
		public int iterations = 10;
		
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
			foreach (Body b  in bodies) b.UpdateVelocity (dt);
			foreach (Shape s in shapes) s.UpdateShape ();
			
			BroadPhase ();
			
			for (LinkedListNode<Arbiter> n = arbiters.First; n != null;)
			{
				Arbiter arb = n.Value; n = n.Next;
				if (stamp - arb.stamp > 3) arb.Remove ();
				else arb.Prestep (1/dt);
			}
			
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
			Body a = sa.body, b = sb.body;
			
			if (a == b || (a.group != 0 && a.group == b.group)) return;
			
			Arbiter arb = null;
			foreach (Arbiter x in arbiters)
				if ((x.sa == sa && x.sb == sb) || (x.sa == sb && x.sb == sa))
					{ arb = x; break; }
			
			if (arb != null) if (arb.stamp == stamp) return;
			
			if (sa.shapeType > sb.shapeType) { var t=sa; sa=sb; sb=t; }
			
			// TODO this is shit
			bool make = arb == null;
			
			if (make) arb = new Arbiter (sa,sb);
			
			if (!Calc.Check (sa, sb, arb)) return;
			
			if (make)
			{
				//arb = new Arbiter (sa,sb);
				a.arbiters.AddFirst (arb);
				b.arbiters.AddFirst (arb);
				arb.Attach (arbiters);
			}
			else
			{ arb.sa = sa; arb.sb = sb; }
			
			arb.stamp = stamp;
		}
		#endregion
	}
	
	
	
	public static class Config
	{
		public const double
			areaMassRatio  = 0.01,
			resolveSlop    = 0.2,
			resolveBias    = 0.2;
		
		public static Material defaultMaterial =
			new Material {restitution=0.2, friction=0.8};
	}
	
	
	
	public class Entry<T> where T : Entry<T>
	{
		internal LinkedListNode<T> node;
		internal virtual void Attach (LinkedList<T> list) { node = list.AddFirst (this as T); }
		internal virtual void Remove () { node.List.Remove (node); }
	}
	
	public class PoolEntry<T> : Entry<T> where T : PoolEntry <T>
	{
		
	}
}
