
using System;
using System.Collections.Generic;

namespace Glaze
{
	public static class Config
	{
		public const double
			areaMassRatio  = 0.01,
			resolveSlop    = 0.1,
			resolveRate    = 0.1,
			
			defaultRestitution = 0.0,
			defaultFriction    = 0.5;
	}
	
	
	
	public class Entry<T> where T : Entry<T>
	{
		internal LinkedListNode<T> node;
		internal virtual void Attach (LinkedList<T> list) { node = list.AddFirst (this as T); }
		internal virtual void Remove () { node.List.Remove (node); }
	}
	
	
	public abstract class Space
	{
		public LinkedList<Body>    bodies;
		public LinkedList<Shape>   shapes;
		public LinkedList<Arbiter> arbiters;
		public LinkedList<Joint>   joints;
		
		internal uint stamp = 0;
		
		internal Space ()
		{
			bodies   = new LinkedList<Body>     ();
			shapes   = new LinkedList<Shape>    ();
			arbiters = new LinkedList<Arbiter>  ();
			joints   = new LinkedList<Joint>    ();
		}
		
		#region CONTROLS
		public void AddBody (Body body)
		{
			foreach (Shape s in body.shapes) s.Attach (shapes);
			body.Attach (bodies);
		}
		
		public void RemoveBody (Body body)
		{
			foreach (Shape s     in body.shapes)   s.Remove ();
			foreach (Arbiter arb in body.arbiters) arb.Remove ();
			body.Remove ();
		}
		
		public void AddJoint    (Joint joint) { joint.Attach (joints); }
		public void RemoveJoint (Joint joint) { joint.Remove (); }
		
		public virtual IEnumerable<Shape> Query (AABB area)
		{
			foreach (Shape s in shapes)
				if (s.aabb.Intersect (area))
					yield return s;
		}
		
		public void RunPhysics (double dt, int iterations)
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
				if (stamp - arb.stamp > 3) arb.Remove (); else arb.Prestep ();
			}
			
			foreach (Joint j in joints) j.Prestep ();
			
			// generally 70% of time spent here
			for (int i=0; i<iterations; i++)
			{
				foreach (Arbiter arb in arbiters) arb.Perform ();
				foreach (Joint j     in joints)   j.Perform ();
			}
			
			foreach (Body b in bodies) b.UpdatePosition (dt);
			
			stamp++;
		}
		#endregion
		
		#region COLLISION DETECTION
		protected abstract void BroadPhase ();
		
		protected void NarrowPhase (Shape sa, Shape sb)
		{
			Body a = sa.body, b = sb.body;
			if (a == b || (a.group != 0 && (a.group&b.group) != 0)) return;
			
			Arbiter arb = null;
			foreach (Arbiter x in a.arbiters) if (x.Belong (sa,sb)) { arb = x; break; }
			
			bool first = arb == null;
			if (sa.shapeType > sb.shapeType) { var t=sa; sa=sb; sb=t; }
			if (!first) { arb.sa=sa; arb.sb=sb; }
			
			if (Calc.Check (sa, sb, ref arb)) // assigns data to arb if successful
			{
				if (first) { if (arb.sa == null) { arb.sa = sa; arb.sb = sb; } arb.Attach (arbiters); }
				arb.stamp = stamp;
			}
		}
		#endregion
	}
	
	public sealed class Arbiter : Entry <Arbiter>
	{
		public Shape sa,sb;
		public double bounce, stick;
		
		internal uint stamp;
		internal int used = 0;
		internal Contact[] contacts;
		
		internal Arbiter (uint n) { contacts = new Contact [n]; }
		
		public bool Belong   (Shape a, Shape b) { return (sa == a && sb == b) || (sa == b && sb == a); }
		public Body GetOther (Body b)           { return sa.body == b ? sb.body : sa.body; }
		
		public IEnumerable<Contact> Contacts { get { for (int i=0; i<used; i++) yield return contacts [i]; } }
		
		internal override void Attach(LinkedList<Arbiter> list)
		{
			base.Attach (list);
			sa.body.arbiters.AddFirst (this);
			sb.body.arbiters.AddFirst (this);
		}
		
		internal override void Remove ()
		{
			base.Remove ();
			sa.body.arbiters.Remove (this);
			sb.body.arbiters.Remove (this);
			for (int i=0; i < contacts.Length && contacts [i] != null; i++) Contact.Retire (contacts [i]);
		}
		
		#region STEP
		internal bool UpdateContact (Vec2 p, Axis a, uint id)
		{
			Contact c;
			for (int i=0; i<used; i++) { c = contacts [i]; if (c.id == id) goto found; }
			
			if (used == contacts.Length) return false;
			
			c = contacts [used] ?? (contacts [used] = Contact.Assign ()); used++;
			c.id = id; c.jnAcc = 0; c.jtAcc = 0;
		found:
			c.p = p; c.con.n = a.n; c.dist = a.d; c.updated = true;
			return true;
		}
		
		internal void Prestep ()
		{
			bounce = Math.Sqrt (sa.restitution * sb.restitution);
			stick  = Math.Sqrt (sa.friction    * sb.friction);
			
			for (int i = used-1; i>=0; i--)
			{
				Contact c = contacts [i];
				if (!c.updated) { if (i < --used) { contacts [i] = contacts [used]; contacts [used] = c; } }
				else            { c.updated = false; c.Prestep (this); }
			}
		}
		
		internal void Perform () { for (int i=0; i<used; i++) contacts [i].Perform (this); }
		#endregion
	}
}
