
using System;
using System.Collections.Generic;

namespace Glaze
{
	public sealed class Arbiter : Entry <Arbiter>
	{
		public Shape sa,sb;
		public double e,u;
		
		internal uint stamp; internal int used = 0;
		internal Contact[] contacts;
		
		internal Arbiter (uint n) { contacts = new Contact [n]; }
		
		public bool Belong (Shape a, Shape b) { return (sa == a && sb == b) || (sa == b && sb == a); }
		public Body GetOther (Body b) { return sa.body == b ? sb.body : sa.body; }
		
		public IEnumerable<Contact> Contacts () { for (int i=0; i<used; i++) yield return contacts [i]; }
		
		#region INTERNALS
		internal bool UpdateContact (Vec2 p, Axis a, uint id)
		{
			Contact c;
			for (int i=0; i<used; i++) { c = contacts [i]; if (c.id == id) goto found; }
			
			if (used == contacts.Length) return false;
			
			c = contacts [used] ?? (contacts [used] = Contact.Assign ()); used++;
			c.id = id; c.jnAcc = 0; c.jtAcc = 0;
		found:
			c.p = p; c.n = a.n; c.dist = a.d; c.updated = true;
			return true;
		}
		
		internal override void Remove ()
		{
			base.Remove ();
			sa.body.arbiters.Remove (this);
			sb.body.arbiters.Remove (this);
			for (int i=0; i < contacts.Length && contacts [i] != null; i++) Contact.Retire (contacts [i]);
		}
		
		internal void Prestep (double dt)
		{
			e = sa.material.restitution * sb.material.restitution;
			u = sa.material.friction    * sb.material.friction;
			
			for (int i = used-1; i>=0; i--)
			{
				Contact c = contacts [i];
				if (!c.updated) { if (i < --used) { contacts [i] = contacts [used]; contacts [used] = c; } }
				else            { c.updated = false; c.Prestep (dt, this); }
			}
		}
		
		internal void Perform () { for (int i=0; i<used; i++) contacts [i].Perform (this); }
		#endregion
	}
	
	
	
	public sealed class Contact
	{
		internal static Stack<Contact> pool = new Stack<Contact> (100);
		internal static Contact Assign () { return pool.Count != 0 ? pool.Pop () : new Contact (); }
		internal static void Retire (Contact c) { pool.Push (c); }
		
		// position, normal, penetration, normal and tangent impulses
		public Vec2 p, n;
		public double dist, jnAcc,jtAcc;
		
		internal Vec2 r1,r2;
		internal double nMass,tMass, bounce, jBias,bias;
		internal uint id; internal bool updated;
		
		#region RESOLUTION MATH
		internal void Prestep (double dt, Arbiter arb)
		{
			Body a = arb.sa.body, b = arb.sb.body;
			
			r1 = p - a.pos; r2 = p - b.pos;
			
			nMass = 1.0 / Calc.KScalar (a,b, this, n);
			tMass = 1.0 / Calc.KScalar (a,b, this, n.Left);
			
			jBias = 0; bias = 1.0/dt * Calc.BiasDist (dist);
			bounce = arb.e * n * Calc.RelativeVelocity (a,b, this);
			
			Calc.NormalImpulse (jnAcc,jtAcc, this, a,b);
		}
		
		internal void Perform (Arbiter arb)
		{
			Body a = arb.sa.body, b = arb.sb.body;
			double jbn, jn, jt; Vec2 vb, vr;
			
			vb   = Calc.RelativeBiasVelocity (a,b, this);
			vr   = Calc.RelativeVelocity     (a,b, this);
			
			jbn  = nMass * (vb * n - bias);
			jn   = nMass * (vr * n + bounce);
			jt   = tMass * (vr * n.Left);
			
			Calc.AddPositive (ref jBias, ref jbn);
			Calc.AddPositive (ref jnAcc, ref jn);
			Calc.AddClamp    (ref jtAcc, ref jt, arb.u * jnAcc);
			
			Calc.NormalBias    (jbn,   this, a,b);
			Calc.NormalImpulse (jn,jt, this, a,b);
		}
		#endregion
	}
	
	
	
	internal static class Calc
	{
		#region AUX
		internal static double BiasDist (double dist) { return Config.resolveBias * Math.Min (0, dist + Config.resolveSlop); }
		
		internal static void NormalBias    (double jbn,           Contact c, Body a, Body b) { Vec2 j = jbn*c.n;                       a.ApplyBias    (-j, c.r1);  b.ApplyBias    (j, c.r2); }
		internal static void NormalImpulse (double jn, double jt, Contact c, Body a, Body b) { Vec2 j = new Vec2 (jn,jt).Rotate (c.n); a.ApplyImpulse (-j, c.r1);  b.ApplyImpulse (j, c.r2); }
		
		internal static void AddPositive (ref double old, ref double change) { change = Math.Max (-old, change); old += change; }
		internal static void AddClamp    (ref double old, ref double change, double limit)
			{ double result = Math.Max (-limit, Math.Min (limit, old+change)); change = result-old; old = result; }
		
		internal static Vec2 RelativeVelocity     (Body a, Body b, Contact c) { return (a.rot     * c.r1.Left + a.vel)     - (b.rot     * c.r2.Left + b.vel); }
		internal static Vec2 RelativeBiasVelocity (Body a, Body b, Contact c) { return (a.rotBias * c.r1.Left + a.velBias) - (b.rotBias * c.r2.Left + b.velBias); }
		
		internal static double KScalar (Body a, Body b, Contact c, Vec2 n)
			{ double r1xn = c.r1.Cross (n), r2xn = c.r2.Cross (n); return a.massInv+b.massInv + a.inertiaInv*r1xn*r1xn + b.inertiaInv*r2xn*r2xn; }
		
		internal static bool ContainsVert (Polygon sa, Vec2 v, Vec2 n) { foreach (Axis a in sa.axisP) if (a.n*n <= 0 && a.n*v > a.d) return false; return true; }
		internal static bool ContainsVert (Polygon sa, Vec2 v)         { foreach (Axis a in sa.axisP) if (a.n*v > a.d)               return false; return true; }
		#endregion
		
		
		#region COLLISIONS
		internal static bool Check (Shape sa, Shape sb, ref Arbiter arb)
		{
			switch ((int)((int)sa.shapeType*5 + sb.shapeType))
			{
				case 0: return Circle2Circle ((Circle)  sa, (Circle)  sb,  ref arb);
				case 1: return Circle2Poly   ((Circle)  sa, (Polygon) sb, ref arb);
				case 6: return Poly2Poly     ((Polygon) sa, (Polygon) sb, ref arb);
			}
			
			return false;
		}
		
		internal static bool Circle2Circle (Circle sa, Circle sb, ref Arbiter arb)
			{ return CircleContact (sa.pos, sb.pos, sa.radius, sb.radius, ref arb); }
		
		internal static bool CircleContact (Vec2 c1, Vec2 c2, double r1, double r2, ref Arbiter arb)
		{
			Vec2 r = c2 - c1; double min = r1 + r2, dist = r.Sq, distInv;
			if (dist >= min*min) return false; dist = Math.Sqrt (dist); distInv = 1.0/dist;
			
			if (arb == null) arb = new Arbiter (1);
			arb.UpdateContact (c1 + (0.5 + distInv * (r1 - min/2)) * r, new Axis {n=distInv*r, d=dist - min}, 0);
			return true;
		}
		
		internal static bool Circle2Poly (Circle circle, Polygon poly, ref Arbiter arb)
		{
			int len = poly.axisP.Length, ix = 0;
			double max = Double.NegativeInfinity;
			
			for (int i=0; i<len; i++)
			{
				double dist = poly.axisP [i].n * circle.pos - poly.axisP [i].d - circle.radius;
				if (dist > 0) return false; if (dist > max) { max = dist; ix = i; }
			}
			
			Vec2 v = poly.vertP [ix], u = poly.vertP [(ix+1)%len]; Axis a = poly.axisP [ix];
			
			double d = a.n.Cross (circle.pos);
			
			if (d > a.n.Cross (v)) return CircleContact (circle.pos, v, circle.radius, 0, ref arb);
			if (d < a.n.Cross (u)) return CircleContact (circle.pos, u, circle.radius, 0, ref arb);
			
			if (arb == null) arb = new Arbiter (1);
			arb.UpdateContact (circle.pos - (circle.radius+max/2) * a.n, new Axis {n=-a.n, d=max}, 0);
			return true;
		}
		
		internal static bool Poly2Poly (Polygon sa, Polygon sb, ref Arbiter arb)
		{
			Axis a1, a2;
			if (!(MinSepAxis (sa,sb, out a1) && MinSepAxis (sb,sa, out a2))) return false;
			
			if (a2.d > a1.d) {var t=sb; sb=sa; sa=t;} // sa is the shape whose axis is used
			if (arb == null) arb = new Arbiter (3);
			arb.sa=sa; arb.sb=sb;
			
			FindVerts (sa,sb, a1.d > a2.d ? a1 : a2, arb);
			return true;
		}
		
		internal static bool MinSepAxis (Polygon sa, Polygon sb, out Axis axis)
		{
			axis = new Axis {d=Double.NegativeInfinity};
			
			foreach (Axis a in sa.axisP)
			{
				double min = Double.PositiveInfinity;
				foreach (Vec2 v in sb.vertP) min = Math.Min (min, a.n*v); min -= a.d;
				if (min > 0) return false; if (min > axis.d) axis = new Axis {n=a.n,d=min};
			}
			
			return true;
		}
		
		internal static void FindVerts (Polygon sa, Polygon sb, Axis a, Arbiter arb)
		{
			uint id = sa.id << 8;
			foreach (Vec2 v in sa.vertP)
				{ if (ContainsVert (sb,v)) if (!arb.UpdateContact (v, a, id)) return; id++; }
			
			id = sb.id << 8;
			foreach (Vec2 v in sb.vertP) 
				{ if (ContainsVert (sa,v,-a.n)) if (!arb.UpdateContact (v, a, id)) return; id++; }
		}
		#endregion
	}
}
