
using System;
using System.Collections.Generic;

namespace Glaze
{
	public class Arbiter : Entry <Arbiter>
	{
		public Shape sa,sb;
		public double e,u;
		
		internal uint stamp, used = 0;
		public Contact[] contacts;
		
		internal Arbiter (uint n) { contacts = new Contact [n]; }
		
		public bool Belong (Shape a, Shape b) { return (sa == a && sb == b) || (sa == b && sb == a); }
		public Body GetOther (Body b) { return sa.body == b ? sb.body : sa.body; }
		
		public IEnumerable<Contact> Contacts ()
			{ for (uint i=0; i<used; i++) yield return contacts [i]; }
		
		internal bool UpdateContact (Vec2 p, Vec2 n, double dist, uint id)
		{
			Contact c; uint i=0;
			for (; i<used; i++) if ((c = contacts [i]).id == id) goto found;
			
			if (used == contacts.Length) return false;
			c = contacts [used] ?? (contacts [used] = Contact.Assign ()); used++;
			
			c.id = id; c.jnAcc = 0; c.jtAcc = 0;
		found:
			c.p = p; c.n = n; c.dist = dist; c.updated = true;
			return true;
		}
		
		internal override void Remove ()
		{
			base.Remove ();
			sa.body.arbiters.Remove (this);
			sb.body.arbiters.Remove (this);
		}
		
		#region RESOLUTION MATH
		internal void Prestep (double dt)
		{
			e = sa.material.restitution * sb.material.restitution;
			u = sa.material.friction    * sb.material.friction;
			
			Body a = sa.body, b = sb.body;
			
			for (uint i=0; i<used; i++)
			{
			start:
				Contact c = contacts [i];
				if (!c.updated)
				{
					if (i == --used) break;
					contacts [i] = contacts [used]; contacts [used] = c; goto start;
				}
				c.updated = false;
				
				c.r1 = c.p - a.pos; c.r2 = c.p - b.pos;
				
				c.nMass = 1.0 / Calc.KScalar (a, b, c.r1, c.r2, c.n);
				c.tMass = 1.0 / Calc.KScalar (a, b, c.r1, c.r2, c.n.Left);
				
				c.jBias = 0; c.bias = 1.0/dt * Calc.BiasDist (c.dist);
				c.bounce = e * c.n * Calc.RelativeVelocity (a,b,c);
				
				Calc.NormalImpulse (c.jnAcc,c.jtAcc, a,b,c);
			}
		}
		
		internal void Perform ()
		{
			Body a = sa.body, b = sb.body;
			
			for (uint i=0; i<used; i++)
			{
				Contact c = contacts [i];
				
				double jbn, jn, jt; Vec2 vb, vr;
				
				vb   = Calc.RelativeBiasVelocity (a,b,c);
				vr   = Calc.RelativeVelocity     (a,b,c);
				
				jbn  = c.nMass * (vb * c.n - c.bias);
				jn   = c.nMass * (vr * c.n + c.bounce);
				jt   = c.tMass * (vr * c.n.Left);
				
				Calc.AddPositive (ref c.jBias, ref jbn);
				Calc.AddPositive (ref c.jnAcc, ref jn);
				Calc.AddClamp    (ref c.jtAcc, ref jt, u * c.jnAcc);
				
				Calc.NormalBiasImpulse (jbn,   a,b,c);
				Calc.NormalImpulse     (jn,jt, a,b,c);
			}
		}
		#endregion
	}
	
	
	
	public class Contact
	{
		internal static Stack<Contact> pool = new Stack<Contact> (100);
		internal static Contact Assign () { return pool.Count != 0 ? pool.Pop () : new Contact (); }
		internal static void Retire (Contact c) { pool.Push (c); }
		
		public Vec2 p, n, r1, r2;
		internal uint id; internal bool updated;
		internal double dist, nMass,tMass, bounce, jnAcc,jtAcc, jBias,bias;
	}
	
	
	
	internal static class Calc
	{
		#region AUX
		internal static double BiasDist (double dist)
			{ return Config.resolveBias * Math.Min (0, dist + Config.resolveSlop); }
		
		internal static void NormalImpulse (double jn, double jt, Body a, Body b, Contact c)
			{ Vec2 v = jn*c.n+jt*c.n.Left; a.ApplyImpulse (-v, c.r1); b.ApplyImpulse (v, c.r2); }
		
		internal static void NormalBiasImpulse (double jbn, Body a, Body b, Contact c)
			{ a.ApplyBiasImpulse (-jbn*c.n, c.r1); b.ApplyBiasImpulse (jbn*c.n, c.r2); }
		
		internal static void AddPositive (ref double old, ref double change)
			{ change = Math.Max (-old, change); old += change; }
		
		internal static void AddClamp (ref double old, ref double change, double limit)
		{
			double result = Math.Max (-limit, Math.Min (limit, old+change));
			change = result-old; old = result;
		}
		
		internal static Vec2 RelativeVelocity (Body a, Body b, Contact c)
			{ return (a.w * c.r1.Left + a.vel) - (b.w * c.r2.Left + b.vel); }
		
		internal static Vec2 RelativeBiasVelocity (Body a, Body b, Contact c)
			{ return (a.wBias * c.r1.Left + a.velBias) - (b.wBias * c.r2.Left + b.velBias); }
		
		internal static double KScalar (Body a, Body b, Vec2 r1, Vec2 r2, Vec2 n)
		{
			double r1xn = r1.Cross (n), r2xn = r2.Cross (n);
			return a.massInv+b.massInv + a.inertiaInv*r1xn*r1xn + b.inertiaInv*r2xn*r2xn;
		}
		
		internal static double Minumum <T> (this IEnumerable<T> list, Func<T,double> f)
		{
			double min = Double.PositiveInfinity;
			foreach (T x in list) min = Math.Min (min, f (x));
			return min;
		}
		#endregion
		
		#region COLLISIONS
		internal static bool Check (Shape sa, Shape sb, ref Arbiter arb)
		{
			switch ((int)((int)sa.shapeType*5 + sb.shapeType))
			{
				case 0: return Circle2Circle ((Circle) sa,  (Circle) sb,  ref arb);
				case 1: return Circle2Poly   ((Circle) sa,  (Polygon) sb, ref arb);
				case 6: return Poly2Poly     ((Polygon) sa, (Polygon) sb, ref arb);
			}
			
			return false;
		}
		
		internal static bool Circle2Circle (Circle sa, Circle sb, ref Arbiter arb)
			{ return CircleContact (sa.pos, sb.pos, sa.radius, sb.radius, ref arb); }
		
		internal static bool CircleContact (Vec2 c1, Vec2 c2, double r1, double r2, ref Arbiter arb)
		{
			Vec2 d = c2 - c1; double min = r1 + r2, dist = d.LengthSq, distInv;
			if (dist >= min*min) return false; dist = Math.Sqrt (dist); distInv = 1.0/dist;
			
			if (arb == null) arb = new Arbiter (1);
			arb.UpdateContact (c1 + (0.5 + distInv * (r1 - min/2)) * d, distInv*d, dist - min, 0);
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
			arb.UpdateContact (circle.pos - (circle.radius+max/2) * a.n, -a.n, max, 0);
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
				double min = sb.vertP.Minumum (v => a.n*v) - a.d;
				if (min > 0) return false; if (min > axis.d) axis = new Axis {n=a.n,d=min};
			}
			
			return true;
		}
		
		internal static void FindVerts (Polygon sa, Polygon sb, Axis a, Arbiter arb)
		{
			uint id = sa.id << 8;
			foreach (Vec2 v in sa.vertP)
			{
				if (ContainsVert (sb,v))
					if (!arb.UpdateContact (v, a.n, a.d, id)) return;
				id++;
			}
			
			id = sb.id << 8;
			foreach (Vec2 v in sb.vertP)
			{
				if (ContainsVert (sa,v,-a.n))
					if (!arb.UpdateContact (v, a.n, a.d, id)) return;
				id++;
			}
		}
		
		internal static bool ContainsVert (Polygon sa, Vec2 v, Vec2 n)
			{ foreach (Axis a in sa.axisP) if (a.n*n <= 0 && a.n*v > a.d) return false; return true; }
		
		internal static bool ContainsVert (Polygon sa, Vec2 v)
			{ foreach (Axis a in sa.axisP) if (a.n*v > a.d) return false; return true; }
		#endregion
	}
}
