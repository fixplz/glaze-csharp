
using System;
using System.Collections.Generic;

namespace Glaze
{
	public sealed class Contact
	{
		internal static readonly Stack<Contact> pool = new Stack<Contact> (100);
		internal static Contact Assign () { return pool.Count != 0 ? pool.Pop () : new Contact (); }
		internal static void Retire (Contact c) { pool.Push (c); }
		
		// position, penetration, normal and tangent impulses
		public Vec2 p;
		public double dist, jnAcc,jtAcc;
		public Connection con;
		
		internal double nMass,tMass, bounce, bias,jBias;
		internal uint id;
		internal bool updated;
		
		#region STEP MATH
		internal void Prestep (Arbiter arb)
		{
			Body a = arb.sa.body, b = arb.sb.body;
			
			con.r1 = p - a.pos; con.r2 = p - b.pos;
			
			nMass = 1.0 / Calc.KScalar (a,b, con, con.n);
			tMass = 1.0 / Calc.KScalar (a,b, con, con.n.Left);
			
			jBias = 0; bias = Calc.BiasDist (dist);
			bounce = arb.bounce * con.n * Calc.RelativeVelocity (a,b, con);
			
			Calc.ContactImpulse (jnAcc,jtAcc, con, a,b);
		}
		
		internal void Perform (Arbiter arb)
		{
			Body a = arb.sa.body, b = arb.sb.body;
			double jbn, jn, jt; Vec2 vb, vr;
			
			vb   = Calc.RelativeBiasVelocity (a,b, con);
			vr   = Calc.RelativeVelocity     (a,b, con);
			
			jbn  = nMass * (vb * con.n - bias);
			jn   = nMass * (vr * con.n + bounce);
			jt   = tMass * (vr * con.n.Left);
			
			Calc.AddPositive (ref jBias, ref jbn);
			Calc.AddPositive (ref jnAcc, ref jn);
			Calc.AddClamp    (ref jtAcc, ref jt, arb.stick * jnAcc);
			
			Calc.NormalBias     (jbn,   con, a,b);
			Calc.ContactImpulse (jn,jt, con, a,b);
		}
		#endregion
	}
	
	
	
	public abstract class Joint : Entry<Joint>
	{
		public Body a,b;
		public Connection con;
		
		internal Vec2 a1,a2;
		
		internal Joint (Body a, Body b, Vec2 a1, Vec2 a2)
		{
			this.a  = a;  this.b  = b;
			this.a1 = (a1-a.pos); this.a2 = (a2-b.pos);
		}
		
		internal abstract void Prestep ();
		internal abstract void Perform ();
	}
	
	
	public class DistanceJoint : Joint
	{
		public   double anchordist, jnAcc;
		internal double nMass, bias;
		
		public DistanceJoint (Body a, Body b, Vec2 anchor1, Vec2 anchor2) : base (a,b,anchor1,anchor2)
			{ anchordist = (anchor1-anchor2).Length; }
		
		#region STEP MATH
		internal override void Prestep ()
		{
			con.r1 = a1.Rotate (a.dir); con.r2 = a2.Rotate (b.dir);
			
			Vec2 d = (b.pos + con.r2) - (a.pos + con.r1);
			double dist = d.Length;
			
			con.n = dist == 0 ? new Vec2 () : 1/dist * d;
			nMass = 1.0 / Calc.KScalar (a,b, con, con.n);
			
			bias = Calc.BiasJoint (dist - anchordist);
			
			Calc.NormalImpulse (jnAcc, con, a,b);
		}
		
		internal override void Perform ()
		{
			Vec2 vb,vr; double jbn, jn;
			
			vb  = Calc.RelativeBiasVelocity (a,b, con);
			vr  = Calc.RelativeVelocity     (a,b, con);
			
			jbn = nMass * (vb * con.n - bias);
			jn  = nMass * (vr * con.n);
			
			jnAcc += jn;
			
			Calc.NormalBias    (jbn, con, a,b);
			Calc.NormalImpulse (jn,  con, a,b);
		}
		#endregion
	}
	
	
	
	public struct Connection
	{
		public Vec2 r1,r2, n;
	}
	
	
	
	internal static class Calc
	{
		#region AUX
		internal static double BiasDist  (double dist) { return Config.resolveRate * Math.Min (0, dist + Config.resolveSlop); }
		internal static double BiasJoint (double dist) { return Config.resolveRate * dist; }
		
		internal static void NormalBias    (double jbn, Connection c, Body a, Body b) { Vec2 j = jbn*c.n; a.ApplyBias    (-j, c.r1);  b.ApplyBias    (j, c.r2); }
		internal static void NormalImpulse (double jn,  Connection c, Body a, Body b) { Vec2 j = jn*c.n;  a.ApplyImpulse (-j, c.r1);  b.ApplyImpulse (j, c.r2); }
		
		internal static void ContactImpulse (double jn, double jt, Connection c, Body a, Body b) { Vec2 j = new Vec2 (jn,jt).Rotate (c.n); a.ApplyImpulse (-j, c.r1);  b.ApplyImpulse (j, c.r2); }
		
		internal static void AddPositive (ref double old, ref double change) { change = Math.Max (-old, change); old += change; }
		internal static void AddClamp    (ref double old, ref double change, double limit) { double result = Math.Max (-limit, Math.Min (limit, old+change)); change = result-old; old = result; }
		
		internal static Vec2 RelativeVelocity     (Body a, Body b, Connection c) { return (a.rot     * c.r1.Left + a.vel)     - (b.rot     * c.r2.Left + b.vel); }
		internal static Vec2 RelativeBiasVelocity (Body a, Body b, Connection c) { return (a.rotBias * c.r1.Left + a.velBias) - (b.rotBias * c.r2.Left + b.velBias); }
		
		internal static double KScalar (Body a, Body b, Connection c, Vec2 n) { return a.massInv+b.massInv + a.inertiaInv * c.r1.Cross (n).Sq () + b.inertiaInv * c.r2.Cross (n).Sq (); }
		
		internal static bool ContainsVert (Polygon sa, Vec2 v, Vec2 n) { foreach (Axis a in sa.axisP) if (a.n*n <= 0 && a.n*v > a.d) return false; return true; }
		internal static bool ContainsVert (Polygon sa, Vec2 v)         { foreach (Axis a in sa.axisP) if (a.n*v > a.d)               return false; return true; }
		
		internal static double Sq (this double x) { return x*x; }
		#endregion
		
		
		#region COLLISIONS
		internal static bool Check (Shape sa, Shape sb, ref Arbiter arb)
		{
			switch ((int)((int)sa.shapeType*5 + sb.shapeType))
			{
				case 0: return Circle2Circle ((Circle)  sa, (Circle)  sb, ref arb);
				case 1: return Circle2Poly   ((Circle)  sa, (Polygon) sb, ref arb);
				case 6: return Poly2Poly     ((Polygon) sa, (Polygon) sb, ref arb);
			}
			
			return false;
		}
		
		internal static bool Circle2Circle (Circle sa, Circle sb, ref Arbiter arb)
			{ return CircleContact (sa.center, sb.center, sa.radius, sb.radius, ref arb); }
		
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
				double dist = poly.axisP [i].n * circle.center - poly.axisP [i].d - circle.radius;
				if (dist > 0) return false; if (dist > max) { max = dist; ix = i; }
			}
			
			Vec2 v = poly.vertP [ix], u = poly.vertP [(ix+1)%len]; Axis a = poly.axisP [ix];
			
			double d = a.n.Cross (circle.center);
			
			if (d > a.n.Cross (v)) return CircleContact (circle.center, v, circle.radius, 0, ref arb);
			if (d < a.n.Cross (u)) return CircleContact (circle.center, u, circle.radius, 0, ref arb);
			
			if (arb == null) arb = new Arbiter (1);
			arb.UpdateContact (circle.center - (circle.radius+max/2) * a.n, new Axis {n=-a.n, d=max}, 0);
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
