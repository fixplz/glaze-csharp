
using System;
using System.Collections.Generic;

namespace Glaze
{
	public class Body : Entry <Body>
	{
		#region PROPERTIES
		// motion components
		public Vec2   pos,   vel;
		public double angle, w;
		
		internal Vec2   velBias, forces;
		internal double wBias,   torque;
		
		public Vec2 gravity;
		
		// local transform
		public Vec2 rot;
		
		// mass and inertia
		public double massInv, inertiaInv;
		
		// sleep components
		internal double  motion   = 0.0002;
		internal bool    sleeping = false;
		
		// bookkeeping
		public LinkedList<Shape>    shapes;
		public LinkedList<Arbiter>  arbiters;
		
		public uint group = 0;
		#endregion
		
		public Body ()
		{
			shapes   = new LinkedList<Shape> ();
			arbiters = new LinkedList<Arbiter> ();
		}
		
		public void AddShape    (Shape s) { shapes.AddFirst (s); s.body = this; }
		public void RemoveShape (Shape s) { shapes.Remove (s); s.Remove (); }
		
		public void CalcProperties ()
		{
			double mass = 0, inertia = 0;
			foreach (Shape s in shapes)
				{ mass += s.mass; inertia += s.mass*s.Inertia; }
			massInv = 1.0/mass; inertiaInv = 1.0/inertia;
		}
		
		#region INTERNAL CONTROLS
		internal void UpdateVelocity (double dt)
		{
			double damping = 1; // TODO damping
			vel = damping * vel + dt * (gravity + massInv * forces);
			w   = damping * w   + dt * inertiaInv * torque;
		}
		
		internal void UpdatePosition (double dt)
		{
			pos += dt * (vel + velBias); angle += dt * (w + wBias); rot = Vec2.Polar (angle);
			//motion = Config.motionBias * motion + (1 - Config.motionBias) * (vel.LengthSq + w*w);
			velBias.Clear (); wBias = 0;
		}
		
		internal void ApplyImpulse (Vec2 j, Vec2 r)
		{
			vel += massInv * j;
			w   += inertiaInv * (r * j.Right);
		}
		
		internal void ApplyBiasImpulse (Vec2 j, Vec2 r)
		{
			velBias += massInv * j;
			wBias   += inertiaInv * (r * j.Right);
		}
		
		internal void ApplyForce (Vec2 f, Vec2 r)
		{
			forces += f;
			torque += (r * f.Right);
		}
		#endregion
	}
	
	
	
	public class Arbiter : Entry <Arbiter>
	{
		public Shape sa,sb;
		public double u = 0, e = 0;
		
		public uint stamp;
		public LinkedList<Contact> contacts;
		
		internal Arbiter (Shape sa, Shape sb)
		{
			this.sa = sa; this.sb = sb;
			contacts = new LinkedList<Contact> ();
			e = sa.material.restitution * sb.material.restitution;
			u = sa.material.friction    * sb.material.friction;
		}
		
		internal void UpdateContact (Vec2 p, Vec2 n, double dist, uint id)
		{
			Contact c = null;
			foreach (Contact x in contacts) if (x.id == id) { c=x; break; }
			if (c == null) { c = new Contact (id); c.Attach (contacts); }
			
			c.p = p; c.n = n; c.dist = dist;
			c.updated = true;
		}
		
		internal override void Remove ()
		{
			base.Remove ();
			sa.body.arbiters.Remove (this);
			sb.body.arbiters.Remove (this);
		}
		
		#region RESOLUTION MATH
		internal void Prestep (double dtInv)
		{
			Body a = sa.body, b = sb.body;
			
			for (LinkedListNode<Contact> n = contacts.First; n != null; n = n.Next)
			{
				Contact c = n.Value;
				if (!c.updated) { contacts.Remove (n); continue; }
				c.updated = false;
				
				c.r1     = c.p - a.pos;
				c.r2     = c.p - b.pos;
				
				c.nMass  = 1.0 / Calc.KScalar (a, b, c.r1, c.r2, c.n);
				c.tMass  = 1.0 / Calc.KScalar (a, b, c.r1, c.r2, c.n.Left);
				
				c.jBias  = 0;
				c.bias   = - dtInv * Config.resolveBias * Math.Min (0, c.dist + Config.resolveSlop);
				
				c.bounce = e * c.n * ((b.w * c.r2.Left + b.vel) - (a.w * c.r1.Left + a.vel));
				
				Vec2 cjT = c.jnAcc * c.n + c.jtAcc * c.n.Left;
				
				a.ApplyImpulse (-cjT, c.r1);
				b.ApplyImpulse ( cjT, c.r2);
			}
		}
		
		internal void Perform ()
		{
			Body a = sa.body, b = sb.body;
			
			foreach (Contact c in contacts)
			{
				double old, jbn, jn, jtMax, jt;
				Vec2 vb, cjT, vr;
				
				vb       = (b.wBias * c.r2.Left + b.velBias) - (a.wBias * c.r1.Left + a.velBias);
				
				jbn      = c.nMass * (c.bias - vb * c.n);
				old      = c.jBias;
				c.jBias  = Math.Max (0, c.jBias + jbn);
				jbn      = c.jBias - old;
				
				cjT      = jbn * c.n;
				
				a.ApplyBiasImpulse (-cjT, c.r1);
				b.ApplyBiasImpulse ( cjT, c.r2);
				
				vr       = (b.w * c.r2.Left + b.vel) - (a.w * c.r1.Left + a.vel);
				
				jn       = - c.nMass * (c.bounce + vr * c.n);
				old      = c.jnAcc;
				c.jnAcc  = Math.Max (0, c.jnAcc + jn);
				jn       = c.jnAcc - old;
				
				jtMax    = u * c.jnAcc;
				jt       = -c.tMass * (vr * c.n.Left);
				old      = c.jtAcc;
				c.jtAcc  = Math.Min (jtMax, Math.Max (-jtMax, c.jtAcc + jt));
				jt       = c.jtAcc - old;
				
				cjT      = c.n.Rotate (new Vec2 {x=jn,y=jt});
				
				a.ApplyImpulse (-cjT, c.r1);
				b.ApplyImpulse ( cjT, c.r2);
			}
		}
		#endregion
	}
	
	
	
	public class Contact : Entry<Contact>
	{
		public Vec2 p, n, r1, r2;
		internal uint id; internal bool updated;
		internal double dist, nMass, tMass, bounce, jnAcc=0, jtAcc=0, jBias, bias;
		internal Contact (uint id) { this.id = id; }
	}
	
	
	
	internal static class Calc
	{
		#region AUX
		internal static double KScalar (Body a, Body b, Vec2 r1, Vec2 r2, Vec2 n)
		{
			double r1xn = r1.Cross (n), r2xn = r2.Cross (n);
			return a.massInv+b.massInv + a.inertiaInv*r1xn*r1xn + b.inertiaInv*r2xn*r2xn;
		}
		#endregion
		
		#region COLLISIONS
		internal static bool Check (Shape sa, Shape sb, Arbiter arb)
		{
			switch ((int)((int)sa.shapeType*5 + sb.shapeType))
			{
				case 0: return Circle2Circle ((Circle) sa,  (Circle) sb,  arb);
				case 1: return Circle2Poly   ((Circle) sa,  (Polygon) sb, arb);
				case 6: return Poly2Poly     ((Polygon) sa, (Polygon) sb, arb);
			}
			
			return false;
		}
		
		internal static bool Circle2Circle (Circle sa, Circle sb, Arbiter arb)
			{ return Circle2Circle (sa.pos, sb.pos, sa.radius, sb.radius, arb); }
		
		internal static bool Circle2Circle (Vec2 c1, Vec2 c2, double r1, double r2, Arbiter arb)
		{
			Vec2 d = c2 - c1; double min = r1 + r2, dist = d.LengthSq, distInv;
			if (dist >= min*min) return false; dist = Math.Sqrt (dist); distInv = 1.0/dist;
			arb.UpdateContact (c1 + (0.5 + distInv * (r1 - min/2)) * d, distInv*d, dist - min, 0);
			return true;
		}
		
		internal static bool Circle2Poly   (Circle circle, Polygon poly, Arbiter arb)
		{
			
			int len = poly.axisP.Length, ix = 0;
			double max = Double.NegativeInfinity;
			
			for (int i=0; i<len; i++)
			{
				double dist = poly.axisP [i].n * circle.pos - poly.axisP [i].d - circle.radius;
				if (dist > 0) return false;
				if (dist > max) { max = dist; ix = i; }
			}
			
			Vec2 v = poly.vertP [ix], u = poly.vertP [(ix+1)%len]; Axis a = poly.axisP [ix];
			
			double d = a.n*circle.pos;
			
			//if (d <  a.n.Cross (u)) return Circle2Circle (circle.pos, u, circle.radius, 0, arb);
			//if (d >= a.n.Cross (v)) return Circle2Circle (circle.pos, v, circle.radius, 0, arb);
			
			arb.UpdateContact (circle.pos - (circle.radius+max/2) * a.n, -a.n, max, 0);
			return true;
		}
		
		internal static bool Poly2Poly  (Polygon sa, Polygon sb, Arbiter arb)
		{
			Axis a1, a2;
			if (!(MinSepAxis (sa,sb, out a1) && MinSepAxis (sb,sa, out a2))) return false;
			FindVerts (sa,sb, a1.d > a2.d ? a1 : -a2, arb);
			return true;
		}
		
		internal static bool MinSepAxis (Polygon sa, Polygon sb, out Axis axis)
		{
			axis = new Axis {d=Double.NegativeInfinity};
			
			foreach (Axis a in sa.axisP)
			{
				double min = Double.PositiveInfinity;
				foreach (Vec2 v in sb.vertP) min = Math.Min (min, a.n*v);
				min -= a.d;
				
				if (min > 0) return false;
				if (min > axis.d) axis = new Axis {n=a.n,d=min};
			}
			
			return true;
		}
		
		internal static void FindVerts  (Polygon sa, Polygon sb, Axis a, Arbiter arb)
		{
			uint id = sa.id > sb.id ? 0u : 65000u;
			foreach (Vec2 v in sa.vertP)
				{ if (ContainsVert (sb,v,-a.n)) arb.UpdateContact (v, a.n, a.d, id); id++; }
			
			id = sa.id > sb.id ? 65000u : 0u;
			foreach (Vec2 v in sb.vertP)
				{ if (ContainsVert (sa,v,a.n))  arb.UpdateContact (v, a.n, a.d, id); id++; }
		}
		
		internal static bool ContainsVert (Polygon sa, Vec2 v, Vec2 n)
		{
			foreach (Axis a in sa.axisP) if (/*a.n*n > 0 &&*/ a.n*v > a.d) return false;
			return true;
		}
		#endregion
	}
}
