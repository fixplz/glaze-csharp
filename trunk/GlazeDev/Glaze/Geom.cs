
using System;
using System.Collections.Generic;

namespace Glaze
{
	public class Body : Entry <Body>
	{
		// motion components
		public Vec2   pos,   vel;
		public double angle, rot;
		
		internal Vec2   velBias;
		internal double rotBias;
		
		public Vec2 gravity;
		
		// local transform
		public Vec2 dir;
		
		// mass and inertia
		public double massInv, inertiaInv;
		
		// bookkeeping
		public LinkedList<Shape>    shapes;
		public LinkedList<Arbiter>  arbiters;
		
		public uint group = 0;
		
		#region DETAILS
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
			foreach (Shape s in shapes) { mass += s.mass; inertia += s.mass*s.Inertia; }
			massInv = 1.0/mass; inertiaInv = 1.0/inertia;
		}
		
		public IEnumerable<Body> Touching ()
		{
			Stack<Body> s = new Stack<Body> ();
			foreach (Arbiter arb in arbiters)
			{
				Body other = arb.GetOther (this);
				if (!s.Contains (other)) { s.Push (other); yield return other; }
			}
		}
		
		internal void UpdateVelocity (double dt)
		{
			double damping = 0.997; // TODO damping
			vel = damping * vel + dt * gravity;
			rot = damping * rot;
		}
		
		internal void UpdatePosition (double dt)
		{
			pos += dt * (vel + velBias); angle += dt * (rot + rotBias); dir = Vec2.Polar (angle);
			velBias.x = velBias.y = rotBias = 0;
		}
		
		public   void ApplyImpulse (Vec2 j, Vec2 r) { vel     += massInv*j; rot     -= inertiaInv*j.Cross (r); }
		internal void ApplyBias    (Vec2 j, Vec2 r) { velBias += massInv*j; rotBias -= inertiaInv*j.Cross (r); }
		#endregion
	}
	
	
	
	public abstract class Shape : Entry <Shape>
	{
		public Material   material = Config.defaultMaterial;
		public ShapeType  shapeType;
		public Body       body;
		public AABB       aabb;
		public double     mass;
		
		public void CalcMass (double density) { mass = density * Area * Config.areaMassRatio; }
		
		public abstract double Area    { get; }
		public abstract double Inertia { get; }
		
		public abstract bool ContainsPoint (Vec2 v);
		public abstract void IntersectRay  (Ray r);
		
		internal abstract void UpdateShape ();
		
		public enum ShapeType { Circle, Polygon }
	}
	
	
	
	public sealed class Circle : Shape
	{
		public double  radius;
		public Vec2    offset, pos;
		
		#region DETAILS
		public Circle () { shapeType = ShapeType.Circle; }
		
		public override double Area    { get { return radius * radius * Math.PI; } }
		public override double Inertia { get { return radius*radius/2 + offset.Sq; } }
		
		internal override void UpdateShape()
			{ aabb.SetExtents (pos = body.pos + offset.Rotate (body.dir), new Vec2 {x=radius, y=radius}); }
		
		public override bool ContainsPoint (Vec2 v) { return (v-pos).Sq < radius*radius; }
		
		public override void IntersectRay (Ray r)
		{
			var dist = r.origin - pos;
			var b = dist*r.dir;
			if (b>0) return;
			
			var d = radius*radius - (dist.Sq - b*b);
			if (d<0) return; d = -b-Math.Sqrt (d);
			
			r.Report (this, d, (r.origin + d*r.dir - pos).Normalize (1));
		}
		#endregion
	}
	
	
	
	public sealed class Polygon : Shape
	{
		public Vec2[] vertL, vertP;
		public Axis[] axisL, axisP;
		
		internal uint id;
		internal static uint next = 0;
		
		#region DETAILS
		public Polygon (Vec2[] vertices)
		{
			id = next++;
			shapeType = ShapeType.Polygon;
			
			int len = vertices.Length;
			
			vertL = vertices;
			vertP = new Vec2 [len];
			axisL = new Axis [len];
			axisP = new Axis [len];
			
			for (int i=0; i<len; i++)
			{
				Vec2 v = vertices [i], u = vertices [(i+1)%len], n = (u-v).Left.Normalize (1);
				axisL [i] = new Axis {n=n, d=n*v};
			}
		}
		
		public override double Area
		{
			get
			{
				double s=0; int len = vertL.Length;
				
				for (int i=0; i<len; i++)
				{
					Vec2 v = vertL [i], u = vertL [(i+1)%len], w = vertL [(i+2)%len];
					s += u.x * (v.y-w.y);
				}
				
				return s/2;
			}
		}
		
		public override double Inertia
		{
			get
			{
				double s1=0, s2=0; int len = vertL.Length;
				
				for (int i=0; i<len; i++)
				{
					Vec2 v = vertL [i], u = vertL [(i+1)%len];
					double a = u.Cross (v), b = v*v + v*u + u*u;
					s1 += a*b; s2 += a;
				}
				
				return s1 / (6*s2);
			}
		}
		
		internal override void UpdateShape()
		{
			AABB box = new AABB ();
			
			vertP [0] = body.pos + vertL [0].Rotate (body.dir);
			box.t = box.b = vertP [0].y;
			box.l = box.r = vertP [0].x;
			
			for (int i=1; i<vertL.Length; i++)
			{
				vertP [i] = body.pos + vertL [i].Rotate (body.dir);
				box.t = Math.Min (box.t, vertP [i].y);
				box.b = Math.Max (box.b, vertP [i].y);
				box.l = Math.Min (box.l, vertP [i].x);
				box.r = Math.Max (box.r, vertP [i].x);
			}
			
			aabb = box;
			
			for (int i=0; i<axisL.Length; i++)
			{
				axisP [i].n = axisL [i].n.Rotate (body.dir);
				axisP [i].d = axisP [i].n * body.pos + axisL [i].d;
			}
		}
		
		public override bool ContainsPoint(Vec2 v) { foreach (Axis a in axisP) if (a.n*v > a.d) return false; return true; }
		
		public override void IntersectRay(Ray r)
		{
			int len = vertP.Length, ix = -1;
			double far = Double.PositiveInfinity, near = 0;
			
			for (int i=0; i<len; i++)
			{
				Vec2 v = vertP [i], n = axisP [i].n;
				double dist = (v-r.origin)*n, slope = r.dir*n;
				
				if (slope == 0) continue;
				double clip = dist/slope;
				
				if (slope < 0) { if (clip > far)  return; if (clip > near) { near = clip; ix = i; } }
				else           { if (clip < near) return; if (clip < far)    far  = clip; }
			}
			
			if (ix == -1) return; Axis a = axisP [ix];
			r.Report (this, -(r.origin*a.n - a.d) / (r.dir*a.n), a.n);
		}
		#endregion
	}
	
	
	
	public struct Material
	{
		public double restitution, friction;
	}
	
	
	
	public struct AABB
	{
		public double t,b,l,r;
		
		public double Width  { get { return r-l; } }
		public double Height { get { return b-t; } }
		
		public void SetExtents (Vec2 c, Vec2 ext) { SetRange (c-ext, c+ext); }
		public void SetRange   (Vec2 v, Vec2 u)   { t=v.y; b=u.y; l=v.x; r=u.x; }
		
		public bool IntersectH (AABB x)  { return x.l < r && l < x.r; }
		public bool Intersect  (AABB x)  { return IntersectH (x) && x.t < b && t < x.b; }
	}
	
	
	
	public struct Axis
	{
		public Vec2 n; public double d;
		public static Axis operator - (Axis a) { return new Axis {n=-a.n,d=a.d}; }
	}
	
	
	public class Ray
	{
		public Vec2 origin,dir, normal;
		public double dist, range;
		public Shape shape;
		
		public Ray (Vec2 origin, Vec2 target, double range)
		{
			this.origin = origin;
			this.range  = dist = range;
			dir = (target-origin).Normalize (1);
		}
		
		public void Reset () { dist = range; shape = null; normal.x = normal.y = 0; }
		
		public void Report (Shape s, double dist, Vec2 normal)
		{
			if (this.dist < dist) return;
			
			this.dist   = dist;
			this.normal = normal;
			this.shape  = s;
		}
	}
	
	
	public struct Vec2
	{
		public double x,y;
		
		public Vec2 (double x, double y) { this.x=x; this.y=y; }
		
		public override string ToString() { return String.Format ("({0},{1})", x,y); }
				
		public Vec2 Right { get { return new Vec2 {x=y,  y=-x}; } }
		public Vec2 Left  { get { return new Vec2 {x=-y, y=x }; } }
		public Vec2 Unit  { get { return (1/Length) * this;     } }
		
		public double Length { get { return Math.Sqrt (Sq); } }
		public double Sq     { get { return this * this;          } }
		
		public double Cross  (Vec2 b)  { return this * b.Right; }
		public Vec2   Rotate (Vec2 b)  { return new Vec2 {x=x*b.x - y*b.y, y=y*b.x + x*b.y}; }
		
		public Vec2 Normalize (double length) { return (length/Length) * this; }
		
		public static Vec2 Polar (double a) { return new Vec2 {x=Math.Cos(a), y=Math.Sin(a)}; }
		
		public static Vec2 operator + (Vec2 a, Vec2 b)   { return new Vec2 {x=a.x+b.x, y=a.y+b.y}; }
		public static Vec2 operator - (Vec2 a, Vec2 b)   { return new Vec2 {x=a.x-b.x, y=a.y-b.y}; }
		public static Vec2 operator * (double s, Vec2 v) { return new Vec2 {x=v.x*s,   y=v.y*s  }; }
		public static Vec2 operator - (Vec2 a)           { return new Vec2 {x=-a.x,    y=-a.y   }; }
		public static Vec2 operator * (Vec2 v, double s) { return s*v; }
		
		public static double operator * (Vec2 a, Vec2 b) { return a.x*b.x + a.y*b.y; }
	}
}
