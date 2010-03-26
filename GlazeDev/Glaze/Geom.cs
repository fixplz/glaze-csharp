
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
		public double damping = 0.999;
		
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
		
		public void ImpulseAtPoint (Vec2 j, Vec2 p) { ApplyImpulse (j, p-pos); }
		
		public void CalcProperties ()
		{
			double mass = 0, inertia = 0;
			foreach (Shape s in shapes) { mass += s.mass; inertia += s.mass*s.Inertia; }
			massInv = 1.0/mass; inertiaInv = 1.0/inertia;
		}
		
		public IEnumerable<Body> Touching
		{ get {
			Stack<Body> s = new Stack<Body> ();
			foreach (Arbiter arb in arbiters)
			{
				Body other = arb.GetOther (this);
				if (!s.Contains (other)) { s.Push (other); yield return other; }
			}
		} }
		
		internal void UpdateVelocity (double dt)
		{
			vel = damping * vel + dt * gravity;
			rot = damping * rot;
		}
		
		internal void UpdatePosition (double dt)
		{
			pos += dt*vel + velBias; dir = Vec2.Polar (angle += dt*rot + rotBias);
			velBias.x = velBias.y = rotBias = 0;
		}
		
		internal void ApplyImpulse (Vec2 j, Vec2 r) { vel     += massInv*j; rot     -= inertiaInv*j.Cross (r); }
		internal void ApplyBias    (Vec2 j, Vec2 r) { velBias += massInv*j; rotBias -= inertiaInv*j.Cross (r); }
		#endregion
	}
	
	
	
	public abstract class Shape : Entry <Shape>
	{
		public ShapeType  shapeType;
		public Body       body;
		public AABB       aabb;
		public double     mass;
		
		public double friction = Config.defaultFriction, restitution = Config.defaultRestitution;
		
		public void CalcMass (double density) { mass = density * Area * Config.areaMassRatio; }
		
		public abstract double Area    { get; }
		public abstract double Inertia { get; }
		
		public abstract bool ContainsPoint (Vec2 v);
		public abstract void IntersectRay  (Ray ray);
		
		internal abstract void UpdateShape ();
		
		public enum ShapeType { Circle, Polygon }
	}
	
	
	
	public sealed class Circle : Shape
	{
		public double  radius;
		public Vec2    offset, center;
		
		#region DETAILS
		public Circle () { shapeType = ShapeType.Circle; }
		
		public override double Area    { get { return radius * radius * Math.PI; } }
		public override double Inertia { get { return radius*radius/2 + offset.Sq; } }
		
		internal override void UpdateShape()
			{ aabb.SetExtents (center = body.pos + offset.Rotate (body.dir), new Vec2 {x=radius, y=radius}); }
		
		public override bool ContainsPoint (Vec2 v) { return (v-center).Sq < radius*radius; }
		
		public override void IntersectRay (Ray ray)
		{
			Vec2 r = center - ray.origin;
			double slope = r*ray.dir;                          if (slope < 0) return;
			double D     = radius*radius + slope*slope - r*r;  if (D     < 0) return;
			double dist  = slope - Math.Sqrt (D);              if (dist  < 0) return;
			
			ray.Report (this, dist, (-r + dist*ray.dir).Normalize (1));
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
			double inner = Double.PositiveInfinity, outer = 0;
			
			for (int i=0; i<len; i++)
			{
				Axis a = axisP [i];
				
				double proj = a.n*r.origin - a.d, slope = -a.n*r.dir; if (slope == 0) continue;
				double dist = proj/slope;
				
				if (slope > 0) { if (dist > inner) return; if (dist > outer) { outer = dist; ix = i; } }
				else           { if (dist < outer) return; if (dist < inner)   inner = dist;           }
			}
			
			if (ix == -1) return; r.Report (this, outer, axisP [ix].n);
		}
		#endregion
	}
	
	
	
	public struct AABB
	{
		public double t,b,l,r;
		
		public double Width  { get { return r-l; } }
		public double Height { get { return b-t; } }
		
		public AABB SetExtents (Vec2 c, Vec2 ext) { SetRange (c-ext, c+ext); return this; }
		public AABB SetRange   (Vec2 v, Vec2 u)   { t=v.y; b=u.y; l=v.x; r=u.x; return this; }
		
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
