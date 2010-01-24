
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
		internal Vec2 rot;
		
		// mass and inertia
		public double massInv, inertiaInv;
		
		// bookkeeping
		public LinkedList<Shape>    shapes;
		public LinkedList<Arbiter>  arbiters;
		
		public uint group = 0;
		#endregion
		
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
			foreach (Shape s in shapes)
				{ mass += s.mass; inertia += s.mass*s.Inertia; }
			massInv = 1.0/mass; inertiaInv = 1.0/inertia;
		}
		
		public IEnumerable<Body> Contacts ()
		{
			Stack<Body> s = new Stack<Body> ();
			foreach (Arbiter arb in arbiters)
			{
				Body other = arb.GetOther (this);
				if (!s.Contains (other)) { s.Push (other); yield return other; }
			}
		}
		#endregion
		
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
			velBias.Clear (); wBias = 0;
		}
		
		internal void ApplyImpulse     (Vec2 j, Vec2 r) { vel     += massInv * j; w     += inertiaInv * (r * j.Right); }
		internal void ApplyBiasImpulse (Vec2 j, Vec2 r) { velBias += massInv * j; wBias += inertiaInv * (r * j.Right); }
		internal void ApplyForce       (Vec2 f, Vec2 r) { forces  += f; torque += (r * f.Right); }
		#endregion
	}
	
	
	
	public abstract class Shape : Entry <Shape>
	{
		public Material   material = Config.defaultMaterial;
		public ShapeType  shapeType;
		public Body       body;
		public AABB       aabb;
		public double     mass;
		
		public void CalcMass (double density)
			{ mass = density * Area * Config.areaMassRatio; }
		
		public abstract double Area    { get; }
		public abstract double Inertia { get; }
		
		internal abstract void UpdateShape ();
		
		public enum ShapeType { Circle, Polygon }
	}
	
	
	
	public class Circle : Shape
	{
		public double  radius;
		public Vec2    offset, pos;
		
		#region DETAILS
		public Circle () { shapeType = ShapeType.Circle; }
		
		public override double Area    { get { return radius * radius * Math.PI; } }
		public override double Inertia { get { return radius*radius/2 + offset.Length; } }
		
		internal override void UpdateShape()
		{
			pos = body.pos + offset.Rotate (body.rot);
			aabb.SetExtents (pos, new Vec2 {x=radius, y=radius});
		}
		#endregion
	}
	
	
	
	public class Polygon : Shape
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
			
			vertP [0] = body.pos + vertL [0].Rotate (body.rot);
			box.t = box.b = vertP [0].y;
			box.l = box.r = vertP [0].x;
			
			for (int i=1; i<vertL.Length; i++)
			{
				vertP [i] = body.pos + vertL [i].Rotate (body.rot);
				box.t = Math.Min (box.t, vertP [i].y);
				box.b = Math.Max (box.b, vertP [i].y);
				box.l = Math.Min (box.l, vertP [i].x);
				box.r = Math.Max (box.r, vertP [i].x);
			}
			
			aabb = box;
			
			for (int i=0; i<axisL.Length; i++)
			{
				axisP [i].n = axisL [i].n.Rotate (body.rot);
				axisP [i].d = axisP [i].n * body.pos + axisL [i].d;
			}
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
		
		public void SetExtents (Vec2 center, Vec2 extents) { SetRange (center-extents, center+extents); }
		public void SetRange (Vec2 v, Vec2 u)              { t=v.y; b=u.y; l=v.x; r=u.x; }
		
		public bool IntersectH (AABB x)                    { return x.l < r && l < x.r; }
		public bool Intersect  (AABB x)                    { return IntersectH (x) && x.t < b && t < x.b; }
	}
	
	
	public struct Axis
	{
		public Vec2 n; public double d;
		public static Axis operator - (Axis a) { return new Axis {n=-a.n,d=a.d}; }
	}
	
	
	
	public struct Vec2
	{
		public double x,y;
		
		public Vec2 (double x, double y) { this.x=x; this.y=y; }
		
		public override string ToString() { return String.Format ("({0},{1})", x,y); }
		
		public void Clear () { x=0; y=0; }
				
		public Vec2 Right { get { return new Vec2 {x=y,  y=-x}; } }
		public Vec2 Left  { get { return new Vec2 {x=-y, y=x }; } }
		public Vec2 Unit  { get { return (1/Length) * this;     } }
		
		public double Length   { get { return Math.Sqrt (LengthSq); } }
		public double LengthSq { get { return this * this;          } }
		
		public double Cross  (Vec2 b)  { return this * b.Right; }
		public Vec2   Rotate (Vec2 b)  { return new Vec2 {x=x*b.x - y*b.y, y=y*b.x + x*b.y}; }
		
		public Vec2 Normalize (double length) { return (length/Length) * this; }
		
		public static Vec2 Polar (double a) { return new Vec2 {x=Math.Cos(a), y=Math.Sin(a)}; }
		
		public static Vec2 operator + (Vec2 a, Vec2 b)   { return new Vec2 {x=a.x+b.x, y=a.y+b.y}; }
		public static Vec2 operator - (Vec2 a, Vec2 b)   { return new Vec2 {x=a.x-b.x, y=a.y-b.y}; }
		public static Vec2 operator * (double s, Vec2 v) { return new Vec2 {x=v.x*s,   y=v.y*s  }; }
		public static Vec2 operator - (Vec2 a)           { return new Vec2 {x=-a.x,    y=-a.y   }; }
		
		public static double operator * (Vec2 a, Vec2 b) { return a.x*b.x + a.y*b.y; }
	}
}
