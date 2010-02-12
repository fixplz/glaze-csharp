
using System;
using System.Collections.Generic;
using System.Linq;
using System.Drawing;

namespace Glaze
{
	public static class Tools
	{
		public static Body CircleBody (double x, double y, double r)
		{
			var b = new Body { pos=new Vec2 (x,y) };
			b.AddShape (CircleShape (0,0, r)); b.CalcProperties ();
			return b;
		}
		
		public static Circle CircleShape (double x, double y, double r)
		{
			var c = new Circle { offset=new Vec2 (x,y), radius=r };
			c.CalcMass (1);
			return c;
		}
		
		public static Body BoxBody (double x, double y, double angle, double width, double height)
		{
			var b = new Body { pos=new Vec2 (x,y) };
			b.AddShape (BoxShape (0,0, angle, width, height)); b.CalcProperties ();
			return b;
		}
		
		public static Polygon BoxShape (double x, double y, double angle, double width, double height)
		{
			Vec2[] verts =
				{new Vec2 (0,0),          new Vec2 (0,height),
				 new Vec2 (width,height), new Vec2 (width,0)};
			
			Vec2 offset = new Vec2 (x,y), rot = Vec2.Polar (angle), half = new Vec2 (width/2,height/2);
			
			for (int i=0; i<verts.Length; i++)
				verts [i] = offset + (verts [i] - half).Rotate (rot);
			
			var p = new Polygon (verts); p.CalcMass (1);
			return p;
		}
		
		
		public static void DrawWorld (Space sp, Graphics g)
		{
			foreach (Shape s in sp.shapes)
			{
				Pen p = new Pen (s.body.arbiters.Count>0 ? Color.Red : Color.Black, 2);
				
				try
				{
					if (s.shapeType == Shape.ShapeType.Circle)
					{
						Circle c = (Circle) s; Vec2 rot = c.radius * c.body.dir;
						g.DrawEllipse (p,  AsRectangle (c.aabb));
						g.DrawLine    (p,  AsPoint (c.pos + 0.4*rot),  AsPoint (c.pos + rot));
					}
					else if (s.shapeType == Shape.ShapeType.Polygon)
					{
						Vec2[] verts = ((Polygon) s).vertP;
						g.DrawLines (p,   verts.Select (v => AsPoint (v)).ToArray ());
						g.DrawLine  (p,   AsPoint (verts.First ()),   AsPoint (verts.Last ()));
					}
				}
				catch (OverflowException) {}
			}
			
			foreach (Arbiter arb in sp.arbiters)
				foreach (Contact c in arb.Contacts ())
					g.FillEllipse (Brushes.Green, (float)c.p.x-2, (float)c.p.y-2, 4,4);
		}
		
		public static void DrawRays (IEnumerable<Ray> rays, Graphics g)
		{
			foreach (Ray r in rays)
			{
				if (r.shape == null) continue; var hit = r.origin+r.dist*r.dir;
				g.DrawLine (new Pen (Color.Blue,  1), AsPoint (r.origin), AsPoint (hit));
				g.DrawLine (new Pen (Color.Green, 2), AsPoint (hit), AsPoint (hit+20*r.normal));
			}
		}
		
		
		public static PointF AsPoint (Vec2 v)
			{ return new PointF ((float)v.x, (float)v.y); }
		
		public static RectangleF AsRectangle (AABB aabb) 
			{ return new RectangleF ((float)aabb.l, (float)aabb.t, (float)aabb.Width, (float)aabb.Height); }
	}
}
