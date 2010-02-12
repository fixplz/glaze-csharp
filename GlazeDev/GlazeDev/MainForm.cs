
using System;
using System.Drawing;
using System.Windows.Forms;
using System.Linq;

using Glaze;

namespace GlazeDev
{
	public partial class MainForm : Form
	{
		public Space space;
		public Shape selected;
		public Vec2  mouse,offset, gravity;
		public Ray[] rays;
		
		public MainForm()
		{
			InitializeComponent();
			
			DoubleBuffered = true;
			
			space = new SortedSpace (); space.iterations = 10;
			gravity = new Vec2 (0,400);
			
			int rnum = 20, spacing = 50;
			rays = new Ray [rnum];
			for (var i=0; i<rnum; i++)
				rays [i] = new Ray (new Vec2 (500+i*spacing-rnum*spacing/2,250), new Vec2 (500,1000), 1000);
			
			var ground = new Body { pos=new Vec2 (300,500) };
			ground.AddShape (Tools.BoxShape (0,0,         0,  1000,100));
			ground.AddShape (Tools.BoxShape (-550,-150, -0.4, 100,400));
			ground.AddShape (Tools.BoxShape ( 550,-150,  0.4, 100,400));
			ground.massInv = 0; ground.inertiaInv = 0;
			space.AddBody (ground);
			
			Test3 ();
			
			var t       = new Timer ();
			t.Interval  = 1000/30;
			t.Tick      += delegate
			{
				int ticks = 10;
				for (int i=0; i<ticks; i++)
				{
					if (selected != null)
					{
						var b = selected.body;
						b.vel = 0.9 * b.vel + 2 * (mouse - (b.pos + offset.Rotate (b.dir)));
					}
					space.RunPhysics (0.05/ticks);
				}
				
				foreach (Ray r in rays) r.Reset ();
				foreach (Shape s in space.Query (new AABB {l=0,t=0,r=1200,b=1000}))
					foreach (Ray r in rays) s.IntersectRay (r);
				
				Invalidate ();
			};
			t.Start ();
		}
		
		protected override void OnMouseMove (MouseEventArgs e) { mouse = new Vec2 (e.X,e.Y); }
		protected override void OnMouseUp   (MouseEventArgs e) { selected = null; }
		
		protected override void OnMouseDown (MouseEventArgs e)
		{
			var area = new AABB (); area.SetExtents (mouse, new Vec2 (1,1));
			selected = space.Query (area).FirstOrDefault (s => s.ContainsPoint (mouse));
			if (selected != null)
			{
				var b = selected.body; 
				offset = (mouse - b.pos).Rotate (new Vec2 (b.dir.x,-b.dir.y));
			}
		}
		
		protected override void OnPaint(PaintEventArgs e)
		{
			e.Graphics.Clear (Color.White);
			e.Graphics.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.HighQuality;
			Tools.DrawWorld (space, e.Graphics);
			Tools.DrawRays  (rays,  e.Graphics);
		}
		
		public void Test1 ()
		{
			for (int i=0; i<15; i++)
			{
				var b1 = Tools.BoxBody    (getrand (600), getrand (400), getrand (6.28), 80,60);
				var b2 = Tools.CircleBody (getrand (600), getrand (400), 40);
				b1.gravity = gravity;
				b2.gravity = gravity;
				space.AddBody (b1);
				space.AddBody (b2);
			}
		}
		
		public void Test2 ()
		{
			for (int i=0; i<200; i++)
			{
				var n = rand.Next (3) + 3;
				var verts = new Vec2 [n]; double r = 20;
				for (int j=0; j<n; j++) verts [j] = r * Vec2.Polar ((double)(n-j)/n * 2*Math.PI);
				
				var p = new Polygon (verts); p.CalcMass (1);
				var b = new Body { gravity = gravity, pos = new Vec2 (getrand (600), getrand (400)) };
				b.AddShape (p); b.CalcProperties ();

				space.AddBody (b);
			}
		}
		
		public void Test3 ()
		{
			for (int r=0; r<30;        r++)
			for (int c=0; c<6+(1+r)%2; c++)
			{
				var b = Tools.BoxBody (130 + 60*((double)(r%2)/2 + c), 445 - r * 10, 0, 60,10);
				b.gravity = gravity; space.AddBody (b);
			}
			
			var b2 = Tools.BoxBody (310,-400, 0, 100, 800);
			b2.gravity = gravity; space.AddBody (b2);
		}
		
		// this test demonstrates an inherent bad behavior with long rectangles - maybe I can fix this
		public void Test4 ()
		{
			for (int r=0; r<20; r++)
			{
				var b = Tools.BoxBody (300, 443 - r * 15, 0, 400,15);
				b.gravity = gravity; b.inertiaInv *= 0.5;
				space.AddBody (b);
			}
		}
		
		public static Random rand = new Random ();
		public static double getrand (double mod) { return rand.NextDouble ()*mod; }
	}
}
