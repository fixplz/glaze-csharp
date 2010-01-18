
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
		public Vec2  mouse, gravity;
		
		public MainForm()
		{
			InitializeComponent();
			
			DoubleBuffered = true;
			
			space = new SortedSpace ();
			gravity = new Vec2 (0,100);
			
			var ground = Tools.BoxBody (300,500, 0, 1000,100);
			ground.massInv = 0; ground.inertiaInv = 0;
			space.AddBody (ground);
			
			Test2 ();
			
			var t       = new Timer ();
			t.Interval  = 1000/40;
			t.Tick      += delegate
			{
				int ticks = 3;
				for (int i=0; i<ticks; i++)
				{
					if (selected != null) selected.body.vel += mouse - selected.body.pos;
					space.RunPhysics (1.0/(ticks*20));
				}
				Invalidate ();
			};
			t.Start ();
		}
		
		protected override void OnMouseMove(MouseEventArgs e) { mouse = new Vec2 (e.X,e.Y); }
		protected override void OnMouseUp(MouseEventArgs e) { selected = null; }
		protected override void OnMouseDown(MouseEventArgs e)
			{ selected = space.Query (new AABB {l=e.X, t=e.Y, r=e.X+1, b=e.Y+1}).FirstOrDefault (); }
		
		
		protected override void OnPaint(PaintEventArgs e)
		{
			e.Graphics.Clear (Color.White);
			e.Graphics.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.HighQuality;
			Tools.DrawWorld (space, e.Graphics);
		}
		
		public void Test1 ()
		{
			for (int i=0; i<60; i++)
			{
				var b1 = Tools.BoxBody    (rand.NextDouble ()*600, rand.NextDouble ()*400, rand.NextDouble ()*6.28, 40,30);
				var b2 = Tools.CircleBody (rand.NextDouble ()*600, rand.NextDouble ()*400, 20);
				b1.gravity = gravity;
				b2.gravity = gravity;
				space.AddBody (b1);
				space.AddBody (b2);
			}
		}
		
		public void Test2 ()
		{
			for (int i=0; i<80; i++)
			{
				var n = rand.Next (5) + 3;
				var verts = new Vec2 [n]; double r = 20;
				for (int j=n-1; j>=0; j--) verts [j] = r * new Vec2
					(-Math.Cos ((double)j/n * 2*Math.PI), Math.Sin ((double)j/n * 2*Math.PI));
				
				var p = new Polygon (verts); p.CalcMass (1);
				var b = new Body
				{
					gravity = gravity,
					pos = new Vec2 (rand.NextDouble ()*600, rand.NextDouble ()*400)
				};
				b.AddShape (p); b.CalcProperties ();
				
				space.AddBody (b);
			}
		}
		
		public void Test3 ()
		{
			for (int r=0; r<30; r++)
			{
				for (int c=0; c<5; c++)
				{
					var b = Tools.BoxBody (200 + ((double)(r%2)/2 + c) * 60, 445 - r * 9.7, 0, 60,10);
					b.gravity = gravity;
					space.AddBody (b);
				}
			}
		}
		
		public static Random rand = new Random ();
	}
}
