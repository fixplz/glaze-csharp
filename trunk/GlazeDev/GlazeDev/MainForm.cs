
using System;
using System.Drawing;
using System.Windows.Forms;

using Glaze;

namespace GlazeDev
{
	public partial class MainForm : Form
	{
		public Space space;
		
		public MainForm()
		{
			InitializeComponent();
			
			DoubleBuffered = true;
			
			space = new SortedSpace ();
			
			
			var ground = G.BoxBody (300,500, 0, 1000,100);
			ground.massInv = 0; ground.inertiaInv = 0;
			space.AddBody (ground);
			
			for (int i=0; i<40; i++)
			{
				/*var b = G.BoxBody (rand.NextDouble ()*600, rand.NextDouble ()*400, rand.NextDouble ()*6.28, 40,30);
				b.gravity = new Vec2 (0,15);*/
				
				var b1 = G.BoxBody    (rand.NextDouble ()*600, rand.NextDouble ()*400, rand.NextDouble ()*6.28, 40,30);
				var b2 = G.CircleBody (rand.NextDouble ()*600, rand.NextDouble ()*400, 20);
				b1.gravity = new Vec2 (0,10);
				b2.gravity = new Vec2 (0,10);
				space.AddBody (b1);
				space.AddBody (b2);
			}
			
			
			var t       = new Timer ();
			t.Interval  = 1000/40;
			t.Tick      += delegate { space.RunPhysics (0.1); Invalidate (); };
			t.Start ();
		}
		
		public static Random rand = new Random ();
		
		protected override void OnPaint(PaintEventArgs e)
		{
			e.Graphics.Clear (Color.White);
			e.Graphics.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.HighQuality;
			G.DrawWorld (space, e.Graphics);
		}
	}
}
