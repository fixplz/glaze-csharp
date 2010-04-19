
using System;
using System.Collections.Generic;

namespace Glaze
{
	public class SortedSpace : Space
	{
		public SortedSpace () : base () {}
		
		protected override void BroadPhase()
		{
			LinkedListNode<Shape> a,b,n;
			
			for (a = shapes.First.Next; a != null; a = n)
			{
				n = a.Next;
				b = a.Previous; if (b.Value.aabb.t < a.Value.aabb.t) continue;
				while (b.Previous != null && b.Previous.Value.aabb.t > a.Value.aabb.t) b = b.Previous;
				shapes.Remove (a); shapes.AddBefore (b, a);
			}
			
			for (a = shapes.First; a != null; a = a.Next)
				for (b = a.Next; b != null && b.Value.aabb.t < a.Value.aabb.b; b = b.Next)
					if (a.Value.aabb.IntersectH (b.Value.aabb))
						NarrowPhase (a.Value, b.Value);
		}
		
		public override IEnumerable<Shape> Query(AABB area)
		{
			foreach (Shape s in shapes)
				if (area.b < s.aabb.t) yield break;
				else if (s.aabb.Intersect (area)) yield return s;
		}
	}
}
