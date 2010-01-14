
using System;
using System.Collections.Generic;

namespace Glaze
{
	public class SortedSpace : Space
	{
		public SortedSpace () : base ()
		{
			
		}
		
		protected override void BroadPhase()
		{
			LinkedListNode<Shape> a,b,n;
			
			for (a = shapes.First.Next; a != null; a = n)
			{
				n = a.Next;
				for (b = a.Previous;
				     b.Previous != null && b.Previous.Value.aabb.t > a.Value.aabb.t;
				     b = b.Previous);
				shapes.Remove (a); shapes.AddBefore (b, a);
			}
			
			for (a = shapes.First; a != null; a = a.Next)
				for (b = a.Next; b != null && b.Value.aabb.t < a.Value.aabb.b; b = b.Next)
					if (a.Value.aabb.IntersectH (b.Value.aabb))
						NarrowPhase (a.Value, b.Value);
		}
		
		public override IEnumerable<Shape> Query(AABB area)
		{
			LinkedListNode<Shape> a = shapes.First;
			for (; a != null && a.Value.aabb.t < area.t; a = a.Next);
			for (; a != null && a.Value.aabb.b < area.b; a = a.Next)
				if (a.Value.aabb.IntersectH (area)) yield return a.Value;
		}
	}
}
