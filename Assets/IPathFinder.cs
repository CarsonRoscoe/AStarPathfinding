using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using PathFinding;
using UnityEngine;

namespace PathFinder {
	/*
		The base class for the AI. This was originally written because I plan on further building this up,
		so I can reuse it for other pathfinding types and general future use, hence why it was abstracted out.
	 */
  public abstract class BasePathFinder {
			/*
			The function called when a path is requested to be found.

				T World: A object that represents the 2D world of a type that extends Node.
				Vector2 start: A 2D vector for the start position of the path
				Vector2 end: The goal to find
		 */
    public abstract List<T> FindPath<T>( T[,] World, Vector2 start, Vector2 end ) where T : Node;

		/*
			The surrounding points in the 2D world
		 */
    public static List<Vector2> GetSurroundingPoints( Vector2 point ) {
	    return new List<Vector2>() {
	        new Vector2( point.x - 1, point.y ),
	        new Vector2( point.x + 1, point.y ),
	        new Vector2( point.x, point.y + 1 ),
	        new Vector2( point.x, point.y - 1 ),
	        new Vector2( point.x - 1, point.y + 1 ),
	        new Vector2( point.x + 1, point.y - 1 ),
	        new Vector2( point.x + 1, point.y + 1 ),
	        new Vector2( point.x - 1, point.y - 1 )
	    };
    }
  }
}
