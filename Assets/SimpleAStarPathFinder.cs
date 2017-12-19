using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using PathFinding;
using PathFinding.Example;
using UnityEngine;

namespace PathFinder {

  //A* Pathfinding class
  public class SimpleAStarPathFinder : BasePathFinder {
    //The world as given to the pathfinder when a path is requested
    Node[,] m_world;
    //The world as seen by the pathfinder when A* is being executed
    AStartNode[,] m_aStartWorld;
    //Width of the world
    int m_width;
    //Height of the world
    int m_height;
    //The goal to find
    Vector2 m_endPoint;
    //The last best F found to date
    float m_lastBestF;

    /*
			The function called when a path is requested to be found.

			T World: A object that represents the 2D world of a type that extends Node.
			Vector2 start: A 2D vector for the start position of the path
			Vector2 end: The goal to find
		 */
    public override List<T> FindPath<T>( T[,] World, Vector2 start, Vector2 end ) {
      m_world = World;
      m_width = World.GetLength(0);
      m_height = World.GetLength(1);
      m_aStartWorld = new AStartNode[m_width, m_height];
      m_endPoint = end;
      m_lastBestF = float.MaxValue;

      PopulateAStarNodes();

      var path = new List<T>();
      var success = Search( m_aStartWorld[(int)start.x, (int)start.y] );
      if (success) {
        var node = m_aStartWorld[(int)m_endPoint.x, (int)m_endPoint.y];
        while(node.Parent != null) {
          path.Add( World[(int)node.Point.x, (int)node.Point.y] );
          node = node.Parent;
        }
        path.Reverse();
      }
      
      m_world = null;
      return path;
    }

    //Setting the starting scenario for the AStarNode representation of the world, which is executed
    //during A*'s execution
    void PopulateAStarNodes() {
      for ( int x = 0; x < m_width; x++ ) {
        for ( int y = 0; y < m_height; y++ ) {
          m_aStartWorld[x, y] = new AStartNode( new Vector2( x, y ), m_world[x, y].TileType != TileType.Wall, (new Vector2(x, y)).Pythag( m_endPoint ) );
        }
      }
    }

    //The method that encases all the logic of A*'s actual path finding.
    //Returns true if a path can be found, false it not.
    //The root parameter is the starting point of the search
		bool Search(AStartNode root) {
      //All the branches hit of possible paths as the best path is found. 
      //The branches are organized by combined F values
			var nodes = new SortedDictionary<float, Queue<AStartNode>>();

      //A action to add a node to the relevant path
			Action<AStartNode> AddNodeToNodes = (node) => {
				if (!nodes.ContainsKey(node.F)) {
					nodes.Add(node.F, new Queue<AStartNode>());
				}
				nodes[node.F].Enqueue(node);
			};

      //A action that finds the node with the lowset F and dequeues it from the list of nodes to try
			Func<AStartNode> GetLowestFs = () => {
				foreach (var key in nodes.Keys.ToList()) {
					if (nodes[key].Count > 0) {
						return nodes[key].Dequeue();
					} else {
						nodes.Remove(key);
					}
				}
				return null;
			};

			AddNodeToNodes(root);

			AStartNode currentNode = GetLowestFs();

      //The 'recusive' loop, written as a do:while
			do{
        //Mark the current node to check as closed
				currentNode.State = AStarState.Closed;

				var surroundingEmptyNodes = GetSurroundingEmptyNodes(currentNode).OrderBy(x => x.F).ToList();
				bool finish = false;

        //For every surrounding empty node ordered by F values
				foreach (var surroundingNode in surroundingEmptyNodes) {
          //If we found the end, mark it as finished and leave the loop
					if (surroundingNode.Point == m_endPoint) {
						finish = true;
						surroundingNode.Load(currentNode);
						break;
          //Otherwise, add the current node to the Nodes dictionary
					} else {
						surroundingNode.Load(currentNode);
						AddNodeToNodes(surroundingNode);
					}
				}

        //If we are finished, break out of the loop as we found the best path
				if (finish) {
					break;
				}

        //Set the current node to check to be that with the lowest F value in the nodes we can check next
				currentNode = GetLowestFs();

			} while (currentNode != null);

      //Return success or fail
			return currentNode != null;
		}

    //Find all nodes that surround a particiular node which are empty
    //it also takes into account you can only consider diagnol nodes empty if there is
    //no walls on either relevant side
		List<AStartNode> GetSurroundingEmptyNodes(AStartNode fromNode) {
      var emptyNodes = new List<AStartNode>();
      var surroundingPoints = GetSurroundingPoints( new Vector2( fromNode.Point.x, fromNode.Point.y ) );

      foreach ( var surroundingPoint in surroundingPoints ) {
        int x = (int)surroundingPoint.x;
        int y = (int)surroundingPoint.y;

        if (x < 0 || x >= m_width || y < 0 || y>= m_height) {
          continue;
        }

        var node = m_aStartWorld[x,y];

        if (!node.IsEmpty) {
          continue;
        }

        if (node.State == AStarState.Closed) {
          continue;
        }

        //Only to be added if their G-value is lower. This means that this route is lower for this node than the previously explored way that considered it
        if (node.State == AStarState.Open) {
                    var preG = fromNode.G + node.Cost;
                    if (preG < node.G) {
                      node.Load(fromNode);
                      emptyNodes.Add( node );
                    }
                    continue;
        } else {
          //If untested, set the parent and flat it as Open for consideration
          node.Load(fromNode);
          node.State = AStarState.Open;
          emptyNodes.Add( node );
        }
      }

      return emptyNodes;
    }
    
    //Find all x/y points that surround a particiular node.
    //it also takes into account you can only consider diagnol nodes empty if there is
    //no walls on either relevant side
    public new List<Vector2> GetSurroundingPoints( Vector2 point ) {
			var result = new List<Vector2> {
				new Vector2( point.x - 1, point.y ),
				new Vector2( point.x + 1, point.y ),
				new Vector2( point.x, point.y + 1 ),
				new Vector2( point.x, point.y - 1 ),
            };

			var emptyLeft =  (point.x - 1 >= 0 && point.x - 1 < m_world.GetLength(0)) ? m_world[(int)point.x - 1, (int)point.y].TileType != TileType.Wall : false;
			var emptyRight = (point.x + 1 >= 0 && point.x + 1 < m_world.GetLength(0)) ? m_world[(int)point.x + 1, (int)point.y].TileType != TileType.Wall : false;
			var emptyAbove = (point.y + 1 >= 0 && point.y + 1 < m_world.GetLength(1)) ? m_world[(int)point.x, (int)point.y + 1].TileType != TileType.Wall : false;
			var emptyBelow = (point.y - 1 >= 0 && point.y - 1 < m_world.GetLength(1)) ? m_world[(int)point.x, (int)point.y - 1].TileType != TileType.Wall : false;

      if (emptyLeft && emptyAbove) {
          result.Add(new Vector2(point.x - 1, point.y + 1));
      }
      if (emptyLeft && emptyBelow) {
          result.Add(new Vector2(point.x - 1, point.y - 1));
      }
      if (emptyRight && emptyAbove) {
          result.Add(new Vector2(point.x + 1, point.y + 1));
      }
      if (emptyRight && emptyBelow) {
          result.Add(new Vector2(point.x + 1, point.y - 1));
      }
      return result;
  }


    /*
      A type of Node that is used in A*'s execution.
     */
    public class AStartNode {
      //The node that leads to this one in the current parth. Null until claimed.
      public AStartNode Parent { get; private set; }
      //The nodes x/y position in the world
      public Vector2 Point { get; private set; }
      //If the node is empty
      public bool IsEmpty { get; private set; }
      //The G value in A*, which is effectively "how many turns until this node"
      public float G { get; set; }
      //The H value, which represents the "total distance" from the goal. Pythagoream theory is used here
      public float H { get; private set; }
      //Whether the node is Unchecked, Open or Closed (it's state in the execution)
      public AStarState State { get; set; }
      //The F value, which is the combination of G and H
      public float F { get { return G + H; } }
      //The cost, which is the pythag of it to it's parent
      public float Cost { get { return Point.Pythag( Parent.Point ); } }

      public AStartNode(Vector2 point, bool isEmpty, float h) {
        Point = point;
        State = AStarState.Untested;
        IsEmpty = isEmpty;
        G = 0.0f;
        H = h;
      }
      bool firstTimeLoading = true;

      //Method called that sets the parent of this node and creates it's visual representation in the world
      public void Load( AStartNode newParent ) {
        Parent = newParent;
        G = Parent.G + Cost;
        if (firstTimeLoading) {
            firstTimeLoading = false;
            UnityEngine.Object.Instantiate(WorldLoader.instance.TileTypes.First(x => x.TileType == TileType.Visited).ToCreate, new Vector3(Point.x, 0.0f, Point.y), Quaternion.identity);
        }
        
      }
    }

    public enum AStarState { Open, Closed, Untested }
  }
}
