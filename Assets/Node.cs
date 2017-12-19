using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PathFinding {
  //The Node type each x/y coordinate on the map is represented by
  public class Node {
    //The represented x position
    public int x;
    //The represented y position
    public int y;
    //The type (e.g. Wall, Start, Goal, etc.) of Tile that can be represented by the node
    public TileType TileType;
  }
}
