using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace PathFinding {
  //What types of tiles a world can be populated by.
  public enum TileType { Floor, Wall, Start, Goal, Path, Visited }

  //A class for use inside Unity's editor when setting up the seen that helps describe how to read the file
  [Serializable]
  public class TileTypeDataClass {
    public char KeyInputValue;
    public TileType TileType;
    public GameObject ToCreate;
  }
}
