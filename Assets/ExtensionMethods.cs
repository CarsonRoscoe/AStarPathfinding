using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace PathFinding {
  public static class ExtensionMethods {
    //Asquare+Bsquare=Csquare extention for quick use
    public static float Pythag(this Vector2 myPoint, Vector2 otherPoint) {
      return Mathf.Sqrt( Mathf.Pow( otherPoint.x - myPoint.x, 2 ) + Mathf.Pow( otherPoint.y - myPoint.y, 2 ) );
    }
  }
}
