﻿using System.Collections.Generic;
using System.IO;
using System.Linq;
using PathFinder;
using UnityEngine;

namespace PathFinding.Example {
  //The class that utilizes PathFinding to generate the world and creates the AI that
  //will steer along the path
  public class WorldLoader : MonoBehaviour {

    public static WorldLoader instance;

    //List used in the Unity editor for describing how to real the file of the world
    public TileTypeDataClass[] TileTypes;
    //The path generated by A*
    public Node[] Path;
    //The AI to create that will steer along the path
    public GameObject SteeringAI;
 
    //The 2D world the path is created for
    [HideInInspector]
    public Node[,] World;
    
    //The end point/goal of the path finding
    [HideInInspector]
    public Vector2 Goal;

    //The starting point in the path finding
    [HideInInspector]
    public Vector2 StartPoint;



    public void Awake() {
      if ( instance == null ) {
        instance = this;
      }
      else {
        Destroy( this );
      }
    }

    //Loading the map from a file, then drawing the map on the screen
    public void LoadMapFromFile() {
      HUDManager.instance.Reset();
      DeleteAllWithTag("WorldDrawn");
      DeleteAllWithTag("PathDrawn");
      World = null;
      Path = null;
      var path = Application.dataPath + "/../";
      LoadWorld( path + "map.txt" );
      DrawWorld();
    }

    //Clears the world from previous A*'s, performs A* on the world, then draws the world
    //with the generated path
    public void InvokeAStar() {
      HUDManager.instance.Reset();
      DeleteAllWithTag("WorldDrawn");
      DeleteAllWithTag("PathDrawn");
      Path = null;
      if (World != null) {
        PerformAStarOnWorld();
        World[(int)Goal.x, (int)Goal.y].TileType = TileType.Goal;
        if (Path.Length == 0) {
          HUDManager.instance.Invalid();
        }
        Debug.Log( "Path Length: " + Path.Length );
        DrawWorld();
      }
    }

    //Clears any leftover AI, then creates the AI that will steer along the path that has been found
    public void StartPathFinding() {
      HUDManager.instance.Reset();
      var car = GameObject.Find("Car");
      if (car != null) {
        Destroy(car);
      }
      if (Path != null) {
        World[(int)Goal.x, (int)Goal.y].TileType = TileType.Goal;
        if (Path.Length == 0) {
          HUDManager.instance.Invalid();
        } else {
          var steeringAI = Instantiate(SteeringAI, new Vector3(StartPoint.x, 0.0f, StartPoint.y), Quaternion.identity);
          steeringAI.GetComponent<Steering.Steering>().BeginMovement(Path);
        }
      }
    }

    void DeleteAllWithTag(string tag) {
      foreach(var toDelete in GameObject.FindGameObjectsWithTag(tag)) {
        Destroy(toDelete);
      }
    }

    //Reads the map from a file and  prepares the World 2D array pathfinding will view the world from
    public void LoadWorld(string mapFile) {
      var streamReader = new StreamReader( mapFile );

      List<List<Node>> initialNodes = new List<List<Node>>();
      int z = 0;

      while ( !streamReader.EndOfStream ) {
        var row = new List<Node>();
        var line = streamReader.ReadLine();

        for ( int x = 0; x < line.Length; x++ ) {
          var letter = line[x];
          var tileType = TileTypes.First( t => t.KeyInputValue == letter );
          row.Add( new Node { x = x, y = z, TileType = tileType.TileType } );
        }

        z++;
        initialNodes.Add( row );
      }

      World = new Node[initialNodes.Count, initialNodes[0].Count];

      var str = string.Empty;
      for ( int x = 0; x < initialNodes.Count; x++ ) {
        for(int y = 0; y < initialNodes[x].Count; y++ ) {
          World[x, y] = initialNodes[x][y];
          if ( World[x, y] .TileType == TileType.Start) {
            StartPoint = new Vector2( x, y );
          }
          if ( World[x, y].TileType == TileType.Goal ) {
            Goal = new Vector2( x, y );
          }
        }
      }

      streamReader.Close();
    }

    //Creates the path finder, finds the path, then assigns the result
    void PerformAStarOnWorld() {
      var pathFinder = new SimpleAStarPathFinder();
      var path = pathFinder.FindPath( World, StartPoint, Goal );
      foreach ( var node in path ) {
        node.TileType = TileType.Path;
      }
      Path = path.ToArray();
    }

    //Draws the world onto the screen
    void DrawWorld() {
      for ( int x = 0; x < World.GetLength(0); x++ ) {
        for ( int y = 0; y < World.GetLength(1); y++ ) {
          DrawTile( World[x,y].TileType, new Vector3( x, 0, y ) );
        }
      }

      for ( int x = -1; x < World.GetLength(0) + 1; x++ ) {
        DrawTile( TileType.Wall, new Vector3( x, 0, -1 ) );
        DrawTile( TileType.Wall, new Vector3( x, 0, World.GetLength(1) ) );
      }
      for ( int y = -1; y < World.GetLength(1) + 1; y++ ) {
        DrawTile( TileType.Wall, new Vector3( -1, 0, y ) );
        DrawTile( TileType.Wall, new Vector3( World.GetLength(0), 0, y ) );
      }
    }

    //Draws a single tile in the world
    void DrawTile( TileType tileType, Vector3 point ) {
      if ( tileType != TileType.Floor ) {
        var objToCreate = TileTypes.First( x => x.TileType == tileType ).ToCreate;
        Instantiate( objToCreate, point, Quaternion.identity );
      }

      Instantiate( TileTypes.First( x => x.TileType == TileType.Floor ).ToCreate, new Vector3( point.x, point.y - 1.0f, point.z ), Quaternion.identity );
    }
  }
}