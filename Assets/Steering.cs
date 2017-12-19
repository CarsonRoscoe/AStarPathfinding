using System;
using System.Collections.Generic;
using PathFinding;
using PathFinding.Example;
using UnityEngine;

namespace Steering {
    /*
        Steering AI that follows the path
     */
    public class Steering : MonoBehaviour {
        //Racceleration rate
        public float AccelerationPerFrame;
        //Max speed the AI can travel
        public float SpeedLimit;
        //The force factor nearby walls invoke on the AI when the AI is too close
        public float MoveFromWallsForceFactor;
        //The current velocity
        Vector3 m_velocity;
        //The closest node we are steering towards
        Node m_currentNode;
        //The node after the current node
        Node m_secondNode;
        //The node after the second node
        Node m_thirdNode;
        //The path to steer towards
        Queue<Node> m_path;
        //Whether we have reached the destination or not
        bool m_landed;

        //Called when movement first begins
        public void BeginMovement(Node[] path) {
            m_landed = false;
            m_velocity = Vector3.zero;
            m_path = new Queue<Node>(path);
            if (m_path.Count > 0) {
                m_currentNode = m_path.Dequeue();
            }
            if (m_path.Count > 0) {
                m_secondNode = m_path.Dequeue();
            }
            if (m_path.Count > 0) {
                m_thirdNode = m_path.Dequeue();
            }

            Debug.Log(m_path.Count);
        }

        //Invoked 60 frames a second, this is the "brain" of the AI
        void Update() {
            //If we have a node to move towards
            if (m_currentNode != null) {
                //and we have not yet reached the destination
                if (!m_landed) {
                    //Assign the direction we're moving towards to be 66% of the currentNode to the AI
                    var directionToMove = (new Vector3(m_currentNode.y, 0.0f, m_currentNode.x) - transform.position).normalized * 0.66f;
                    //Assign 22% of the direction we're moving towards to be towards the secondNode
                    if (m_secondNode != null) {
                        directionToMove += (new Vector3(m_secondNode.y, 0.0f, m_secondNode.x) - transform.position).normalized * 0.22f;
                    }
                    //Assign the last 12% of the direction we're moving towards to be the thirdNode
                    if (m_thirdNode != null) {
                        directionToMove += (new Vector3(m_thirdNode.y, 0.0f, m_thirdNode.x) - transform.position).normalized * 0.12f;
                    }
                    //Normalize our direction and update our velocity to be the end speed after taking all forces into account
                    directionToMove.Normalize();
                    m_velocity = Move(m_velocity, directionToMove);

                    //Should we start landing
                    if (!TryUpdateNode()) {
                        m_landed = true;
                    }
                } else {
                    //Landing logic to slowly reach the end goal once we are close enough
                    var vel = m_velocity.magnitude;
                    var currentNodePos = new Vector3(m_currentNode.y, 0.0f, m_currentNode.x);
                    var dir = currentNodePos - transform.position;
                    m_velocity = dir.normalized * vel;
                    m_velocity *= 0.95f;
			        transform.position += m_velocity * Time.deltaTime;
                    if (dir.magnitude <= 0.01f) {
                        m_velocity = Vector3.zero;
                        m_currentNode = null;
                        HUDManager.instance.Success();
                    }
                }
            }
        }

        //When we move each frame, this takes into account moving in that direction, with that velocity,
        //how much we accelerate and how we react to nearby walls
        Vector3 Move(Vector3 currentVelocity, Vector3 directionToMove) {
            var oldVelocity = currentVelocity;

            //Apply acceleration and desired movement to our end velocity
            var velocityOffset = directionToMove * AccelerationPerFrame;
            currentVelocity += velocityOffset;
            //var avoidanceVelocity = Vector3.zero;

            //Apply Wall Avoidance, moving away from nearby walls if we're too close
            var pointHeadingTowards = transform.position + currentVelocity * Time.deltaTime * 3.0f;
            foreach (var nearbyWall in GetAllNearbyWalls(pointHeadingTowards)) {
                var distance = new Vector3(pointHeadingTowards.x - nearbyWall.x, 0.0f, pointHeadingTowards.z - nearbyWall.y);
                //If we're too close, we move away from this wall
                if (distance.magnitude <= 0.85f) {
                    // Force is inverse distance squared, so the closer we are, the more we move away
                    var power = (1.0f / distance.magnitude) * (1.0f / distance.magnitude) * MoveFromWallsForceFactor;
                    currentVelocity += distance * power;
                    Debug.Log("Distance: " + distance.magnitude + " : Force:" + distance * power);
                }
            }

            //Update our velocity to take into account the result from the walls
            currentVelocity = currentVelocity.normalized * (oldVelocity.magnitude + AccelerationPerFrame * Time.deltaTime);

            //Cap movement 
            if (currentVelocity.magnitude > SpeedLimit) {
                currentVelocity = currentVelocity.normalized * SpeedLimit;
			}

            //Apply movement
			transform.position += currentVelocity * Time.deltaTime;
			return currentVelocity;
        }

        //On return true, update success. On return false, failed, meaning no node to update to.
        //So success on landing, or failure if it can't even start
        bool TryUpdateNode() {
            var distToCollide = 0.75f;

            var node3Check = CheckIfNodeHitAndUpdateAccordingly(m_thirdNode, distToCollide, 3);
            var node2Check = CheckIfNodeHitAndUpdateAccordingly(m_secondNode, distToCollide, 2);
            var node1Check = CheckIfNodeHitAndUpdateAccordingly(m_currentNode, distToCollide, 1);
            return node3Check && node2Check && node1Check;
        }

        bool CheckIfNodeHitAndUpdateAccordingly(Node node, float distanceToCollide, int nodeNumber) {
            //Current node check
            if (node != null && Vector3.Distance(transform.position, new Vector3(node.y, 0.0f, node.x)) < distanceToCollide) {
                if (!TryAddNextNodes(nodeNumber)) {
                    return false;
                }
            }
            return true;
        }

        bool TryAddNextNodes(int many) {
            var left = many;
            while (left > 0) {
                if (m_path.Count > 2) {
                    m_currentNode = m_secondNode;
                    m_secondNode = m_thirdNode;
                    m_thirdNode = m_path.Dequeue();
                } else if (m_path.Count > 1) {
                    m_currentNode = m_secondNode;
                    m_secondNode = m_path.Dequeue();
                    m_thirdNode = null;
                } else if (m_path.Count == 1) {
                    m_currentNode = m_path.Dequeue();
                    m_secondNode = null;
                } else {
                    return false;
                }
                left--;
            }
            return true;
        }

        //Returns all walls around a point
        List<Vector2> GetAllNearbyWalls(Vector3 fromPoint) {
            var result = new List<Vector2>();
            var roughX = (int)Mathf.Round(fromPoint.x);
            var roughY = (int)Mathf.Round(fromPoint.z);
            var surroundingPoints = PathFinder.BasePathFinder.GetSurroundingPoints(new Vector2(roughX, roughY));
            foreach (var point2D in surroundingPoints) {
                if (point2D.x >= 0 && point2D.x < WorldLoader.instance.World.GetLength(0) && point2D.y >= 0 && point2D.y < WorldLoader.instance.World.GetLength(1)) {
                    var node = WorldLoader.instance.World[(int)point2D.x, (int)point2D.y];
                    if (node.TileType == TileType.Wall ) {
                        //result.Add(node);
                        result.Add(new Vector2(point2D.x - 0.5f, point2D.y - 0.5f));
                        result.Add(new Vector2(point2D.x + 0.5f, point2D.y + 0.5f));
                        result.Add(new Vector2(point2D.x - 0.5f, point2D.y + 0.5f));
                        result.Add(new Vector2(point2D.x + 0.5f, point2D.y - 0.5f));
                    }
                }
            }
            return result;
        }
    }
}
