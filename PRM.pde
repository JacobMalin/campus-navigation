//You will only be turning in this file
//Your solution will be graded based on it's runtime (smaller is better), 
//the optimality of the path you return (shorter is better), and the
//number of collisions along the path (it should be 0 in all cases).

//You must provide a function with the following prototype:
// ArrayList<Integer> planPath(Vec2 startPos, Vec2 goalPos, Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes);
// Where: 
//    -startPos and goalPos are 2D start and goal positions
//    -centers and radii are arrays specifying the center and radius of obstacles
//    -numObstacles specifies the number of obstacles
//    -nodePos is an array specifying the 2D position of roadmap nodes
//    -numNodes specifies the number of nodes in the PRM
// The function should return an ArrayList of node IDs (indexes into the nodePos array).
// This should provide a collision-free chain of direct paths from the start position
// to the position of each node, and finally to the goal position.
// If there is no collision-free path between the start and goal, return an ArrayList with
// the 0'th element of "-1".

//Your code can safely make the following assumptions:
//  - The function connectNeighbors() will always be called before planPath()
//  - The variable maxNumNodes has been defined as a large static int, and it will
//    always be bigger than the numNodes variable passed into planPath()
//  - None of the positions in the nodePos array will ever be inside an obstacle
//  - The start and the goal position will never be inside an obstacle

// There are many useful functions in CollisionLibrary.pde and Vec2.pde
// which you can draw on in your implementation. Please add any additional 
// functionality you need to this file (PRM.pde) for compatabilty reasons.

// Here we provide a simple PRM implementation to get you started.
// Be warned, this version has several important limitations.
// For example, it uses BFS which will not provide the shortest path.
// Also, it (wrongly) assumes the nodes closest to the start and goal
// are the best nodes to start/end on your path on. Be sure to fix 
// these and other issues as you work on this assignment. This file is
// intended to illustrate the basic set-up for the assignmtent, don't assume 
// this example funcationality is correct and end up copying it's mistakes!).



import java.util.PriorityQueue;



//Here, we represent our graph structure as a neighbor list
//You can use any graph representation you like
ArrayList<Integer>[] neighbors = new ArrayList[maxNumNodes];  //A list of neighbors can can be reached from a given node

//Set which nodes are connected to which neighbors (graph edges) based on PRM rules
void connectNeighbors(Vec2[][] shapes, float eps, Vec2[] nodePos, int numNodes){
  for (int i = 0; i < numNodes; i++){
    neighbors[i] = new ArrayList<Integer>();  //Clear neighbors list
    for (int j = 0; j < numNodes; j++){
      if (i == j) continue; //don't connect to myself
      Vec2 dir = nodePos[j].minus(nodePos[i]).normalized();
      float distBetween = nodePos[i].distanceTo(nodePos[j]);
      boolean circleListCheck = rayShapeListIntersect(shapes, nodePos[i], dir, distBetween, eps);
      if (!circleListCheck){
        neighbors[i].add(j);
      }
    }
  }
}

ArrayList<Integer> planPath(Vec2 startPos, Vec2 goalPos, Vec2[][] shapes, float eps, Vec2[] nodePos, int numNodes){
  ArrayList<Integer> path = new ArrayList();
  
  // If there is line of sight
  if (!rayShapeListIntersect(shapes, startPos, goalPos.minus(startPos).normalized(), goalPos.distanceTo(startPos), eps)) return path;
  
  boolean startInObstacle = pointInShapeList(shapes, startPos, eps);
  boolean goalInObstacle = pointInShapeList(shapes, goalPos, eps);
  
  if (startInObstacle || goalInObstacle) {
    path.add(0, -1);
    return path;
  }
  
  path = runAStar(nodePos, numNodes, startPos, goalPos, shapes, eps);
  
  return path;
}

// Straight line heuristic
float heuristic(int id, Vec2 goalPos) {
  return nodePos[id].distanceTo(goalPos);
}

// Creates the path by following previous backwards
ArrayList<Integer> reconstruct_path(int currentID, ArrayList<Integer> previous) {
  ArrayList<Integer> path = new ArrayList();
  while (currentID != -1) {
    //println("path:", currentID);
    path.add(0, currentID);
    currentID = previous.get(currentID);
  }
  
  return path;
}

// A* - referenced from wikipedia's pseudocode
ArrayList<Integer> runAStar(Vec2[] nodePos, int numNodes, Vec2 startPos, Vec2 goalPos, Vec2[][] shapes, float eps) {
  //println("\n\n\n\ns:", startID, "g:", goalID);
  PriorityQueue<node> fringe = new PriorityQueue(numNodes);
  ArrayList<Float> cheapestPath = new ArrayList(numNodes);
  ArrayList<Float> heuristic = new ArrayList(numNodes);
  ArrayList<Integer> previous = new ArrayList(numNodes);
  
  // Goal Node
  int goalID = -1;
  float minGoalPath = MAX_FLOAT;
  
  // Set cheapestPath and heuristic to inf and previous to -1
  for (int i = 0; i < numNodes; i++) {
    cheapestPath.add(MAX_FLOAT);
    heuristic.add(MAX_FLOAT);
    previous.add(-1);
  }
  
  // Add all nodes visible to the start node to the fringe
  for (int i = 0; i < numNodes; i++) {
    if (!rayShapeListIntersect(shapes, startPos, nodePos[i].minus(startPos).normalized(), startPos.distanceTo(nodePos[i]), eps)) {
      cheapestPath.add(i, 0.0);
      heuristic.set(i, heuristic(i, goalPos));
      fringe.add(new node(i, 0.0 + heuristic.get(i)));
    }
  }
  
  // While the optimal goal path has not been popped
  while (!fringe.isEmpty()) {
    node current = fringe.poll();
    //println("current", current.id, cheapestPath.get(current.id));
    
    // If it takes longer to get to the current node than the minGoalPos
    if (current.priority > minGoalPath) {
      return reconstruct_path(goalID, previous);
    }
    
    // If the current node has line of sight to the goal
    if (!rayShapeListIntersect(shapes, nodePos[current.id], goalPos.minus(nodePos[current.id]).normalized(), goalPos.distanceTo(nodePos[current.id]), eps)) {
      float tentativeCheapestGoalPath = cheapestPath.get(current.id) + nodePos[current.id].distanceTo(goalPos);
      if (tentativeCheapestGoalPath < minGoalPath) {
        minGoalPath = tentativeCheapestGoalPath;
        goalID = current.id;
      }
    }
    
    // Add each neighbor of the current node
    //println("n size:", neighbors[current.id].size());
    for (int i = 0; i < neighbors[current.id].size(); i++){
      int neighborID = neighbors[current.id].get(i);
      float tentativeCheapestPath = cheapestPath.get(current.id) + nodePos[current.id].distanceTo(nodePos[neighborID]);
      
      //println("n:", neighborID, tentativeCheapestPath);
      // If the path from current to the neighbor is the shortest path yet
      if (tentativeCheapestPath < cheapestPath.get(neighborID)) {
        cheapestPath.set(neighborID, tentativeCheapestPath);
        heuristic.set(neighborID, heuristic(neighborID, goalPos));
        previous.set(neighborID, current.id);
        
        // Update fringe
        node n = new node(neighborID, cheapestPath.get(neighborID) + heuristic.get(neighborID));
        if (!fringe.contains(n)) fringe.add(n);
      }
    }
  }
  
  //println("goal:", goalID, previous);
  if (goalID != -1) {
    return reconstruct_path(goalID, previous);
  }
    
  ArrayList<Integer> path = new ArrayList();
  path.add(0,-1);
  return path; 
}




// Node class
class node implements Comparable<node> {
  public int id;
  public float priority;
  
  public node(int id, float priority) {
    this.id = id;
    this.priority = priority;
  }
  
  @Override
  public int compareTo(node o) {
      return o.priority < this.priority ? 1 : -1;
  }
  
  public int equals(node o) {
      return o.id == this.id ? 1 : -1;
  }
}
