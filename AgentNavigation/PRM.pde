//You will only be turning in this file
//Your solution will be graded based on it's runtime (smaller is better), 
//the optimality of the path you return (shorter is better), and the
//number of collisions along the path (it should be 0 in all cases).

//You must provide a function with the following prototype:
// ArrayList<Integer> planPath(Vec2 startPos, Vec2 goalPos, Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes);
// Where: 
//    -startPos and goalPos are 2D start and goal positions
//    -centers and radii are arrays specifying the center and radius
//    -numObstacles specifies the number of obstacles
//    -nodePos is an array specifying the 2D position of roadmap nodes
//    -numNodes specifies the number of nodes in the PRM
// The function should return an ArrayList of node IDs (indexes into the nodePos array).
// This should provide a collision-free chain of direct paths from the start position
// to the position of each node, and finally to the goal position.
// If there is no collision-free path between the start and goal, return an ArrayList with
// the 0'th element of "-1".

// Your code can safely make the following assumptions:
//   - The function connectNeighbors() will always be called before planPath()
//   - The variable maxNumNodes has been defined as a large static int, and it will
//     always be bigger than the numNodes variable passed into planPath()
//   - None of the positions in the nodePos array will ever be inside an obstacle
//   - The start and the goal position will never be inside an obstacle

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



//Here, we represent our graph structure as a neighbor list
//You can use any graph representation you like

int getBest(ArrayList<Node> frontier){
  float min=9999999;
  int minID=0;
  
 for (int i = 0; i < frontier.size(); i++){
  // println(frontier.get(i).cost );
 if (frontier.get(i).cost<min){
  // println( i, frontier.get(i).cost) ;
 min=frontier.get(i).cost;
 minID=i;
 }
 }
 return minID;
}

boolean hasGoal(ArrayList<Node> rs,int goal){
for (int i = 0; i < rs.size(); i++){
  if (rs.get(i).idx==goal) return true;
}
 return false; 
}


ArrayList<Integer>[] neighbors = new ArrayList[maxNumNodes];  //A list of neighbors can can be reached from a given node
//We also want some help arrays to keep track of some information about nodes we've visited
Boolean[] visited = new Boolean[maxNumNodes]; //A list which store if a given node has been visited
int[] parent = new int[maxNumNodes]; //A list which stores the best previous node on the optimal path to reach this node
float[] rotate= new float[maxNumNodes];
//Set which nodes are connected to which neighbors (graph edges) based on PRM rules
void connectNeighbors(Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes){
  for (int i = 0; i < numNodes; i++){
    neighbors[i] = new ArrayList<Integer>();  //Clear neighbors list
    for (int j = 0; j < numNodes; j++){
      if (i == j) continue; //don't connect to myself 
      Vec2 dir = nodePos[j].minus(nodePos[i]).normalized();
      float distBetween = nodePos[i].distanceTo(nodePos[j]);
      hitInfo circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, nodePos[i], dir, distBetween);
      if (!circleListCheck.hit){
        neighbors[i].add(j);
      }
    }
  }
}

//This is probably a bad idea and you shouldn't use it...
int closestNode(Vec2 point, Vec2[] nodePos, int numNodes){
  int closestID = -1;
  float minDist = 999999;
  for (int i = 0; i < numNodes; i++){
    float dist = nodePos[i].distanceTo(point);
    if (dist < minDist){
      closestID = i;
      minDist = dist;
    }
  }
  return closestID;
}

ArrayList<Integer> planPath(Vec2 startPos, Vec2 goalPos, Vec2[] centers, float[] radii, int numObstacles, Vec2[] nodePos, int numNodes){
  ArrayList<Integer> path = new ArrayList();
  
  int startID = closestNode(startPos, nodePos, numNodes);
  int goalID = closestNode(goalPos, nodePos, numNodes);
  
  path = runAstar(nodePos, numNodes, startID, goalID);
  //path = runBFS(nodePos, numNodes, startID, goalID);
  return path;
}

ArrayList<Integer> runAstar(Vec2[] nodePos, int numNodes, int startID, int goalID){
ArrayList<Integer> fringe = new ArrayList();  //New empty fringe
 println("From ",startID, " to ",goalID);
  ArrayList<Node> frontier = new ArrayList<Node> ();
  for (int i = 0; i < numNodes; i++) { //Clear visit tags and parent pointers
    visited[i] = false;
    parent[i] = -1; //No parent yet
  }
  int patience=10000;
  //println("\nBeginning Search");
  
  visited[startID] = false;
  fringe.add(startID);
  float old=9999999;
  ArrayList<Integer> path= new ArrayList<Integer> ();
  //path.add(startID);
  Node bestNode = new Node( startID, 0, goalID,nodePos,path );
   // println("Before");
  frontier.add( new Node( startID, 0, goalID,nodePos,path )  );
 // println("Start node added");
  for(int i=0;i<patience;i++){
   // println(i);
  int nodetoexpand = getBest(frontier);
  //println("Best node ",nodetoexpand);
  if (frontier.get(nodetoexpand).idx==goalID){println(frontier.get(nodetoexpand).path);return frontier.get(nodetoexpand).path;}
  ArrayList<Node> newnodes=frontier.get(nodetoexpand).expandNode();
  if(hasGoal(newnodes,goalID) ){
    bestNode=newnodes.get( getBest(newnodes) );
  old = bestNode.cost;
  }
   frontier.remove(nodetoexpand);
  frontier.addAll(newnodes);
   if (frontier.size()==0) break;
  }
  ArrayList<Integer> deadpath= new ArrayList();
  deadpath.add(startID);
  println("cant find path, run out of patience");
  return deadpath;
}

//BFS (Breadth First Search)
ArrayList<Integer> runBFS(Vec2[] nodePos, int numNodes, int startID, int goalID){
  ArrayList<Integer> fringe = new ArrayList();  //New empty fringe
  ArrayList<Integer> path = new ArrayList();
  for (int i = 0; i < numNodes; i++) { //Clear visit tags and parent pointers
    visited[i] = false;
    parent[i] = -1; //No parent yet
  }

  //println("\nBeginning Search");
  
  visited[startID] = true;
  fringe.add(startID);
  //println("Adding node", startID, "(start) to the fringe.");
  //println(" Current Fringe: ", fringe);
  
  while (fringe.size() > 0){
    int currentNode = fringe.get(0);
    fringe.remove(0);
    if (currentNode == goalID){
      //println("Goal found!");
      break;
    }
    for (int i = 0; i < neighbors[currentNode].size(); i++){
      int neighborNode = neighbors[currentNode].get(i);
      if (!visited[neighborNode]){
        visited[neighborNode] = true;
        parent[neighborNode] = currentNode;
        fringe.add(neighborNode);
        //println("Added node", neighborNode, "to the fringe.");
        //println(" Current Fringe: ", fringe);
      }
    } 
  }
  
  if (fringe.size() == 0){
    //println("No Path");
    path.add(0,-1);
    return path;
  }
    
  //print("\nReverse path: ");
  int prevNode = parent[goalID];
  path.add(0,goalID);
  //print(goalID, " ");
  while (prevNode >= 0){
    //print(prevNode," ");
    path.add(0,prevNode);
    prevNode = parent[prevNode];
  }
  //print("\n");
  
  return path;
}

public class Node {
  public int idx;
  public int goalID;
  public float cost;
  public ArrayList<Integer> path;
  public Vec2[] nodePos;

  public Node(int id,float oldcost,int goalID, Vec2[]  nodePos,ArrayList<Integer> path){
    this.idx = id;
    this.path= new ArrayList<Integer> (path);
    this.path.add(id);
    this.goalID=goalID;
    this.cost=oldcost+nodePos[id].distanceTo(nodePos[goalID]);
    
    this.nodePos=nodePos;
    
  }
  
  public ArrayList<Node> expandNode(){
   ArrayList<Node> result= new ArrayList();
  if(!visited[this.idx]){
   visited[this.idx]=true;
  for (int i = 0; i < neighbors[this.idx].size(); i++){
     Node toadd=new Node(neighbors[this.idx].get(i) ,this.cost+ nodePos[this.idx].distanceTo( nodePos[neighbors[this.idx].get(i) ] )-nodePos[this.idx].distanceTo(nodePos[goalID]) ,this.goalID,this.nodePos,this.path );
   // println(toadd.idx);
    result.add( toadd );
  }
    }
    //println(this.path);
 // println( this.idx,result.size() ,neighbors[this.idx].size());
// println(neighbors[0]);
  return result;
  }    

}
