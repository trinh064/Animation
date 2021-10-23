//CSCI 5611 - Graph Search & Planning
//PRM Sample Code [Proj 1]
//Instructor: Stephen J. Guy <sjguy@umn.edu>

//This is a test harness designed to help you test & debug your PRM.

//USAGE:
// On start-up your PRM will be tested on a random scene and the results printed
// Left clicking will set a red goal, right clicking the blue start
// The arrow keys will move the circular obstacle with the heavy outline
// Pressing 'r' will randomize the obstacles and re-run the tests
import java.lang.Math;
//Change the below parameters to change the scenario/roadmap size
int numObstacles = 20;
int numNodes  = 300;
  
  
//A list of circle obstacles
static int maxNumObstacles = 1000;
Vec2 circlePos[] = new Vec2[maxNumObstacles]; //Circle positions
float circleRad[] = new float[maxNumObstacles];  //Circle radii

Vec2 recPos[] = new Vec2[maxNumObstacles]; //Circle positions
//float circleRad[] = new float[maxNumObstacles];  //Circle radii

Vec2 startPos = new Vec2(100,500);
Vec2 goalPos = new Vec2(500,200);

static int maxNumNodes = 1000;
Vec2[] nodePos = new Vec2[maxNumNodes];

static int maxNumAgents = 20;
int numAgents = 20;

float k_goal = 2.5;  //TODO: Tune this parameter to agent stop naturally on their goals
float k_avoid = 8;
float goalSpeed = 40;
int agentw=36;
int agenth=60;
float agentRad = agenth-10;

//The agent states
Vec2[] agentPos = new Vec2[maxNumAgents];
Vec2[] agentVel = new Vec2[maxNumAgents];
Vec2[] agentAcc = new Vec2[maxNumAgents];

//The agent goals
Vec2[] goalpos = new Vec2[maxNumAgents];

float computeTTC(Vec2 pos1, Vec2 vel1, float radius1, Vec2 pos2, Vec2 vel2, float radius2){
  float combinedRadius = radius1+radius2; //<>//
  Vec2 relativeVelocity = vel1.minus(vel2);
  float ttc = rayCircleIntersectTime(pos2, combinedRadius, pos1, relativeVelocity);
  return ttc;
}
/*
Vec2 computeAgentForces(int id){
  //TODO: Make this better
  Vec2 acc = new Vec2(0,0);
  Vec2 goal_vel=goalpos[id].minus(agentPos[id]);
    goal_vel.setToLength(goalSpeed);

  //goal_vel.setToLength(0);
  //Vec2 goal_vel= new Vec2 (0,0);
  Vec2 goal_force=goal_vel.minus(agentVel[id]);
  goal_force.mul(k_goal);
  acc.add(goal_force);
  
  return acc;
}
*/
Vec2 computeAgentForces(int id){
  //TODO: Make this better
  Vec2 acc = new Vec2(0,0);
        
  Vec2 goal_vel=tempgoal[id].minus(agentPos[id]);
  
  goal_vel.setToLength(goalSpeed);
  //Vec2 goal_vel= new Vec2 (0,0);
  Vec2 goal_force=goal_vel.minus(agentVel[id]);
  // println("path ",tempgoal[id].x);
  goal_force.mul(k_goal);
  acc.add(goal_force);
  
  return acc;
}

float rayCircleIntersectTime(Vec2 center, float r, Vec2 l_start, Vec2 l_dir){
 
  //Compute displacement vector pointing from the start of the line segment to the center of the circle
  Vec2 toCircle = center.minus(l_start);
 
  //Solve quadratic equation for intersection point (in terms of l_dir and toCircle)
  float a = l_dir.length();
  float b = -2*dot(l_dir,toCircle); //-2*dot(l_dir,toCircle)
  float c = toCircle.lengthSqr() - (r*r); //different of squared distances
 
  float d = b*b - 4*a*c; //discriminant
 
  if (d >=0 ){
    //If d is positive we know the line is colliding
    float t = (-b - sqrt(d))/(2*a); //Optimization: we typically only need the first collision!
    if (t >= 0) return t;
    return -1;
  }
 
  return -1; //We are not colliding, so there is no good t to return
}
// Compute attractive forces to draw agents to their goals,
// and avoidance forces to anticipatory avoid collisions


int[]progress=new int[numAgents];
Vec2[] tempgoal=new Vec2[numAgents];
boolean [] done=new boolean[numAgents];

float [] rot=new float[numAgents];
boolean [] donerot=new boolean[numAgents];

void moveAgent(float dt){
  //Compute accelerations for every agents
  for (int i = 0; i < numAgents; i++){
    Vec2 totaforce=new Vec2(0,0);
      if(done[i]) continue;
    if(agentPos[i].distanceTo(nodePos[ paths[i].get(progress[i]) ] ) <agentw ){if( progress[i]<paths[i].size()-1 ){progress[i]+=1;donerot[i]=false; tempgoal[i]=nodePos[paths[i].get(progress[i])];} else{done[i]=true;}   }
    
    agentAcc[i] = computeAgentForces(i);
     
    for (int j = 0; j < numAgents; j++){
       if(done[j]) continue;
    if (j!=i){
      if (agentPos[i].distanceTo(tempgoal[i])>10 ){
      float ttc=computeTTC( agentPos[i], agentVel[i], agentRad, agentPos[j], agentVel[j], agentRad);
      if (ttc!=-1 && ttc<1  ){
      //  println("collide",ttc);
      Vec2 A_future = agentPos[i].plus(agentVel[i].times(ttc));
      Vec2 B_future = agentPos[j].plus(agentVel[j].times(ttc));
      Vec2 relative_future_direction = A_future.minus(B_future).normalized();
     //totaforce.add(  relative_future_direction.times(1/ttc)  );
     //println( totaforce.x, totaforce.y);
     //println(  relative_future_direction.times(1/ttc).times(k_avoid));
      if ( (abs(A_future.minus(B_future).y/A_future.minus(B_future).x ) - abs(agentVel[i].y/agentVel[i].x) <0.001) || (A_future.minus(B_future).y == 0 && agentVel[i].y==0) || (A_future.minus(B_future).x == 0 && agentVel[i].x==0))    
        {agentAcc[i].add(  relative_future_direction.perpendicular().times(1/ttc).times(k_avoid) ); }
        else{
        agentAcc[i].add(  relative_future_direction.times(1/ttc).times(k_avoid) );
        //println( agentAcc[i].x, agentAcc[i].y);
        }
    }
    }
    }
     //println( totaforce.normalized().times(k_avoid).x, totaforce.normalized().times(k_avoid).y);
     //agentAcc[i].add( totaforce.normalized().times(k_avoid) );
     
if(agentAcc[i].length()>500){
//println( agentAcc[i].length());
agentAcc[i].setToLength(500);}
  }}
   for (int i = 0; i < numAgents; i++){
     float aim=0;
     if(agentVel[i].y==0){aim=3.1416/2;}
  else{aim=-atan(agentVel[i].x/agentVel[i].y);}
     if( (agentVel[i].x<0 && agentVel[i].y<0) || (agentVel[i].x>0 && agentVel[i].y<0) ){aim-=3.1416;}
     if(abs(aim-rot[i]) < 2*3.1416 - abs(aim-rot[i]))
     {rot[i]+=(aim-rot[i])*2*dt;}
     else {
       if(aim-rot[i]>0)
     {rot[i]+=((aim-rot[i])-2*3.1416)*3*dt;}
     else{
     rot[i]+=((aim-rot[i])+2*3.1416)*3*dt;}
     }
     if(abs(aim-rot[i])<0.01) {donerot[i]=true;}
    //println( rot[i]);
    float oldvel=agentVel[i].length();
   }
      
  //Update position and velocity using (Eulerian) numerical integration
  for (int i = 0; i < numAgents; i++){
   if( !done[i])
    {agentVel[i].add(agentAcc[i].times(dt));
    agentPos[i].add(agentVel[i].times(dt));}
  }

}

  float [] offsetrad=new float[circleRad.length];

//Generate non-colliding PRM nodes
void generateRandomNodes(int numNodes, Vec2[] circleCenters, float[] circleRadii){
  
  for (int i = 0; i < numNodes; i++){
    Vec2 randPos = new Vec2(random(width),random(height));
    boolean insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos,2);
    //boolean insideBox = pointInBox(boxTopLeft, boxW, boxH, randPos);
    //boolean vcheck=voronoi_check(randPos);
    while (insideAnyCircle  ){
      randPos = new Vec2(random(width),random(height));
      insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos,2);
      //vcheck=voronoi_check(randPos);
      //insideBox = pointInBox(boxTopLeft, boxW, boxH, randPos);
    }
    nodePos[i] = randPos;
  }
  
  for (int i = 0; i < numAgents; i++){
   
  nodePos[i] = new Vec2 (agentPos[i].x,agentPos[i].y);
  nodePos[i+numAgents] = new Vec2 (goalpos[i].x,goalpos[i].y);
  }
}

void placeRandomObstacles(int numObstacles){
  //Initial obstacle position
  
  for (int i = 0; i < numObstacles; i++){
    circlePos[i] = new Vec2(random(50,950),random(50,700));
    circleRad[i] = random(20,60);
  offsetrad[i]= circleRad[i]+agentw/2+8;
   // println("offset",offsetrad[i]);

  }
  
  circleRad[0] = 30; //Make the first obstacle big
}

void placeRandomAgents(){
  //Initial obstacle position
 
  for (int i = 0; i < numAgents; i++){
   agentPos[i]=sampleFreePos();
   goalpos[i]=sampleFreePos();
 
  }
   // println("placing", circleRad[i]);
 return;
}

ArrayList<Integer> curPath;

ArrayList<Integer>[] paths = new ArrayList[numAgents];
PImage agentim,obim,bg; 
int strokeWidth = 2;
void setup(){
 
  size(1024,768); //<>//
  agentim = loadImage("agent.png"); //What image to load, experiment with transparent images 
  obim=loadImage("obstacle.png");
  bg=loadImage("background.jpg");
  bg.resize(1024,768);
  agentim.resize(agentw,agenth);
  //paths=new ArrayList<Integer>[numAgents] ;
  //Set initial agent positions and goals
  /*
  agentPos[0] = new Vec2(50,50);
  agentPos[1] = new Vec2(290,650);
  agentPos[2] = new Vec2(360,50);
  agentPos[3] = new Vec2(120,50);
  agentPos[4] = new Vec2(90,650);
  
  goalpos[0] = new Vec2(900,700);
  goalpos[1] = new Vec2(500,120);
  goalpos[2] = new Vec2(300,520);
  goalpos[3] = new Vec2(400,520);
  goalpos[4] = new Vec2(400,120);
 */
  //Set initial velocities to cary agents towards their goals
    placeRandomObstacles(numObstacles);

    placeRandomAgents();
    //agentPos[0] = new Vec2(220,610);
 // agentPos[1] = new Vec2(520,310);
  
  //goalpos[0] = new Vec2(520,310);
  //goalpos[1] = new Vec2(220,610);

  for (int i = 0; i < numAgents; i++){
    agentVel[i] = goalpos[i].minus(agentPos[i]);
    if (agentVel[i].length() > 0)
      agentVel[i].setToLength(goalSpeed);
  }
  
  testPRM();
  pause=true;
}

int numCollisions;
float pathLength;
boolean reachedGoal;
void pathQuality(){
  Vec2 dir;
  hitInfo hit;
  float segmentLength;
  numCollisions = 9999; pathLength = 9999;
  if (curPath.size() == 1 && curPath.get(0) == -1) return; //No path found  
  
  pathLength = 0; numCollisions = 0;
  
  if (curPath.size() == 0 ){ //Path found with no nodes (direct start-to-goal path)
    segmentLength = startPos.distanceTo(goalPos);
    pathLength += segmentLength;
    dir = goalPos.minus(startPos).normalized();
    hit = rayCircleListIntersect(circlePos, circleRad, numObstacles, startPos, dir, segmentLength);
    if (hit.hit) numCollisions += 1;
    return;
  }
  
  segmentLength = startPos.distanceTo(nodePos[curPath.get(0)]);
  pathLength += segmentLength;
  dir = nodePos[curPath.get(0)].minus(startPos).normalized();
  hit = rayCircleListIntersect(circlePos, circleRad, numObstacles, startPos, dir, segmentLength);
  if (hit.hit) numCollisions += 1;
  
  
  for (int i = 0; i < curPath.size()-1; i++){
    int curNode = curPath.get(i);
    int nextNode = curPath.get(i+1);
    segmentLength = nodePos[curNode].distanceTo(nodePos[nextNode]);
    pathLength += segmentLength;
    
    dir = nodePos[nextNode].minus(nodePos[curNode]).normalized();
    hit = rayCircleListIntersect(circlePos, circleRad, numObstacles, nodePos[curNode], dir, segmentLength);
    if (hit.hit) numCollisions += 1;
  }
  
  int lastNode = curPath.get(curPath.size()-1);
  segmentLength = nodePos[lastNode].distanceTo(goalPos);
  pathLength += segmentLength;
  dir = goalPos.minus(nodePos[lastNode]).normalized();
  hit = rayCircleListIntersect(circlePos, circleRad, numObstacles, nodePos[lastNode], dir, segmentLength);
  if (hit.hit) numCollisions += 1;
}
boolean voronoi_check(Vec2 randPos){
  float k1=999999;
  float k2=999999;
  int id=0;
  for (int i=0;i<numObstacles;i++){
   float d= randPos.distanceTo(circlePos[i]);
  if(d<k1 ) {
  k1=d;
  id=i;
  }}
  
  for (int i=0;i<numObstacles;i++){
   float d= randPos.distanceTo(circlePos[i]);
  if(d<k2 && id !=i) {
  k2=d;
  }
  }
  println(k1,k2,k1!=k2);
  return k1!=k2;
}
Vec2 sampleFreePos(){
  Vec2 randPos = new Vec2(random(width),random(height));
  
  boolean insideAnyCircle = pointInCircleList(circlePos,offsetrad,numObstacles,randPos,2);
  while (insideAnyCircle ){
    randPos = new Vec2(random(width),random(height));
    insideAnyCircle = pointInCircleList(circlePos,offsetrad,numObstacles,randPos,2);
  }
  return randPos;
}

void testPRM(){
  long startTime, endTime;
  
  startPos = sampleFreePos();
  goalPos = sampleFreePos();
  
  generateRandomNodes(numNodes, circlePos, offsetrad);
  connectNeighbors(circlePos, offsetrad, numObstacles, nodePos, numNodes);
  
  //startTime = System.nanoTime();
  //curPath = (startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
  curPath=new ArrayList<Integer>();
  
 // endTime = System.nanoTime();
  //pathQuality();
  
  //println("Nodes:", numNodes," Obstacles:", numObstacles," Time (us):", int((endTime-startTime)/1000),
  //        " Path Len:", pathLength, " Path Segment:", curPath.size()+1,  " Num Collisions:", numCollisions);
  
  for (int i=0;i<numAgents;i++){
  paths[i]=planPath(agentPos[i], goalpos[i], circlePos, circleRad, numObstacles, nodePos, numNodes);
  progress[i]=0;
  //println( progress[i]);
  tempgoal[i]=nodePos[paths[i].get(progress[i])];
  //agentVel[i]=tempgoal[i].minus(agentPos[i]);
  //agentVel[i]=agentVel[i].setToLength(goalSpeed);
  done[i]=false;
  if(agentVel[i].y==0){rot[i]=3.1416/2;}
  else{rot[i]=-atan(agentVel[i].x/agentVel[i].y);}
  donerot[i]=false;
  //println(agentVel[i].x, agentVel[i].y);
  }
 return;
  
}

void draw(){
  
  //println("FrameRate:",frameRate);
  strokeWeight(1);
  background(bg); //Grey background
  //camera(mouseX, height/2, (height/2) / tan(PI/6), width/2, height/2, 0, 0, 1, 0);
  //translate(width/2, height/2, -100);
  stroke(0,0,0);
  fill(255,255,255);
  
  
  //Draw the circle obstacles
  for (int i = 0; i < numObstacles; i++){
    Vec2 c = circlePos[i];
    float r = circleRad[i];
    //circle(c.x,c.y,r*2);
    image(obim,c.x-r,c.y-r,r*2,r*2);
  }
  //Draw the first circle a little special b/c the user controls it
  fill(240);
  strokeWeight(2);
  //circle(circlePos[0].x,circlePos[0].y,circleRad[0]*2);
  image(obim,circlePos[0].x-circleRad[0],circlePos[0].y-circleRad[0],circleRad[0]*2,circleRad[0]*2);
  strokeWeight(1);
  
  //Draw PRM Nodes
  fill(0);
  for (int i = 0; i < numNodes; i++){
  //  circle(nodePos[i].x,nodePos[i].y,5);
  }
  
  //Draw graph
  /*
  stroke(100,100,100);
  strokeWeight(1);
  for (int i = 0; i < numNodes; i++){
    for (int j : neighbors[i]){
      //line(nodePos[i].x,nodePos[i].y,nodePos[j].x,nodePos[j].y);
    }
  }
  */
  //Draw Start and Goal
  //fill(20,60,250);
  //circle(nodePos[startNode].x,nodePos[startNode].y,20);
  //circle(startPos.x,startPos.y,20);
 // fill(250,30,50);
  //circle(nodePos[goalNode].x,nodePos[goalNode].y,20);
 // circle(goalPos.x,goalPos.y,20);
  
  if (curPath.size() >0 && curPath.get(0) == -1) return; //No path found
  
  //Draw Planned Path
  stroke(20,255,40);
  strokeWeight(4);
  /*if (curPath.size() == 0){
    //line(startPos.x,startPos.y,goalPos.x,goalPos.y);
    return;
  }
  //line(startPos.x,startPos.y,nodePos[curPath.get(0)].x,nodePos[curPath.get(0)].y);
  for (int i = 0; i < curPath.size()-1; i++){
    int curNode = curPath.get(i);
    int nextNode = curPath.get(i+1);
    line(nodePos[curNode].x,nodePos[curNode].y,nodePos[nextNode].x,nodePos[nextNode].y);
  }*/
  //line(goalPos.x,goalPos.y,nodePos[curPath.get(curPath.size()-1)].x,nodePos[curPath.get(curPath.size()-1)].y);
  
  for (int a=0;a<numAgents;a++){
  
    for (int i = 0; i < paths[a].size()-1; i++){
    int curNode = paths[a].get(i);
    int nextNode = paths[a].get(i+1);
    //(nodePos[curNode].x,nodePos[curNode].y,nodePos[nextNode].x,nodePos[nextNode].y);
  }
    
  }
   
 strokeWeight(1);
  //Draw orange goal rectangle
  fill(100,150,50);
  for (int i = 0; i < numAgents; i++){
    rect(goalpos[i].x-15, goalpos[i].y-15, 30, 30);
  }
 
  //Draw the green agents
  fill(20,200,150);
  for (int i = 0; i < numAgents; i++){
    
   pushMatrix();
  translate(agentPos[i].x,agentPos[i].y);
  rotate(rot[i]);
  //rect(-agentw/2,-agenth/2,agentw, agenth);
  image(agentim, -agentw/2 ,-agenth/2);
  popMatrix();
   //circle(agentPos[i].x, agentPos[i].y, agentRad*2);
  }
  
  if(!pause){
   moveAgent(1.0/frameRate);
}

}

void drawimage(PImage im, Vec2 pos ){
int w=im.width;
int h=im.height;
  beginShape();
  texture(im);
  // vertex( x, y, z, u, v) where u and v are the texture coordinates in pixels
  vertex(-pos.x-w/2,-pos.x-h/2, 0, 0, 0);
  vertex(pos.x+w/2, -pos.x-h/2, 0, im.width, 0);
  vertex(pos.x+w/2, pos.y+h/2, 0, im.width, im.height);
  vertex(-pos.x-w/2, pos.y+h/2, 0, 0, im.height);
  endShape();
  
  return;
}

boolean pause=true;
boolean shiftDown = false;
void keyPressed(){
  if (key == 'r'){
    setup();
    return;
  }
  
  if (keyCode == SHIFT){
    shiftDown = true;
  }
  if (key == ' '){
    pause = !pause;
  }
  
  float speed = 10;
  if (shiftDown) speed = 30;
  if (keyCode == RIGHT){
    circlePos[0].x += speed;
  }
  if (keyCode == LEFT){
    circlePos[0].x -= speed;
  }
  if (keyCode == UP){
    circlePos[0].y -= speed;
  }
  if (keyCode == DOWN){
    circlePos[0].y += speed;
  }
  connectNeighbors(circlePos, offsetrad, numObstacles, nodePos, numNodes);
  curPath = planPath(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
}

void keyReleased(){
  if (keyCode == SHIFT){
    shiftDown = false;
  }
}
/*
void mousePressed(){
  if (mouseButton == RIGHT){
    startPos = new Vec2(mouseX, mouseY);
    //println("New Start is",startPos.x, startPos.y);
  }
  else{
    goalPos = new Vec2(mouseX, mouseY);
    //println("New Goal is",goalPos.x, goalPos.y);
  }
  //curPath = planPath(startPos, goalPos, circlePos, circleRad, numObstacles, nodePos, numNodes);
}*/
