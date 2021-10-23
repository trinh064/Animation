

//Simulation paramaters
static int maxParticles =500;
Vec2 spherePos = new Vec2(300,400);
float sphereRadius = 60;
float r = 5;
float genRate = 20;
float obstacleSpeed = 200;
float COR = 0.7;
float smoothR=50;
Vec2 gravity = new Vec2(0,400);
boolean ver = false;
boolean hor = false;
//Initalalize variable
Vec2 pos[] = new Vec2[maxParticles];
Vec2 posOld[] = new Vec2[maxParticles];
Vec2 vel[] = new Vec2[maxParticles];
Vec2 acc[] = new Vec2[maxParticles];

float displace[]=new float[maxParticles];
int numParticles = 500;
//float life[]= new float[maxParticles];
//float lifecycle=10;
float k_stiff=3000;
float k_stiffN=10000;
float restden=5;
void setup(){
  size(800,600,P3D);
  surface.setTitle("Particle System [CSCI 5611 Example]");
  strokeWeight(2); //Draw thicker lines 
  initscene();

}

Vec2 obstacleVel = new Vec2(0,0);

void initscene(){
  for (int i = 0; i < numParticles; i++){
    if (numParticles > maxParticles) break;
    pos[i] = new Vec2(20+random(20),300+random(20));
    posOld[i]=new Vec2(20,200);
    vel[i] = new Vec2(0,0); 
    acc[i] = new Vec2(0,gravity.y);
    displace[i]=0;
   // life[numParticles]=500;
   println("init");
  }
  return;
}
void update(float dt){

  for (int i = 0; i <  numParticles; i++){
   
    if (pos[i].y > height - r){
      pos[i].y = height-r;
      vel[i].y *= -COR;
    }
    if (pos[i].y < r){
      pos[i].y = r;
      vel[i].y = -vel[i].y;
    }
    if (pos[i].x > width - r){
      pos[i].x = width - r;
      vel[i].x *= -COR;
    }
    if (pos[i].x < r){
      pos[i].x = r;
      vel[i].x = -vel[i].x;
    }
  }
  
 
  
  for (int i = 0; i <  numParticles; i++){
    //vel[i]=pos[i].minus(posOld[i]).times(1/dt);
    posOld[i]=pos[i];
    if(g )vel[i].y+=gravity.y*dt;
   
    pos[i].add(vel[i].times(dt) );
  }
  
    for (int i = 0; i <  numParticles; i++){
    float den=0;
    float denN=0;
    float press=0;
    float pressN=0;
    //Get density
    for (int j = 0; j <  numParticles; j++){
      float dist=posOld[i].distanceTo(pos[j]);
      if(dist>0 && dist<smoothR){
      float q,q2,q3;
      q=1-dist/smoothR;
      q2=q*q;
      q3=q*q*q;
      den+=q2;
      denN+=q3;
      }
  }
  //get displace and move
  for (int j = 0; j <  numParticles; j++){
      float dist=posOld[i].distanceTo(pos[j]);
      if(dist>0 && dist<smoothR){
      float q,q2,q3;
      q=1-dist/smoothR;
      q2=q*q;
      q3=q*q*q;
      //println(den);
      press=k_stiff*(den-restden);
      pressN=k_stiffN*denN;
      float d=(press*q+pressN*q2)*dt*dt;
      Vec2 b2a=posOld[j].minus(posOld[i]).normalized();
      vel[i].add(b2a.times(-d/dt) );
      vel[j].add(b2a.times(d/dt) );
      }
  }}
  

  
}

boolean leftPressed, rightPressed, upPressed, downPressed, shiftPressed,reset;
void keyPressed(){
  if (keyCode == LEFT) {leftPressed = true;hor=true;}
  if (keyCode == RIGHT) {rightPressed = true;hor=true;}
  if (keyCode == UP) {upPressed = true; ver=true;}
  if (keyCode == DOWN) {downPressed = true;ver=true;}
  if (keyCode == SHIFT) shiftPressed = true;
  if (key == ' ') paused = !paused;
  if (key == 'g') {g = !g;}
}

void keyReleased(){
  if (key == 'r'){
    println("Reseting the System");
    reset=true;
  }
  if (keyCode == LEFT) {leftPressed = false;hor=false;}
  if (keyCode == RIGHT) {rightPressed = false;hor=false;}
  if (keyCode == UP) {upPressed = false; ver=false;}
  if (keyCode == DOWN) {downPressed = false;ver=false;}
  if (keyCode == SHIFT) shiftPressed = false;
}


boolean paused = true;
boolean g=true;
void draw(){
  
  if (!paused) {for (int i=0;i<20;i++) {update(1.0/(20*frameRate));}   
  background(255); //White background
  stroke(0,0,0);
  for (int i = 0; i < numParticles; i++){
    //fill(20,20,life[i]);
    circle(pos[i].x, pos[i].y, r*2); //(x, y, diameter)
  }
  fill(20,20,200);
  //circle(spherePos.x, spherePos.y, sphereRadius*2); //(x, y, diameter)
} }






// Begin the Vec2 Libraray

//Vector Library
//CSCI 5611 Vector 2 Library [Example]
// Stephen J. Guy <sjguy@umn.edu>

public class Vec2 {
  public float x, y;
  
  public Vec2(float x, float y){
    this.x = x;
    this.y = y;
  }
  
  public String toString(){
    return "(" + x+ "," + y +")";
  }
  
  public float length(){
    return sqrt(x*x+y*y);
  }
  
  public Vec2 plus(Vec2 rhs){
    return new Vec2(x+rhs.x, y+rhs.y);
  }
  
  public void add(Vec2 rhs){
    x += rhs.x;
    y += rhs.y;
  }
  
  public Vec2 minus(Vec2 rhs){
    return new Vec2(x-rhs.x, y-rhs.y);
  }
  
  public void subtract(Vec2 rhs){
    x -= rhs.x;
    y -= rhs.y;
  }
  
  public Vec2 times(float rhs){
    return new Vec2(x*rhs, y*rhs);
  }
  
  public void mul(float rhs){
    x *= rhs;
    y *= rhs;
  }
  
  public void clampToLength(float maxL){
    float magnitude = sqrt(x*x + y*y);
    if (magnitude > maxL){
      x *= maxL/magnitude;
      y *= maxL/magnitude;
    }
  }
  
  public void setToLength(float newL){
    float magnitude = sqrt(x*x + y*y);
    x *= newL/magnitude;
    y *= newL/magnitude;
  }
  
  public void normalize(){
    float magnitude = sqrt(x*x + y*y);
    x /= magnitude;
    y /= magnitude;
  }
  
  public Vec2 normalized(){
    float magnitude = sqrt(x*x + y*y);
    return new Vec2(x/magnitude, y/magnitude);
  }
  
  public float distanceTo(Vec2 rhs){
    float dx = rhs.x - x;
    float dy = rhs.y - y;
    return sqrt(dx*dx + dy*dy);
  }
}

Vec2 interpolate(Vec2 a, Vec2 b, float t){
  return a.plus((b.minus(a)).times(t));
}

float interpolate(float a, float b, float t){
  return a + ((b-a)*t);
}

float dot(Vec2 a, Vec2 b){
  return a.x*b.x + a.y*b.y;
}

Vec2 projAB(Vec2 a, Vec2 b){
  return b.times(a.x*b.x + a.y*b.y);
}
