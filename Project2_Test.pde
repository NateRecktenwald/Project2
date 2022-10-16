//Create Window
String windowTitle = "Swinging Rope";
void setup() {
  size(1028, 720, P3D);
  surface.setTitle(windowTitle);
  initScene();
}

//Simulation Parameters
float floor = 500;
Vec3 gravity = new Vec3(0,400,0);
float radius = 5;
//Vec2 stringTop = new Vec3(200,50);
float restLen = 10;

//causes curtain effect
//float restLenX = 18;
//float restLenY = 8;

float mass = 1.0; //TRY-IT: How does changing mass affect resting length of the rope?
float k = 2000; //TRY-IT: How does changing k affect resting length of the rope?
float kv = 1000; //TRY-IT: How big can you make kv?

//Initial positions and velocities of masses
int rows = 20;
int cols = 20;
static int maxNodes = 500;
Vec3 pos[][] = new Vec3[rows+1][cols+1];
Vec3 vel[][] = new Vec3[rows+1][cols+1];
Vec3 acc[][] = new Vec3[rows+1][cols+1];

//obsticle
Vec3 obsticlePos = new Vec3(300, 500, 200);
float obsticleRadius = 50;


int numNodes = 20*20;

void initScene(){
  float x = 300;
  for (int i = 0; i < rows; i++){
    float z = 100;
    for(int j = 0; j < cols; j++){
    pos[i][j] = new Vec3(x,300,z);
    vel[i][j] = new Vec3(0,0,0);
    z = z + 10;
  }
  x = x + 10;
}
}

void update(float dt){
  //Vec3 velTemp[][] = new Vec3[rows+1][cols+1];
  //for(int i = 0; i < rows; i++){
  //  for(int j = 0; j < cols; j++){
  //    velTemp[i][j] = vel[i][j];
  //  }
  //}
  
  //for(int i = 0; i < rows-1; i++){
  // for(int j = 0; j < cols; j++){
  //  Vec3 diff = pos[i+1][j].minus(pos[i][j]);
  //  diff.normalize();
  //  float v1 = dot(diff, vel[i][j]);
  //  float v2 = dot(diff, vel[i+1][j]);
  //  float force = -k*(10-1)-kv*(v1-v2);
  //  velTemp[i][j].add(diff.times(force*dt));
  //  velTemp[i+1][j].subtract(diff.times(force*dt));
  // }
  //}
  
  //for(int i = 0; i < rows; i++){
  // for(int j = 0; j < cols-1; j++){
  //  Vec3 diff = pos[i][j+1].minus(pos[i][j]);
  //  diff.normalize();
  //  float v1 = dot(diff, vel[i][j]);
  //  float v2 = dot(diff, vel[i][j+1]);
  //  float force = -k*(10-1)-kv*(v1-v2);
  //  velTemp[i][j].add(diff.times(force*dt));
  //  velTemp[i][j+1].subtract(diff.times(force*dt));
  // }
  //}
  
  //for(int i = 0; i < rows; i++){
  // for(int j = 0; j < cols; j++){
  //   velTemp[i][j].add(gravity);
  //   velTemp[0][j] = new Vec3(0, 0, 0);
  //   vel[i][j] = velTemp[i][j];
  //   pos[i][j].add(vel[i][j].times(dt));
  // }
  //}
  
  
  
  //Reset accelerations each timestep (momenum only applies to velocity)
  for (int i = 0; i < rows; i++){
    for (int j = 0; j < cols; j++) {
      acc[i][j] = new Vec3(0,0,0);
      acc[i][j].add(gravity);
    }
  }
  
  //Compute (damped) Hooke's law for each spring
  
  //virtical springs
  for (int i = 0; i < rows-1; i++){
    for (int j = 0; j < cols; j++) {
      Vec3 diffy = pos[i+1][j].minus(pos[i][j]);
      float stringFy = -k*(diffy.length() - restLen);
      //println(stringF,diff.length(),restLen);
      
      
      
      Vec3 stringDiry = diffy.normalized();
      float projVboty = dot(vel[i][j], stringDiry);
      float projVtopy = dot(vel[i+1][j], stringDiry);
      float dampFy = -kv*(projVtopy - projVboty);
      
      
      Vec3 forcey = stringDiry.times(stringFy+dampFy);
      //System.out.println(force + "    " + stringF + "     " + dampF);
      acc[i][j].add(forcey.times(-1.0/mass));
      acc[i+1][j].add(forcey.times(1.0/mass));
      }
  }
  
  for(int i = 0; i < rows; i++){
    for(int j = 0; j < cols - 1; j++){
          Vec3 diffx = pos[i][j + 1].minus(pos[i][j]);
        float stringFx = -k*(diffx.length() - restLen);
        //println(stringF,diff.length(),restLen);
        
        
        
        Vec3 stringDirx = diffx.normalized();
        float projVbotx = dot(vel[i][j], stringDirx);
        float projVtopx = dot(vel[i][j+ 1], stringDirx);
        float dampFx = -kv*(projVtopx - projVbotx);
        
        
        Vec3 forcex = stringDirx.times(stringFx+dampFx);
        //System.out.println(force + "    " + stringF + "     " + dampF);
        acc[i][j].add(forcex.times(-1.0/mass));
        acc[i][j+1].add(forcex.times(1.0/mass));
    }
  }
  
  
  //Eulerian integration
  for (int i = 1; i < rows; i++){
    for (int j = 0; j < cols; j++) {
      vel[i][j].add(acc[i][j].times(dt));
      pos[i][j].add(vel[i][j].times(dt));
    }
  }
  
  //Collision detection and response
  for (int i = 0; i < rows; i++){
    for (int j = 0; j < cols; j++) {
      if (pos[i][j].y+radius > floor){
        vel[i][j].y *= -.9;
        pos[i][j].y = floor - radius;
      }
    }
  }
  
  //Collision detection
  for(int i = 0; i < rows; i++){
    for(int j = 0; j < cols; j++){
      float dist = obsticlePos.distanceTo(pos[i][j]);
      if(dist < obsticleRadius+0.09){
       Vec3 normal = (obsticlePos.minus(pos[i][j])).times(-1);
       normal.normalize();
       Vec3 bounce = normal.times(dot(vel[i][j], normal));
       vel[i][j].subtract(bounce.times(.5));
       pos[i][j].add(normal.times(.1+obsticleRadius-dist));
      }
    }
  }
}

//Draw the scene: one sphere per mass, one line connecting each pair
boolean paused = true;
void draw() {
  background(255,255,255);
  for(int i = 0; i < 1000; i++){
    if (!paused) update(1/(1000*frameRate));
  }
  fill(0,0,0);
  
  //spheres
  for (int i = 0; i < rows; i++){
    for (int j = 0; j < cols; j++) {
      pushMatrix();
      translate(pos[i][j].x,pos[i][j].y, pos[i][j].z);
      sphere(1);
      popMatrix();
    }
  }
  
  pushMatrix();
  noStroke();
  fill(255,0,0);
  lights();
  translate(obsticlePos.x, obsticlePos.y, obsticlePos.z);
  sphere(obsticleRadius);
  stroke(0,0,0);
  popMatrix();
  
  //virtical lines
  for (int i = 0; i < rows-1; i++){
    for (int j = 0; j < cols; j++) {
      line(pos[i][j].x,pos[i][j].y,pos[i][j].z,pos[i+1][j].x,pos[i+1][j].y,pos[i+1][j].z);
    }
  }
  
  //horizontal line
  for (int i = 0; i < rows; i++){
    for (int j = 0; j < cols-1; j++) {
      line(pos[i][j].x,pos[i][j].y,pos[i][j].z,pos[i][j+1].x,pos[i][j+1].y,pos[i][j+1].z);
    }
  }
  
  if (paused)
    surface.setTitle(windowTitle + " [PAUSED]");
  else
    surface.setTitle(windowTitle + " "+ nf(frameRate,0,2) + "FPS");
}

void keyPressed(){
  if (key == ' ')
    paused = !paused;
  if(key == 'r')
     initScene();
}
