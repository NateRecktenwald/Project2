//PDEs and Integration
//CSCI 5611 Swinging Rope [Exercise]
//Stephen J. Guy <sjguy@umn.edu>

//NOTE: The simulation starts paused, press "space" to unpause

//TODO:
//  1. The rope moves very slowly now, this is because the timestep is 1/20 of realtime
//      a. Make the timestep realtime (20 times faster than the inital code), what happens?
//      b. Call the small 1/20th timestep update, 20 times each frame (in a for loop) -- why is it different?
//  2. When the rope hanging down fully the spacing between the links is not equal, even though they
//     where initalized with an even spacing between each node. What is this?
//      - If this is a bug, fix the corisponding code
//      - If this why is not a bug, explain why this is the expected behavior
//  3. By default, the rope starts vertically. Change initScene() so it starts at an angle. The rope should
//     then swing backc and forth.
//  4. Try changing the mass and the k value. How do they interact wich each other?
//  5. Set the kv value very low, does the rope bounce a lot? What about a high kv (not too high)?
//     Why doesn't the rope stop swinging at high values of kv?
//  6. Add friction/drag so that the rope eventually stops. An easy friction model is a scaled force 
//     in the opposite direction of a nodes current velocity. 

//Challenge:
//  - Set the top of the rope to be wherever the user's mouse is, and allow the user to drag the rope around the scene.
//  - Keep the top of the rope fixed, but allow the user to click and drag one of the balls of the rope to move it around.
//  - Place a medium-sized, static 2D ball in the scene, have the nodes on the rope experience a "bounce" force if they collide with this ball.


//Create Window

//camera movement variables
//boolean moveUp = false;
//boolean moveDown = false; 
//boolean moveLeft = false;
//boolean moveRight = false;
int x = -6900;
int y = -50;
int z = -550;
int eyeX = 950;
int eyeY = 150;
int eyeZ = -950;
float rotX = -0.5235988;
float rotY = -4.450589;
float rotZ = 6.544984; 

String windowTitle = "Swinging Rope";
void setup() {
  size(2000, 1200, P3D);
  surface.setTitle(windowTitle);
  beginCamera();
  camera(eyeX, eyeY, eyeZ, x, y, z, 1.0, 1.0, 0.0);
  rotateX(rotX);
  rotateY(rotY);
  rotateZ(rotZ);
  endCamera();
  initScene();
}

//Simulation Parameters
float floor = 1000;
Vec3 gravity = new Vec3(0,400,0);
float radius = 5;
//Vec3 stringTop = new Vec3(500,50,50);
float restLenY = 8;
float restLenX = 18;
float mass = 1.15; //TRY-IT: How does changing mass affect resting length of the rope?
float k = 180; //TRY-IT: How does changing k affect resting length of the rope?
float kv = 100; //TRY-IT: How big can you make kv?



//Initial positions and velocities of masses
static int maxNodes = 100;
Vec3 pos[][] = new Vec3[maxNodes/2][maxNodes];
Vec3 vel[][] = new Vec3[maxNodes/2][maxNodes];
Vec3 acc[][] = new Vec3[maxNodes/2][maxNodes];

int numNodes = 21;

void initScene(){
  for (int i = 0; i < numNodes - 5; i++){
    for (int j = 0; j < numNodes - 1; j++) {
      pos[i][j] = new Vec3(0,0,0);
      pos[i][j].x = 1200 - (25 * j) + (25 * i);
      pos[i][j].y = 270 + 8*i; //Make each node a little lower
      pos[i][j].z = 50 + (20 * j);
      vel[i][j] = new Vec3(0,0,0);
    }
  }
}

void update(float dt){

  //Reset accelerations each timestep (momenum only applies to velocity)
  for (int i = 0; i < numNodes - 5; i++){
    for (int j = 0; j < numNodes - 1; j++) {
      acc[i][j] = new Vec3(0,0,0);
      acc[i][j].add(gravity);
    }
  }
  
  //Compute (damped) Hooke's law for each spring
  
  //virtical springs
  for (int i = 0; i < numNodes - 6; i++){
    for (int j = 0; j < numNodes - 1; j++) {
      Vec3 diffy = pos[i+1][j].minus(pos[i][j]);
      float stringFy = -k*(diffy.length() - restLenY);
      //println(stringF,diff.length(),restLen);
      
      
      
      Vec3 stringDiry = diffy.normalized();
      float projVboty = dot(vel[i][j], stringDiry);
      float projVtopy = dot(vel[i+1][j], stringDiry);
      float dampFy = -kv*(projVtopy - projVboty);
      
      
      Vec3 forcey = stringDiry.times(stringFy+dampFy);
      //System.out.println(force + "    " + stringF + "     " + dampF);
      acc[i][j].add(forcey.times(-1.0/mass));
      acc[i+1][j].add(forcey.times(1.0/mass));
      
      if(j != numNodes - 2) {
        Vec3 diffx = pos[i][j + 1].minus(pos[i][j]);
        float stringFx = -k*(diffx.length() - restLenX);
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
  }
  
  
  //Eulerian integration
  for (int i = 1; i < numNodes - 5; i++){
    for (int j = 0; j < numNodes - 1; j++) {
      vel[i][j].add(acc[i][j].times(dt));
      pos[i][j].add(vel[i][j].times(dt));
    }
  }
  
  //Collision detection and response
  for (int i = 0; i < numNodes - 5; i++){
    for (int j = 0; j < numNodes - 1; j++) {
      if (pos[i][j].y+radius > floor){
        vel[i][j].y *= -.9;
        pos[i][j].y = floor - radius;
      }
    }
  }
  
  //camera movements
  
  beginCamera();
  camera(eyeX, eyeY, eyeZ, x, y, z, 1.0, 1.0, 0.0);
  System.out.println("eyeX: " + eyeX + "  eyeY: " + eyeY + "  eyeZ: " + eyeZ + "  x: " + x + "  y: " + y + "  z: " + z);
  rotateX(rotX);
  rotateY(rotY);
  rotateZ(rotZ);
  System.out.println("rotX: " + rotX + "  rotY: " + rotY + "  rotZ: " + rotZ);
  endCamera();
  
}

//Draw the scene: one sphere per mass, one line connecting each pair
boolean paused = true;
void draw() {
  background(255,255,255);
  for (int i = 0; i < 200; i++) {
    if (!paused) update(1/(200 * frameRate));
  }
  fill(0,0,0);
  
  //spheres
  for (int i = 0; i < numNodes - 5; i++){
    for (int j = 0; j < numNodes - 2; j++) {
      pushMatrix();
      translate(pos[i][j].x,pos[i][j].y, pos[i][j].z);
      sphere(radius);
      popMatrix();
    }
  }
  
  //virtical lines
  for (int i = 0; i < numNodes - 6; i++){
    for (int j = 0; j < numNodes - 2; j++) {
      line(pos[i][j].x,pos[i][j].y,pos[i][j].z,pos[i+1][j].x,pos[i+1][j].y,pos[i+1][j].z);
    }
  }
  
  //horizontal line
  for (int i = 0; i < numNodes - 5; i++){
    for (int j = 0; j < numNodes - 3; j++) {
      line(pos[i][j].x,pos[i][j].y,pos[i][j].z,pos[i][j+1].x,pos[i][j+1].y,pos[i][j+1].z);
    }
  }
  
  
  if (paused)
    surface.setTitle(windowTitle + " [PAUSED]");
  else
    surface.setTitle(windowTitle + " "+ nf(frameRate,0,2) + "FPS");
}

void keyPressed(){
  if (key == ' ') {
    paused = !paused;
  }
  if (key == 'w') {
    z += 50;
  }
  if (key == 's') {
    z -= 50;
  }
  if (key == 'd') {
    x += 50;
  }
  if (key == 'a') {
    x -= 50;
  }
  if (key == '1') {
    y += 50;
  }
  if (key == '2') {
    y -= 50;
  }
    if (keyCode == UP) {
    eyeZ += 50;
  }
  if (keyCode == DOWN) {
    eyeZ -= 50;
  }
  if (keyCode == RIGHT) {
    eyeX += 50;
  }
  if (keyCode == LEFT) {
    eyeX -= 50;
  }
  if (key == '3') {
    eyeY += 50;
  }
  if (key == '4') {
    eyeY -= 50;
  }
  if (key == '5') {
    rotX += PI/6;
  }
  if (key == '6') {
    rotX -= PI/6;
  }
  if (key == '7') {
    rotY += PI/12;
  }
  if (key == '8') {
    rotY -= PI/12;
  }
  if (key == '9') {
    rotZ += PI/12;
  }
  if (key == '0') {
    rotZ -= PI/12;
  }
   
}

//void keyReleased() {
//  if (key == 'w') {
//    z += 0;
//  }
//  if (key == 's') {
//    z += 0;
//  }
//  if (key == 'd') {
//    x += 0;
//  }
//  if (key == 'a') {
//    x += 0;
//  }
//    if (key == UP) {
//    y += 50;
//  }
//  if (key == DOWN) {
//    y -= 50;
//  }
//}
