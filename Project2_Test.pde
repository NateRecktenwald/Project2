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

//obsticle
Vec3 obsticlePos = new Vec3(9940, 580, 230);
float obsticleRadius = 50;

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
Vec3 gravity = new Vec3(0,400,0);
float radius = 5;
//Vec3 stringTop = new Vec3(500,50,50);
float restLenY = 15;
float restLenX = 18;
float mass = 1; //TRY-IT: How does changing mass affect resting length of the rope?
float k = 2500; //TRY-IT: How does changing k affect resting length of the rope?
float kv = 1200; //TRY-IT: How big can you make kv?



//Initial positions and velocities of masses
static int maxNodes = 100;
Vec3 pos[][] = new Vec3[maxNodes][maxNodes];
Vec3 vel[][] = new Vec3[maxNodes][maxNodes];
Vec3 acc[][] = new Vec3[maxNodes][maxNodes];

int numNodes = 26;

void initScene(){
  for (int i = 0; i < numNodes - 1; i++){
    for (int j = 0; j < numNodes - 1; j++) {
      pos[i][j] = new Vec3(0,0,0);
      pos[i][j].x = 1200 - (20 * j) + (15 * i);
      pos[i][j].y = 270 - 4*i; //Make each node a little lower
      pos[i][j].z = 50 + (15 * j);
      vel[i][j] = new Vec3(0,0,0);
    }
  }
}

void update(float dt){

  //Reset accelerations each timestep (momenum only applies to velocity)
  for (int i = 0; i < numNodes - 1; i++){
    for (int j = 0; j < numNodes - 1; j++) {
      acc[i][j] = new Vec3(0,0,0);
      acc[i][j].add(gravity);
    }
  }
  
   // //air drag calculation
   //for(int i = 0; i < numNodes - 1; i++){
   //   for(int j = 0; j < numNodes - 1; j++){

   //   }
   // }
  
  //Compute (damped) Hooke's law for each spring
    //virtical springs
  for (int i = 0; i < numNodes - 2; i++){
    for (int j = 0; j < numNodes - 1; j++) {
      Vec3 diffy = pos[i+1][j].minus(pos[i][j]);
      float stringFy = -k*(diffy.length() - restLenY);
      //println(stringF,diff.length(),restLen);
      
      
      
      Vec3 stringDiry = diffy.normalized();
      float projVboty = dot(vel[i][j], stringDiry);
      float projVtopy = dot(vel[i+1][j], stringDiry);
      float dampFy = -kv*(projVtopy - projVboty);
      
      
        Vec3 forcey;
        if(j < numNodes - 3 && i < numNodes - 2) {
          float dist1 = pos[i][j].distanceTo(pos[i+1][j]);
          float dist2 = pos[j+1][j].distanceTo(pos[i][j+1]);
          float area1 = dist1 * dist2;
          //float beta = asin(vel[i][j].x/vel[i][j].length());
          float constants = 0.5*1.2*0.01;
          float airDragY = (vel[i][j].y*vel[i][j].y*area1*constants)/mass; //* cos(beta);
          if(vel[i][j].y > 0) {
            airDragY *= -1;
          }
          forcey = stringDiry.times(stringFy+dampFy+airDragY);
         }
         else {
          forcey = stringDiry.times(stringFy+dampFy);
         }
       
       //forcey = stringDiry.times(stringFy+dampFy);
      //System.out.println(force + "    " + stringF + "     " + dampF);
      acc[i][j].add(forcey.times(-1.0/mass));
      acc[i+1][j].add(forcey.times(1.0/mass));
      }
  }
  
  for(int i = 0; i < numNodes - 1; i++){
    for(int j = 0; j < numNodes - 2; j++){
        Vec3 diffx = pos[i][j + 1].minus(pos[i][j]);
        float stringFx = -k*(diffx.length() - restLenX);
        //println(stringF,diff.length(),restLen);
        
        
        
        Vec3 stringDirx = diffx.normalized();
        float projVbotx = dot(vel[i][j], stringDirx);
        float projVtopx = dot(vel[i][j+ 1], stringDirx);
        float dampFx = -kv*(projVtopx - projVbotx);
        Vec3 forcex;
        if(j < numNodes - 3 && i < numNodes - 2) {
          float dist1 = pos[i][j].distanceTo(pos[i+1][j]);
          float dist2 = pos[j+1][j].distanceTo(pos[i][j+1]);
          float area1 = dist1 * dist2;
          //float beta = asin(vel[i][j].x/vel[i][j].length()); 
          float constants = 0.5*1.2*0.01;
          float airDragX = (vel[i][j].x*vel[i][j].x*area1*constants)/mass; //* sin(beta);
          if(vel[i][j].y > 0) {
            airDragX *= -1;
          }
          forcex = stringDirx.times(stringFx+dampFx+airDragX);
        }
        else {
          forcex = stringDirx.times(stringFx+dampFx);
        }
      
        forcex = stringDirx.times(stringFx+dampFx);
        //System.out.println(force + "    " + stringF + "     " + dampF);
        acc[i][j].add(forcex.times(-1.0/mass));
        acc[i][j+1].add(forcex.times(1.0/mass));
    }
  }

  
  
  //Eulerian integration
  //for (int i = 1; i < numNodes - 5; i++){
  //  for (int j = 0; j < numNodes - 1; j++) {
  //    vel[i][j].add(acc[i][j].times(dt));
  //    pos[i][j].add(vel[i][j].times(dt));
  //  }
  //}
  
  //midpoint
  for (int i = 1; i < numNodes - 1; i++){
    for (int j = 0; j < numNodes - 1; j++) {
      Vec3 k1 = acc[i][j];
      Vec3 k2 = acc[i][j].plus(k1.div(2));
      vel[i][j].add(k2.times(dt));
      
      Vec3 k3 = vel[i][j];
      Vec3 k4 = vel[i][j].plus(k3.div(2));
      pos[i][j].add(k4.times(dt));
    }
  }
  ////rk4 integration
  //for (int i = 1; i < numNodes - 5; i++){
  //  for (int j = 0; j < numNodes - 1; j++) {
  //    Vec3 k1 = acc[i][j];
  //    Vec3 k2 = acc[i][j].plus(k1.times(dt).div(2));
  //    Vec3 k3 = acc[i][j].plus(k2.times(dt).div(2));
  //    Vec3 k4 = acc[i][j].plus(k3.times(dt));
  //    k1.plus(k2.times(2));
  //    k1.plus(k3.times(2));
  //    k1.plus(k4);
  //    k1.times(dt/6);
  //    vel[i][j].add(k1);
      
  //    Vec3 k5 = vel[i][j];
  //    Vec3 k6 = vel[i][j].plus(k5.times(dt).div(2));
  //    Vec3 k7 = vel[i][j].plus(k6.times(dt).div(2));
  //    Vec3 k8 = vel[i][j].plus(k7.times(dt));
  //    k1.plus(k6.times(2));
  //    k1.plus(k7.times(2));
  //    k1.plus(k8);
  //    k5.times(dt/6);
  //    pos[i][j].add(k5);
  //  }
  //}
  

  
  //camera movements
  
  beginCamera();
  camera(eyeX, eyeY, eyeZ, x, y, z, 1.0, 1.0, 0.0);
  //System.out.println("eyeX: " + eyeX + "  eyeY: " + eyeY + "  eyeZ: " + eyeZ + "  x: " + x + "  y: " + y + "  z: " + z);
  rotateX(rotX);
  rotateY(rotY);
  rotateZ(rotZ);
  //System.out.println("rotX: " + rotX + "  rotY: " + rotY + "  rotZ: " + rotZ);
  endCamera();
  
  //System.out.println("rotX: " + obsticlePos.x + "  rotY: " + obsticlePos.y + "  rotZ: " + obsticlePos.z);
  
  //Collision detection
  for(int i = 0; i < numNodes - 1; i++){
    for(int j = 0; j < numNodes - 1; j++){
      float dist = obsticlePos.distanceTo(pos[i][j]);
      if(dist < obsticleRadius){
       Vec3 normal = (obsticlePos.minus(pos[i][j])).times(-1);
       normal.normalize();
       Vec3 bounce = normal.times(dot(vel[i][j], normal));
       vel[i][j].subtract(bounce.times(2));
       pos[i][j].add(normal.times(2+obsticleRadius-dist));
      }
    }
  }
  
  
}

//Draw the scene: one sphere per mass, one line connecting each pair
boolean paused = true;
void draw() {
  background(255,255,255);
  for (int i = 0; i < 200; i++) {
    if (!paused) update(1/(200 * frameRate));
  }
  
  pushMatrix();
  noStroke();
  fill(255,0,0);
  lights();
  translate(obsticlePos.x, obsticlePos.y, obsticlePos.z);
  sphere(obsticleRadius);
  stroke(0,0,0);
  popMatrix();
  
  pushMatrix();
  noStroke();

  for(int y = 0; y < numNodes - 2; y++){
     beginShape(TRIANGLE_STRIP);
     fill(0,150,100 + y * 10);
   for(int x = 0; x < numNodes - 1; x++){
       vertex(pos[y][x].x, pos[y][x].y, pos[y][x].z);
       vertex(pos[y + 1][x].x, pos[y + 1][x].y, pos[y + 1][x].z);
   }
     endShape();
  }
  popMatrix();
  
  
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
    if (key == 'u') {
    eyeZ += 50;
  }
  if (key == 'j') {
    eyeZ -= 50;
  }
  if (key == 'k') {
    eyeX += 50;
  }
  if (key == 'h') {
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
  if(key == 'r')
    initScene();
  if(keyCode == RIGHT)
    obsticlePos.x += 10;
  if(keyCode == LEFT)
    obsticlePos.x -= 10;
  if(keyCode == UP)
    obsticlePos.y -= 10;
  if(keyCode == DOWN)
    obsticlePos.y += 10;
  if(key == 'n')
    obsticlePos.z += 10;
  if(key == 'm')
    obsticlePos.z -= 10;
   
}
