// Node struct
class Node {
  Vec3 pos;
  Vec3 vel;
  Vec3 last_pos;
  float radius = 0.0379;

  Node(Vec3 pos) {
    this.pos = pos;
    this.vel = new Vec3(0, 0, 0);
    this.last_pos = pos;
  }
}

class Sphere {
  Vec3 pos;
  float radius;
  
  Sphere(Vec3 pos, float radius){
    this.pos = pos;
    this.radius = radius;
  }
}

//Wind vars
Vec3 wind = new Vec3(8, 0, 0);
float k = 0.5;
float time_wind = 0;
float billow_frequency = 0.5f;
float billow_magnitude = 0.2f;

int totalNodes = 20;

Sphere ball = new Sphere(new Vec3(4.8, 5.5, 4.6), 0.3);


Node[][] nodes = new Node[totalNodes][totalNodes];


PrintWriter output;
Camera camera;

PImage myTexture;

// Link length
float link_length = 0.05;

float total_length_error(){
  //down
  float sum = 0;
  for(int x = 0; x < totalNodes-1; x++){
    for(int y = 0; y < totalNodes; y++){
      sum += nodes[x+1][y].pos.distanceTo(nodes[x][y].pos);
     }
  }
  //across
  for(int x = 0; x < totalNodes; x++){
    for(int y = 0; y < totalNodes-1; y++){
      sum += nodes[x][y+1].pos.distanceTo(nodes[x][y].pos);
    }
  }
  return abs(sum - 10);
}

float total_energy(){
  float kinetic = 0;
  float potential = 0;
  for(int i = 0; i < totalNodes; i++){
    for(int j = 0; j < totalNodes; j++){
      kinetic += 0.5 * nodes[i][j].vel.lengthSqr();
      potential += gravity.y * ((height - nodes[i][j].pos.y * scene_scale) / scene_scale);
    }
  }
  return kinetic + potential;
}

// Gravity
Vec3 gravity = new Vec3(0, 10, 0);

// Scaling factor for the scene
float scene_scale = width / 10.0f;

// Physics Parameters
int relaxation_steps = 230;
int sub_steps = 10;

void resolveCollisionsWithSphere() {
  for(int i = 0; i < totalNodes; i++){
    for(int j = 0; j < totalNodes; j++){
      Vec3 d = nodes[i][j].pos.minus(ball.pos);
      float dist = d.length();

      if(dist < ball.radius + nodes[i][j].radius) { 
          // The node is inside the sphere
          // Move the node to the surface of the sphere
          Vec3 correction = d.normalized().times(ball.radius + nodes[i][j].radius - dist);
          nodes[i][j].pos = nodes[i][j].pos.plus(correction);
      }
    }
  }
}

void setup() {
  //camera(width/2.0 - 50, height/2.0, (height/2.0) / tan(PI*30.0 / 180.0) - 50, width/2.0, height/2.0, 0, 0, 1, 0);
  camera = new Camera();
  for (int i = 0; i < totalNodes; i++) {
    for (int j = 0; j < totalNodes; j++) {
        nodes[i][j] = new Node(new Vec3(4.8 + (i * link_length), 5, 5.1 - (j * link_length)));
    }
  }

  
  output = createWriter("positions.txt"); 
  size(500, 500, P3D);
  surface.setTitle("Double Pendulum");
  scene_scale = width / 10.0f;
  smooth();
  ambientLight(50, 50, 50);
  directionalLight(255, 255, 255, 1, -1, -0.5);
  pointLight(250, 250, 250, width/2, height/2, 200);
  myTexture = loadImage("texture.png");
}

void update_physics(float dt) {
  time_wind += dt;
  float modulated_wind_magnitude = 1.0f + billow_magnitude * (float)Math.sin(billow_frequency * time_wind);
  Vec3 modulated_wind = wind.times(modulated_wind_magnitude);
  for(int i = 0; i < totalNodes; i++){
    for(int j = 0; j < totalNodes; j++){
      // Apply wind drag based on node's velocity relative to the modulated wind
      Vec3 relativeWind = modulated_wind.minus(nodes[i][j].vel);
      Vec3 windForce = relativeWind.times(k);

      // Combine forces (Gravity and Wind)
      Vec3 totalForce = gravity.plus(windForce);

      // Update velocities
      nodes[i][j].vel = nodes[i][j].vel.plus(totalForce.times(dt));

      // Update positions
      nodes[i][j].last_pos = nodes[i][j].pos;
      nodes[i][j].pos = nodes[i][j].pos.plus(nodes[i][j].vel.times(dt));
    }
  }
  
  resolveCollisionsWithSphere();
  
  // Constrain the distance between nodes to the link length
  for (int i = 0; i < relaxation_steps; i++) {
    //down
    for(int x = 0; x < totalNodes-1; x++){
      for(int y = 0; y < totalNodes; y++){
        Vec3 delta = nodes[x+1][y].pos.minus(nodes[x][y].pos);
        float delta_len = delta.length();
        float correction = (delta_len - link_length) *.1;
        Vec3 delta_normalized = delta.normalized();
        nodes[x+1][y].pos = nodes[x+1][y].pos.minus(delta_normalized.times(correction / 2));
        nodes[x][y].pos = nodes[x][y].pos.plus(delta_normalized.times(correction / 2));
      }
    }
    //across
    for(int x = 0; x < totalNodes; x++){
      for(int y = 0; y < totalNodes-1; y++){
        Vec3 delta = nodes[x][y+1].pos.minus(nodes[x][y].pos);
        float delta_len = delta.length();
        float correction = delta_len - link_length;
        Vec3 delta_normalized = delta.normalized();
        nodes[x][y+1].pos = nodes[x][y+1].pos.minus(delta_normalized.times(correction / 2));
        nodes[x][y].pos = nodes[x][y].pos.plus(delta_normalized.times(correction / 2));
      }
    }
    
    for (int x = 0; x < totalNodes; x++) {
      nodes[0][x].pos = new Vec3(4.8, 5, 5.1 - (x * link_length));
    }
  }

  // Update the velocities (PBD)
  for(int i = 0; i < totalNodes; i++){
    for(int j = 0; j < totalNodes; j++){
      nodes[i][j].vel = nodes[i][j].pos.minus(nodes[i][j].last_pos).times(1 / dt);
    }
  }
}

boolean paused = false;

void keyPressed() {
  camera.HandleKeyPressed();
  if (key == ' ') {
    paused = !paused;
    output.flush();
    output.close();
  }
}

void keyReleased()
  {
  camera.HandleKeyReleased();
  }

float time = 0;
void draw() {
  float dt = 1.0 / 20; //Dynamic dt: 1/frameRate;
  if (!paused) {
    output.println(total_length_error());
    for (int i = 0; i < sub_steps; i++) {
      time += dt / sub_steps;
      update_physics(dt / sub_steps);
    }
  }

  background(255);
  camera.Update(dt);
  
  
  //draw nodes
  fill(0, 255, 0);
  noStroke();
  lights();
  strokeWeight(link_length * scene_scale); //HERE
  for(int i = 0; i < totalNodes; i++){
    for(int j = 0; j < totalNodes; j++){
      pushMatrix();
      translate(nodes[i][j].pos.x * scene_scale, nodes[i][j].pos.y * scene_scale, nodes[i][j].pos.z * scene_scale);
      //sphere(nodes[i][j].radius * scene_scale);
      popMatrix(); 
    }
  }

  ambient(127);
  specular(255);
  shininess(200);
  
  fill(255, 0, 0);
  pushMatrix();
  translate(ball.pos.x * scene_scale, ball.pos.y * scene_scale, ball.pos.z * scene_scale);
  sphere(ball.radius * scene_scale);
  popMatrix();

  //draw lines
  textureMode(NORMAL);
  beginShape(TRIANGLES);
  texture(myTexture);
  float textureStepX = 1.0 / (totalNodes - 1);
  float textureStepY = 1.0 / (totalNodes - 1);
  for (int x = 0; x < totalNodes-1; x++) {
      for (int y = 0; y < totalNodes-1; y++) {
          // Triangle 1
          vertex(nodes[x][y].pos.x * scene_scale, nodes[x][y].pos.y * scene_scale, nodes[x][y].pos.z * scene_scale, x * textureStepX, y * textureStepY);
          vertex(nodes[x + 1][y].pos.x * scene_scale, nodes[x + 1][y].pos.y * scene_scale, nodes[x + 1][y].pos.z * scene_scale, (x + 1) * textureStepX, y * textureStepY);
          vertex(nodes[x][y + 1].pos.x * scene_scale, nodes[x][y + 1].pos.y * scene_scale, nodes[x][y + 1].pos.z * scene_scale, x * textureStepX, (y + 1) * textureStepY);
  
          // Triangle 2
          vertex(nodes[x + 1][y].pos.x * scene_scale, nodes[x + 1][y].pos.y * scene_scale, nodes[x + 1][y].pos.z * scene_scale, (x + 1) * textureStepX, y * textureStepY);
          vertex(nodes[x + 1][y + 1].pos.x * scene_scale, nodes[x + 1][y + 1].pos.y * scene_scale, nodes[x + 1][y + 1].pos.z * scene_scale, (x + 1) * textureStepX, (y + 1) * textureStepY);
          vertex(nodes[x][y + 1].pos.x * scene_scale, nodes[x][y + 1].pos.y * scene_scale, nodes[x][y + 1].pos.z * scene_scale, x * textureStepX, (y + 1) * textureStepY);
      }
  }
  endShape();
}



//---------------
//Vec 3 Library
//---------------

//Vector Library
//CSCI 5611 Vector 2 Library [Example]
// Stephen J. Guy <sjguy@umn.edu>

public class Vec3 {
  public float x, y, z;

  public Vec3(float x, float y, float z) {
    this.x = x;
    this.y = y;
    this.z = z;
  }

  public String toString() {
    return "(" + x + "," + y + "," + z + ")";
  }

  public float lengthSqr() {
    return x * x + y * y + z * z;
  }

  public Vec3 plus(Vec3 rhs) {
    return new Vec3(x + rhs.x, y + rhs.y, z + rhs.z);
  }

  public Vec3 times(float rhs) {
    return new Vec3(x * rhs, y * rhs, z * rhs);
  }
  
  public Vec3 minus(Vec3 rhs) {
    return new Vec3(x - rhs.x, y - rhs.y, z - rhs.z);
  }
  
  public float length() {
    return sqrt(x * x + y * y + z * z);
  }
  
  public Vec3 normalized() {
    float magnitude = sqrt(x * x + y * y + z * z);
    return new Vec3(x / magnitude, y / magnitude, z / magnitude);
  }
  
  public float distanceTo(Vec3 rhs) {
    float dx = rhs.x - x;
    float dy = rhs.y - y;
    float dz = rhs.z - z;
    return sqrt(dx * dx + dy * dy + dz * dz);
  }
  
  public Vec3 cross(Vec3 other) {
    float newX = this.y * other.z - this.z * other.y;
    float newY = this.z * other.x - this.x * other.z;
    float newZ = this.x * other.y - this.y * other.x;
    return new Vec3(newX, newY, newZ);
  }
  
  public float dot(Vec3 other) {
    return this.x * other.x + this.y * other.y + this.z * other.z;
  }
  
  public Vec3 multiply(Vec3 rhs){
    return new Vec3(this.x * rhs.x, this.y * rhs.y, this.z * rhs.z);
  }
  
  public void add(Vec3 rhs){
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
  }
  
  public void subtract(Vec3 rhs){
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
  }
}