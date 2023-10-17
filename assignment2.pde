// Node struct
class Node {
  Vec3 pos;
  Vec3 vel;
  Vec3 last_pos;
  float radius = 0.01;

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

// Nodes
Node base1 = new Node(new Vec3(5, 5, 5));
Node base2 = new Node(new Vec3(5, 5, 4.8));
Node base3 = new Node(new Vec3(5, 5, 4.6));
Node base4 = new Node(new Vec3(5, 5, 4.4));
Node base5 = new Node(new Vec3(5, 5, 4.2));

Node node1 = new Node(new Vec3(5.2, 5, 5));
Node node2 = new Node(new Vec3(5.2, 5, 4.8));
Node node3 = new Node(new Vec3(5.2, 5, 4.6));
Node node4 = new Node(new Vec3(5.2, 5, 4.4));
Node node5 = new Node(new Vec3(5.2, 5, 4.2));

Node node6 = new Node(new Vec3(5.4, 5, 5));
Node node7 = new Node(new Vec3(5.4, 5, 4.8));
Node node8 = new Node(new Vec3(5.4, 5, 4.6));
Node node9 = new Node(new Vec3(5.4, 5, 4.4));
Node node10 = new Node(new Vec3(5.4, 5, 4.2));

Node node11 = new Node(new Vec3(5.6, 5, 5));
Node node12 = new Node(new Vec3(5.6, 5, 4.8));
Node node13 = new Node(new Vec3(5.6, 5, 4.6));
Node node14 = new Node(new Vec3(5.6, 5, 4.4));
Node node15 = new Node(new Vec3(5.6, 5, 4.2));

Node node16 = new Node(new Vec3(5.8, 5, 5));
Node node17 = new Node(new Vec3(5.8, 5, 4.8));
Node node18 = new Node(new Vec3(5.8, 5, 4.6));
Node node19 = new Node(new Vec3(5.8, 5, 4.4));
Node node20 = new Node(new Vec3(5.8, 5, 4.2));

Node[][] nodes = new Node[5][5];

//sphere
Sphere ball = new Sphere(new Vec3(4.8, 5.5, 4.6), 0.3);

PrintWriter output;
Camera camera;

PImage myTexture;

//Collision detection
boolean nodeSphereCollision(Node object1, Sphere object2) {
    if (object1.pos.distanceTo(object2.pos) <= object1.radius + object2.radius) {
        return true;
    }
    return false;
}

// Link length
float link_length = 0.2;

float total_length_error(){
  //down
  float sum = 0;
  for(int x = 0; x < 4; x++){
    for(int y = 0; y < 5; y++){
      sum += nodes[x+1][y].pos.distanceTo(nodes[x][y].pos);
     }
  }
  //across
  for(int x = 0; x < 5; x++){
    for(int y = 0; y < 4; y++){
      sum += nodes[x][y+1].pos.distanceTo(nodes[x][y].pos);
    }
  }
  return abs(sum - 10);
}

float total_energy(){
  float kinetic = 0;
  float potential = 0;
  for(int i = 0; i < 5; i++){
    for(int j = 0; j < 5; j++){
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
int relaxation_steps = 100;
int sub_steps = 1;


void setup() {
  //camera(width/2.0 - 50, height/2.0, (height/2.0) / tan(PI*30.0 / 180.0) - 50, width/2.0, height/2.0, 0, 0, 1, 0);
  camera = new Camera();
  nodes[0][0] = base1;
  nodes[0][1] = base2;
  nodes[0][2] = base3;
  nodes[0][3] = base4;
  nodes[0][4] = base5;

  nodes[1][0] = node1;
  nodes[1][1] = node2;
  nodes[1][2] = node3;
  nodes[1][3] = node4;
  nodes[1][4] = node5;

  nodes[2][0] = node6;
  nodes[2][1] = node7;
  nodes[2][2] = node8;
  nodes[2][3] = node9;
  nodes[2][4] = node10;

  nodes[3][0] = node11;
  nodes[3][1] = node12;
  nodes[3][2] = node13;
  nodes[3][3] = node14;
  nodes[3][4] = node15;

  nodes[4][0] = node16;
  nodes[4][1] = node17;
  nodes[4][2] = node18;
  nodes[4][3] = node19;
  nodes[4][4] = node20;

  
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
  // Semi-implicit Integration
  for(int i = 0; i < 5; i++){
    for(int j = 0; j < 5; j++){
      nodes[i][j].last_pos = nodes[i][j].pos;
      nodes[i][j].vel = nodes[i][j].vel.plus(gravity.times(dt));
      nodes[i][j].pos = nodes[i][j].pos.plus(nodes[i][j].vel.times(dt));
    }  
  }
  
  for(int i = 0; i < 5; i++){
    for(int j = 0; j < 5; j++){
      if(nodeSphereCollision(nodes[i][j], ball)){
        // Move balls out of collision
        //float overlap = 0.5f * (dist - balls[i].radius - balls[j].radius);
        //balls[i].pos.subtract(delta.normalized().times(overlap));
        //balls[j].pos.add(delta.normalized().times(overlap));
        Vec3 delta = nodes[i][j].pos.minus(ball.pos);
        float dist = delta.length();
        float overlap = (dist - nodes[i][j].radius - ball.radius);
        nodes[i][j].pos.subtract(delta.normalized().times(overlap));
      }
    }
  }
  
  // Constrain the distance between nodes to the link length
  for (int i = 0; i < relaxation_steps; i++) {
    //down
    for(int x = 0; x < 4; x++){
      for(int y = 0; y < 5; y++){
        Vec3 delta = nodes[x+1][y].pos.minus(nodes[x][y].pos);
        float delta_len = delta.length();
        float correction = (delta_len - link_length) *.1;
        Vec3 delta_normalized = delta.normalized();
        nodes[x+1][y].pos = nodes[x+1][y].pos.minus(delta_normalized.times(correction / 2));
        nodes[x][y].pos = nodes[x][y].pos.plus(delta_normalized.times(correction / 2));
      }
    }
    //across
    for(int x = 0; x < 5; x++){
      for(int y = 0; y < 4; y++){
        Vec3 delta = nodes[x][y+1].pos.minus(nodes[x][y].pos);
        float delta_len = delta.length();
        float correction = delta_len - link_length;
        Vec3 delta_normalized = delta.normalized();
        nodes[x][y+1].pos = nodes[x][y+1].pos.minus(delta_normalized.times(correction / 2));
        nodes[x][y].pos = nodes[x][y].pos.plus(delta_normalized.times(correction / 2));
      }
    }
    
    base1.pos = new Vec3(5, 5, 5); // Fix the base nodes in place
    base2.pos = new Vec3(5, 5, 4.8);
    base3.pos = new Vec3(5, 5, 4.6);
    base4.pos = new Vec3(5, 5, 4.4);
    base5.pos = new Vec3(5, 5, 4.2);
  }

  // Update the velocities (PBD)
  for(int i = 0; i < 5; i++){
    for(int j = 0; j < 5; j++){
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
  strokeWeight(0.02 * scene_scale);
  for(int i = 0; i < 5; i++){
    for(int j = 0; j < 5; j++){
      pushMatrix();
      translate(nodes[i][j].pos.x * scene_scale, nodes[i][j].pos.y * scene_scale, nodes[i][j].pos.z * scene_scale);
      sphere(nodes[i][j].radius * scene_scale);
      popMatrix(); 
    }
  }

  ambient(127);
  specular(255);
  shininess(200);
  
  //draw ball
  fill(255, 0, 0);
  pushMatrix();
  translate(ball.pos.x * scene_scale, ball.pos.y * scene_scale, ball.pos.z * scene_scale);
  sphere(ball.radius * scene_scale);
  popMatrix();
  
  //draw lines
  textureMode(NORMAL);
  beginShape(TRIANGLES);
  texture(myTexture);
  for (int x = 0; x < 4; x++) {
      for (int y = 0; y < 4; y++) {
          // Triangle 1
          vertex(nodes[x][y].pos.x * scene_scale, nodes[x][y].pos.y * scene_scale, nodes[x][y].pos.z * scene_scale, (float)x / 4, (float)y / 4);
          vertex(nodes[x + 1][y].pos.x * scene_scale, nodes[x + 1][y].pos.y * scene_scale, nodes[x + 1][y].pos.z * scene_scale, (float)(x + 1) / 4, (float)y / 4);
          vertex(nodes[x][y + 1].pos.x * scene_scale, nodes[x][y + 1].pos.y * scene_scale, nodes[x][y + 1].pos.z * scene_scale, (float)x / 4, (float)(y + 1) / 4);
  
          // Triangle 2
          vertex(nodes[x + 1][y].pos.x * scene_scale, nodes[x + 1][y].pos.y * scene_scale, nodes[x + 1][y].pos.z * scene_scale, (float)(x + 1) / 4, (float)y / 4);
          vertex(nodes[x + 1][y + 1].pos.x * scene_scale, nodes[x + 1][y + 1].pos.y * scene_scale, nodes[x + 1][y + 1].pos.z * scene_scale, (float)(x + 1) / 4, (float)(y + 1) / 4);
          vertex(nodes[x][y + 1].pos.x * scene_scale, nodes[x][y + 1].pos.y * scene_scale, nodes[x][y + 1].pos.z * scene_scale, (float)x / 4, (float)(y + 1) / 4);
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
  
  
  
