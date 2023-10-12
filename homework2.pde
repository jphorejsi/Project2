// Node struct
class Node {
  Vec3 pos;
  Vec3 vel;
  Vec3 last_pos;

  Node(Vec3 pos) {
    this.pos = pos;
    this.vel = new Vec3(0, 0, 0);
    this.last_pos = pos;
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

PrintWriter output;

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
  camera(width +1/2.0, height+1/2.0, (height/2.0) / tan(PI*30.0 / 180.0), width/2.0, height/2.0, 0, 0, 1, 0);
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
  
  // Constrain the distance between nodes to the link length
  for (int i = 0; i < relaxation_steps; i++) {
    //down
    for(int x = 0; x < 4; x++){
      for(int y = 0; y < 5; y++){
        Vec3 delta = nodes[x+1][y].pos.minus(nodes[x][y].pos);
        float delta_len = delta.length();
        float correction = delta_len - link_length;
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
  if (key == ' ') {
    paused = !paused;
    output.flush();
    output.close();
  }
}

float time = 0;
void draw() {
  //ortho(-width/2, width/2, -height/2, height/2);
  float dt = 1.0 / 20; //Dynamic dt: 1/frameRate;
  if (!paused && time <= 30) {
    println(time);
    //output.println(total_energy() + " " + time);
    output.println(total_length_error());
    for (int i = 0; i < sub_steps; i++) {
      time += dt / sub_steps;
      update_physics(dt / sub_steps);
    }
  }

  background(255);
  
  //draw nodes
  fill(0, 255, 0);
  noStroke();
  lights();
  strokeWeight(0.02 * scene_scale);
  for(int i = 0; i < 5; i++){
    for(int j = 0; j < 5; j++){
      pushMatrix();
      translate(nodes[i][j].pos.x * scene_scale, nodes[i][j].pos.y * scene_scale, nodes[i][j].pos.z * scene_scale);
      sphere(0.3 * scene_scale/4);
      popMatrix(); 
    }
  }
  
  //draw lines
  stroke(0);
  strokeWeight(3);
  //down
  for(int x = 0; x < 4; x++){
      for(int y = 0; y < 5; y++){
        line(nodes[x+1][y].pos.x * scene_scale, nodes[x+1][y].pos.y * scene_scale, nodes[x+1][y].pos.z * scene_scale, nodes[x][y].pos.x * scene_scale, nodes[x][y].pos.y * scene_scale, nodes[x][y].pos.z * scene_scale);
      }
    }
  //across
  for(int x = 0; x < 5; x++){
      for(int y = 0; y < 4; y++){
        line(nodes[x][y+1].pos.x * scene_scale, nodes[x][y+1].pos.y * scene_scale, nodes[x][y+1].pos.z * scene_scale, nodes[x][y].pos.x * scene_scale, nodes[x][y].pos.y * scene_scale, nodes[x][y].pos.z * scene_scale);
      }
    }
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
}
  
  
  
