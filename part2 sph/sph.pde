float aspect_ratio = 5/8;

float k_smooth_radius = 0.035;
float k_stiff = 150.0;
float k_rest_density = 0.2;
float grab_radius = 0.08;

public class Particle {
  Vec2 pos, oldPos, vel;
  float press, dens;
  boolean grabbed;
 
  public Particle(float x, float y){
    this.pos = new Vec2(x, y);
    this.oldPos = new Vec2(x, y);
    this.vel = new Vec2(0, 0);
    this.press = 0.0;
    this.dens = 0.0;
    this.grabbed = false;
  }
}

int r = 35;
int n = 40;
int numParticles = 0;
Particle[] particles = new Particle[1400];


void setup(){
  size(800,500);
  surface.setTitle("Double Pendulum");
  
  for(int i = 0; i < r; i++){
    for(int j = 0; j < n; j++){
      particles[i * n + j] = new Particle(-1+2*j/n+0.1*i/r,aspect_ratio*(0.5 - 1.5*i/r));
    }
  }  
}

public class Pair{
  Particle p1, p2;
  float q, q2, q3;
  
  public Pair(Particle p1, Particle p2, float q){
    this.p1 = p1;
    this.p2 = p2;
    this.q = q;
    this.q2 = q * q;
    this.q3 = q * q * q;
  }
}

 
void simulateSPH(Particle[] particles, int numParticles, float k_smooth_radius, float k_rest_density, float k_stiff, Vec2 mousePos, float grab_radius, float aspect_ratio, float dt){
  for(int i = 0; i < numParticles; i++){
    particles[i].vel = (particles[i].pos.minus(particles[i].oldPos)).times(1/dt);
    particles[i].vel = particles[i].vel.plus(new Vec2(0.0, -5.0).times(dt));
    
    //if(particles[i].pos.y < -aspect_ratio){
    //  particles[i].pos.y = -aspect_ratio;
    //  particles[i].vel.y *= -0.3;
    //}
    //if(particles[i].pos.x < -1.0){
    //  particles[i].pos.x = -1.0;
    //  particles[i].vel.x *= -0.3;
    //}
    //if(particles[i].pos.x > 1.0){
    //  particles[i].pos.x = 1.0;
    //  particles[i].vel.x *= -0.3;
    //}
    
    float front = 10 * dt;
    Vec2 mid = new Vec2(mousePos.x - particles[i].pos.x, mousePos.y - particles[i].pos.y).times(1/grab_radius);
    particles[i].vel = particles[i].vel.plus((mid.minus(particles[i].vel)).times(front));
  }
  
  //Find all neighboring particles [TODO: Slow for large SPH systems!]
  Pair[] pairs = new Pair[1400];
  float pair_length = 0;
  for(int i = 0; i < numParticles; i++){
    for(int j = 0; j < numParticles; j++){
      float dist = particles[i].pos.distanceTo(particles[j].pos);
      if(dist < k_smooth_radius && i < j){
        float q = 1 - (dist/k_smooth_radius);
        pairs[i] = new Pair(particles[i], particles[j], q);
        pair_length++;
      }
    } 
  }
  
 //Accumulate per-particle density 
  for(int i = 0; i < pair_length; i++){
    pairs[i].p1.dens += pairs[i].q2;
    pairs[i].p2.dens += pairs[i].q2;
  }
  
  
  //Computer per-particle pressure: stiffness*(density - rest_density)
  for(int i = 0; i < numParticles; i++){
    particles[i].press = k_stiff*(particles[i].dens - k_rest_density);
    if (particles[i].press > 20){
      particles[i].press = 20;  //maximum pressure
    }
  }
  
  //for pair in pairs
  for(int i = 0; i < pair_length; i++){
    Particle a = pairs[i].p1;
    Particle b = pairs[i].p2;
    float displace = (a.press + b.press) * pairs[i].q * (dt*dt);
    a.pos = a.pos.plus((a.pos.minus(b.pos)).normalized().times(displace));
    b.pos = b.pos.plus((b.pos.minus(a.pos)).normalized().times(displace));
  }
}


boolean paused = false;
void keyPressed() {
  if (key == ' ') {
    paused = !paused;
  }
}

void draw(){
  background(255);
  float dt = 1.0 / 20;
  Vec2 mousePos = new Vec2(mouseX, mouseY);
  
  //if mouse.clicked
  //for p in particles p.grabbed = true if mousePos.distanceTo(p.pos) < grab_radius
  //if mouse.released
  //p.grabbed = false for p in particles 
  
  if(mousePressed){
    for(int i = 0; i < numParticles; i++){
       if(mousePos.distanceTo(particles[i].pos) < grab_radius){
         particles[i].grabbed = true; 
       }
    }
  }
  if(!mousePressed){
    for(int i = 0; i < numParticles; i++){
      particles[i].grabbed = false; 
    }
  }
  
  
   //Substep simulation for increased numerical stability
    for(int i = 0; i < 10; i++){
      float sim_dt = dt/10;
      if(sim_dt > 0.003){
        sim_dt = 0.003;
      }
      simulateSPH(particles,numParticles,k_smooth_radius,k_rest_density,k_stiff,mousePos,grab_radius,aspect_ratio,sim_dt);
    }
   
    for(int i = 0; i < numParticles; i++){
      float q = (particles[i].press/20);
      fill(0.4-q*0.4,0.8-q*0.3,1.0-q*0.1);
      ellipse(particles[i].pos.x, particles[i].pos.y, 0.012, 0.012);
    }
}

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
