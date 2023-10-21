ArrayList<Particle> particles = new ArrayList<Particle>();
float h = 16.0;  // interaction radius
float mass = 1.0;
float rho0 = 1.0;  // rest density
float k = 100.0;  // stiffness
float mu = 50.0;  // viscosity coefficient
float dt = 0.02;  // time step

void setup() {
  size(800, 600);
  background(255);
  
  for (int i = 0; i < 200; i++) {
    particles.add(new Particle(random(100, 300), random(100, 300)));
  }
}

void draw() {
  background(255);

  for (Particle p : particles) {
    p.computeDensityPressure(particles);
  }

  for (Particle p : particles) {
    p.computeForces(particles);
    p.integrate();
    p.display();
  }
}

class Particle {
  PVector pos, vel, acc;
  float rho;  // density
  float pressure;

  Particle(float x, float y) {
    pos = new PVector(x, y);
    vel = new PVector();
    acc = new PVector();
  }

  void computeDensityPressure(ArrayList<Particle> particles) {
    rho = 0;
    for (Particle pj : particles) {
      float r = PVector.dist(pos, pj.pos);
      if (r < h) {
        rho += mass * poly6Kernel(r, h);
      }
    }
    pressure = k * (rho - rho0);
  }

  void computeForces(ArrayList<Particle> particles) {
    PVector f_pressure = new PVector();
    PVector f_viscosity = new PVector();

    for (Particle pj : particles) {
      if (pj != this) {
        PVector rij = PVector.sub(pos, pj.pos);
        float r = rij.mag();

        if (r < h) {
          PVector D = PVector.mult(rij, -1 * mass * (pressure + pj.pressure) / (2 * pj.rho) * spikyGradient(r, h));
          f_pressure.add(D);

          PVector U = PVector.sub(pj.vel, vel);
          U.mult(mass * mu / pj.rho * viscousLaplacian(r, h));
          f_viscosity.add(U);
        }
      }
    }

    PVector f_gravity = new PVector(0, 9.8 * rho);

    acc.set(0, 0);
    acc.add(PVector.div(f_pressure, rho));
    acc.add(PVector.div(f_viscosity, rho));
    acc.add(f_gravity);
  }

  void integrate() {
    vel.add(PVector.mult(acc, dt));
    pos.add(PVector.mult(vel, dt));
  }

  void display() {
    fill(50, 150, 255, 150);
    noStroke();
    ellipse(pos.x, pos.y, h/2, h/2);
  }
}

// Kernel functions
float poly6Kernel(float r, float h) {
  float hr = h - r;
  return 315.0 / (64.0 * PI * pow(h, 9)) * pow(hr, 3) * pow(hr, 3);
}

float spikyGradient(float r, float h) {
  float hr = h - r;
  return -45.0 / (PI * pow(h, 6)) * hr * hr;
}

float viscousLaplacian(float r, float h) {
  return 45.0 / (PI * pow(h, 6)) * (h - r);
}
