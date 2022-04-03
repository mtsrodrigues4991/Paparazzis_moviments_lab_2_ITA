// eight path parameters
float radius = 30;
PVector center1 = new PVector(450, 600);
PVector center2 = new PVector(300, 500);

// initial vehicle position
PVector position = new PVector(500,50);

// initialize the drone
Vehicle v;
// initialize the wind
FlowField wind;

void setup(){
  size(800,800);
  background(255);

  wind = new FlowField(10); // flowfield with resolution 10 - Perlin Noise
  v = new Vehicle(position, center1, center2, radius*2.5, wind);
}

void draw(){
  background(255);
  // draw the eight path on the screen
  //draw_path(50, center1, center2);
  
  v.state_machine();
  //v.eight_movement();
  v.update();
  v.display();
}
