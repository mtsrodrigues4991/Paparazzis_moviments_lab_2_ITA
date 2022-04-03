class Vehicle {
  float r = 6; // dimension to draw the drone
  float theta; // angle of rotation in the circle
  float angPos; // angular position
  float angVel; // look in front of the actual position
  float maxspeed;
  float maxforce;
  PVector acceleration;
  PVector velocity;
  PVector location;
  boolean first_approach;
  boolean change_color = false; // auxiliar to help visualization
  
  // center of circles 1 and 2, and their radius
  PVector center_1;
  PVector center_2;
  float radius;
  
  PVector middle_point;
  PVector auxiliar_point1_circle2 = new PVector();
  PVector auxiliar_point2_circle2 = new PVector();
  PVector auxiliar_point1_circle1 = new PVector();
  PVector auxiliar_point2_circle1 = new PVector();
  
  // auxiliary variables for the eight movement
  boolean part1 = false;
  boolean part2 = false;
  boolean part3 = false;
  boolean part4 = false;  
  
  // is there to be wind
  boolean is_wind = true;
  float wind_force = 0.5;
  FlowField wind;
  
  // variable for way point, in what circle to start
  boolean c1 = true;
  
  // mininum number of interations to the state machine to change states
  // condition for change
  boolean complete = false;
  int counter = 0;
  int state = 0;
  
  Vehicle(PVector position, PVector center1, PVector center2, float rad, FlowField w){
    
    // copy the wind
    wind = w;
    
    // trajectory variables and geometry variables
    center_1 = center1.get();
    center_2 = center2.get();
    radius = rad;
    
    middle_point = new PVector((center_1.x+center_2.x)/2, (center_1.y+center_2.y)/2);
  
    // line from circle 1 to to 2
    PVector from1to2 = new PVector();
   
    // find the two auxiliary points on circle 2, perpendicular to the line linking both circles
    from1to2 = PVector.sub(center2, center1);
    
    auxiliar_point1_circle2 = from1to2.get();
    auxiliar_point1_circle2.normalize();
    auxiliar_point1_circle2.rotate(PI/2).mult(radius);
    auxiliar_point1_circle2.add(center2);
    
    auxiliar_point2_circle2 = from1to2.get();
    auxiliar_point2_circle2.normalize();
    auxiliar_point2_circle2.rotate(-PI/2).mult(radius);
    auxiliar_point2_circle2.add(center2);  
    
    // line from circle 2 to to 1
    PVector from2to1 = new PVector();
  
    // find the two auxiliary points on circle 1, perpendicular to the line linking both circles
    from2to1 = PVector.sub(center1, center2);
    
    auxiliar_point1_circle1 = from2to1.get();
    auxiliar_point1_circle1.normalize();
    auxiliar_point1_circle1.rotate(PI/2).mult(radius);
    auxiliar_point1_circle1.add(center1);
    
    auxiliar_point2_circle1 = from2to1.get();
    auxiliar_point2_circle1.normalize();
    auxiliar_point2_circle1.rotate(-PI/2).mult(radius);
    auxiliar_point2_circle1.add(center1);      
      
    // drone variables
    acceleration = new PVector(0,0);
    velocity = new PVector(0,0);
    location = position.get(); // make a copy of the position vectors
    
    // has the drone made the first approach to start the movement?
    first_approach = false;
    
    angPos = 0;
    angVel = 0.8;
    maxspeed = 3.7;
    maxforce = 0.5;
    
    angVel = PI/18; // 10 degrees
  }
  
  void state_machine(){
    // if it is the first time it is being called, start with the eight movement
    if (complete == false){
      if (state == 0){
        eight_movement();
      } else if (state == 1){
        seek_around(center_1, radius);
      } else if (state == 2){
        arrive();
      } else if (state == 3){
        oval_movement();
      }
    }
    
    if (complete){
      complete = false;
      float rand = random(100);

      if (rand < 25){
        eight_movement();
        state=0;
      } else if (rand < 50) {
        seek_around(center_1, radius);
        state=1;
      } else if(rand < 75) {
        arrive();
        state=2;
      } else {
        oval_movement();
        state=3;
      }
    }
    
    // increment counter 
    counter = counter + 1;

    if (counter > 500){
      complete = true; // change state
      counter = 0; // reset counter
    }
  }
  
  void eight_movement(){
    
    if (first_approach == false){
      approach(); 
    } else {
      eight(); 
    }
  }
    
  void eight(){
    // sequence of points
    // P1.circle2 -> P2.circle2 -> P2.circle1 -> P1.circle1
    // part1 -> part2 -> part3 -> part4
    PVector desired = new PVector();
    // reduce max speed
    maxspeed = 2.5;
     
    if (part1){
     // go to point 1 of circle 2
     desired = PVector.sub(auxiliar_point2_circle2, location);

     desired.normalize(null);
     desired.mult(maxspeed);

     PVector steer = PVector.sub(desired, velocity); // formula for steering velocity
     steer.limit(maxforce);
     applyForce(steer);
     
     if (desired.mag() < 60) {
       change_color = false;
       // go to part 2 of the trajectory
       part1 = false;
       part2 = true;
     } 
     }
     // go around circle 2 until reaching point 1
     if (part2){
         
       PVector centerToPerimeter = new PVector();       
       
       PVector posToCenter = PVector.sub(center_2, location);
       centerToPerimeter = posToCenter.normalize(null).mult(-radius); //Set to null to create a new vector
       
       // calculate moving target
       theta = atan2(centerToPerimeter.y, centerToPerimeter.x);
       theta += angVel;

       PVector target = new PVector();
       target.x = radius*cos(theta);
       target.y = radius*sin(theta);
       target.add(center_2);
       desired = PVector.sub(target, location);   
              
       desired.normalize(null);
       desired.mult(maxspeed);
  
       PVector steer = PVector.sub(desired, velocity); // formula for steering velocity
       steer.limit(maxforce);
       applyForce(steer);
       
       // if it arrived at the destiny, go to next stage of movement)
       if (dist(location.x, location.y, auxiliar_point1_circle2.x, auxiliar_point1_circle2.y) < 20){
         change_color = true;
         // go to part 2 of the trajectory
         part2 = false;
         part3 = true;       
       }
           
       fill(0,255,0);
       ellipse(center_2.x + centerToPerimeter.x, center_2.y+centerToPerimeter.y, 6, 6);

       fill(255,0,0);
       ellipse(target.x, target.y, 6, 6);
     } 
     
    // go to point 1 of of circle 1
    if (part3){
     // go to point 1 of circle 1
     desired = PVector.sub(auxiliar_point1_circle1, location);

     desired.normalize(null);
     desired.mult(maxspeed);

     PVector steer = PVector.sub(desired, velocity); // formula for steering velocity
     steer.limit(maxforce);
     applyForce(steer);
     
     if (desired.mag() < 60) {
       change_color = false;
       // go to part 2 of the trajectory
       part3 = false;
       part4 = true;
     } 
     }
    
    // go around circle 1 until reaching point 2
     if (part4){
         
       PVector centerToPerimeter = new PVector();       
       
       PVector posToCenter = PVector.sub(center_1, location);
       centerToPerimeter = posToCenter.normalize(null).mult(-radius); //Set to null to create a new vector
       
       // calculate moving target
       theta = atan2(centerToPerimeter.y, centerToPerimeter.x);
       theta -= angVel;

       PVector target = new PVector();
       target.x = radius*cos(theta);
       target.y = radius*sin(theta);
       target.add(center_1);
       desired = PVector.sub(target, location);   
              
       desired.normalize(null);
       desired.mult(maxspeed);
  
       PVector steer = PVector.sub(desired, velocity); // formula for steering velocity
       steer.limit(maxforce);
       applyForce(steer);
       
       // if it arrived at the destiny, go to next stage of movement)
       if (dist(location.x, location.y, auxiliar_point2_circle1.x, auxiliar_point2_circle1.y) < 20){
         change_color = true;
         // go back to part 1
         part4 = false;
         part1 = true;       
       }
       
       fill(0,255,0);
       ellipse(center_1.x + centerToPerimeter.x, center_1.y+centerToPerimeter.y, 6, 6);

       fill(255,0,0);
       ellipse(target.x, target.y, 6, 6);
     } 
     
  }
  
  void oval_movement(){
    // sequence of points
    // P1.circle2 -> P2.circle2 -> P2.circle1 -> P1.circle1
    // part1 -> part2 -> part3 -> part4
    PVector desired = new PVector();
    // reduce max speed
    maxspeed = 2.5;
     
    if (part1){
     // go to point 1 of circle 2
     desired = PVector.sub(auxiliar_point2_circle2, location);  //quando começa e procura o ponto abaixo do circ esquerdo

     desired.normalize(null);
     desired.mult(maxspeed);

     PVector steer = PVector.sub(desired, velocity); // formula for steering velocity
     steer.limit(maxforce);
     applyForce(steer);
     
     if (desired.mag() < 20) {
       change_color = false;
       // go to part 2 of the trajectory
       part1 = false;
       part2 = true;
     } 
     }
     // go around circle 2 until reaching point 1
     if (part2){
         
       PVector centerToPerimeter = new PVector();       
       
       PVector posToCenter = PVector.sub(center_2, location);
       centerToPerimeter = posToCenter.normalize(null).mult(-radius); //Set to null to create a new vector
       
       // calculate moving target
       theta = atan2(centerToPerimeter.y, centerToPerimeter.x);
       theta += angVel;

       PVector target = new PVector();
       target.x = radius*cos(theta);
       target.y = radius*sin(theta);
       target.add(center_2);      
       desired = PVector.sub(target, location);   
              
       desired.normalize(null);
       desired.mult(maxspeed);
  
       PVector steer = PVector.sub(desired, velocity); // formula for steering velocity
       steer.limit(maxforce);
       applyForce(steer);
       
       // if it arrived at the destiny, go to next stage of movement)
       if (dist(location.x, location.y, auxiliar_point1_circle2.x, auxiliar_point1_circle2.y) < 20){
         change_color = true;
         // go to part 2 of the trajectory
         part2 = false;
         part3 = true;       
       }
           
       fill(0,255,0);
       ellipse(center_2.x + centerToPerimeter.x, center_2.y+centerToPerimeter.y, 6, 6);

       fill(255,0,0);
       ellipse(target.x, target.y, 6, 6);
     } 
     
    // go to point 2 of of circle 1
    if (part3){
     // go to point 2 of circle 1
     desired = PVector.sub(auxiliar_point2_circle1, location);  

     desired.normalize(null);
     desired.mult(maxspeed);

     PVector steer = PVector.sub(desired, velocity); // formula for steering velocity
     steer.limit(maxforce);
     applyForce(steer);
     
     if (desired.mag() < 20) {
       change_color = false;
       // go to part 2 of the trajectory
       part3 = false;
       part4 = true;
     } 
     }
      // go around circle 1 until reaching point 1
       if (part4){
         
       PVector centerToPerimeter = new PVector();       
       
       PVector posToCenter = PVector.sub(center_1, location);
       centerToPerimeter = posToCenter.normalize(null).mult(-radius); //Set to null to create a new vector
       
       // calculate moving target
       theta = atan2(centerToPerimeter.y, centerToPerimeter.x);
       theta += angVel;

       PVector target = new PVector();
       target.x = radius*cos(theta);
       target.y = radius*sin(theta);
       target.add(center_1);      
       desired = PVector.sub(target, location);   
              
       desired.normalize(null);
       desired.mult(maxspeed);
  
       PVector steer = PVector.sub(desired, velocity); // formula for steering velocity
       steer.limit(maxforce);
       applyForce(steer);
       
       // if it arrived at the destiny, go to next stage of movement)
       if (dist(location.x, location.y, auxiliar_point1_circle1.x, auxiliar_point1_circle1.y) < 20){
         change_color = true;
         // go to part 2 of the trajectory
         part4 = false;
         part1 = true; 
       }
       
       fill(0,255,0);
       ellipse(center_1.x + centerToPerimeter.x, center_1.y+centerToPerimeter.y, 6, 6);

       fill(255,0,0);
       ellipse(target.x, target.y, 6, 6);
     } 
     
     
  }
  
  // in the beginning of the movement, approach the point between the circles
  void approach(){
     PVector desired = new PVector();
     
     // move the drone to the middle position between the two circles
     desired = PVector.sub(middle_point, location);

     desired.normalize(null);
     desired.mult(maxspeed);
     PVector steer = PVector.sub(desired, velocity); // formula for steering velocity

     // if we are far, go max speed
     if (desired.mag() > 10){
       steer.limit(maxforce);
     } else { // if we are close, go stop
       steer = PVector.sub(new PVector(0,0), velocity);
       steer.limit(maxforce); 
     }
     applyForce(steer);
  
      // if very close, say approach is done
      if (desired.mag() < 80) {
        first_approach = true;
        change_color = true;
        // start part 1 of the eight trajectory
        part1 = true;
      }
  }

  void seek_around(PVector center, float radius){
    
    PVector desired = new PVector();
    PVector posToPerimeter = new PVector();
    PVector centerToPerimeter = new PVector();
    PVector posToCircle = new PVector();
    
    PVector posToCenter = PVector.sub(center, location);
    
    float dis_pos_center = PVector.dist(center, location);
    // if it is close to the center
    if (dis_pos_center < radius) {

      centerToPerimeter = posToCenter.normalize(null).mult(-radius); //Set to null to create a new vector
      posToPerimeter = PVector.add(posToCenter, centerToPerimeter);
    }
    else if (dis_pos_center > radius) {

      centerToPerimeter = posToCenter.normalize(null).mult(-radius);      
      
      posToPerimeter = PVector.sub(center, location);
    }
    // if far from the perimeter
    
    posToCircle = PVector.add(posToCenter, centerToPerimeter);
    
    if (posToCircle.mag() > 100){
      // go full speed
      desired = posToPerimeter.get();
      change_color = false;
    } else {
      // if close to the perimeter, calculate moving target
      float theta = atan2(centerToPerimeter.y, centerToPerimeter.x);
      theta += angVel;
    
      PVector target = new PVector();
      target.x = radius*cos(theta);
      target.y = radius*sin(theta);
      target.add(center);
      desired = PVector.sub(target, location);
      
      stroke(255,0,0);
      line(center.x, center.y, target.x, target.y);
      
      fill(255,0,0);
      ellipse(target.x, target.y, 6,6);
      
      change_color = true;
    }
    
    desired.normalize();
    desired.mult(maxspeed);
    PVector steer = PVector.sub(desired, velocity); // formula for steering velocity
    steer.limit(maxforce);
    applyForce(steer);
    
    // plot some helpful lines
    stroke(0,255,0);
    line(center.x, center.y, center.x+centerToPerimeter.x, center.y+centerToPerimeter.y);
    
    fill(0,255,0);
    ellipse(center.x + centerToPerimeter.x, center.y+centerToPerimeter.y, 6, 6);
      
  }

  // implement a desaceleration as the vehicle approaches the target
  // there is a circunference around the target
  // if the vehicle is outside that circunference, it should go full speed
  // if the vehicle is inside that circunference, the distance should be
  // between 0 and maximum speed, mapped according to the distance
  // Arrive steering behavior
  void arrive(){
    // go to the center of c1 or c2
    PVector desired = new PVector(0,0);
    
    if (c1){
      desired = PVector.sub(center_1, location);
    } else {
      desired = PVector.sub(center_2, location);
    }
    
    float d = desired.mag(); // the distance is the magnitude of the vector pointing to the target
    
    desired.normalize();
    
    // if we are closer than 100 pixels, we should go slower
    if (d<50){
      float m = map(d, 0, 100, 0, maxspeed); // map the speed from 0 to maxspeed
      desired.mult(m); // multiply the normalized vector by the magnitude of the speed
      // if we arrived, we should go back
      if (d < 10){
        if (c1) {
          c1 = false;
        } else {
          c1 = true;
        }
      }
    } else {
      desired.mult(maxspeed);
    }
    
    PVector steer = PVector.sub(desired, velocity); 
    steer.limit(maxforce);
    applyForce(steer);
  }

  void applyForce(PVector force) {
    acceleration.add(force); // Newton's second law, with force accumulation and mass = 1
  }  
  
  void update() {
    velocity.add(acceleration);
    velocity.limit(maxspeed);
    location.add(velocity);

    // angle of the drone
    theta = velocity.heading() + PI/2; //Calculates the angle of rotation for a vector (2D vectors only)

    // if there is wind, add it to the fose
    if (is_wind){
      acceleration.add(PVector.mult(wind.lookup(location), wind_force));
      velocity.add(acceleration);
      location.add(velocity);
    }
    
    acceleration.mult(0); // we clear the acceleration for each frame
    // otherwise it would accumulate the forces being applied
  }  

  void display(){
    draw_path();
   
    if (change_color){
      fill(255,0,0);
    }else{
    fill(175);
    }
    
    stroke(0);
    pushMatrix();
    translate(location.x, location.y);
    rotate(theta);
    beginShape();
    vertex(0, -r*2);
    vertex(-r, r*2);
    vertex(r, r*2);
    endShape(CLOSE);
    popMatrix();
    

  }
  
  void draw_path(){
    // draw the two circles that form the eight
    stroke(0);
    fill(255);
    ellipse(center_1.x, center_1.y, 2*radius, 2*radius);
    ellipse(center_2.x, center_2.y, 2*radius, 2*radius);
    
    // draw center of both circles
    stroke(0);
    fill(0,0,255);
    ellipse(center_1.x, center_1.y, 6, 6);
    ellipse(center_2.x, center_2.y, 6, 6);
    //line(center_1.x, center_1.y, center_2.x, center_2.y);
    
    // draw important points for the vehicle trajectory
    stroke(255,0,0);
    fill(255,0,0);
    // point between the two circles
    ellipse((center_1.x+center_2.x)/2, (center_1.y+center_2.y)/2, 6, 6);
    
    stroke(255,0,0);
    fill(255,0,0);
    ellipse(auxiliar_point1_circle2.x, auxiliar_point1_circle2.y, 6, 6);
    stroke(0,0,255);
    fill(0,0,255);
    ellipse(auxiliar_point2_circle2.x, auxiliar_point2_circle2.y, 6, 6);
  
    stroke(255,0,0);
    fill(255,0,0);
    ellipse(auxiliar_point1_circle1.x, auxiliar_point1_circle1.y, 6, 6);
    stroke(0,0,255);
    fill(0,0,255);
    ellipse(auxiliar_point2_circle1.x, auxiliar_point2_circle1.y, 6, 6);
  
  }  
  
  
  
  
}
