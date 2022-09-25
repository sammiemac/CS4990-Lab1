/// In this file, you will have to implement seek and waypoint-following
/// The relevant locations are marked with "TODO"

// Okay, this is another test because I messed up the last time.

class Crumb
{
  PVector position;
  Crumb(PVector position)
  {
     this.position = position;
  }
  void draw()
  {
     fill(255);
     noStroke(); 
     circle(this.position.x, this.position.y, CRUMB_SIZE);
  }
}

class Boid
{
   Crumb[] crumbs = {};
   int last_crumb;
   float acceleration;
   float rotational_acceleration;
   KinematicMovement kinematic;
   PVector target;
   // distance
   float dist;
   
   Boid(PVector position, float heading, float max_speed, float max_rotational_speed, float acceleration, float rotational_acceleration)
   {
     this.kinematic = new KinematicMovement(position, heading, max_speed, max_rotational_speed);
     this.last_crumb = millis();
     this.acceleration = acceleration;
     this.rotational_acceleration = rotational_acceleration;
   }


   void update(float dt)
   {
     if (target != null)
     {  
        // TODO: Implement seek here
        
        // Visual debug, draws a line between the boid and the target destination
        line(target.x, target.y, kinematic.position.x, kinematic.position.y);
        noFill();
        circle(target.x, target.y, 25);
        
        // Length of adjacent side
        float dx = kinematic.position.x - target.x;

        // Length of opposite side
        float dy = kinematic.position.y - target.y;
        
        // Angle to target
        float angleTo = degrees(atan2(dy, dx));
        angleTo = normalize_angle(angleTo - kinematic.heading);
        println("Angle to Target: " + angleTo);
        println("Kinematic Heading: " + kinematic.getHeading());
        
        // Hopefully this turns the boid lmao
        if (angleTo > kinematic.getHeading()) {
          kinematic.increaseSpeed(3*dt, 100000*dt);
        } else if (angleTo < kinematic.getHeading()) {
          kinematic.increaseSpeed(3*dt, -100000*dt);
        }
        
        // Hopefully this stops the boid lmao
        float updateDist = PVector.dist(target, kinematic.position);
        println("Current Distance: " + updateDist);
        if (updateDist < (dist * 0.25) && kinematic.speed > 0) {
          kinematic.increaseSpeed(-15*dt, 0);
        }
        
        
        
        //kinematic.increaseSpeed(3*dt,100000*dt);
     }
     
     // place crumbs, do not change     
     if (LEAVE_CRUMBS && (millis() - this.last_crumb > CRUMB_INTERVAL))
     {
        this.last_crumb = millis();
        this.crumbs = (Crumb[])append(this.crumbs, new Crumb(this.kinematic.position));
        if (this.crumbs.length > MAX_CRUMBS)
           this.crumbs = (Crumb[])subset(this.crumbs, 1);
     }
     
     // do not change
     this.kinematic.update(dt);
     
     draw();
   }
   
   void draw()
   {
     for (Crumb c : this.crumbs)
     {
       c.draw();
     }
     
     fill(255);
     noStroke(); 
     float x = kinematic.position.x;
     float y = kinematic.position.y;
     float r = kinematic.heading;
     circle(x, y, BOID_SIZE);
     // front
     float xp = x + BOID_SIZE*cos(r);
     float yp = y + BOID_SIZE*sin(r);
     
     // left
     float x1p = x - (BOID_SIZE/2)*sin(r);
     float y1p = y + (BOID_SIZE/2)*cos(r);
     
     // right
     float x2p = x + (BOID_SIZE/2)*sin(r);
     float y2p = y - (BOID_SIZE/2)*cos(r);
     triangle(xp, yp, x1p, y1p, x2p, y2p);
   } 
   
   // added dist
   void seek(PVector target, float dist)
   {
      this.target = target;
      this.dist = dist;
      
   }
   
   void follow(ArrayList<PVector> waypoints)
   {
      // TODO: change to follow *all* waypoints
      this.target = waypoints.get(0);
      
   }
}
