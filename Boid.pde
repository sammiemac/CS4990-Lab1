/// In this file, you will have to implement seek and waypoint-following
/// The relevant locations are marked with "TODO"

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
   // added distance, waypoints array, and index
   float dist;
   ArrayList<PVector> waypoints;
   int waypointsIndex;
   
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
        float angleTo = atan2(dy, dx);
        angleTo = normalize_angle_left_right(angleTo - kinematic.getHeading());
        //println("Angle to Target: " + angleTo);
        //println("Kinematic Heading: " + kinematic.getHeading());
        float angleNext;
        
        // Distance to target
        float updateDist = PVector.dist(target, kinematic.position);
        //println("Current Distance: " + updateDist); 
        
        // The following section covers takeoff from current location to target
        
        // begin movement if not on path or on last lastpoint
        if (waypoints == null || waypointsIndex == waypoints.size() - 2)
        {
          // resets waypoints when left-clicking or when on last waypoint in ArrayList
          waypoints = null;
          waypointsIndex = 0;
          
          // ensures boid is always turning toward target
          if (angleTo < 0)
            kinematic.increaseSpeed(0, 1000*dt);
          else if (angleTo > 0)
            kinematic.increaseSpeed(0, -1000*dt);
          else
            kinematic.increaseSpeed(0, 0);
            
          // speed up depending on distance  
          if (updateDist < 30 || kinematic.getSpeed() == 0)
            kinematic.increaseSpeed(5*dt, 0);
          else
            kinematic.increaseSpeed(20*dt, 0);
          
          if ((updateDist < dist*0.1 || updateDist < 40) && kinematic.speed > 10)
            kinematic.increaseSpeed(-1.5*kinematic.getSpeed()*dt, 0);
          
          // stop when boid reaches target  
          if (updateDist < 0.5)
          {
            println("Ya done now");
            kinematic.speed = 0;
            kinematic.rotational_velocity = 0;
            target = null;
            entering_path = false;
          }
        }
        // movement when on path
        else
        {
          angleNext = atan2(target.y - waypoints.get((waypointsIndex+1)%waypoints.size()).y,
                              target.x - waypoints.get((waypointsIndex+1)%waypoints.size()).x);
          angleNext = normalize_angle_left_right(angleNext - kinematic.getHeading());
          println("Angle next = " + angleNext);
          if (angleTo < 0)
            kinematic.increaseSpeed(0, 1000*dt);
          else if (angleTo > 0)
            kinematic.increaseSpeed(0, -1000*dt);
          else
            kinematic.increaseSpeed(0, 0);
            
          if (updateDist < 30 || kinematic.getSpeed() == 0)
            kinematic.increaseSpeed(5*dt, 0);
          else
             kinematic.increaseSpeed(20*dt, 0);
             
          if ((updateDist < dist*0.15 || updateDist < 40) || (abs(angleNext) < 0.5 && abs(angleNext) > (TAU-0.5)))
          {
            kinematic.increaseSpeed(-0.5*kinematic.getSpeed()*dt, 0);
          }
             
          if (updateDist < 4)
            target = waypoints.get(++waypointsIndex);
          if (waypointsIndex == waypoints.size() - 1)
            waypoints = null;
        }
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
   
   // added dist
   void follow(ArrayList<PVector> waypoints, float dist)
   {
      // TODO: change to follow *all* waypoints
      this.waypointsIndex = 0;
      this.target = waypoints.get(0);
      this.dist = dist;
      this.waypoints = waypoints; 
   }
}
