// Useful to sort lists by a custom key
import java.util.Comparator;


/// In this file you will implement your navmesh and pathfinding. 

/// This node representation is just a suggestion
class Node
{
   int id;
   ArrayList<Wall> polygon;
   PVector center;
   ArrayList<Node> neighbors;
   ArrayList<Wall> connections;
}



class NavMesh
{   
  
  ArrayList<PVector> reflex = new ArrayList<PVector>();
  ArrayList<Wall> edge = new ArrayList<Wall>();
  
   void bake(Map map)
   {
       /// generate the graph you need for pathfinding
       reflex.clear();
       edge.clear();
       ArrayList<Wall> pts = map.walls;
       for (int i = 0; i < pts.size(); i++)
       {
         if (pts.get(i).normal.dot(pts.get((i+1)%pts.size()).direction) > 0)
         {
           PVector point = pts.get(i).end;
           reflex.add(point);
         }
       }
       
       for (int i = 0; i < reflex.size(); i++)
       {
         for (int j = 0; j < pts.size(); j++)
         {
           if (!map.collides(reflex.get(i), pts.get(j).start))
           {
             println("I added a wall!");
             edge.add(new Wall(reflex.get(i), pts.get(j).end));
           }
         }
       }
   }
   
   ArrayList<PVector> findPath(PVector start, PVector destination)
   {
      /// implement A* to find a path
      ArrayList<PVector> result = null;
      return result;
   }
   
   
   void update(float dt)
   {
      draw();
   }
   
   void draw()
   {
     for (PVector w : reflex) {
       stroke(255, 0 ,0);
       fill(255, 0, 0);
       circle(w.x, w.y, 10);
     }
     for (Wall w : edge) {
      w.draw();
     }
   }
}
