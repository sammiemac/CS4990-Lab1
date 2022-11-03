// Useful to sort lists by a custom key
import java.util.Comparator;

// In this file you will implement your navmesh and pathfinding. 

// This node representation is just a suggestion
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
  ArrayList<Integer> indices = new ArrayList<Integer>();
  ArrayList<Wall> edge = new ArrayList<Wall>();
  
   void bake(Map map)
   {
       // generate the graph you need for pathfinding
       
       // clears reflex ArrayList
       reflex.clear();
       // clears indices ArrayList
       indices.clear();
       // clears edge ArrayList
       edge.clear();
       
       ArrayList<Wall> pts = map.outline;
       
       // checks if the angle at the node is reflex, if it is, add to the reflex ArrayList
       for (int i = 0; i < pts.size(); i++)
       {
         if (pts.get(i).normal.dot(pts.get((i+1)%pts.size()).direction) > 0)
         {
           PVector point = pts.get(i).end;
           reflex.add(point);
           indices.add(i + 1);
         }
       }
       
       //// if the edge from the reflex to the map.start node doesn't cross the map's walls, adds the edge to the ArrayList
       //for (int i = 0; i < pts.size(); i++)
       //{
       //  for (int j = 0; j < reflex.size(); j++)
       //  {
       //      // shrinkin temp wall so it doesn't get deteced by the collide function
       //      Wall temp = new Wall(reflex.get(j), pts.get(i).start);
       //      PVector.add(temp.start, PVector.mult(temp.direction, 0.01));
       //      PVector.add(temp.end, PVector.mult(temp.direction, -0.01));
       //      if (!map.collides(temp.start, temp.end)) // no edges get made?
       //        edge.add(i, new Wall(reflex.get(j), pts.get(i).start));
       //  }
       //}
       
       // if all conditions are met, add edge from reflex point to outline vertices
       for (int i = 0; i < reflex.size(); i++)
       {
         for (int j = 0; j < pts.size(); j++)
         {
           // disregards neighbors of reflex
           if (j == indices.get(i) + 1 || j == indices.get(i) - 1)
             continue;
           else
           {
             Wall temp = new Wall(reflex.get(i), pts.get(j).start);
             temp.start = PVector.add(temp.start, PVector.mult(temp.direction, 0.01));
             temp.end = PVector.add(temp.end, PVector.mult(temp.direction, -0.01));
             // checks if temp will collide with wall and is within map
             if (!map.collides(temp.start, temp.end) && isPointInPolygon(temp.center(), pts))
             {
               // checks if edge will collide with other edges, removes unnecessary edges
               boolean intersectsNavMesh = false;
               for (int k = 0; k < edge.size(); k++)
               {
                 if (temp.crosses(edge.get(k).start, edge.get(k).end))
                 {
                   intersectsNavMesh = true;
                   break;
                 }
               }
               if (!intersectsNavMesh)
                 edge.add(i, temp);
             }
           }
         }
       }
       
       //// checks if the edges crosses the map's wall, if it does, remove the edge
       //for (int i = 0; i < pts.size(); i++)
       //{
       //  for (int j = 0; j < edge.size(); j++)
       //  {
       //    Wall temp = edge.get(j);
       //    temp.start = PVector.add(temp.start, PVector.mult(temp.direction, 0.01));
       //    temp.end = PVector.add(temp.end, PVector.mult(temp.direction, -0.01));
         
       //    if (temp.crosses(pts.get(i).start, pts.get(i).end))
       //    {
       //      edge.remove(j);
       //      println("removed edge");
       //      println("new size: " + edge.size());
       //      //i = 0;
       //    }
       //  }
       //}
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
       stroke(0, 255 , 150);
       fill(0, 255, 100);
       circle(w.x, w.y, 10);
     }
     for (Wall w : edge) {
      w.draw();
     }
   }
}
