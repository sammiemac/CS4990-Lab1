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
   
   Node(int id, ArrayList<Wall> polygon, ArrayList<Node> neighbors)
   {
     this.id = id;
     this.polygon = polygon;
     this.neighbors = neighbors;
     float xSum = 0;
     float ySum = 0;
     for (Wall w : polygon)
     {
       xSum += w.start.x + w.end.x;
       ySum += w.start.y + w.end.y;
     }
     this.center = new PVector(xSum / polygon.size(), ySum / polygon.size());
     for (int i = 0; i < neighbors.size(); i++)
     {
       Wall connect = new Wall(center, neighbors.get(i).center);
       connections.add(i, connect);
     }
   }
}

class Polygon
{
  int id;
  ArrayList<Wall> walls;
  ArrayList<PVector> points;
}

class NavMesh
{   
  
  ArrayList<PVector> reflex = new ArrayList<PVector>(); // holds reflex vertices
  ArrayList<Integer> indices = new ArrayList<Integer>(); // holds indices of map's vertices that are reflex
  ArrayList<Wall> edges = new ArrayList<Wall>(); // holds new edges of nav mesh
  ArrayList<Wall> perimeter = map.outline; // holds map outline
  ArrayList<Wall> allEdges = new ArrayList<Wall>(); // holds all the edges of the map (perimeter + edges)
  ArrayList<PVector> allPoints = new ArrayList<PVector>(); // holds all the points of the map
  
   void bake(Map map)
   {
       // generate the graph you need for pathfinding
       
       // resets nav mesh whenever new map is generated
       reflex.clear();
       indices.clear();
       edges.clear();
       allEdges.clear();
       allPoints.clear();
       
       // checks if the angle at the node is reflex, if it is, add to the reflex ArrayList
       for (int i = 0; i < perimeter.size(); i++)
       {
         if (perimeter.get(i).normal.dot(perimeter.get((i+1)%perimeter.size()).direction) > 0)
         {
           PVector point = perimeter.get(i).end;
           reflex.add(point);
           indices.add(i + 1);
         }
       }
       
       // if all conditions are met, add edge from reflex point to outline vertices
       for (int i = 0; i < reflex.size(); i++)
       {
         for (int j = 0; j < perimeter.size(); j++)
         {
           // disregards neighbors of reflex
           if (j == indices.get(i) + 1 || j == indices.get(i) - 1)
             continue;
           else
           {
             Wall temp = new Wall(reflex.get(i), perimeter.get(j).start);
             temp.start = PVector.add(temp.start, PVector.mult(temp.direction, 0.01));
             temp.end = PVector.add(temp.end, PVector.mult(temp.direction, -0.01));
             // checks if temp will collide with wall and is within map
             if (!map.collides(temp.start, temp.end) && isPointInPolygon(temp.center(), perimeter))
             {
               // checks if edge will collide with other edges, removes unnecessary edges
               boolean intersectsNavMesh = false;
               for (int k = 0; k < edges.size(); k++)
               {
                 if (temp.crosses(edges.get(k).start, edges.get(k).end))
                 {
                   intersectsNavMesh = true;
                   break;
                 }
               }
               if (!intersectsNavMesh)
                 edges.add(i, temp);
             }
           }
         }
       }
       
       // adds the walls of the perimeter and the navmesh to allWalls
       allEdges.addAll(perimeter);
       allEdges.addAll(edges);
       
       // adds all the start points of each wall in allEdges to allPoints
       for (int i = 0 ; i < allEdges.size(); i++)
       {
         allPoints.add(i, allEdges.get(i).start);
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
     
     for (Wall w : edges)
     {
       stroke(150, 0, 255);
       w.draw();
     }
     
     for (PVector p : allPoints)
     {
       stroke(255, 0 , 150);
       fill(255, 0, 100);
       circle(p.x, p.y, 10);
     }
     
     for (PVector w : reflex)
     {
       stroke(0, 255 , 150);
       fill(0, 255, 100);
       circle(w.x, w.y, 10);
     }
     
   }
}
