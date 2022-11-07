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
   
   Node(int id, ArrayList<Wall> polygon)
   {
     this.id = id;
     this.polygon = polygon;
     //this.neighbors = neighbors;
     float xSum = 0;
     float ySum = 0;
     for (Wall w : polygon)
     {
       xSum += w.start.x;
       ySum += w.start.y;
     }
     this.center = new PVector(xSum / polygon.size(), ySum / polygon.size());
     //for (int i = 0; i < neighbors.size(); i++)
     //{
     //  Wall connect = new Wall(center, neighbors.get(i).center);
     //  connections.add(i, connect);
     //}
   }
}

class Reflex
{
  int id; // id of reflex point
  PVector pt; // location of reflex
  ArrayList<Wall> connections = new ArrayList<Wall>();
  
  Reflex(int id, PVector pt, ArrayList<Wall> connections)
  {
    this.id = id;
    this.pt = pt;
    this.connections = connections;
  }
}

class NavMesh
{   
  
  ArrayList<Reflex> reflex = new ArrayList<Reflex>(); // holds reflex vertices
  //ArrayList<Integer> indices = new ArrayList<Integer>(); // holds indices of map's vertices that are reflex
  ArrayList<Wall> edges = new ArrayList<Wall>(); // holds new edges of nav mesh
  ArrayList<Wall> perimeter = map.outline; // holds map outline
  ArrayList<Wall> allEdges = new ArrayList<Wall>(); // holds all the edges of the map (perimeter + edges)
  ArrayList<PVector> allPoints = new ArrayList<PVector>(); // holds all the points of the map
  ArrayList<Node> polygonCenter = new ArrayList<Node>(); // holds polygons and nodes
  
   void bake(Map map)
   {
       // generate the graph you need for pathfinding
       
       // resets nav mesh whenever new map is generated
       reflex.clear();
       //indices.clear();
       edges.clear();
       allEdges.clear();
       allPoints.clear();
       polygonCenter.clear();
       
       // checks if the angle at the node is reflex, if it is, add to the reflex ArrayList
       for (int i = 0; i < perimeter.size(); i++)
       {
         if (perimeter.get(i).normal.dot(perimeter.get((i+1)%perimeter.size()).direction) > 0)
         {
           PVector point = perimeter.get(i).end;
           ArrayList<Wall> walls = new ArrayList<Wall>();
           Wall temp = new Wall(point, perimeter.get(i).start);
           walls.add(temp);
           temp = new Wall(point, perimeter.get(i+1).end);
           walls.add(temp); 
           reflex.add(new Reflex(i+1, point, walls));
         }
       }
       
       // if all conditions are met, add edge from reflex point to outline vertices
       for (int i = 0; i < reflex.size(); i++)
       {
         for (int j = 0; j < perimeter.size(); j++)
         {
           // disregards neighbors of reflex
           if (j == reflex.get(i).id + 1 || j == reflex.get(i).id - 1)
             continue;
           else
           {
             Wall temp = new Wall(reflex.get(i).pt, perimeter.get(j).start);
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
               {
                 edges.add(i, temp);
                 reflex.get(i).connections.add(temp);
               }
             }
           }
         }
       }
       
       ArrayList<Wall> polygon = new ArrayList<Wall>();
       
       for (int i = 0; i < reflex.size(); i++)
       {
         // moves the edge to the right of the reflex (perimeter wall) to the last index
         //Wall temp = reflex.get(i).connections.get(0);
         //reflex.get(i).connections.add(temp);
         //reflex.get(i).connections.remove(0);
         
         // from each reflex point, make a polygon using the reflex point and the endpoints of the neighboring connections
         for (int j = 0; j < reflex.get(i).connections.size() - 1; j++)
         {
            PVector[] nodes = {reflex.get(i).pt, reflex.get(i).connections.get(j).end, reflex.get(i).connections.get(j+1).end};
            AddPolygon(polygon, nodes);
            Node node = new Node(0, polygon);
            polygonCenter.add(j, node);
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
     
     for (int i = 0; i < reflex.size(); i++)
     {
       stroke(0, 255 , 150);
       fill(0, 255, 100);
       circle(reflex.get(i).pt.x, reflex.get(i).pt.y, 10);
     }
     
     ////for (Node n : polygonCenter.get()
     ////{
     //  for (Wall w : polygonCenter.get(0).polygon)
     //  {
     //    stroke(150, 150, 100);
     //    w.draw();
     //  }
     //  stroke(0, 150 , 150);
     //  fill(0, 255, 100);
     //  //circle(n.center.x, n.center.y, 10);
     ////}
     
     // DEBUGGING CODE
     
     /*SHOWS EVERY POLYGON FROM A REFLEX VERTEX*/
     //for (Wall w : polygonCenter.get(0).polygon)
     //{
     //  stroke(150, 150, 100);
     //  w.draw();
     //}
     //stroke(0, 150 , 150);
     //fill(0, 255, 100);
     
     /*SHOWS ALL CONNECTIONS FROM A REFLEX VERTEX*/
     //for (Wall w : reflex.get(0).connections)
     //{
     //  stroke(150, 0, 100);
     //  w.draw();
     //}
     
     
     
     //stroke(150, 150, 100);
     //reflex.get(0).connections.get(0).draw();
     
     //polygonCenter.get(1).polygon.draw();
     
   }
}
