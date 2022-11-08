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

class PointInfo
{
  
  int id; // holds the id of the point
  PVector pt; // holds the location of the point
  ArrayList<EdgeInfo> connections = new ArrayList<EdgeInfo>(); // holds the connections from this point
  
  PointInfo(int id, PVector pt)
  {
    this.id = id;
    this.pt = pt;
  }
}

class EdgeInfo
{

  int id; // holds the id of the edge
  int start; // holds the starting point that makes this edge
  int end; // holds the ending point that makes this edge
  Wall wall; // holds the wall that creates this edge
  
  EdgeInfo(int id, PointInfo start, PointInfo end)
  {
    this.id = id;
    this.start = start.id;
    this.end = end.id;
    this.wall = new Wall(start.pt, end.pt);
  } 
}

class PointCompare implements Comparator<PointInfo>
{

  int compare(PointInfo a, PointInfo b)
  {
     if (a.id < b.id) return -1;
     if (a.id > b.id) return 1;
     return 0;
  }  
}

class Polygon
{
  int id;
  ArrayList<Wall> sides = new ArrayList<Wall>();
  
  Polygon(int id)
  
}

class NavMesh
{
  ArrayList<Wall> perimeter = map.outline; // holds map outline
  ArrayList<PointInfo> allPoints = new ArrayList<PointInfo>(); // holds all the points of the map
  ArrayList<PointInfo> reflex = new ArrayList<PointInfo>(); // holds reflex vertices
  ArrayList<EdgeInfo> edges = new ArrayList<EdgeInfo>(); // holds new edges of nav mesh
  ArrayList<Node> polygonNode = new ArrayList<Node>(); // holds a node's information
  
   void bake(Map map)
   {
       // generate the graph you need for pathfinding
       
       // resets nav mesh whenever new map is generated
       allPoints.clear();
       reflex.clear();
       edges.clear();
       polygonNode.clear();
       
       // orders point IDs into allPoints and identifies reflex vertices
       for (int i = 0; i < perimeter.size(); i++)
       {
         allPoints.add(new PointInfo(i, perimeter.get(i).start)); // add the current point to the array list
         println("Point added: " + allPoints.get(i).id);
         
         if (perimeter.get(i).normal.dot(perimeter.get((i+1)%perimeter.size()).direction) > 0)
         {
           PVector point = perimeter.get(i).end;
           reflex.add(new PointInfo(i+1, point));
           println("Reflex at: " + allPoints.get(i).id);
         }
       }
       println("Amount of points: " + allPoints.size());
       
       // if all conditions are met, add edge from reflex point to outline vertices
       int edgeCounter = 0;
       EdgeInfo temp;
       for (int i = 0; i < reflex.size(); i++)
       {
         println("Started loop");
         for (int j = reflex.get(i).id + 1; j != reflex.get(i).id;)
         {
           if (j == (reflex.get(i).id + 1) % perimeter.size())
           {
             println("We are at the next neighbor of reflex");
             temp = new EdgeInfo(edgeCounter++, reflex.get(i), allPoints.get(j));
             reflex.get(i).connections.add(temp);
             j = (j+1)%perimeter.size();
             continue;
           }
           else if (j == (reflex.get(i).id - 1) % perimeter.size())
           {
             println("We are at the previous neighbor of reflex");
             temp = new EdgeInfo(edgeCounter++, reflex.get(i), allPoints.get(j));
             reflex.get(i).connections.add(temp);
             j = (j+1)%perimeter.size();
             continue;
           }
           else
           {
             println("We are trying to make a wall");
             temp = new EdgeInfo(edgeCounter, reflex.get(i), allPoints.get(j));
             temp.wall.start = PVector.add(temp.wall.start, PVector.mult(temp.wall.direction, 0.01));
             temp.wall.end = PVector.add(temp.wall.end, PVector.mult(temp.wall.direction, -0.01));
             // checks if temp will collide with wall and is within map
             if (!map.collides(temp.wall.start, temp.wall.end) && isPointInPolygon(temp.wall.center(), perimeter))
             {
               // checks if edge will collide with other edges, removes unnecessary edges
               boolean intersectsNavMesh = false;
               for (int k = 0; k < edges.size(); k++)
               {
                 if (temp.wall.crosses(edges.get(k).wall.start, edges.get(k).wall.end))
                 {
                   intersectsNavMesh = true;
                   // does not add edge but adds a connection
                   if (temp.start == edges.get(k).end && temp.end == edges.get(k).start)
                     reflex.get(i).connections.add(temp);
                   break;
                 }
               }
               if (!intersectsNavMesh)
               {
                 edges.add(temp);
                 reflex.get(i).connections.add(temp);
               }
             }
             j = (j+1)%perimeter.size();
           }
         }
       }
       
       /*MAKING POLYGON CODE*/
       int polygonCounter = 0;
       ArrayList<Wall> polygon = new ArrayList<Wall>();
       Node node;
       for (int i = 0; i < reflex.size(); i++)
       {        
         // from each reflex point, make a polygon using the reflex point and the endpoints of the neighboring connections
         println("Size of connections array: " + reflex.get(i).connections.size());
         for (int j = 0; j < reflex.get(i).connections.size() - 1; j++)
         {
            PVector[] nodes = {reflex.get(i).pt, reflex.get(i).connections.get(j).wall.end,
                               reflex.get(i).connections.get((j+1)%reflex.get(i).connections.size()).wall.end};
            AddPolygon(polygon, nodes);
            node = new Node(polygonCounter++, polygon);
            polygonNode.add(node);
            polygon.clear();
            nodes = null;
         }
       }
       
       //// adds the walls of the perimeter and the navmesh to allWalls
       //allEdges.addAll(perimeter);
       //allEdges.addAll(edges);
       
       //// adds all the start points of each wall in allEdges to allPoints
       //for (int i = 0 ; i < allEdges.size(); i++)
       //{
       //  allPoints.add(i, allEdges.get(i).start);
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
     
     for (EdgeInfo w : edges)
     {
       stroke(150, 0, 255);
       w.wall.draw();
     }
     
     for (PointInfo p : allPoints)
     {
       stroke(255, 0 , 150);
       fill(255, 0, 100);
       circle(p.pt.x, p.pt.y, 10);
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
     for (Wall w : polygonNode.get(0).polygon)
     {
       stroke(255, 255, 50);
       w.draw();
     }
     println("Size of polygonNode array: " + polygonNode.size());
     
     /*SHOWS ALL CONNECTIONS FROM A REFLEX VERTEX*/
     //for (EdgeInfo w : reflex.get(0).connections)
     //{
     //  stroke(150, 0, 100);
     //  w.wall.draw();
     //}
     
     /*SHOWS A SINGLE POLYGON*/
     //ArrayList<Wall> test = new ArrayList<Wall>();
     //PVector[] nodesTest = {reflex.get(0).pt, reflex.get(0).connections.get(0).wall.end,
     //                   reflex.get(0).connections.get(1).wall.end};
     //AddPolygon(test, nodesTest);
     //for (Wall w : test)
     //{
     //  stroke(50, 100, 100);
     //  w.draw();
     //}
       
     /*SHOWS A SINGLE CONNECTION*/
     //stroke(0, 100, 100);
     //reflex.get(0).connections.get(0).wall.draw();
     
     /*SHOWS A SINGLE POINT*/
     //stroke(0, 0, 150);
     //fill(0, 0, 150);
     //circle(allPoints.get(4).pt.x, allPoints.get(4).pt.y, 10);
     
     /*SHOWS A SINGLE EDGE*/
     //stroke(0, 0, 150);
     //edges.get(5).wall.draw();
     
   }
}
