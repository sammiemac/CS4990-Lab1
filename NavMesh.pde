// Useful to sort lists by a custom key
import java.util.Comparator;

// In this file you will implement your navmesh and pathfinding. 

// This node representation is just a suggestion
class Node
{
   int id; // holds the id of the node
   ArrayList<Wall> polygon; // holds the walls that make the polygon surrounding the node
   PVector center; // holds the location of the node
   ArrayList<Node> neighbors; // holds the node's neighbors
   ArrayList<Wall> branches; // holds the connections to the node's neighbors
   int cost; // holds the cost to get to this node
   int heuristic; // holds the distance between this node and the goal
   
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

class Point
{
  
  int id; // holds the id of the point
  PVector pt; // holds the location of the point
  ArrayList<Wall> connections = new ArrayList<Wall>(); // holds the connections from this point
  
  Point(int id, PVector pt)
  {
    this.id = id;
    this.pt = pt;
  }
  
  Point(int id, PVector pt, ArrayList<Wall> c)
  {
    this.id = id;
    this.pt = pt;
    this.connections = c;
  }
  
}

class PointCompare implements Comparator<Point>
{

  int compare(Point a, Point b)
  {
     if (a.id < b.id) return -1;
     if (a.id > b.id) return 1;
     return 0;
  }
  
}

class Edge
{

  int id; // holds the id of the edge
  Point start; // holds the starting point that makes this edge
  Point end; // holds the ending point that makes this edge
  Wall w; // holds the wall that creates this edge
  
  Edge(int id, Point start, Point end)
  {
    this.id = id;
    this.start = start;
    this.end = end;
    this.w = new Wall(start.pt, end.pt);
  }
  
}

class NavMesh
{   
  
  ArrayList<Point> points = new ArrayList<Point>(); // holds the points on the map
  ArrayList<Point> reflex = new ArrayList<Point>(); // holds reflex vertices
  
  ArrayList<Edge> edges = new ArrayList<Edge>(); // holds new edges of nav mesh
  ArrayList<Wall> perimeter = map.outline; // holds map outline
  
  ArrayList<Node> polygonCenter = new ArrayList<Node>(); // holds polygons and nodes
  
   void bake(Map map)
   {
       // generate the graph you need for pathfinding
       
       // resets nav mesh whenever new map is generated
       points.clear();
       reflex.clear();
       edges.clear();
       polygonCenter.clear();
       
       // checks if the angle at the node is reflex, if it is, add to the reflex ArrayList
       for (int i = 0; i < perimeter.size(); i++)
       {
         points.add(new Point((i+1)%perimeter.size(), perimeter.get(i).start)); // add the current point to the array list
         println("Point added: " + points.get(i).id);
         
         if (perimeter.get(i).normal.dot(perimeter.get((i+1)%perimeter.size()).direction) > 0)
         {
           PVector pt = perimeter.get(i).end;
           reflex.add(new Point(i+1, pt));
           println("Reflex at: " + points.get(i).id);
         }
       }
       println("Amount of points: " + points.size());
       
       println("Before sort:");
       for (Point p : points) print(p.id + " ");
       points.sort(new PointCompare());
       println("\nAfter sort");
       for (Point p : points) print(p.id + " ");
       println();
       
       
       // edge generation
       // if all conditions are met, add edge from reflex point to outline vertices
       Edge temp;
       Point tempstart;
       Point tempend;
       for (int i = 0; i < reflex.size(); i++) // i = index of reflex
       {
         println("Started loop");
         println("At reflex: " + reflex.get(i).id);
         for (int j = reflex.get(i).id + 1; j != reflex.get(i).id;) // j = index of perimeter
         {
           println("Looking at: " + j);
           
           // checks if we are at the neighbor of the current reflex 
           // if we are, don't make an edge but add a connection to the reflex
           if (j == (reflex.get(i).id + 1) % perimeter.size())
           {
             println("We are at the next neighbor of reflex");
             tempstart = new Point(reflex.get(i).id, reflex.get(i).pt); // the current reflex
             tempend = new Point(points.get(j).id, points.get(j).pt); // the next point
             temp = new Edge(j, tempstart, tempend);
             reflex.get(i).connections.add(temp.w);
             println("Connection made");
             j = (j+1)%perimeter.size();
             continue;
           }
           else if (j == (reflex.get(i).id - 1) % perimeter.size())
           {
             println("We are at the previous neighbor of reflex");
             tempstart = new Point(points.get(j).id, points.get(j).pt); // the previous point
             tempend = new Point(reflex.get(i).id, reflex.get(i).pt); // the current reflex
             temp = new Edge(j, tempstart, tempend);
             reflex.get(i).connections.add(temp.w);
             println("Connection made");
             j = (j+1)%perimeter.size();
             continue;
           }
           else // if we are not at a neighbor, then we can try to make a wall
           {
             println("We are trying to make a wall");
             tempstart = new Point(reflex.get(i).id, reflex.get(i).pt); // the current reflex
             tempend = new Point(points.get(j).id, points.get(j).pt); // the point we're trying to connect to, btw this is probably wrong
             temp = new Edge(j, tempstart, tempend);
             
             // give the start and end points some clearance
             temp.w.start = PVector.add(temp.w.start, PVector.mult(temp.w.direction, 0.01));
             temp.w.end = PVector.add(temp.w.end, PVector.mult(temp.w.direction, -0.01));
             // checks if temp will collide with wall and is within map
             if (!map.collides(temp.w.start, temp.w.end) && isPointInPolygon(temp.w.center(), perimeter))
             {
               // checks if edge will collide with other edges, removes unnecessary edges
               boolean intersectsNavMesh = false;
               for (int k = 0; k < edges.size(); k++) // k = index of edges
               {
                 if (temp.w.crosses(edges.get(k).start.pt, edges.get(k).end.pt))
                 {
                   intersectsNavMesh = true;
                   if (temp.start.id == edges.get(k).end.id && temp.end.id == edges.get(k).start.id)
                   {
                     reflex.get(i).connections.add(temp.w);
                     println("Same edge, connection made");
                   }
                   println("Crosses another edge, no edge made");
                   break;
                 }
               }
               if (!intersectsNavMesh)
               {
                 edges.add(temp);
                 reflex.get(i).connections.add(temp.w);
                 println("Meets conditions, edge and connection made");
               }   
             }
             else
               println("Crosses with wall, no edge made");
             
             j = (j+1)%perimeter.size();
           }
         }
       }
       
       //ArrayList<Wall> polygon = new ArrayList<Wall>();
       
       //for (int i = 0; i < reflex.size(); i++)
       //{         
       //  // from each reflex point, make a polygon using the reflex point and the endpoints of the neighboring connections
       //  println("Size of connections array: " + reflex.get(i).connections.size());
       //  for (int j = 0; j < reflex.get(i).connections.size() - 1; j++)
       //  {
       //     PVector[] nodes = {reflex.get(i).pt, reflex.get(i).connections.get(j).end, reflex.get(i).connections.get(j+1).end};
       //     AddPolygon(polygon, nodes);
       //     Node node = new Node(0, polygon);
       //     polygonCenter.add(j, node);
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
     
     for (Edge e : edges)
     {
       stroke(150, 0, 255);
       e.w.draw();
     }
     
     for (Point p : points)
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
     //for (Wall w : polygonCenter.get(0).polygon)
     //{
     //  stroke(255, 255, 50);
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
     
     /*SHOWS A SINGLE CONNECTION*/
     //stroke(0, 100, 100);
     //reflex.get(0).connections.get(0).draw();
     
     /*SHOWS ALL THE VERTEX POINTS ON THE MAP*/
     for (Point pt : points)
     {
       stroke(255, 255, 0);
       fill(255, 255, 0);
       circle(pt.pt.x, pt.pt.y, 10);
     }
     
     /*SHOWS ALL THE REFLEX POINTS ON THE MAP*/
     for (Point r: reflex)
     {
       stroke(255, 0, 0);
       fill(255, 255, 0);
       circle(r.pt.x, r.pt.y, 12);
     }
     
     //stroke(150, 150, 100);
     //reflex.get(0).connections.get(0).draw();
     
     //polygonCenter.get(1).polygon.draw();
     
   }
}
