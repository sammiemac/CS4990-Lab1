// Useful to sort lists by a custom key
import java.util.*;

class Node
{
   int id; // holds ID of polygon node
   ArrayList<Wall> polygon; // holds walls of the polygon
   ArrayList<Integer> cornerIDs; // holds ID of each of the corners
   PVector center; // holds coordinates of polygon's center
   ArrayList<Node> neighbors; // holds Nodes of polygons that are adjacent to current polygon
   ArrayList<Wall> connections; // holds graph of walls connecting polygon center to midpoints of adjacent edges
                                // 
   
   Node(int id, ArrayList<Wall> polygon, ArrayList<Integer> cornerIDs)
   {
     this.id = id;
     this.polygon = new ArrayList<Wall>();
     for (Wall w : polygon)
       this.polygon.add(w);
     this.cornerIDs = new ArrayList<Integer>();
     for (Integer w : cornerIDs)
       this.cornerIDs.add(w);
     float xSum = 0;
     float ySum = 0;
     for (Wall w : polygon)
     {
       xSum += w.start.x;
       ySum += w.start.y;
     }
     this.center = new PVector(xSum / polygon.size(), ySum / polygon.size());
     this.neighbors = new ArrayList<Node>();
     this.connections = new ArrayList<Wall>();
   }
   
   void getNodeNeighbors(int id, ArrayList<Node> otherNodes, ArrayList<EdgeInfo> edges)
   {
     int edgeNeighborCounter, point1, point2, point3, nextPoint1, nextPoint2, nextPoint3;
     PVector tempmidpoint;
     point1 = cornerIDs.get(0);
     println("Polygon " + id + "'s Corner 0: " + cornerIDs.get(0));
     point2 = cornerIDs.get(1);
     println("Polygon " + id + "'s Corner 1: " + cornerIDs.get(1));
     point3 = cornerIDs.get(2);
     println("Polygon " + id + "'s Corner 2: " + cornerIDs.get(2));
     ArrayList<Integer> tempID = new ArrayList<Integer>();
     ArrayList<PVector> midpoints = new ArrayList<PVector>();
     
     for (int i = 0; i < otherNodes.size(); i++)
     {
       edgeNeighborCounter = 0;
       tempID.clear();
       if (i == this.id)
         continue;
       println("Comparing Polygon " + id + " with Polygon " + i);
       nextPoint1 = otherNodes.get(i).cornerIDs.get(0);
       nextPoint2 = otherNodes.get(i).cornerIDs.get(1);
       nextPoint3 = otherNodes.get(i).cornerIDs.get(2);
       if (point1 == nextPoint1 || point1 == nextPoint2 || point1 == nextPoint3)
       {
         edgeNeighborCounter += 1;
         println("Added Point " + point1 + " to edge");
         tempID.add(point1);
       }
       if (point2 == nextPoint1 || point2 == nextPoint2 || point2 == nextPoint3)
       {
         edgeNeighborCounter += 1;
         println("Added Point " + point2 + " to edge");
         tempID.add(point2);
       }
       if (point3 == nextPoint1 || point3 == nextPoint2 || point3 == nextPoint3)
       {
         edgeNeighborCounter += 1;
         println("Added Point " + point3 + " to edge");
         tempID.add(point3);
       }
       if (edgeNeighborCounter == 2)
       {
         println("Added midpoint!");
         this.neighbors.add((otherNodes.get((i)%otherNodes.size())));
         for (int j = 0; j < edges.size(); j++)
         {
           if ((tempID.get(0) == edges.get(j).start && tempID.get(1) == edges.get(j).end) ||
               (tempID.get(0) == edges.get(j).end && tempID.get(1) == edges.get(j).start))
           {
             tempmidpoint = new PVector((edges.get(j).wall.start.x + edges.get(j).wall.end.x)/2,
                                       (edges.get(j).wall.start.y + edges.get(j).wall.end.y)/2);
             midpoints.add(tempmidpoint);
           }
         }
       }
     }
     for (int i = 0; i < this.neighbors.size(); i++)
     {
       Wall connect = new Wall(this.center, midpoints.get(i));
       connections.add(connect);
       connect = new Wall(midpoints.get(i), neighbors.get(i).center);
       connections.add(connect);
     }
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
  PVector midpoint; // holds the midpoint of this edge
  
  EdgeInfo(int id, PointInfo start, PointInfo end)
  {
    this.id = id;
    this.start = start.id;
    this.end = end.id;
    this.wall = new Wall(start.pt, end.pt);
        
    this.midpoint = new PVector( (wall.start.x + wall.end.x)/2, (wall.start.y + wall.end.y)/2 );
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

void bubbleSort(int[] input)
{
  int n = input.length;
  {
    for (int i = 0; i < n - 1; i++)
    {
      for (int j = 0; j < n - i - 1; j++)
      {
        if (input[j] > input[j + 1])
        {
          // swap input[j+1] and input[j]
          int temp = input[j];
          input[j] = input[j + 1];
          input[j + 1] = temp;
        }
      }
    }
  }
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
       // resets nav mesh whenever new map is generated
       allPoints.clear();
       reflex.clear();
       edges.clear();
       polygonNode.clear();
       
       // orders point IDs into allPoints and identifies reflex vertices
       for (int i = 0; i < perimeter.size(); i++)
       {
         allPoints.add(new PointInfo(i, perimeter.get(i).start)); // add the current point to the array list
         
         if (perimeter.get(i).normal.dot(perimeter.get((i+1)%perimeter.size()).direction) > 0)
         {
           PVector point = perimeter.get(i).end;
           reflex.add(new PointInfo(i+1, point));
         }
       }
       
       // if all conditions are met, add edge from reflex point to outline vertices
       int edgeCounter = 0;
       EdgeInfo temp;
       for (int i = 0; i < reflex.size(); i++)
       {
         for (int j = reflex.get(i).id + 1; j != reflex.get(i).id;)
         {
           if (j == (reflex.get(i).id + 1) % perimeter.size())
           {
             // We are at the next neighbor of reflex
             temp = new EdgeInfo(edgeCounter++, reflex.get(i), allPoints.get(j));
             reflex.get(i).connections.add(temp);
             j = (j+1)%perimeter.size();
             continue;
           }
           else if (j == (reflex.get(i).id - 1) % perimeter.size())
           {
             // We are at the previous neighbor of reflex
             temp = new EdgeInfo(edgeCounter++, reflex.get(i), allPoints.get(j));
             reflex.get(i).connections.add(temp);
             j = (j+1)%perimeter.size();
             continue;
           }
           else
           {
             // We are trying to make a wall
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
       ArrayList<Integer> nodeIDs = new ArrayList<Integer>();
       ArrayList<int[]> listOfIDs = new ArrayList<int[]>();
       boolean polygonExists = false;
       Node node;
       for (int i = 0; i < reflex.size(); i++)
       {        
         // from each reflex point, make a polygon using the reflex point and the endpoints of the neighboring connections
         for (int j = 0; j < reflex.get(i).connections.size() - 1; j++)
         {
            PVector[] nodes = {reflex.get(i).pt, reflex.get(i).connections.get(j).wall.end,
                               reflex.get(i).connections.get((j+1)%reflex.get(i).connections.size()).wall.end};
            int[] tempNodes = {reflex.get(i).id, reflex.get(i).connections.get(j).end, reflex.get(i).connections.get((j+1)%reflex.get(i).connections.size()).end};
            bubbleSort(tempNodes);
            listOfIDs.add(0, tempNodes);
            for (int k = 1; k < listOfIDs.size(); k++)
            {
               if (Arrays.equals(tempNodes, listOfIDs.get(k)) && listOfIDs.size() != 1)
               {
                 polygonExists = true;
                 listOfIDs.remove(k);
                 break;
               }
            }
            if (!polygonExists)
            {
              nodeIDs.add(tempNodes[0]);
              nodeIDs.add(tempNodes[1]);
              nodeIDs.add(tempNodes[2]);
              AddPolygon(polygon, nodes);
              node = new Node(polygonCounter++, polygon, nodeIDs);
              polygonNode.add(node);
            }
            polygonExists = false;
            polygon.clear();
            nodeIDs.clear();
         }
       }
       
       for (int i = 0; i < polygonNode.size(); i++)
         polygonNode.get(i).getNodeNeighbors(i, polygonNode, edges);
   }
   
   ArrayList<PVector> findPath(PVector start, PVector destination)
   {
      // implement A* to find a path
      
/*    From an array of nodes (that contain nodes that represent the middle of a polygon and the midpoints of each edge)
         - find the distance between each node to each neighbor (this will be each node's cost)
         - find the straight line distance from each node to the target, then add this distance to an array (this will be each node's heuristic)
         - find the node that is closest in distance to the boid agent
         - find the node that is closest in distance to the target location
          
      A* Algo:
        - from the node that is closest to the boid, look at that node's neighbors and find the total sum of it's cost and it's heuristic
        - add this sum to a sorted array, with the smallest total sum at index position 0 (the first object in the array)
        - add the node with the lowest total to another array that hold's the boid's waypoints
        - from this node, look at this node's neighbors and find the total sum of it's cost (this cost to get to this node + the cost to get to its neighbor) and it's heursitic
        - add this sum to a sorted array, with the smallest total sum at index position 0 (the first object in the array)
        - add the node with the lowest total to another array that hold's the boid's waypoints
        - repeat until at destination
        - **if two or more nodes have the same total sum of cost and heuristics, add them to separate arrays and navigate on both of them
        - once the last node has been found, add the target location as the final point in the waypoints in the array
        - navigate
*/
      
      ArrayList<PVector> result = null;
      return result;
   }
   
   
   void update(float dt)
   {
      draw();
   }
   
   void draw()
   {
     
     /*Draws the newly made edges for the NavMesh*/
     for (EdgeInfo w : edges)
     {
       stroke(150, 0, 255);
       w.wall.draw();
     }
     
     /*Draws all the points on the map*/
     for (PointInfo p : allPoints)
     {
       stroke(255, 0 , 150);
       fill(255, 0, 100);
       circle(p.pt.x, p.pt.y, 10);
     }
     
     /*Draws all the reflex points on the map*/
     for (int i = 0; i < reflex.size(); i++)
     {
       stroke(0, 255 , 150);
       fill(0, 255, 100);
       circle(reflex.get(i).pt.x, reflex.get(i).pt.y, 10);
     }
     
     for (Node n : polygonNode)
     {
       //for (Wall w : n.polygon)
       //{
       //  stroke(255, 255, 100);
       //  w.draw();
       //}
       stroke(0, 150 , 150);
       fill(0, 255, 100);
       circle(n.center.x, n.center.y, 10);
       for (Wall w : n.connections)
       {
         stroke(0, 255, 100);
         w.draw();
       }
     }
     
     //for (EdgeInfo e : edges)
     //{
     //  stroke(0, 255 , 100);
     //  fill(0, 150, 150);
     //  circle(e.midpoint.x, e.midpoint.y, 10);
     //}
     
     
     // DEBUGGING CODE
     
     /*SHOWS EVERY POLYGON FROM A REFLEX VERTEX*/
     //for (Node p : polygonNode)
     //{
     //  for (Wall w : p.polygon)
     //  {
     //    stroke(255, 255, 50);
     //    w.draw();
     //    println("Printing polygon");
     //  }
     //}
     //println("Size of polygonNode array: " + polygonNode.size());
     
     /*SHOWS ALL CONNECTIONS FROM A REFLEX VERTEX*/
     //for (EdgeInfo w : reflex.get(0).connections)
     //{
     //  stroke(150, 0, 100);
     //  w.wall.draw();
     //}
     
     /*SHOWS ALL CONNECTIONS FROM A POLYGON CENTER*/
     //for (Wall w : polygonNode.get(3).connections)
     //{
     //  stroke(150, 0, 100);
     //  w.draw();
     //}
     //println("Polygon 2's points: " + polygonNode.get(2).cornerIDs.get(0) + " " +
     //        polygonNode.get(2).cornerIDs.get(1) + " " + polygonNode.get(2).cornerIDs.get(2));
     //println("Polygon 3's points: " + polygonNode.get(3).cornerIDs.get(0) + " " +
     //        polygonNode.get(3).cornerIDs.get(1) + " " + polygonNode.get(3).cornerIDs.get(2));
     //println("Polygon 4's points: " + polygonNode.get(4).cornerIDs.get(0) + " " +
     //        polygonNode.get(4).cornerIDs.get(1) + " " + polygonNode.get(4).cornerIDs.get(2));
     
     
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
     //stroke(255, 255, 0);
     //fill(255, 255, 0);
     //circle(reflex.get(0).pt.x, reflex.get(0).pt.y, 10);
     
     /*SHOWS A SINGLE EDGE*/
     //stroke(0, 0, 150);
     //edges.get(5).wall.draw();
     
   }
}
