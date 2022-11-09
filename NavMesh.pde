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
                                // and then to centers of adjacent polygons
  ArrayList<PVector> betweenMidpoints;
   
   Node(int id, ArrayList<Wall> polygon, ArrayList<Integer> cornerIDs)
   {
     this.id = id;
     this.polygon = new ArrayList<Wall>();
     for (Wall w : polygon)
       this.polygon.add(w);
     this.cornerIDs = new ArrayList<Integer>();
     for (Integer w : cornerIDs)
       this.cornerIDs.add(w);
     // center == mean of points' coordinates
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
     this.betweenMidpoints = new ArrayList<PVector>();
   }
   
   void getNodeNeighbors(ArrayList<Node> otherNodes, ArrayList<EdgeInfo> edges)
   {
     int sharedPoints; // counter to see how many shared points a polygon has with another
     int point1, point2, point3; // holds IDs of this polygon's points
     int nextPoint1, nextPoint2, nextPoint3; // holds IDs of polygon we are comparing
     PVector tempMidpoint;
     point1 = cornerIDs.get(0);
     point2 = cornerIDs.get(1);
     point3 = cornerIDs.get(2);
     ArrayList<Integer> tempIDs = new ArrayList<Integer>(); // space to compare IDs of shared points
     ArrayList<PVector> midpoints = new ArrayList<PVector>();
     
     for (int i = 0; i < otherNodes.size(); i++)
     {
       sharedPoints = 0;
       // resets whenever we look at new polygon
       tempIDs.clear();
       // skip if comparing with itself
       if (i == this.id)
         continue;
       nextPoint1 = otherNodes.get(i).cornerIDs.get(0);
       nextPoint2 = otherNodes.get(i).cornerIDs.get(1);
       nextPoint3 = otherNodes.get(i).cornerIDs.get(2);
       
       // polygon is adjacent if it shares an edge i.e., it shares two points
       if (point1 == nextPoint1 || point1 == nextPoint2 || point1 == nextPoint3)
       {
         sharedPoints += 1;
         tempIDs.add(point1);
       }
       if (point2 == nextPoint1 || point2 == nextPoint2 || point2 == nextPoint3)
       {
         sharedPoints += 1;
         tempIDs.add(point2);
       }
       if (point3 == nextPoint1 || point3 == nextPoint2 || point3 == nextPoint3)
       {
         sharedPoints += 1;
         tempIDs.add(point3);
       }
       if (sharedPoints == 2)
       {
         // add compared polygon to current polygon's list of neighbors
         this.neighbors.add((otherNodes.get((i)%otherNodes.size())));
         // look for the ID of the edge that is shared, find its midpoint, then add it as an intermediate
         // point for the polygon to connect to before connecting to the adjacent polygon's center
         for (int j = 0; j < edges.size(); j++)
         {
           if ((tempIDs.get(0) == edges.get(j).start && tempIDs.get(1) == edges.get(j).end) ||
               (tempIDs.get(0) == edges.get(j).end && tempIDs.get(1) == edges.get(j).start))
           {
             tempMidpoint = edges.get(j).midpoint;
             midpoints.add(tempMidpoint);
           }
         }
       }
     }
     
     // creating navmesh graph i.e., the connections
     for (int i = 0; i < this.neighbors.size(); i++)
     {
       if (i == midpoints.size())
         break;
       Wall connect = new Wall(this.center, midpoints.get(i));
       connections.add(connect);
       PVector mid = midpoints.get(i);
       betweenMidpoints.add(mid);
       connect = new Wall(midpoints.get(i), neighbors.get(i).center);
       connections.add(connect);
     }
   }
   
   ArrayList<PVector> getNavigableGraph()
   {
     return new ArrayList<PVector>();
   }
   
}

// added PointInfo and EdgeInfo classes to give points and edges IDs
class PointInfo
{
  
  int id; // holds ID of the point
  PVector pt; // holds location of the point
  ArrayList<EdgeInfo> connections = new ArrayList<EdgeInfo>(); // holds the connections from this point
  int cost; // holds the distance between the current point to this point
  int heuristic; // holds the distance between this point and the target location
  
  PointInfo(int id, PVector pt)
  {
    this.id = id;
    this.pt = pt;
  }
}

class EdgeInfo
{

  int id; // holds ID of the edge
  int start; // holds ID of starting point that makes this edge
  int end; // holds ID of ending point that makes this edge
  Wall wall; // holds wall that creates this edge
  PVector midpoint; // holds midpoint of this edge
  
  EdgeInfo(int id, PointInfo start, PointInfo end)
  {
    this.id = id;
    this.start = start.id;
    this.end = end.id;
    this.wall = new Wall(start.pt, end.pt);    
    this.midpoint = this.wall.center();
  } 
}

// PointCompare and bubbleSort for utility
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
  ArrayList<PointInfo> allPoints = new ArrayList<PointInfo>(); // gives IDs to all points of the map outline
  ArrayList<PointInfo> reflex = new ArrayList<PointInfo>(); // gives IDs to all reflex vertices
  ArrayList<EdgeInfo> edges = new ArrayList<EdgeInfo>(); // gives IDs to new edges of nav mesh
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
       allPoints.add(new PointInfo(i, perimeter.get(i).start)); // add the current point to the array list, assigns ID in order
         
       // while traveling the perimeter, if we need to make any turn to the right, it is a reflex
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
                 // does not add edge but adds a connection if edge already exists
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
      
      // creating polygons
      int polygonCounter = 0;
      ArrayList<Wall> polygon = new ArrayList<Wall>();
      ArrayList<Integer> cornerIDs = new ArrayList<Integer>(); // holds IDs of corners of polygon
      ArrayList<int[]> compareIDs = new ArrayList<int[]>(); // 
      boolean polygonExists = false;
      Node node;
      for (int i = 0; i < reflex.size(); i++)
      {        
        // from each reflex point, make a polygon using the reflex point and the endpoints of the neighboring connections
        for (int j = 0; j < reflex.get(i).connections.size() - 1; j++)
        {
          // holds PVectors of polygon's corners
          PVector[] nodes = {reflex.get(i).pt, reflex.get(i).connections.get(j).wall.end,
                             reflex.get(i).connections.get((j+1)%reflex.get(i).connections.size()).wall.end};
          // holds IDs of polygon's corneres
          int[] tempNodes = {reflex.get(i).id, reflex.get(i).connections.get(j).end,
                             reflex.get(i).connections.get((j+1)%reflex.get(i).connections.size()).end};
          // sorts order of IDs from least to greatest for better comparison
          bubbleSort(tempNodes);
          compareIDs.add(0, tempNodes);
          for (int k = 1; k < compareIDs.size(); k++)
          {
            // if we found a connection already, remove from compareIDs
            if (Arrays.equals(tempNodes, compareIDs.get(k)) && compareIDs.size() != 1)
            {
              polygonExists = true;
              compareIDs.remove(k);
              break;
            }
          }
          if (!polygonExists)
          {
            cornerIDs.add(tempNodes[0]);
            cornerIDs.add(tempNodes[1]);
            cornerIDs.add(tempNodes[2]);
            AddPolygon(polygon, nodes);
            node = new Node(polygonCounter++, polygon, cornerIDs);
            polygonNode.add(node);
          }
          // reset comparison structures
          polygonExists = false;
          polygon.clear();
          cornerIDs.clear();
       }
     }
     
     // find neighbors and connections of each polygons
     for (int i = 0; i < polygonNode.size(); i++)
       polygonNode.get(i).getNodeNeighbors(polygonNode, edges);
   }
   
   ArrayList<PVector> findPath(PVector start, PVector destination)
   {
      // implement A* to find a path
      
      /*The following is an incomplete implementation of A* search (pseudocode). 
      While the code doesn't actually work, it captures the idea we had 
      for implementing this portion of the lab.*/
      
/*    Node currentNode = new Node(); // holds the node that the boid is currently at
      Node lookingAt; // holds the node that we are thinking of moving to next
      float totalCost = 0; // holds the total cost
      ArrayList<Node> distances = new ArrayList<Node>(); // holds all of the midpoints sorted by their distance from the starting location
      ArrayList<Node> priorityQueue = new ArrayList<Node>(); // holds the priority queue for a*
      ArrayList<PVectors> result = new ArrayList<PVectors>(); // holds all the nodes we've visited in order
      
      // from an arraylist of midpoints, find the distance from the starting location to the midpoint
      for(int i = 0; i < midpoints.size(); i++)
      {
        Node temp = midpoints.get(i);
        temp.dist = (start - midpoints.get(i).pt).mag;
        distances.add(temp);
      }
      distances.sort(dist); // sort the distances from shortest to longest
      currentNode = distances.get(0); // when first initiated, set the current node to the closest midpoint from the starting location
      result.add(distances.get(0)); // add the first midpoint visited to results
      
      // as long as we aren't at the destination, pathfind using a*
      while(currentNode.pt != destination)
      {
        // look at the neighbors of the current node
        for (int i = 0; i < currentNode.neighbors.size(); i++)
        {
          lookingAt = currentNode.neighbors.get(i); // look at the neighbor
          lookingAt.cost = (lookingAt.pt - currentNode.pt).mag; // the cost to get to this neighbor (distance between the nodes)
          totalCost += lookingAt.cost; // add the cost to the total cost
          lookingAt.heuristic = (destination - lookingAt.pt).mag; // the heuristic is the distance between the node and the destination
          lookingAt.totalSum = (totalCost + lookingAt.heuristic); // the value of this node, the sum of the total cost traveled and the node's heuristic
          priorityQueue.add(lookingAt); // add to the priority queue
        }
        priorityQueue.sort(totalSum); // sort the priority queue by the totalSum, from smallest to largest
        
        result.add(priorityQueue.get(0)); // add the node with the smallest value to the result
        currentNode = priorityQueue.get(0); // move to the next node
      }
      
/*    From an array of nodes (that contain nodes that represent the midpoints of each edge)
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
