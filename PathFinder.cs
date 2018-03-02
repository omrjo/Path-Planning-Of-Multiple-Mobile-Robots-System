using System;
using System.Collections.Generic;
using System.Drawing;
using System.Text;

namespace X80Demo
{
    //PathFinder class contain information and functions needed for the search 
    public class PathFinder
    {
        private int width;
        private int height;
        private Node[,] nodes;
        private Node startNode;
        private Node endNode;
        private SearchParameters searchParameters;

        //Constructor of class PathFinder
        public PathFinder(SearchParameters searchParameters)
        {
            this.searchParameters = searchParameters;
            InitializeNodes(searchParameters.Map);
            this.startNode = this.nodes[searchParameters.StartLocation.X, searchParameters.StartLocation.Y];
            this.startNode.State = NodeState.Open;
            this.endNode = this.nodes[searchParameters.EndLocation.X, searchParameters.EndLocation.Y];
        }

        //FinPath function return list of the path from start to goal position
        public List<Point> FindPath()
        {
            List<Point> path = new List<Point>();
            // success return true if the path found and use parent information to add the path to list 
            //if success return false then there is no path found
            bool success = Search(startNode);
            if (success)
            {
                Node node = this.endNode;
                while (node.ParentNode != null)
                {
                    path.Add(node.Location);
                    node = node.ParentNode;
                }

                // Reverse to get the path in correct order from the start
                path.Reverse();
            }

            return path;
        }
        //InitializeNodes function take 2-dimension array which represent the grid map as true/false
        //for every square of the map create Node contain information about that square
        private void InitializeNodes(bool[,] map)
        {
            this.width = map.GetLength(0);
            this.height = map.GetLength(1);
            this.nodes = new Node[this.width, this.height];
            for (int y = 0; y < this.height; y++)
            {
                for (int x = 0; x < this.width; x++)
                {
                    this.nodes[x, y] = new Node(x, y, map[x, y], this.searchParameters.EndLocation);
                }
            }
        }
        //Search function perform the adjusted A* algorithm to find if there is path or not
        private bool Search(Node currentNode)
        {
            //the current Node expanded for the search set to closed to not search it again
            currentNode.State = NodeState.Closed;
            List<Node> nextNodes = GetAdjacentWalkableNodes(currentNode);

            //for every walkable adjecent nodes sort the nodes to always get the lowest cost node
            nextNodes.Sort((node1, node2) => node1.F.CompareTo(node2.F));
            foreach (var nextNode in nextNodes)
            {
                if (nextNode.Location == this.endNode.Location)
                {
                    //return true if path found
                    return true;
                }
                else
                {
                    //if didn't reach the goal position start the search from next node
                    if (Search(nextNode))
                        return true;
                }
            }

            //return false if path not found 
            return false;
        }

        //Get walkable adjacent nodes for the current node
        private List<Node> GetAdjacentWalkableNodes(Node fromNode)
        {
            List<Node> walkableNodes = new List<Node>();
            List<Point> nextLocations = GetAdjacentLocations(fromNode.Location);
            List<Point> list = new List<Point>();

            //after get the next locations check diognal locations near the obstacles as the robot cannot move through it
            foreach (var location in nextLocations)
            {
                int x = location.X;
                int y = location.Y;

                if (x < 0 || x >= this.width || y < 0 || y >= this.height)
                    continue;

                Node node = this.nodes[x, y];
                if (!node.IsWalkable)
                {
                    if (x == fromNode.Location.X && y == fromNode.Location.Y + 1)
                    {
                        Point diognal = new Point(x - 1, y);
                        Point diognal2 = new Point(x + 1, y);
                        list.Add(diognal);
                        list.Add(diognal2);
                    }
                    else if (x == fromNode.Location.X + 1 && y == fromNode.Location.Y)
                    {
                        Point diognal = new Point(x, y + 1);
                        Point diognal2 = new Point(x, y - 1);
                        list.Add(diognal);
                        list.Add(diognal2);

                    }
                    else if (x == fromNode.Location.X && y == fromNode.Location.Y - 1)
                    {
                        Point diognal = new Point(x - 1, y);
                        Point diognal2 = new Point(x + 1, y);
                        list.Add(diognal);
                        list.Add(diognal2);

                    }
                    else if (x == fromNode.Location.X - 1 && y == fromNode.Location.Y)
                    {
                        Point diognal = new Point(x, y + 1);
                        Point diognal2 = new Point(x, y - 1);
                        list.Add(diognal);
                        list.Add(diognal2);

                    }
                }
            }
            foreach (var point in list)
            {
                //remove diognal location near obstacles from next adjacent walkable nodes
                nextLocations.Remove(point);
            }

            //for each next location check if we will consider it or not
            foreach (var location in nextLocations)
            {
                int x = location.X;
                int y = location.Y;

                // Stay within the grid's boundaries
                if (x < 0 || x >= this.width || y < 0 || y >= this.height)
                    continue;

                Node node = this.nodes[x, y];
                // Ignore non-walkable nodes
                if (!node.IsWalkable)
                    continue;

                //Ignore closed nodes
                if (node.State == NodeState.Closed)
                    continue;

                //open nodes are only added to the list if their G-value is lower through current route.
                if (node.State == NodeState.Open)
                {
                    float traversalCost = Node.GetTraversalCost(node.Location, fromNode.Location);
                    float gTemp = fromNode.G + traversalCost;
                    if (gTemp < node.G)
                    {
                        node.ParentNode = fromNode;
                        walkableNodes.Add(node);
                    }
                }
                else
                {
                    // If it's untested, set the parent and make it Open for consideration next time
                    node.ParentNode = fromNode;
                    node.State = NodeState.Open;
                    walkableNodes.Add(node);
                }
            }

            return walkableNodes;
        }

        //Get next locations of the current node 
        private static List<Point> GetAdjacentLocations(Point fromLocation)
        {
            List<Point> list = new List<Point>
            {
                new Point(fromLocation.X-1, fromLocation.Y-1),
                new Point(fromLocation.X-1, fromLocation.Y  ),
                new Point(fromLocation.X-1, fromLocation.Y+1),
                new Point(fromLocation.X,   fromLocation.Y+1),
                new Point(fromLocation.X+1, fromLocation.Y+1),
                new Point(fromLocation.X+1, fromLocation.Y  ),
                new Point(fromLocation.X+1, fromLocation.Y-1),
                new Point(fromLocation.X,   fromLocation.Y-1)
            };
            return list;
        }
    }
}
