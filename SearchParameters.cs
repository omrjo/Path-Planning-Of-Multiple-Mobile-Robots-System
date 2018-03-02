using System;
using System.Collections.Generic;
using System.Drawing;

namespace X80Demo
{
    //search parameters used if every search: the map, start location, goal location
    public class SearchParameters
    {
        public Point StartLocation { get; set; }

        public Point EndLocation { get; set; }

        public bool[,] Map { get; set; }

        //constructor for class SearchParameters
        public SearchParameters(Point startLocation, Point endLocation, bool[,] map)
        {
            this.StartLocation = startLocation;
            this.EndLocation = endLocation;
            this.Map = map;
        }
    }
}
