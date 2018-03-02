using System;
using System.Collections.Generic;
using System.Windows.Forms;
using System.Drawing;

namespace X80Demo
{
    public class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        public bool[,] map;
        private SearchParameters searchParameters;

        [STAThread]
        static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new Form1());
        }

        public List<Point> Run(Point s, Point g)
        {
            Point start = s;
            Point goal = g;
            //PathFinder pathFinder = new PathFinder(searchParameters);
            //List<Point> path = pathFinder.FindPath();
           // Point point = new Point(1, 1);
            //AddObestacle(point);
            //point = new Point(1,0);
            //AddObestacle(point);
            PathFinder pathFinder = new PathFinder(searchParameters);
            List<Point> path = pathFinder.FindPath();
            return path;
        }
        public void InitializeMap(Point s, Point g)
        {
            map = new bool[4, 4];
            for (int y = 0; y < 4; y++)
                for (int x = 0; x < 4; x++)
                    map[x, y] = true;

            var startLocation = s;
            var endLocation = g;
            searchParameters = new SearchParameters(startLocation, endLocation, map);
        }

        public void AddObestacle(Point point)
        {
            map[point.X, point.Y] = false;
        }
    }
}