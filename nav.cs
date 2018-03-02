using System;
using System.Collections.Generic;
using System.Text;
using System.Drawing;

namespace X80Demo
{
    public class nav
    {
        public static short direction = 0,nxtdirection = 0;
        public static short middlesensor = short.MaxValue,leftsensor = short.MaxValue,rightsensor = short.MaxValue;
        public static short BackMiddleSensor = short.MaxValue,BackLeftSensor = short.MaxValue,BackRightSensor = short.MaxValue;
        public static bool obstacle = false, diagonalobstacle = false,backword = false;
        public static int firstm,firstl, firstr;
    }
}
