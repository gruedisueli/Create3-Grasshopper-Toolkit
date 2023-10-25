using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino;
using Rhino.Geometry;

namespace RolyPoly.Settings
{
    internal static class GlobalSettings
    {
        public static readonly double ANIMATION_TIME_INC_SECONDS = 0.15;
        public static readonly double ROBOT_RADIUS_METERS = 0.2;
        public static readonly double ROBOT_DIRECTION_ARROW_LENGTH_METERS = 0.1;
        public static readonly double ROBOT_DIRECTION_ARROW_HEAD_LEG_LENGTH_METERS = 0.05;
        public static readonly double ROBOT_DIRECTION_ARROW_HEAD_ANGLE_RADIANS = 3 * Math.PI / 4;
        public static readonly Plane ROBOT_DEFAULT_DRIVE_PLANE = Plane.WorldXY;
        public static readonly double ROBOT_ROTATION_SPEED_RADIANS_PER_SECOND = 0.2;
        public static readonly double ROBOT_DRIVE_SPEED_METERS_PER_SECOND = 0.075;
        public static readonly double STANDARD_TOLERANCE = 0.001;
    }
}
