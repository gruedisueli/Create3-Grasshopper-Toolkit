using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;
using RolyPoly.Interfaces;
using RolyPoly.Settings;
using RolyPoly.Utilities;

namespace RolyPoly.Objects
{
    /// <summary>
    /// A drive straight action
    /// </summary>
    internal class RobotActionDriveStraight : IRobotAction
    {
        public string RobotName { get; set; } = "undefined";
        /// <summary>
        /// How far to drive.
        /// </summary>
        public double DriveDistanceMeters { get; }
        /// <summary>
        /// Segment representing path in Rhino space.
        /// </summary>
        public Line LineSegment { get; }
        /// <summary>
        /// How fast to drive. Consult iRobot's documentation for acceptable ranges!
        /// </summary>
        public double DriveSpeedMetersPerSecond { get; set; } = GlobalSettings.ROBOT_DRIVE_SPEED_METERS_PER_SECOND;
        public Vector3d StartRobotDir { get; }
        public Vector3d EndRobotDir { get; }

        public RobotActionDriveStraight(Line segment, bool isForwards)
        {
            LineSegment = segment;
            var scaleFactor = GeometryUtils.GetDistanceScaleFactorToRoombaUnits();
            var driveDist = segment.Length * scaleFactor;
            if (isForwards)
            {
                StartRobotDir = LineSegment.Direction;
            }
            else
            {
                driveDist *= -1;
                StartRobotDir = LineSegment.Direction * -1;
            }

            EndRobotDir = StartRobotDir;

            DriveDistanceMeters = driveDist;
        }

        public string[] GetCommandStrings()
        {
            return new [] {$"MessageDriveStraight(\"{RobotName}\", {StringUtils.GetFormattedDoubleString(DriveDistanceMeters)}, {StringUtils.GetFormattedDoubleString(DriveSpeedMetersPerSecond)})"};
        }
    }
}
