using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Eto.Forms;
using Rhino.Geometry;
using RolyPoly.Interfaces;
using RolyPoly.Settings;
using RolyPoly.Utilities;

namespace RolyPoly.Objects
{
    /// <summary>
    /// A drive arc action.
    /// </summary>
    internal class RobotActionDriveArc : IRobotAction
    {
        public string RobotName { get; set; } = "undefined";
        /// <summary>
        /// Angle inscribed by the arc.
        /// </summary>
        public double AngleRadians { get; }
        /// <summary>
        /// Radius of the arc.
        /// </summary>
        public double ArcRadiusMeters { get; }
        /// <summary>
        /// If true, the robot will drive forwards.
        /// </summary>
        public bool IsForwards { get; }
        /// <summary>
        /// Segment representing path in Rhino space.
        /// </summary>
        public Arc ArcSegment { get; }
        /// <summary>
        /// How fast to drive. Consult iRobot's documentation for acceptable ranges!
        /// </summary>
        public double DriveSpeedMetersPerSecond { get; set; } = GlobalSettings.ROBOT_DRIVE_SPEED_METERS_PER_SECOND;
        public Vector3d StartRobotDir { get; }
        public Vector3d EndRobotDir { get; }


        public RobotActionDriveArc(Arc arc, bool isForwards, Plane drivePln)
        {
            var scaleFactor = GeometryUtils.GetDistanceScaleFactorToRoombaUnits();
            ArcSegment = arc;
            ArcRadiusMeters = arc.Radius * scaleFactor;
            IsForwards = isForwards;
            var arcCrv = arc.ToNurbsCurve();
            Vector3d endTangent = arcCrv.TangentAtEnd;
            Vector3d startTangent = arcCrv.TangentAtStart;
            if (isForwards)
            {
                StartRobotDir = startTangent;
                EndRobotDir = endTangent;
            }
            else
            {
                StartRobotDir = startTangent * -1;
                EndRobotDir = endTangent * -1;
            }
            Vector3d toCenter = arc.Center - arc.StartPoint;
            Vector3d cP = Vector3d.CrossProduct(toCenter, startTangent);
            var angle = Vector3d.VectorAngle(cP, drivePln.ZAxis);
            bool isCW = angle < Math.PI / 2;//angle should either be very close to 0 or Pi radians, given that we are assuming prior verification that drive plane and arc plane are coplanar.
            AngleRadians = isCW ? arc.Angle * -1 : arc.Angle;
        }

        public string[] GetCommandStrings()
        {
            return new []{$"MessageDriveArc(\"{RobotName}\", {StringUtils.GetFormattedDoubleString(AngleRadians)}, {StringUtils.GetFormattedDoubleString(ArcRadiusMeters)}, {StringUtils.GetFormattedDoubleString(DriveSpeedMetersPerSecond)}, {IsForwards})"};
        }
    }
}
