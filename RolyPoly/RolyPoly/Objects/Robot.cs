using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino;
using Rhino.Geometry;
using RolyPoly.Settings;
using RolyPoly.Utilities;

namespace RolyPoly.Objects
{
    /// <summary>
    /// Contains information related to a specific robot that is being controlled.
    /// </summary>
    internal class Robot
    {
        /// <summary>
        /// The namespace of the robot, per its configuration settings (not defined in Grasshopper, the namespace is set via the robot's web interface)
        /// </summary>
        public string RobotName { get; }
        /// <summary>
        /// Direction that robot is facing
        /// </summary>
        public Vector3d RobotDirection { get; private set; }
        /// <summary>
        /// Center point of the robot.
        /// </summary>
        public Point3d RobotCenter { get; private set; }
        /// <summary>
        /// Additional geometry that is attached to this robot and which you want to simulate the movement of, for example, a payload.
        /// </summary>
        public List<GeometryBase> AdditionalGeometry { get; }
        /// <summary>
        /// Preview geometry, generated at instantiation, which represents the overall shape and direction of the robot.
        /// </summary>
        public List<GeometryBase> RobotGeometry { get; }
        /// <summary>
        /// Plane the robot should be moving in.
        /// </summary>
        public Plane RobotMovementPlane { get; }

        public Robot(string name, Plane movementPlane, Line orientationLine, List<GeometryBase> additionalGeom)
        {
            RobotName = name;
            RobotMovementPlane = movementPlane;
            RobotDirection = orientationLine.Direction;
            RobotCenter = orientationLine.From;
            if (additionalGeom != null)
            {
                AdditionalGeometry = (from g in additionalGeom select g.Duplicate()).ToList();
            }
            var scaleFactor = GeometryUtils.GetDistanceScaleFactorToModelUnits();
            Circle robotCircle = new Circle(RobotMovementPlane, RobotCenter, GlobalSettings.ROBOT_RADIUS_METERS * scaleFactor);
            var arrow = GeometryUtils.BuildArrow(movementPlane, RobotCenter, RobotDirection, GlobalSettings.ROBOT_DIRECTION_ARROW_LENGTH_METERS * scaleFactor, GlobalSettings.ROBOT_DIRECTION_ARROW_HEAD_LEG_LENGTH_METERS * scaleFactor, GlobalSettings.ROBOT_DIRECTION_ARROW_HEAD_ANGLE_RADIANS);
            RobotGeometry = new List<GeometryBase>() { robotCircle.ToNurbsCurve() };
            RobotGeometry.AddRange(arrow);
        }

        /// <summary>
        /// Creates a copy of a Robot.
        /// </summary>
        public Robot(Robot robot)
        {
            RobotDirection = robot.RobotDirection;
            RobotCenter = robot.RobotCenter;
            AdditionalGeometry = (from g in robot.AdditionalGeometry select g.Duplicate()).ToList();
            RobotGeometry = (from g in robot.RobotGeometry select g.Duplicate()).ToList();
            RobotMovementPlane = robot.RobotMovementPlane;
        }

        public void DriveForward(double distMeters)
        {
            var scaleFactor = GeometryUtils.GetDistanceScaleFactorToModelUnits();
            var dist = distMeters * scaleFactor;
            var d = RobotDirection;
            d.Unitize();
            Transform t = Transform.Translation(d * dist);
            TransformAll(t);
        }

        public void Rotate(double angleRad)
        {
            Transform t = Transform.Rotation(angleRad, RobotMovementPlane.ZAxis, RobotCenter);
            TransformAll(t);
        }

        public void DriveArc(double angleRad, double radiusMeters, bool isForward)
        {
            var scaleFactor = GeometryUtils.GetDistanceScaleFactorToModelUnits();
            var radius = radiusMeters * scaleFactor;
            Vector3d leftDir = Vector3d.CrossProduct(RobotMovementPlane.ZAxis, RobotDirection);
            Vector3d rightDir = leftDir * -1;
            Vector3d centerDir;
            if ((angleRad < 0 && isForward) || (angleRad > 0 && !isForward))
            {
                centerDir = rightDir;
            }
            else
            {
                centerDir = leftDir;
            }
            centerDir.Unitize();
            Point3d rotCenter = RobotCenter + (centerDir * radius);
            Transform t = Transform.Rotation(angleRad, RobotMovementPlane.ZAxis, rotCenter);
            TransformAll(t);
        }

        private void TransformAll(Transform t)
        {
            foreach (var g in RobotGeometry)
            {
                g.Transform(t);
            }
            foreach (var g in AdditionalGeometry)
            {
                g.Transform(t);
            }

            var dirCrv = new Line(RobotCenter, RobotDirection, 1).ToNurbsCurve();
            dirCrv.Transform(t);
            var dir = dirCrv.PointAtEnd - dirCrv.PointAtStart;
            dir.Unitize();
            RobotDirection = dir;

            var c = RobotCenter;
            c.Transform(t);
            RobotCenter = c;
        }
    }
}
