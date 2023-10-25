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
    /// A rotate action
    /// </summary>
    internal class RobotActionRotate : IRobotActionSimple
    {
        public string RobotName { get; set; } = "undefined";
        /// <summary>
        /// How many radians to rotate during this action. Positive = counter-clockwise
        /// </summary>
        public double RotationAngleRadians { get; }

        /// <summary>
        /// Angular rotation speed in radians per second.
        /// </summary>
        public double RotationSpeedRadiansPerSecond { get; set; } = GlobalSettings.ROBOT_ROTATION_SPEED_RADIANS_PER_SECOND;

        public Plane RotationPlane { get; private set; } = GlobalSettings.ROBOT_DEFAULT_DRIVE_PLANE;
        public Vector3d StartRobotDir { get; private set; } = Vector3d.Unset;
        public Vector3d EndRobotDir { get; private set; } = Vector3d.Unset;

        public RobotActionRotate(double rotationAngleRad)
        {
            RotationAngleRadians = rotationAngleRad;
        }

        /// <summary>
        /// Sets the rotation plane, and updates the end direction.
        /// </summary>
        public void SetRotationPlane(Plane pln)
        {
            RotationPlane = pln;
            SetEndDirection();
        }

        /// <summary>
        /// Sets the start direction and end directions for this step.
        /// </summary>
        public void SetStartAndEndDirections(Vector3d startDir)
        {
            StartRobotDir = startDir;
            SetEndDirection();
        }

        /// <summary>
        /// Sets the end direction.
        /// </summary>
        private void SetEndDirection()
        {
            if (StartRobotDir == Vector3d.Unset) return;
            var endDir = StartRobotDir;
            endDir.Rotate(RotationAngleRadians, RotationPlane.ZAxis);
            EndRobotDir = endDir;
        }

        public string[] GetCommandStrings()
        {
            return new [] {$"MessageRotate(\"{RobotName}\", {StringUtils.GetFormattedDoubleString(RotationAngleRadians)}, {StringUtils.GetFormattedDoubleString(RotationSpeedRadiansPerSecond)})"};
        }
    }
}
