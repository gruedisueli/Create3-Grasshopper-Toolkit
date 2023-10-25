using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;
using RolyPoly.Interfaces;
using RolyPoly.Objects;
using RolyPoly.Settings;

namespace RolyPoly.Utilities
{
    internal static class ActionUtils
    {
        /// <summary>
        /// Takes a list of actions and inserts rotations as needed between driving robot steps so that the robot follows the path of a polycurve.
        /// </summary>
        public static List<IRobotAction> InsertRotationsBetweenSegments(List<IRobotAction> rawActionList, Vector3d initialRobotDirection, double rotSpeedBetweenSegmentsRadiansPerSecond, Plane drivePln)
        {
            var currDir = initialRobotDirection;
            currDir.Unitize();
            var finalActionsList = new List<IRobotAction>();
            foreach(var action in rawActionList)
            {
                if (action is IRobotActionSimple aS)
                {
                    aS.SetStartAndEndDirections(currDir);//directions are not set by default for these actions when the user creates them.
                }
                else if (IsTangentDifferent(currDir, action.StartRobotDir, drivePln, out var angle))
                {
                    var aR = new RobotActionRotate(angle);
                    aR.RotationSpeedRadiansPerSecond = rotSpeedBetweenSegmentsRadiansPerSecond;
                    aR.SetRotationPlane(drivePln);//this may not be strictly necessary, including for completeness
                    aR.SetStartAndEndDirections(currDir);//this may not be strictly necessary, including for completeness
                    finalActionsList.Add(aR);
                }

                finalActionsList.Add(action);
                currDir = action.EndRobotDir;
                currDir.Unitize();
            }

            return finalActionsList;
        }

        /// <summary>
        /// Applies the specified name to all actions in the list.
        /// </summary>
        public static void UpdateActionsWithRobotName(ref List<IRobotAction> actions, string name)
        {
            foreach (var a in actions)
            {
                a.RobotName = name;
            }
        }

        /// <summary>
        /// Returns true if the two directions are different, as well as angle between them.
        /// </summary>
        private static bool IsTangentDifferent(Vector3d dir1, Vector3d dir2, Plane drivePln, out double angle)
        {
            angle = Vector3d.VectorAngle(dir1, dir2, drivePln.ZAxis);
            if (angle > Math.PI)
            {
                angle -= Math.PI * 2;
            }
            return Math.Abs(angle) > GlobalSettings.STANDARD_TOLERANCE;
        }
    }
}
