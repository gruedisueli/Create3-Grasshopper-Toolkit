using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;
using RolyPoly.Interfaces;

namespace RolyPoly.Objects
{
    /// <summary>
    /// A wait action
    /// </summary>
    internal class RobotActionWait : IRobotActionSimple
    {
        public string RobotName { get; set; } = "undefined";
        /// <summary>
        /// The number of steps to pause for during this wait.
        /// </summary>
        public int WaitCount { get; }
        public Vector3d StartRobotDir { get; private set; } = Vector3d.Unset;
        public Vector3d EndRobotDir { get; private set; } = Vector3d.Unset;

        public RobotActionWait(int waitCt)
        {
            WaitCount = waitCt;
        }

        /// <summary>
        /// Sets the start direction and end directions for this step.
        /// </summary>
        public void SetStartAndEndDirections(Vector3d startDir)
        {
            StartRobotDir = startDir;
            EndRobotDir = startDir;
        }

        public string[] GetCommandStrings()
        {
            var m = new string[WaitCount];
            for (int i = 0; i < WaitCount; i++)
            {
                m[i] = $"MessageWait(\"{RobotName}\")";
            }

            return m;
        }
    }
}
