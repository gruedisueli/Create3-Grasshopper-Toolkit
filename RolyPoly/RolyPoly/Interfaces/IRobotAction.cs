using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RolyPoly.Interfaces
{
    /// <summary>
    /// Standard interface that robot actions inherit from.
    /// </summary>
    internal interface IRobotAction
    {
        /// <summary>
        /// Name of the robot this command will be sent to.
        /// </summary>
        string RobotName { get; set; }
        /// <summary>
        /// Direction robot should face at start of action.
        /// </summary>
        Vector3d StartRobotDir { get; }
        /// <summary>
        /// Direction robot should face at end of action.
        /// </summary>
        Vector3d EndRobotDir { get; }
        /// <summary>
        /// Returns the command(s) we will add to the python script we use to send messages to the robot. This is our own custom syntax, not specific to iRobot.
        /// </summary>
        string[] GetCommandStrings();
    }
}
