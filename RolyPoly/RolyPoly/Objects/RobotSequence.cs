using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RolyPoly.Interfaces;

namespace RolyPoly.Objects
{
    /// <summary>
    /// The set of steps a robot will perform.
    /// </summary>
    internal class RobotSequence
    {
        /// <summary>
        /// The name of the robot that will execute this sequence.
        /// </summary>
        public string RobotName { get; set; }
        /// <summary>
        /// All steps the robot will carry-out, start to finish.
        /// </summary>
        public List<string> RobotSteps { get; set; }
    }
}
