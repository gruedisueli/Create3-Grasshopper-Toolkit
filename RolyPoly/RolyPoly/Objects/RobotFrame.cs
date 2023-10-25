using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RolyPoly.Objects
{
    /// <summary>
    /// Holds information about a specific frame of a robot validation animation
    /// </summary>
    internal class RobotFrame
    {
        /// <summary>
        /// The data of the robot in its current frame.
        /// </summary>
        public Robot CurrentRobotData { get; set; }
        /// <summary>
        /// The step of the sequence this corresponds to.
        /// </summary>
        public int CurrentSequenceStep { get; set; }
        /// <summary>
        /// The action the robot is currently completing.
        /// </summary>
        public string CurrentAction { get; set; }

        /// <summary>
        /// Creates a copy of a frame.
        /// </summary>
        public RobotFrame(RobotFrame frame)
        {
            CurrentRobotData = new Robot(frame.CurrentRobotData);
            CurrentSequenceStep = frame.CurrentSequenceStep;
            CurrentAction = frame.CurrentAction;
        }

        public RobotFrame()
        {

        }
    }
}
