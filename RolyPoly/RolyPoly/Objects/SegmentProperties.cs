using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RolyPoly.Interfaces;

namespace RolyPoly.Objects
{
    internal class SegmentProperties
    {
        /// <summary>
        /// Index of the segment on the polycurve these properties should correspond to.
        /// </summary>
        public int SegmentIndex { get; }
        /// <summary>
        /// If true, the robot will drive forwards along this segment. Otherwise it will drive backwards.
        /// </summary>
        public bool IsDriveForwards { get; set; } = true;
        /// <summary>
        /// Actions to execute prior to this segment, if any, in the order that they should occur.
        /// </summary>
        public List<IRobotAction> ActionsAtStartPoint { get; set; } = new List<IRobotAction>();
        /// <summary>
        /// Actions to execute after to this segment, if any, in the order that they should occur.
        /// </summary>
        public List<IRobotAction> ActionsAtEndPoint { get; set; } = new List<IRobotAction>();

        public SegmentProperties(int segmentIdx)
        {
            SegmentIndex = segmentIdx;
        }
    }
}
