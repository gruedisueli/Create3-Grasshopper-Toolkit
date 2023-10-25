using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RolyPoly.Objects
{
    /// <summary>
    /// Contains a collection of robot frames for passing between Grasshopper components
    /// </summary>
    internal class RobotFrames
    {
        public Dictionary<string, List<RobotFrame>> AllRobotFrames { get; set; }
    }
}
