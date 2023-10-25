using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RolyPoly.Interfaces
{
    /// <summary>
    /// Robot actions that don't have curve paths associated with them.
    /// </summary>
    internal interface IRobotActionSimple : IRobotAction
    {
        void SetStartAndEndDirections(Vector3d startDir);
    }
}
