using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino;
using Rhino.Geometry;
using RolyPoly.Settings;

namespace RolyPoly.Utilities
{
    internal static class GeometryUtils
    {
        /// <summary>
        /// Builds a simple arrowhead.
        /// </summary>
        public static List<GeometryBase> BuildArrow(Plane plane, Point3d startPt, Vector3d direction, double length, double headLegLength, double headAngleRadians)
        {
            Line tail = new Line(startPt, direction, length);
            Curve arrowEdge = new Line(tail.To, tail.Direction, headLegLength).ToNurbsCurve();
            Transform rotA = Transform.Rotation(headAngleRadians, plane.ZAxis, tail.To);
            Transform rotB = Transform.Rotation(headAngleRadians * -1, plane.ZAxis, tail.To);
            Curve eA = arrowEdge.DuplicateCurve();
            Curve eB = arrowEdge.DuplicateCurve();
            eA.Transform(rotA);
            eB.Transform(rotB);
            return new List<GeometryBase>() { tail.ToNurbsCurve(), eA, eB };
        }

        /// <summary>
        /// Gets the scale factor from Roomba units (meters) to the model unit system that the file is set to.
        /// </summary>
        public static double GetDistanceScaleFactorToModelUnits()
        {
            return RhinoMath.UnitScale(UnitSystem.Meters, RhinoDoc.ActiveDoc.ModelUnitSystem);
        }

        /// <summary>
        /// Gets the scale factor from model units to units used by Roomba (meters)
        /// </summary>
        public static double GetDistanceScaleFactorToRoombaUnits()
        {
            return RhinoMath.UnitScale(RhinoDoc.ActiveDoc.ModelUnitSystem, UnitSystem.Meters);
        }

        /// <summary>
        /// Returns true if line is within the plane, given standard tolerances.
        /// </summary>
        public static bool IsLineCoplanar(Line line, Plane pln)
        {
            return pln.DistanceTo(line.From) < GlobalSettings.STANDARD_TOLERANCE &&
                    pln.DistanceTo(line.To) < GlobalSettings.STANDARD_TOLERANCE;
        }

        /// <summary>
        /// Returns true if arc is within the plane, given standard tolerances.
        /// </summary>
        public static bool IsArcCoplanar(Arc arc, Plane pln)
        {
            var arcPln = arc.Plane;
            var refZ = arcPln.ZAxis;
            var a = Vector3d.VectorAngle(pln.ZAxis, refZ);
            if (Vector3d.VectorAngle(pln.ZAxis, refZ) > Math.PI / 2)
            {
                refZ.Reverse();
            }

            var angle = Vector3d.VectorAngle(pln.ZAxis, refZ);
            return angle < GlobalSettings.STANDARD_TOLERANCE &&
                   pln.DistanceTo(arcPln.Origin) < GlobalSettings.STANDARD_TOLERANCE;
        }
    }
}
