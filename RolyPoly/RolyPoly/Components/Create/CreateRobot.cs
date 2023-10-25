using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using RolyPoly.Objects;
using RolyPoly.Settings;
using RolyPoly.Utilities;

namespace RolyPoly.Components.Create
{
    public class CreateRobot : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the CreateRoomba class.
        /// </summary>
        public CreateRobot()
          : base("CreateRobot", "CreateRobot",
              "Makes the Robot object for generating program and simulation",
              "RolyPoly", "Create")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddLineParameter("Robot Direction Line", "Robot Direction Line", "Initial direction line from the center of the robot in the forward direction of the robot, before robot has started moving", GH_ParamAccess.item);//0
            pManager.AddTextParameter("Robot Name", "Robot Name", " The namespace of the robot, per its configuration settings (not defined in Grasshopper, the namespace is set via the robot's web interface)", GH_ParamAccess.item);//1
            pManager.AddGeometryParameter("Additional Geometry (o)", "Additional Geometry (o)", "(optional) Any additional geometry that is attached to the robot that you would like to simulate the movement of", GH_ParamAccess.list);//2

            pManager[2].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.Register_GenericParam("Robot", "Robot", "Robot object for use in program generation and simulation");
            pManager.Register_GeometryParam("Debug Geometry", "Debug Geometry", "Geometry for debug purposes only");
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var dirLine = Line.Unset;
            string robotName = "";
            var additionalGeom = new List<GeometryBase>();

            DA.GetData(0, ref dirLine);
            DA.GetData(1, ref robotName);
            DA.GetDataList(2, additionalGeom);

            var drivePln = GlobalSettings.ROBOT_DEFAULT_DRIVE_PLANE;

            //check that line is within the plane.
            if (!GeometryUtils.IsLineCoplanar(dirLine, drivePln))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Provided direction line is not coplanar with robot movement plane");
                return;
            }

            var roomba = new Robot(robotName, drivePln, dirLine, additionalGeom);
            var dbgGeom = new List<GeometryBase>();
            dbgGeom.AddRange(roomba.RobotGeometry);
            dbgGeom.AddRange(roomba.AdditionalGeometry);

            DA.SetData(0, roomba);
            DA.SetDataList(1, dbgGeom);
        }

        /// <summary>
        /// Provides an Icon for the component.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                //You can add image files to your project resources and access them like this:
                // return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("D9B371F2-875B-41AE-B88A-C896F1C417D1"); }
        }
    }
}