using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using RolyPoly.Objects;
using RolyPoly.Settings;

namespace RolyPoly.Components.Create
{
    public class CreateActionDriveArc : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the CreateActionDriveArc class.
        /// </summary>
        public CreateActionDriveArc()
          : base("CreateActionDriveArc", "CreateActionDriveArc",
              "Creates a drive arc action",
              "RolyPoly", "Create")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddArcParameter("Arc Segment", "Arc Segment", "Arc that the robot should drive along", GH_ParamAccess.item);//0
            pManager.AddBooleanParameter("Is Forwards (o)", "Is Forwards (o)", "(optional) If false, robot will drive backwards along the arc", GH_ParamAccess.item);//1
            pManager.AddNumberParameter("Drive Speed Meters Per Second (o)", "Drive Speed Meters Per Second (o)", "(optional), How fast to drive. Consult iRobot's documentation for acceptable ranges!", GH_ParamAccess.item);//2

            pManager[1].Optional = true;
            pManager[2].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.Register_GenericParam("Drive Arc Action", "Drive Arc Action", "Drive Arc Action");//0
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var arcSegment = Arc.Unset;
            bool isForwards = true;
            var driveSpd = GlobalSettings.ROBOT_DRIVE_SPEED_METERS_PER_SECOND;

            DA.GetData(0, ref arcSegment);
            DA.GetData(1, ref isForwards);
            DA.GetData(2, ref driveSpd);

            var action = new RobotActionDriveArc(arcSegment, isForwards, GlobalSettings.ROBOT_DEFAULT_DRIVE_PLANE)
            {
                DriveSpeedMetersPerSecond = driveSpd
            };

            DA.SetData(0, action);
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
            get { return new Guid("64A08833-B05F-4C4D-B1C9-EE2FDAABEAC4"); }
        }
    }
}