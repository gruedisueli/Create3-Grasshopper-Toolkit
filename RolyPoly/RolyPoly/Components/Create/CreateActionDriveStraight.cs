using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using RolyPoly.Objects;
using RolyPoly.Settings;

namespace RolyPoly.Components.Create
{
    public class CreateActionDriveStraight : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the CreateActionDriveStraight class.
        /// </summary>
        public CreateActionDriveStraight()
          : base("CreateActionDriveStraight", "CreateActionDriveStraight",
              "Creates a robot action to drive straight forwards or backwards",
              "RolyPoly", "Create")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddLineParameter("Line Segment", "Line Segment", "Line Segment Representing Path of Robot in Rhino Space", GH_ParamAccess.item);//0
            pManager.AddBooleanParameter("Is Forwards (o)", "Is Forwards (o)", "(optional) If false, robot drives backwards along segment", GH_ParamAccess.item);//1
            pManager.AddNumberParameter("Drive Speed Meters Per Second (o)", "Drive Speed Meters Per Second (o)", "(optional), How fast to drive. Consult iRobot's documentation for acceptable ranges!", GH_ParamAccess.item);//2

            pManager[1].Optional = true;
            pManager[2].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.Register_GenericParam("Drive Straight Action", "Drive Straight Action", "Drive Straight Action");//0
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var segment = Line.Unset;
            bool isForwards = true;
            double driveSpeed = GlobalSettings.ROBOT_DRIVE_SPEED_METERS_PER_SECOND;

            DA.GetData(0, ref segment);
            DA.GetData(1, ref isForwards);
            DA.GetData(2, ref driveSpeed);

            var action = new RobotActionDriveStraight(segment, isForwards)
            {
                DriveSpeedMetersPerSecond = driveSpeed
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
            get { return new Guid("3FDE0B0E-A3A6-461D-9B98-4AFA300D1445"); }
        }
    }
}