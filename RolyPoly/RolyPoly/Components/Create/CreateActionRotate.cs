using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino;
using Rhino.Geometry;
using RolyPoly.Objects;
using RolyPoly.Settings;

namespace RolyPoly.Components.Create
{
    public class CreateActionRotate : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the CreateActionRotate class.
        /// </summary>
        public CreateActionRotate()
          : base("CreateActionRotate", "CreateActionRotate",
              "Create a rotate action",
              "RolyPoly", "Create")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddNumberParameter("Rotate Angle Degrees", "Rotate Angle Degrees", "Angle the robot should rotate during this action. Positive = counter-clockwise, Negative = clockwise", GH_ParamAccess.item);//0
            pManager.AddNumberParameter("Rotation Speed Degrees/Second (o)", "Rotation Speed Degrees/Second (o)", "(optional) Rotation speed in degrees/second. If you change this, refer to iRobot's documentation for acceptable ranges!", GH_ParamAccess.item);//1

            pManager[1].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.Register_GenericParam("Rotate Action", "Rotate Action", "Rotate Action");//0
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            double rotAngleDeg = 0;
            double rotSpeedDegreesPerSecond = 0;
            double rotSpeedRadiansPerSecond = GlobalSettings.ROBOT_ROTATION_SPEED_RADIANS_PER_SECOND;

            DA.GetData(0, ref rotAngleDeg);
            if (DA.GetData(1, ref rotSpeedDegreesPerSecond))
            {
                rotSpeedRadiansPerSecond = RhinoMath.ToRadians(rotSpeedDegreesPerSecond);
            }

            var action = new RobotActionRotate(RhinoMath.ToRadians(rotAngleDeg))
            {
                RotationSpeedRadiansPerSecond = rotSpeedRadiansPerSecond
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
            get { return new Guid("982EA334-1C8B-4248-95EB-2CA7F5AE333F"); }
        }
    }
}