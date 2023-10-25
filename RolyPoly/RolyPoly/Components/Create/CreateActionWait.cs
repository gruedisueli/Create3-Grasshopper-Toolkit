using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using RolyPoly.Objects;

namespace RolyPoly.Components.Create
{
    public class CreateActionWait : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the CreateActionWait class.
        /// </summary>
        public CreateActionWait()
          : base("CreateActionWait", "CreateActionWait",
              "Creates a wait action",
              "RolyPoly", "Create")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddIntegerParameter("Wait Count", "Wait Count", "Number of steps the robot should pause for during this action", GH_ParamAccess.item);//0
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.Register_GenericParam("Wait Action", "Wait Action", "Wait Action");//0
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            int waitCt = 0;

            DA.GetData(0, ref waitCt);

            if (waitCt <= 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "The wait count must be greater than 0");
                return;
            }

            DA.SetData(0, new RobotActionWait(waitCt));
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
            get { return new Guid("289B3E80-9272-407D-8496-635B77A9241F"); }
        }
    }
}