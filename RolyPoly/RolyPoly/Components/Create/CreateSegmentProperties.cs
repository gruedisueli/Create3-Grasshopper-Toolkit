using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using RolyPoly.Interfaces;
using RolyPoly.Objects;

namespace RolyPoly.Components.Create
{
    public class CreateSegmentProperties : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the CreateSegmentProperties class.
        /// </summary>
        public CreateSegmentProperties()
          : base("CreateSegmentProperties", "CreateSegmentProperties",
              "Creates segment properties to override the default properties for a segment along the polycurve representing the robot path",
              "RolyPoly", "Create")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddIntegerParameter("Segment Index", "Segment Index", "The index of the segment of the polycurve these properties should override.", GH_ParamAccess.item);//0
            pManager.AddBooleanParameter("Is Drive Forwards (o)", "Is Drive Forwards (o)", "(optional) If true, robot will drive forwards along this segment. Default = true", GH_ParamAccess.item);//1
            pManager.AddGenericParameter("Actions At Start Point (o)", "Actions At Start Point (o)", "(optional) Actions to execute prior to this segment, if any, in the order that they should occur.", GH_ParamAccess.list);//1
            pManager.AddGenericParameter("Actions At End Point (o)", "Actions At End Point (o)", "(optional) Actions to execute after to this segment, if any, in the order that they should occur.", GH_ParamAccess.list);//2

            pManager[1].Optional = true;
            pManager[2].Optional = true;
            pManager[3].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.Register_GenericParam("Segment Properties", "Segment Properties", "Properties for this specific segment along the polycurve path");//0
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            int segmentIdx = 0;
            bool isForwards = true;
            var actionsAtStart = new List<IRobotAction>();
            var actionsAtEnd = new List<IRobotAction>();

            DA.GetData(0, ref segmentIdx);
            DA.GetData(1, ref isForwards);
            DA.GetDataList(2, actionsAtStart);
            DA.GetDataList(3, actionsAtEnd);

            var properties = new SegmentProperties(segmentIdx)
            {
                IsDriveForwards = isForwards,
                ActionsAtStartPoint = actionsAtStart,
                ActionsAtEndPoint = actionsAtEnd
            };

            DA.SetData(0, properties);
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
            get { return new Guid("94A714AC-3E2D-4B6A-B362-C0947EC3C1F2"); }
        }
    }
}