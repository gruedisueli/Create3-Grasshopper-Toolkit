using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using RolyPoly.Interfaces;
using RolyPoly.Objects;
using RolyPoly.Utilities;

namespace RolyPoly.Components.Create
{
    public class ActionsToRobotSequence : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public ActionsToRobotSequence()
          : base("ActionsToRobotSequence", "ActionsToRobotSequence",
              "Takes a basic list of robot actions and turns it into a sequence.",
              "RolyPoly", "Create")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Actions", "Actions", "List of actions to combine into sequence", GH_ParamAccess.list);//0
            pManager.AddGenericParameter("Robot", "Robot", "Robot that will be performing these actions", GH_ParamAccess.item);//1
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.Register_GenericParam("Robot Sequence", "Robot Sequence", "Sequence of steps for this robot");
            pManager.Register_StringParam("Debug Actions", "Debug Actions", "List of all steps the robot will perform, for debugging purposes only");
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var actionsList = new List<IRobotAction>();
            Robot robot = null;

            DA.GetDataList(0, actionsList);
            DA.GetData(1, ref robot);

            ActionUtils.UpdateActionsWithRobotName(ref actionsList, robot.RobotName);

            var dbgMessages = new List<string>();
            foreach (var a in actionsList)
            {
                dbgMessages.AddRange(a.GetCommandStrings());
            }
            var sequence = new RobotSequence()
            {
                RobotName = robot.RobotName,
                RobotSteps = dbgMessages
            };

            DA.SetData(0, sequence);
            DA.SetDataList(1, dbgMessages);
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
            get { return new Guid("407377B0-FA48-44D3-8028-F1A52FF2C82C"); }
        }
    }
}