using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using RolyPoly.Objects;

namespace RolyPoly.Components.Create
{
    public class RobotSequencesToProgram : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the RobotSequencesToProgram class.
        /// </summary>
        public RobotSequencesToProgram()
          : base("RobotSequencesToProgram", "RobotSequencesToProgram",
              "Takes one or more robot sequences and combines them into a program that can be verified and copy/pasted into a python script for running on the robot",
              "RolyPoly", "Create")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Robot Sequences", "Robot Sequences", "List of robot sequences to combine into a program, where robots act in synchronized fashion from step to step", GH_ParamAccess.list);//0
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.Register_StringParam("Program", "Program", "Program that can be passed into verification components and also copy/pasted into a python script for running on robot");//0
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var sequences = new List<RobotSequence>();

            DA.GetDataList(0, sequences);

            int stepCt = sequences[0].RobotSteps.Count;
            for (int i = 1; i < sequences.Count; i++)
            {
                if (sequences[i].RobotSteps.Count == stepCt) continue;
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Error: input sequence lengths do not match. Based on the current tool framework, all sequences must have the same number of steps. Possible solution: insert 'Wait' actions at strategic positions in sequence by using the 'Segment Properties' input on 'PolycurveToRobotSequence'.");
                return;
            }

            var program = new List<string>();
            program.Add("moves = [");
            string stepHeader = "    SequenceStep([";
            string stepCloser = "]),";
            string msgPrefix = "        ";
            string msgCloser = ",";
            for (int step = 0; step < stepCt; step++)
            {
                program.Add(stepHeader);
                for (int seqIdx = 0; seqIdx < sequences.Count; seqIdx++)
                {
                    string suffix = seqIdx < sequences.Count - 1 ? msgCloser : stepCloser;
                    program.Add(msgPrefix + sequences[seqIdx].RobotSteps[step] + suffix);
                }
            }
            program.Add("]");

            DA.SetDataList(0, program);
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
            get { return new Guid("0EA8C9E2-85C7-49C4-9D37-9F8C616AB1CA"); }
        }
    }
}