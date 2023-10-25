using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using RolyPoly.Objects;
using System.Linq;

namespace RolyPoly.Components.Create
{
    public class CreateActionPlaySong : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the CreateActionPlaySong class.
        /// </summary>
        public CreateActionPlaySong()
          : base("CreateActionPlaySong", "CreateActionPlaySong",
              "Creates a song that the robot can play",
              "RolyPoly", "Create")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddNumberParameter("Note Durations Seconds", "Note Durations Seconds", "For each note in the song, how long it should be, in seconds", GH_ParamAccess.list);//0
            pManager.AddIntegerParameter("Note Frequencies Hertz", "Note Frequencies Hertz", "(integers) For each note in the song, its frequency, in hertz", GH_ParamAccess.list);//1
        }

        /// <summary>A
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.Register_GenericParam("Play Song Action", "Play` Song Action", "Play Song Action");//0
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var durationsSeconds = new List<double>();
            var frequencies = new List<int>();

            DA.GetDataList(0, durationsSeconds);
            DA.GetDataList(1, frequencies);

            if (durationsSeconds.Count != frequencies.Count)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "There must be an equal number of durations and frequencies, one pair of them for each note");
                return;
            }

            if (durationsSeconds.Count == 0 || frequencies.Count == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Lists must have at least one value in them");
                return;
            }

            var durationsNanoSecs = (from d in durationsSeconds select (int)Math.Floor(d * 1000000000)).ToList();

            DA.SetData(0, new RobotActionPlaySong(durationsNanoSecs, frequencies));
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
            get { return new Guid("C49C55E0-5324-40B8-9159-64F5D7C988D1"); }
        }
    }
}