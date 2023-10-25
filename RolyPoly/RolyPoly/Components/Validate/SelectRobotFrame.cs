using System;
using System.Drawing;
using System.Collections.Generic;
using System.Linq;
using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Rhino;
using Rhino.Geometry;
using RolyPoly.Objects;
using RolyPoly.Settings;
using RolyPoly.Utilities;

namespace RolyPoly.Components.Validate
{
    public class SelectRobotFrame : GH_Component
    {
        private List<Point3d> _currentPositions = new List<Point3d>();
        private List<string> _currentActions = new List<string>();
        private Plane _drivingPlane = Plane.Unset;

        /// <summary>
        /// Initializes a new instance of the PreviewRobotFrame class.
        /// </summary>
        public SelectRobotFrame()
          : base("SelectRobotFrame", "SelectRobotFrame",
              "Selects a single robot frame from validation animation",
              "RolyPoly", "Validate")
        {
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            base.DrawViewportWires(args);

            for (int i = 0; i < _currentActions.Count; i++)
            {
                var tD = new TextDot(_currentActions[i], _currentPositions[i])
                {
                    FontHeight = 10
                };
                args.Display.DrawDot(tD, Color.White, Color.Black, Color.Black);
            }
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("Robot Frames", "Robot Frames", "Robot frames object", GH_ParamAccess.item);//0
            pManager.AddNumberParameter("Animation Position (0-1)", "Animation Position (0-1)", "Animation position for frame selection, a fractional value between 0 and 1", GH_ParamAccess.item);//1
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.Register_GeometryParam("Robot Geometry", "Robot Geometry", "Robot Geometry");//0
            pManager.Register_StringParam("Current Program Message", "Current Program Message", "The step and program line / message that this frame is being generated from");//1
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            RobotFrames frames = null;
            double animPos = 0;

            DA.GetData(0, ref frames);
            DA.GetData(1, ref animPos);

            _currentPositions.Clear();
            _currentActions.Clear();

            animPos = RhinoMath.Clamp(animPos, 0, 1);
            var firstRobot = frames.AllRobotFrames.Keys.First();
            int currentFrame = (int)Math.Floor((frames.AllRobotFrames[firstRobot].Count - 1) * animPos);
            _currentActions = UnpackFrameStep(frames, currentFrame);
            _drivingPlane = frames.AllRobotFrames[firstRobot][0].CurrentRobotData.RobotMovementPlane;//they should be the same, at least unless the user has done something unusual, in which case the preview will be a little off, but that's ok.

            //get positions for text dots. They should float over the robots.
            foreach (var framesList in frames.AllRobotFrames.Values)
            {
                var p = framesList[currentFrame].CurrentRobotData.RobotCenter + _drivingPlane.YAxis * GlobalSettings.ROBOT_RADIUS_METERS * GeometryUtils.GetDistanceScaleFactorToModelUnits() * 3;
                _currentPositions.Add(p);
            }

            DA.SetDataTree(0, UnpackFrameGeometry(frames, currentFrame));
            DA.SetDataList(1, _currentActions);
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
            get { return new Guid("C874CD9B-84D1-49A2-BA2C-9D68011F9351"); }
        }

        private DataTree<GeometryBase> UnpackFrameGeometry(RobotFrames frames, int frameIdx)
        {
            var tree = new DataTree<GeometryBase>();
            int i = 0;
            foreach(var frameList in frames.AllRobotFrames.Values)
            {
                var frame = frameList[frameIdx];
                var path = new GH_Path(i);
                tree.AddRange(frame.CurrentRobotData.RobotGeometry, path);
                tree.AddRange(frame.CurrentRobotData.AdditionalGeometry, path);
                tree.Add(new Rhino.Geometry.Point(frame.CurrentRobotData.RobotCenter), path);
                i++;
            }

            return tree;
        }

        private List<string> UnpackFrameStep(RobotFrames frames, int frameIdx)
        {
            var list = new List<string>();
            foreach (var kvp in frames.AllRobotFrames)
            {
                var frame = kvp.Value[frameIdx];
                var message = $"Robot: {kvp.Key}\nStep Index: {frame.CurrentSequenceStep}\n{frame.CurrentAction?.Trim()}";
                list.Add(message);
            }

            return list;
        }
    }
}