using System;
using System.Collections.Generic;
using System.Linq;
using System.Drawing;
using Rhino.Display;
using Grasshopper.Kernel;
using Rhino.Geometry;
using RolyPoly.Interfaces;
using RolyPoly.Objects;
using RolyPoly.Settings;
using RolyPoly.Utilities;

namespace RolyPoly.Components.Create
{
    public class PolycurveToRobotSequence : GH_Component
    {
        /// <summary>
        /// Holds preview information for actions that occur at a specific location in the model.
        /// </summary>
        private class PositionActionsLabel
        {
            /// <summary>
            /// All the actions that occur at this position, in order.
            /// </summary>
            public List<string> AllActions { get; private set; } = new List<string>();
            /// <summary>
            /// World coordinates of these actions.
            /// </summary>
            public Point3d Position { get; }
            /// <summary>
            /// The starting index of this actions label, in terms of the overall step sequence.
            /// </summary>
            public int StartStepIndex { get; private set; }
            /// <summary>
            /// Text cached for previews.
            /// </summary>
            private string _tagText = "";
            /// <summary>
            /// True if actions have been added to this object
            /// </summary>
            private bool _initialized = false;

            public PositionActionsLabel(Point3d position)
            {
                Position = position;
            }

            public void SetStartStepIndex(int idx)
            {
                StartStepIndex = idx;
                UpdateTagText();
            }

            public void AddAction(string action)
            {
                _initialized = true;
                AllActions.Add(action);
                UpdateTagText();
            }

            public void Draw(IGH_PreviewArgs args)
            {
                if (!_initialized) return;
                var tD = new TextDot(_tagText, Position)
                {
                    FontHeight = 10
                };
                args.Display.DrawDot(tD, Color.White, Color.Black, Color.Black);
            }

            private void UpdateTagText()
            {
                _tagText = "";
                int stepIdx = StartStepIndex;
                foreach(var a in AllActions)
                {
                    _tagText += $"Step {stepIdx}: {a}\n";
                    stepIdx++;
                }
            }
        }

        private List<Tuple<Curve, bool>> _previewInputCrvs = new List<Tuple<Curve, bool>>();
        private List<PositionActionsLabel> _previewPositionActions = new List<PositionActionsLabel>();

        /// <summary>
        /// Initializes a new instance of the CreateRoombaSequence class.
        /// </summary>
        public PolycurveToRobotSequence()
          : base("PolycurveToRobotSequence", "PolycurveToRobotSequence",
              "Builds the initial list of actions for a single Roomba",
              "RolyPoly", "Create")
        {
        }

        public override void DrawViewportWires(IGH_PreviewArgs args)
        {
            foreach (var t in _previewInputCrvs)
            {
                var color = t.Item2 ? Color.White : Color.Red;
                args.Display.DrawCurve(t.Item1, color, 1);
            }
            foreach (var a in _previewPositionActions)
            {
                a.Draw(args);
            }
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("Polycurve Path", "Polycurve Path", "Single joined polycurve consisting of sub-segments that are either Lines or Arcs. No other curve types are supported", GH_ParamAccess.item);//0
            pManager.AddGenericParameter("Robot", "Robot", "Robot object created by CreateRobot", GH_ParamAccess.item);//1
            pManager.AddGenericParameter("Segment Properties (o)", "Segment Properties (o)", "(optional) List of modifications to specific segment properties of the path. Note that you should supply no more than one input for each segment of the polycurve", GH_ParamAccess.list);//2
            pManager.AddNumberParameter("Rotation Speed Between Segments Radians Per Sec (o)", "Rotation Speed Between Segments Radians Per Sec (o)", "(optional) Rotations are inserted between segments to align the robot position as needed prior to continuing. This speed governs how fast this occurs. If not specified the default is used. Consult iRobot documentation for correct ranges!", GH_ParamAccess.item);//3
            pManager.AddNumberParameter("Drive Speed Meters Per Second (o)", "Drive Speed Meters Per Second (o)", "(optional), How fast to drive. Consult iRobot's documentation for acceptable ranges!", GH_ParamAccess.item);//4


            pManager[2].Optional = true;
            pManager[3].Optional = true;
            pManager[4].Optional = true;
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
            Curve inputPath = null;
            Robot robot = null;
            var segmentModifications = new List<SegmentProperties>();
            double rotSpd = GlobalSettings.ROBOT_ROTATION_SPEED_RADIANS_PER_SECOND;
            double driveSpd = GlobalSettings.ROBOT_DRIVE_SPEED_METERS_PER_SECOND;

            _previewInputCrvs.Clear();
            _previewPositionActions.Clear();

            DA.GetData(0, ref inputPath);
            DA.GetData(1, ref robot);
            DA.GetDataList(2, segmentModifications);
            DA.GetData(3, ref rotSpd);
            DA.GetData(4, ref driveSpd);

            List<Curve> inputCrvSegments;
            if (inputPath is PolyCurve polyCrv)
            {
                inputCrvSegments = ExplodePolyCurve(polyCrv);
            }
            else if (inputPath is PolylineCurve pLCrv)
            {
                inputCrvSegments = pLCrv.DuplicateSegments().ToList();
            }
            else
            {
                inputCrvSegments = new List<Curve>() { inputPath };
            }

            var segmentActions = new List<IRobotAction>();
            bool isError = false;
            var tagPoints = new List<Point3d>();
            var segmentPositionActions = new List<PositionActionsLabel>();
            for (int i = 0; i < inputCrvSegments.Count; i++)
            {
                var c = inputCrvSegments[i];
                Point3d midPt, startPt;
                string error = "";
                //all actions are "forwards" unless overridden above.
                if (c.TryGetArc(out var a))
                {
                    if (!GeometryUtils.IsArcCoplanar(a, robot.RobotMovementPlane))
                    {
                        AddRuntimeMessage(GH_RuntimeMessageLevel.Error, $"Arc sub-segment of input polycurve at segment {i} is not coplanar to robot movement plane");
                        isError = true;
                        error = ": NOT COPLANAR";
                    }

                    var action = new RobotActionDriveArc(a, true, robot.RobotMovementPlane)
                    {
                        DriveSpeedMetersPerSecond = driveSpd
                    };
                    segmentActions.Add(action);
                    midPt = a.MidPoint;
                    startPt = a.StartPoint;
                }
                else if (c.IsLinear(GlobalSettings.STANDARD_TOLERANCE))
                {
                    var line = new Line(c.PointAtStart, c.PointAtEnd);
                    if (!GeometryUtils.IsLineCoplanar(line, robot.RobotMovementPlane))
                    {
                        AddRuntimeMessage(GH_RuntimeMessageLevel.Error, $"Line sub-segment of input polycurve at segment {i} is not coplanar to robot movement plane");
                        isError = true;
                        error = ": NOT COPLANAR";
                    }

                    var action = new RobotActionDriveStraight(line, true)
                    {
                        DriveSpeedMetersPerSecond = driveSpd
                    };
                    segmentActions.Add(action);
                    midPt = line.PointAt(0.5);
                    startPt = line.From;
                }
                else
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Subcurve of the polycurve is neither of Arc type nor Line type. All subcurves must be one of those two types of objects.");
                    isError = true;
                    error = ": NOT LINE OR ARC";
                    midPt = c.PointAtNormalizedLength(0.5);
                    startPt = c.PointAtStart;
                }

                tagPoints.Add(startPt);
                _previewInputCrvs.Add(Tuple.Create(c, error == ""));
                var label = new PositionActionsLabel(midPt);
                label.AddAction("Segment: " + i + error);
                segmentPositionActions.Add(label);
            }
            tagPoints.Add(inputPath.PointAtEnd);

            if (isError) return;

            segmentModifications = segmentModifications.OrderByDescending(m => m.SegmentIndex).ToList();

            var modifiedSegments = new List<int>();
            foreach (var m in segmentModifications)
            {
                if (modifiedSegments.Contains(m.SegmentIndex))
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Two inputs are modifying the same curve segment. Only one set of properties is allowed per segment");
                    return;
                }

                if (m.SegmentIndex >= inputCrvSegments.Count)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Segment index in an input property is outside the range of indices of polycurve subsegments. Check your inputs.");
                    return;
                }

                var segment = segmentActions[m.SegmentIndex];

                if (!m.IsDriveForwards)
                {
                    IRobotAction action;
                    if (segment is RobotActionDriveStraight dS)
                    {
                        action = new RobotActionDriveStraight(dS.LineSegment, false)
                        {
                            DriveSpeedMetersPerSecond = dS.DriveSpeedMetersPerSecond
                        };
                    }
                    else if (segment is RobotActionDriveArc dA)
                    {
                        action = new RobotActionDriveArc(dA.ArcSegment, false, robot.RobotMovementPlane)
                        {
                            DriveSpeedMetersPerSecond = dA.DriveSpeedMetersPerSecond
                        };
                    }
                    else
                    {
                        AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Internal Error: segment action list contains unexpected action type");
                        return;
                    }
                    segmentActions[m.SegmentIndex] = action;
                }

                if (m.SegmentIndex == segmentActions.Count - 1)
                {
                    segmentActions.AddRange(m.ActionsAtEndPoint);
                }
                else
                {
                    segmentActions.InsertRange(m.SegmentIndex + 1, m.ActionsAtEndPoint);
                }
                segmentActions.InsertRange(m.SegmentIndex, m.ActionsAtStartPoint);

                modifiedSegments.Add(m.SegmentIndex);
            }

            var finalActions = ActionUtils.InsertRotationsBetweenSegments(segmentActions, robot.RobotDirection, rotSpd, robot.RobotMovementPlane);
            ActionUtils.UpdateActionsWithRobotName(ref finalActions, robot.RobotName);

            var dbgMessages = new List<string>();
            foreach (var a in finalActions)
            {
                dbgMessages.AddRange(a.GetCommandStrings());
            }
            var sequence = new RobotSequence()
            {
                RobotName = robot.RobotName,
                RobotSteps = dbgMessages
            };

            int currentSegmentIdx = 0;
            int currentTagIdx = 0;
            int currentActionIdx = 0;
            _previewPositionActions = new List<PositionActionsLabel>() { new PositionActionsLabel(tagPoints[currentSegmentIdx]) };
            _previewPositionActions[0].SetStartStepIndex(0);

            foreach(var a in finalActions)
            {
                if (a is IRobotActionSimple)
                {
                    var previewAction = _previewPositionActions[currentTagIdx];
                    if (a is RobotActionWait aW)
                    {
                        for (int w = 0; w < aW.WaitCount; w++)
                        {
                            currentActionIdx++;
                            previewAction.AddAction("Wait");
                        }
                    }
                    else if (a is RobotActionRotate aR)
                    {
                        currentActionIdx++;
                        previewAction.AddAction($"Rotate: {StringUtils.GetFormattedDoubleString(aR.RotationAngleRadians)} radians");
                    }
                    else if (a is RobotActionPlaySong aS)
                    {
                        currentActionIdx++;
                        previewAction.AddAction("Play Song");
                    }
                    else
                    {
                        AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Internal Error: unrecognized simple action type being unpacked for previews");
                    }
                }
                else
                {
                    _previewPositionActions.Add(segmentPositionActions[currentSegmentIdx]);
                    _previewPositionActions.Last().SetStartStepIndex(currentActionIdx);
                    currentSegmentIdx++;
                    currentTagIdx++;
                    if (currentSegmentIdx < tagPoints.Count)
                    {
                        _previewPositionActions.Add(new PositionActionsLabel(tagPoints[currentSegmentIdx]));
                        _previewPositionActions.Last().SetStartStepIndex(currentActionIdx + 1);
                        currentTagIdx++;
                    }
                    currentActionIdx++;
                }
            }

            DA.SetData(0, sequence);
            DA.SetDataList(1, dbgMessages);

        }

        /// <summary>
        /// Provides an Icon for the component.
        /// </summary>
        protected override Bitmap Icon
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
            get { return new Guid("27B31EA8-D0BA-4E55-A5F3-FA9D029447AB"); }
        }

        /// <summary>
        /// Explodes a polycurve into subcurves, especially with regards to exploding polylines within it.
        /// Polycurve segments can include entire polylines that need to be exploded.
        /// </summary>
        private List<Curve> ExplodePolyCurve(PolyCurve polyCrv)
        {
            var segments = new List<Curve>();
            for (int i = 0; i < polyCrv.SegmentCount; i++)
            {
                var c = polyCrv.SegmentCurve(i);
                if (c.TryGetPolyline(out var pL))
                {
                    for (int vIdx = 0; vIdx < pL.Count - 1; vIdx++)
                    {
                        segments.Add(new Line(pL[vIdx], pL[vIdx + 1]).ToNurbsCurve());
                    }
                }
                else segments.Add(c);
            }

            return segments;
        }
    }
}