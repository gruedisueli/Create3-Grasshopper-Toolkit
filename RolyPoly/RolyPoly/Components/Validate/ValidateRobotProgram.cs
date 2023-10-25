using System;
using System.Collections.Generic;
using System.Linq;
using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Rhino;
using Rhino.Geometry;
using RolyPoly.Objects;
using RolyPoly.Settings;

namespace RolyPoly.Components.Validate
{
    public class ValidateRobotProgram : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the ValidateRobotProgram class.
        /// </summary>
        public ValidateRobotProgram()
          : base("ValidateRobotProgram", "ValidateRobotProgram",
              "Takes a string of text based on our custom syntax and simulates robot movements from it. Use this tool to validate whether your program works as intended prior to copying it into a python script and sending it to the robot. Note that the simulation it provides is approximate and does not take into account the acceleration curves of the robot. Use the simulation to get a general idea of robot behavior",
              "RolyPoly", "Validate")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("Program", "Program", "Program to validate. Note: it must be of type string, from a Grasshopper 'panel' set to 'Multiline data'", GH_ParamAccess.list);//0
            pManager.AddGenericParameter("Robot List", "Robot List", "List of all the robots", GH_ParamAccess.list);//1
            pManager.AddNumberParameter("Time increment seconds (0.05 to 0.5 value) (o)", "Time increment seconds (0 to 0.5 value) (o)", "(optional) Time increment that is used to sample robot position throughout simulation. Value must be between 0.05 and 0.5 seconds.", GH_ParamAccess.item);//2
            pManager[2].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.Register_GenericParam("Robot Frames", "Robot Frames", "Object containing all robot frames, to be unpacked by a separate tool.");//0
            pManager.Register_GeometryParam("Dbg All Frames Geometry", "Dbg All Frames Geometry", "(Debug) All the frames of the simulation. You can use our frame selector to get a single one by passing the 'Robot Frames' output into it");
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var program = new List<string>();
            var robots = new List<Robot>();
            double timeInc = GlobalSettings.ANIMATION_TIME_INC_SECONDS;

            DA.GetDataList(0, program);
            DA.GetDataList(1, robots);
            DA.GetData(2, ref timeInc);

            timeInc = RhinoMath.Clamp(timeInc, 0.05, 0.5);

            var robotDict = new Dictionary<string, Robot>();
            foreach (var r in robots)
            {
                robotDict.Add(r.RobotName, r);
            }

            var animationFrames = new Dictionary<string, List<RobotFrame>>();

            foreach (var kvp in robotDict)
            {
                var firstFrame = new RobotFrame()
                {
                    CurrentRobotData = new Robot(kvp.Value)
                };
                animationFrames.Add(kvp.Key, new List<RobotFrame>(){firstFrame});
            }

            //int robotIdx = -1;
            int stepCt = -1;
            foreach (var line in program)
            {

                bool isStraight = false;
                bool isRotate = false;
                bool isArc = false;
                bool isWait = false;
                bool isPlaySong = false;
                if (line.Contains("Step"))
                {
                   // robotIdx = 0;
                    stepCt++;
                    if (stepCt > 0)//equalize list lengths so that robots that had shorter steps wait for others to finish
                    {
                        EqualizeListLengths(ref animationFrames);
                    }
                    continue;
                }
                
                if (line.Contains("Wait"))
                {
                    isWait = true;
                }
                else if (line.Contains("Straight"))
                {
                    isStraight = true;
                }
                else if (line.Contains("Rotate"))
                {
                    isRotate = true;
                }
                else if (line.Contains("Arc"))
                {
                    isArc = true;
                }
                else if (line.Contains("PlaySong"))
                {
                    isPlaySong = true;
                }
                else
                {
                    continue;
                }

                var args = GetArgs(line);
                if (args == null || args.Count == 0)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Failed to extract any arguments from message");
                    return;
                }
                string robotName = args[0];
                robotName = robotName.Trim('"');
                if (!animationFrames.ContainsKey(robotName))
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Unrecognized robot name in message");
                    return;
                }
                var robotFrameList = animationFrames[robotName];
                var currPos = robotFrameList.Last();

                if (isWait)
                {
                    var newPos = new RobotFrame(currPos);
                    newPos.CurrentSequenceStep = stepCt;
                    newPos.CurrentAction = "Wait";
                    robotFrameList.Add(newPos);
                }
                else if (isPlaySong)
                {
                    var newPos = new RobotFrame(currPos);
                    newPos.CurrentSequenceStep = stepCt;
                    newPos.CurrentAction = "Play Song";
                    robotFrameList.Add(newPos);
                }
                else if (isStraight)
                {
                    robotFrameList.AddRange(GetStraightPositions(currPos, args, timeInc, stepCt, line));
                }
                else if (isRotate)
                {
                    robotFrameList.AddRange(GetRotationPositions(currPos, args, timeInc, stepCt, line));
                }
                else if (isArc)
                {
                    robotFrameList.AddRange(GetArcPositions(currPos, args, timeInc, stepCt, line));
                }

                //robotIdx++;
            }

            EqualizeListLengths(ref animationFrames);

            var robotFramesOutput = new RobotFrames()
            {
                AllRobotFrames = animationFrames
            };

            DA.SetData(0, robotFramesOutput);
            DA.SetDataTree(1, GetAllFramesGeom(animationFrames));
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
            get { return new Guid("033B6358-2387-4AAF-BEFB-C42DA07D5BC7"); }
        }


        /// <summary>
        /// Ensures that all lists are the same end length by padding out shorter lists with copied data.
        /// </summary>
        private void EqualizeListLengths(ref Dictionary<string, List<RobotFrame>> robotFramesDict)
        {
            var listLengths = new List<Tuple<string, int>>();
            foreach(var kvp in robotFramesDict)
            {
                listLengths.Add(Tuple.Create(kvp.Key, kvp.Value.Count));
            }

            listLengths = listLengths.OrderByDescending(list => list.Item2).ToList();
            int longest = listLengths[0].Item2;
            for (int i = 1; i < listLengths.Count; i++)
            {
                string listName = listLengths[i].Item1;
                var robotList = robotFramesDict[listName];
                PadList(ref robotList, longest);
            }
        }

        /// <summary>
        /// Pads a list with duplicate items at end until list is desired length
        /// </summary>
        private void PadList(ref List<RobotFrame> positions, int desiredLength)
        {
            var last = positions[positions.Count - 1];
            for (int i = positions.Count; i < desiredLength; i++)
            {
                positions.Add(new RobotFrame(last));
            }
        }

        /// <summary>
        /// Gets a list of positions along a straight path. Null on failure.
        /// </summary>
        private List<RobotFrame> GetStraightPositions(RobotFrame startingPos, List<string> msgArgs, double timeInc, int stepIdx, string stepMsg)
        {
            if (msgArgs.Count != 3)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Error parsing straight message");
                return null;
            }
            double d;//distance in meters
            double v;//max velocity in meters per second (varies a bit in real-life)
            if (!TryParseDouble(msgArgs[1], out d) || !TryParseDouble(msgArgs[2], out v))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Unable to parse straight message parameters");
                return null;
            }
            double t = Math.Abs(d / v);//time to complete operation.
            int steps = GetStepCt(t, timeInc);
            double distInc = d / (double)steps;

            var positions = new List<RobotFrame>();
            var currentPos = new RobotFrame(startingPos);
            currentPos.CurrentSequenceStep = stepIdx;
            currentPos.CurrentAction = stepMsg;
            for (int i = 0; i < steps; i++)
            {
                var p = new RobotFrame(currentPos);
                p.CurrentRobotData.DriveForward(distInc);
                positions.Add(p);
                currentPos = p;
            }

            return positions;
        }

        /// <summary>
        /// Gets positions within a rotational action.
        /// </summary>
        private List<RobotFrame> GetRotationPositions(RobotFrame startingPos, List<string> msgArgs, double timeInc, int stepIdx, string stepMsg)
        {
            if (msgArgs.Count != 3)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Error parsing rotate message");
                return null;
            }
            double a;//angle to rotate in radians
            double v;//max angular speed radians per second
            if (!TryParseDouble(msgArgs[1], out a) || !TryParseDouble(msgArgs[2], out v))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Error parsing rotate message parameters");
                return null;
            }
            double t = Math.Abs(a / v);//time to complete operation
            int steps = GetStepCt(t, timeInc);
            double angInc = a / (double)steps;

            var positions = new List<RobotFrame>();
            var currentPos = new RobotFrame(startingPos);
            currentPos.CurrentSequenceStep = stepIdx;
            currentPos.CurrentAction = stepMsg;
            for (int i = 0; i < steps; i++)
            {
                var p = new RobotFrame(currentPos);
                p.CurrentRobotData.Rotate(angInc);
                positions.Add(p);
                currentPos = p;
            }

            return positions;
        }

        /// <summary>
        /// Gets positions within an arc action.
        /// </summary>
        private List<RobotFrame> GetArcPositions(RobotFrame startingPos, List<string> msgArgs, double timeInc, int stepIdx, string stepMsg)
        {
            if (msgArgs.Count != 5)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Error parsing arc message");
                return null;
            }
            double a;//angle of arc in radians
            double r;//radius of arc in meters
            double v;//max velocity in meters per second (varies a bit in real-life)
            if (!TryParseDouble(msgArgs[1], out a) || !TryParseDouble(msgArgs[2], out r) || !TryParseDouble(msgArgs[3], out v))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Error parsing rotate message arguments");
                return null;
            }
            double arcLen = (2 * Math.PI * r) * (a / (2 * Math.PI));
            double t = Math.Abs(arcLen / v);//time to complete operation
            bool isForward = bool.Parse(msgArgs[4]);
            int steps = GetStepCt(t, timeInc);
            double angInc = a / (double)steps;

            var positions = new List<RobotFrame>();
            var currentPos = new RobotFrame(startingPos);
            currentPos.CurrentSequenceStep = stepIdx;
            currentPos.CurrentAction = stepMsg;
            for (int i = 0; i < steps; i++)
            {
                var p = new RobotFrame(currentPos);
                p.CurrentRobotData.DriveArc(angInc, r, isForward);
                positions.Add(p);
                currentPos = p;
            }

            return positions;
        }

        /// <summary>
        /// Gets the arguments from message.
        /// </summary>
        private List<string> GetArgs(string msg)
        {
            var prelimArgs = msg.Split(new char[] { '(', '[', ']', ',', ')', ' ' });
            List<string> args = new List<string>();
            foreach (string a in prelimArgs)
            {
                if (a == "" || a.Length == 0 || a.Contains("Message")) continue;
                args.Add(a);
            }
            return args;
        }

        /// <summary>
        /// Gets number of steps to complete current operation given the time increment (approx)
        /// </summary>
        private int GetStepCt(double totalTime, double timeInc)
        {
            return (int)Math.Ceiling(totalTime / timeInc);
        }

        /// <summary>
        /// Tries to parse a float from a string. False on failure.
        /// </summary>
        private bool TryParseDouble(string s, out double d)
        {
            if (!double.TryParse(s, out d))
            {
                return false;
            }
            return true;
        }
        private DataTree<GeometryBase> GetAllFramesGeom(Dictionary<string, List<RobotFrame>> allFramesLists)
        {
            var tree = new DataTree<GeometryBase>();
            var firstRobot = allFramesLists.Keys.First();
            for (int frameIdx = 0; frameIdx < allFramesLists[firstRobot].Count; frameIdx++)//all robot lists should have exactly the same number of frames.
            {
                int listIdx = 0;
                foreach(var list in allFramesLists.Values)
                {
                    var path = new GH_Path(new []{ frameIdx, listIdx });
                    var frame = list[frameIdx];
                    tree.AddRange(frame.CurrentRobotData.RobotGeometry, path);
                    tree.AddRange(frame.CurrentRobotData.AdditionalGeometry, path);
                    tree.Add(new Point(frame.CurrentRobotData.RobotCenter), path);
                    listIdx++;
                }
            }

            return tree;
        }
    }
}