using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;
using RolyPoly.Interfaces;

namespace RolyPoly.Objects
{
    /// <summary>
    /// Contains information for playing a robot song.
    /// </summary>
    internal class RobotActionPlaySong : IRobotActionSimple
    {
        public string RobotName { get; set; } = "undefined";
        public Vector3d StartRobotDir { get; private set; } = Vector3d.Unset;
        public Vector3d EndRobotDir { get; private set; } = Vector3d.Unset;
        /// <summary>
        /// The duration, in nanoseconds, for each note in the song
        /// </summary>
        public List<int> NotesDurationsNanoSec { get; } 
        /// <summary>
        /// The frequency, in hertz, for each note in the song
        /// </summary>
        public List<int> NotesFrequenciesHz { get; }

        public RobotActionPlaySong(List<int> notesDurationsNanoSec, List<int> notesFrequenciesHz)
        {
            NotesDurationsNanoSec = notesDurationsNanoSec;
            NotesFrequenciesHz = notesFrequenciesHz;
        }

        /// <summary>
        /// Sets the start direction and end directions for this step.
        /// </summary>
        public void SetStartAndEndDirections(Vector3d startDir)
        {
            StartRobotDir = startDir;
            EndRobotDir = startDir;
        }

        public string[] GetCommandStrings()
        {
            return new[] { $"MessagePlaySong(\"{RobotName}\", {ListToString(NotesFrequenciesHz)}, {ListToString(NotesDurationsNanoSec)})" };
        }

        /// <summary>
        /// Converts a list to a string, for the purposes of our song generation.
        /// </summary>
        private string ListToString(List<int> list)
        {
            string d = ",";
            string msg = "[";
            for (int i = 0; i < list.Count; i++)
            {
                msg += list[i].ToString();
                if (i < list.Count - 1)
                {
                    msg += d;
                }
            }
            msg += "]";

            return msg;
        }
    }
}
