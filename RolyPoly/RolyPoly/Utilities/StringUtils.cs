using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RolyPoly.Utilities
{
    internal static class StringUtils
    {
        /// <summary>
        /// Gets string formatted as we need for sending commands to the robot in the python script.
        /// </summary>
        public static string GetFormattedDoubleString(double d)
        {
            return d.ToString("F4", CultureInfo.InvariantCulture);
        }
    }
}
