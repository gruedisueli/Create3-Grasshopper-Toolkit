using Grasshopper;
using Grasshopper.Kernel;
using System;
using System.Drawing;

namespace RolyPoly
{
    public class RolyPolyInfo : GH_AssemblyInfo
    {
        public override string Name => "RolyPoly";

        //Return a 24x24 pixel bitmap to represent this GHA library.
        public override Bitmap Icon => null;

        //Return a short string describing the purpose of this GHA library.
        public override string Description => "A Grasshopper plugin to create commands to send to your iRobot Create3";

        public override Guid Id => new Guid("fb5fb742-94dc-4559-9e2a-8ccf090f18a9");

        //Return a string identifying you or your company.
        public override string AuthorName => "Gavin Ruedisueli";

        //Return a string representing your preferred contact details.
        public override string AuthorContact => "";
    }
}