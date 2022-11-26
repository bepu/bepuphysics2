using BepuPhysics.Trees;
using BepuUtilities;
using DemoContentLoader;
using Demos.SpecializedTests;
using DemoUtilities;
using OpenTK;
using System.Runtime.Intrinsics;

namespace Demos
{
    class Program
    {
        static void Main()
        {
            var window = new Window("pretty cool multicolored window",
                new Int2((int)(DisplayDevice.Default.Width * 0.75f), (int)(DisplayDevice.Default.Height * 0.75f)), WindowMode.Windowed);
            var loop = new GameLoop(window);
            ContentArchive content;
            using (var stream = typeof(Program).Assembly.GetManifestResourceStream("Demos.Demos.contentarchive"))
            {
                content = ContentArchive.Load(stream);
            }
            HeadlessTest.Test<ShapePileTestDemo>(content, 4, 32, 512);
            var demo = new DemoHarness(loop, content);
            loop.Run(demo);
            loop.Dispose();
            window.Dispose();
        }
    }
}