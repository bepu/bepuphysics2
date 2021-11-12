using BepuPhysics;
using BepuUtilities;
using DemoContentLoader;
using Demos.Demos;
using Demos.SpecializedTests;
using DemoUtilities;
using OpenTK;

namespace Demos
{
    class Program
    {
        static void Main(string[] args)
        {
            var window = new Window("pretty cool multicolored window",
                new Int2((int)(DisplayDevice.Default.Width * 0.75f), (int)(DisplayDevice.Default.Height * 0.75f)), WindowMode.Windowed);
            var loop = new GameLoop(window);
            ContentArchive content;
            using (var stream = typeof(Program).Assembly.GetManifestResourceStream("Demos.Demos.contentarchive"))
            {
                content = ContentArchive.Load(stream);
            }
            //HeadlessTest.Test<NewtDemo>(content, 4, 32, 512);
            //HeadlessTest.Test<ColosseumDemo>(content, 8, 32, 1024);
            //HeadlessTest.Test<RopeTwistDemo>(content, 4, 32, 512);
            //HeadlessTest.Test<RagdollTubeDemo>(content, 4, 128, 1024);
            DeterminismTest<FountainStressTestDemo>.Test(content, 1, 16384);
            var demo = new DemoHarness(loop, content);
            loop.Run(demo);
            loop.Dispose();
            window.Dispose();
        }
    }
}