using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Memory;
using DemoContentLoader;
using DemoRenderer.UI;
using Demos.Properties;
using Demos.SpecializedTests;
using DemoUtilities;
using OpenTK;
using System.Diagnostics;
using System.IO;

namespace Demos
{
    class Program
    {
        static void Main(string[] args)
        {
            HeadlessDemo.Test<SimpleDemo>(512, 512);
            //ConstraintDescriptionMappingTests.Test();
            //Console.ReadKey();

            var window = new Window("pretty cool multicolored window",
                new Int2((int)(DisplayDevice.Default.Width * 0.75f), (int)(DisplayDevice.Default.Height * 0.75f)), WindowMode.Windowed);
            var loop = new GameLoop(window);
            ContentArchive content;
            using (var stream = new MemoryStream(Resources.Content))
            {
                content = ContentArchive.Load(stream);
            }
            //var fontContent = content.Load<FontContent>(@"Content\Courier Prime Sans.ttf");
            var fontContent = content.Load<FontContent>(@"Content\Carlito-Regular.ttf");
            var font = new Font(loop.Surface.Device, loop.Surface.Context, fontContent);
            var demo = new DemoHarness(window, loop.Input, loop.Camera, font);
            loop.Run(demo);
            loop.Dispose();
            window.Dispose();

        }
    }
}