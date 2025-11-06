using BepuUtilities;
using DemoContentLoader;
using DemoUtilities;
using OpenTK;
using System.Threading;
using System.Threading.Tasks;

namespace Demos;

class Program
{


    static async Task Main()
    {
        var window = new Window("pretty cool multicolored window",
            new Int2((int)(DisplayDevice.Default.Width * 0.75f), (int)(DisplayDevice.Default.Height * 0.75f)), WindowMode.Windowed);
        var loop = new GameLoop(window);
        ContentArchive content;
        using (var stream = typeof(Program).Assembly.GetManifestResourceStream("Demos.Demos.contentarchive"))
        {
            content = ContentArchive.Load(stream);
        }
        //HeadlessTest.Test<ShapePileTestDemo>(content, 4, 32, 512);
        var demo = new DemoHarness(loop, content);
        var dt = 0.12f;
        loop.Update(dt);

        while (true)
        {
            loop.SingleFrame(demo, dt);
            Thread.Sleep(1000);
        }

        loop.Dispose();
        window.Dispose();
    }

    static void OldMain()
    {
        var window = new Window("pretty cool multicolored window",
            new Int2((int)(DisplayDevice.Default.Width * 0.75f), (int)(DisplayDevice.Default.Height * 0.75f)), WindowMode.Windowed);
        var loop = new GameLoop(window);
        ContentArchive content;
        using (var stream = typeof(Program).Assembly.GetManifestResourceStream("Demos.Demos.contentarchive"))
        {
            content = ContentArchive.Load(stream);
        }
        //HeadlessTest.Test<ShapePileTestDemo>(content, 4, 32, 512);
        var demo = new DemoHarness(loop, content);
        loop.Run(demo);
        loop.Dispose();
        window.Dispose();
    }
}