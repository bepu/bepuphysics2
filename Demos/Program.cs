using BepuUtilities;
using DemoContentLoader;
using DemoUtilities;
using OpenTK;
using System;
using System.Runtime.InteropServices;
using System.Threading;



namespace Demos;



class Program
{

    [DllImport("user32.dll")]
    static extern short GetAsyncKeyState(int key);

    static bool IsKeyPressed(int key) => (GetAsyncKeyState(key) & 0x8000) != 0;




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
        //HeadlessTest.Test<ShapePileTestDemo>(content, 4, 32, 512);
        var demo = new DemoHarness(loop, content);
        var dt = 0.12f;
        loop.Update(dt);
        DateTime? arrowDownTime = null;
        var holdDownTimeMs = 500;
        while (true)
        {
            if (IsKeyPressed(0x1B)) break; // escape
            if (!IsKeyPressed(0x27))
            {
                arrowDownTime = null;
                Thread.Sleep(10);
                continue;
            }
            if (arrowDownTime is null || (DateTime.Now - arrowDownTime.Value).TotalMilliseconds > holdDownTimeMs)
            {
                loop.SingleFrame(demo, dt);
                Thread.Sleep(50);
            }
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