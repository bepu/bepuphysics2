using BepuPhysics;
using BepuUtilities;
using DemoContentLoader;
using Demos.Demos;
using Demos.SpecializedTests;
using DemoUtilities;
using OpenTK;
using System;
using System.Runtime.CompilerServices;

namespace Demos
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine($"aasgh: {Unsafe.SizeOf<MotionState>()}");
            var window = new Window("pretty cool multicolored window",
                new Int2((int)(DisplayDevice.Default.Width * 0.75f), (int)(DisplayDevice.Default.Height * 0.75f)), WindowMode.Windowed);
            var loop = new GameLoop(window);
            ContentArchive content;
            using (var stream = typeof(Program).Assembly.GetManifestResourceStream("Demos.Demos.contentarchive"))
            {
                content = ContentArchive.Load(stream);
            }
            HeadlessTest.Test<NewtDemo>(content, 8, 32, 512);
            var demo = new DemoHarness(loop, content);
            loop.Run(demo);
            loop.Dispose();
            window.Dispose();
        }
    }
}