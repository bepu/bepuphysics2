using BepuUtilities;
using DemoRenderer;
using BepuPhysics;
using BepuPhysics.Collidables;
using System;
using System.Numerics;
using BepuUtilities.Collections;
using DemoContentLoader;
using BepuPhysics.Constraints;
using DemoUtilities;

namespace Demos.SpecializedTests
{
    public class SimpleTestDemo : Demo
    {
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-30, 10, -30);
            //camera.Yaw = MathHelper.Pi ; 
            camera.Yaw = MathHelper.Pi * 3f / 4;
            //camera.Pitch = MathHelper.PiOver2 * 0.999f;
            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(4, 1));

            var box = new Box(1f, 3f, 2f);

            var boxInertia = box.ComputeInertia(1);
            var boxIndex = Simulation.Shapes.Add(box);
            const int width = 256;
            const int height = 1;
            const int length = 256;
            var shapeCount = 0;
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    for (int k = 0; k < length; ++k)
                    {
                        var location = new Vector3(6, 3, 6) * new Vector3(i, j, k) + new Vector3(-width * 3, 5.5f, -length * 3);
                        var bodyDescription = BodyDescription.CreateDynamic(location, boxInertia, boxIndex, -0.01f);
                        var index = shapeCount++;
                        Simulation.Bodies.Add(bodyDescription);
                    }
                }
            }

            Simulation.Statics.Add(new StaticDescription(new Vector3(), Simulation.Shapes.Add(new Box(5000, 1, 5000))));
            //var mesh = DemoMeshHelper.CreateDeformedPlane(128, 128, (x, y) => new Vector3(x - 64, 2f * (float)(Math.Sin(x * 0.5f) * Math.Sin(y * 0.5f)), y - 64), new Vector3(4, 1, 4), BufferPool);
            //Simulation.Statics.Add(new StaticDescription(new Vector3(), Simulation.Shapes.Add(mesh)));
        }

        double time = 0;
        long frameCount = 0;
        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            base.Update(window, camera, input, dt);
            const long minimumFrameToMeasure = 256;
            frameCount++;
            if (frameCount >= minimumFrameToMeasure)
            {
                var frameTime = Simulation.Profiler[Simulation.BroadPhaseOverlapFinder];
                time += frameTime;
                Console.WriteLine($"coldet time (ms): {1e3 * frameTime}, average (ms): {1e3 * time / (frameCount - minimumFrameToMeasure)}");
            }

        }
    }
}


