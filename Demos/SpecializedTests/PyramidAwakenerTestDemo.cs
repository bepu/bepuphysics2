using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace Demos.Demos
{
    /// <summary>
    /// Repeatedly checks for bugs related to multithreaded awakening and narrow phase flushing.
    /// </summary>
    public class PyramidAwakenerTestDemo : Demo
    {
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-30, 8, -110);
            camera.Yaw = MathHelper.Pi * 3f / 4;

            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));

            var boxShape = new Box(1, 1, 1);
            boxShape.ComputeInertia(1, out var boxInertia);
            var boxIndex = Simulation.Shapes.Add(boxShape);
            const int pyramidCount = 10;
            for (int pyramidIndex = 0; pyramidIndex < pyramidCount; ++pyramidIndex)
            {
                const int rowCount = 20;
                for (int rowIndex = 0; rowIndex < rowCount; ++rowIndex)
                {
                    int columnCount = rowCount - rowIndex;
                    for (int columnIndex = 0; columnIndex < columnCount; ++columnIndex)
                    {
                        Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(
                            (-columnCount * 0.5f + columnIndex) * boxShape.Width,
                            (rowIndex + 0.5f) * boxShape.Height,
                            (pyramidIndex - pyramidCount * 0.5f) * (boxShape.Length + 4)),
                            boxInertia,
                            new CollidableDescription(boxIndex, 0.1f),
                            new BodyActivityDescription(0.01f)));
                    }
                }
            }

            var staticShape = new Box(250, 1, 250);
            var staticShapeIndex = Simulation.Shapes.Add(staticShape);

            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -0.5f, 0), new CollidableDescription(staticShapeIndex, 0.1f)));

        }
        
        int frameIndex;
        Random random = new Random(5);
        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            frameIndex++;
            if (frameIndex % 64 == 0)
            {
                var bulletShape = new Sphere(0.5f + 5 * (float)random.NextDouble());
                bulletShape.ComputeInertia(bulletShape.Radius * bulletShape.Radius * bulletShape.Radius, out var bulletInertia);
                var bulletShapeIndex = Simulation.Shapes.Add(bulletShape);
                var bodyDescription = BodyDescription.CreateConvexDynamic(
                    new Vector3(0, 8, -130), new BodyVelocity(new Vector3(0, 0, 350)), bulletShape.Radius * bulletShape.Radius * bulletShape.Radius, Simulation.Shapes, bulletShape);
                Simulation.Bodies.Add(bodyDescription);
            }
            if (frameIndex % 192 == 0)
            {
                Simulation.Dispose();
                BufferPool.Clear();
                for (int i = 0; i < ThreadDispatcher.ThreadCount; ++i)
                    ThreadDispatcher.GetThreadMemoryPool(i).Clear();
                Initialize(null, camera);
            }
            base.Update(window, camera, input, dt);
        }

    }
}
