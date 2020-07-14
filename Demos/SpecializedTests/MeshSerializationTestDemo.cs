using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using System;
using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using System.Diagnostics;

namespace Demos.SpecializedTests
{
    public class MeshSerializationTestDemo : Demo
    {
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-30, 8, -60);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = 0;

            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new PositionFirstTimestepper());

            var startTime = Stopwatch.GetTimestamp();
            DemoMeshHelper.CreateDeformedPlane(1025, 1025, (x, y) => new Vector3(x * 0.125f, MathF.Sin(x) + MathF.Sin(y), y * 0.125f), Vector3.One, BufferPool, out var originalMesh);
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, 0, 0), new CollidableDescription(Simulation.Shapes.Add(originalMesh), 0.1f)));
            var endTime = Stopwatch.GetTimestamp();
            var freshConstructionTime = (endTime - startTime) / (double)Stopwatch.Frequency;
            Console.WriteLine($"Fresh construction time (ms): {freshConstructionTime * 1e3}");

            BufferPool.Take(originalMesh.GetSerializedByteCount(), out var serializedMeshBytes);
            originalMesh.Serialize(serializedMeshBytes);
            startTime = Stopwatch.GetTimestamp();
            var loadedMesh = new Mesh(serializedMeshBytes, BufferPool);
            endTime = Stopwatch.GetTimestamp();
            var loadTime = (endTime - startTime) / (double)Stopwatch.Frequency;
            Console.WriteLine($"Load time (ms): {(endTime - startTime) * 1e3 / Stopwatch.Frequency}");
            Console.WriteLine($"Relative speedup: {freshConstructionTime / loadTime}");
            Simulation.Statics.Add(new StaticDescription(new Vector3(128, 0, 0), new CollidableDescription(Simulation.Shapes.Add(loadedMesh), 0.1f)));


            BufferPool.Return(ref serializedMeshBytes);

            var random = new Random(5);
            var shapeToDrop = new Box(1, 1, 1);
            shapeToDrop.ComputeInertia(1, out var shapeToDropInertia);
            var descriptionToDrop = BodyDescription.CreateDynamic(new Vector3(), shapeToDropInertia, new CollidableDescription(Simulation.Shapes.Add(shapeToDrop), 0.1f), new BodyActivityDescription(0.01f));
            for (int i = 0; i < 1024; ++i)
            {
                descriptionToDrop.Pose.Position = new Vector3(8 + 240 * (float)random.NextDouble(), 10 + 10 * (float)random.NextDouble(), 8 + 112 * (float)random.NextDouble());
                Simulation.Bodies.Add(descriptionToDrop);
            }

        }
    }
}
