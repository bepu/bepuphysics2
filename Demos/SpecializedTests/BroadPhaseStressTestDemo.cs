using BepuUtilities;
using DemoRenderer;
using DemoUtilities;
using BepuPhysics;
using BepuPhysics.Collidables;
using System;
using System.Numerics;
using System.Diagnostics;
using BepuUtilities.Memory;
using BepuUtilities.Collections;
using System.Runtime.CompilerServices;
using DemoContentLoader;

namespace Demos.SpecializedTests
{
    public class BroadPhaseStressTestDemo : Demo
    {
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-20f, 13, -20f);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = MathHelper.Pi * 0.1f;
            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));

            var shape = new Sphere(0.5f);
            shape.ComputeInertia(1, out var sphereInertia);
            var shapeIndex = Simulation.Shapes.Add(shape);
            const int width = 64;
            const int height = 64;
            const int length = 64;
            var spacing = new Vector3(1.01f);
            var halfSpacing = spacing / 2;
            float randomization = 0.9f;
            var randomizationSpan = (spacing - new Vector3(1)) * randomization;
            var randomizationBase = randomizationSpan * -0.5f;
            var random = new Random(5);
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    for (int k = 0; k < length; ++k)
                    {
                        var r = new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble());
                        var location = spacing * (new Vector3(i, j, k) + new Vector3(-width, 1, -length)) + randomizationBase + r * randomizationSpan;
                        if ((i + j + k) % 2 == 1)
                        {
                            var bodyDescription = new BodyDescription
                            {
                                Activity = new BodyActivityDescription { MinimumTimestepCountUnderThreshold = 32, SleepThreshold = -0.1f },
                                Pose = new RigidPose
                                {
                                    Orientation = BepuUtilities.Quaternion.Identity,
                                    Position = location
                                },
                                Collidable = new CollidableDescription
                                {
                                    Continuity = new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Discrete },
                                    SpeculativeMargin = 0.1f,
                                    Shape = shapeIndex
                                },
                                LocalInertia = sphereInertia
                            };
                            Simulation.Bodies.Add(bodyDescription);
                        }
                        else
                        {
                            var staticDescription = new StaticDescription
                            {
                                Pose = new RigidPose
                                {
                                    Orientation = BepuUtilities.Quaternion.Identity,
                                    Position = location
                                },
                                Collidable = new CollidableDescription
                                {
                                    Continuity = new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Discrete },
                                    SpeculativeMargin = 0.1f,
                                    Shape = shapeIndex
                                }
                            };
                            Simulation.Statics.Add(staticDescription);
                        }

                    }
                }
            }
            refineTimes = new TimingsRingBuffer(sampleCount, BufferPool);
            testTimes = new TimingsRingBuffer(sampleCount, BufferPool);
        }

        const int sampleCount = 128;
        TimingsRingBuffer refineTimes;
        TimingsRingBuffer testTimes;
        long frameCount;
        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            base.Update(window, camera, input, dt);
            refineTimes.Add(Simulation.Profiler[Simulation.BroadPhase]);
            testTimes.Add(Simulation.Profiler[Simulation.BroadPhaseOverlapFinder]);
            if (frameCount++ % sampleCount == 0)
            {
                var refineStats = refineTimes.ComputeStats();
                var testStats = testTimes.ComputeStats();
                Console.WriteLine($"Refine: {refineStats.Average * 1000} ms average, {refineStats.StdDev * 1000} stddev");
                Console.WriteLine($"Test:   {testStats.Average * 1000} ms average, {testStats.StdDev * 1000} stddev");
            }
        }

    }
}
