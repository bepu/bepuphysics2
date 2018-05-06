using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuUtilities;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using Quaternion = BepuUtilities.Quaternion;

namespace Demos.SpecializedTests
{
    public class SweepTestDemo : Demo
    {
        struct TestFilter : ISweepFilter
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowTest(int childA, int childB)
            {
                return true;
            }
        }

        public unsafe override void Initialize(Camera camera)
        {
            camera.Position = new Vector3(-30, 8, -60);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            Simulation = Simulation.Create(BufferPool, new TestCallbacks());
            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);

            var spherePairTask = Simulation.NarrowPhase.SweepTaskRegistry.GetTask<Sphere, Sphere>();
            var a = new Sphere(0.5f);
            var b = new Sphere(0.5f);
            var filter = new TestFilter();
            var intersected = spherePairTask.Sweep(
                &a, a.TypeId, Quaternion.Identity, new BodyVelocity { Linear = new Vector3(1, 0, 0) },
                &b, b.TypeId, new Vector3(1, 0, 0), Quaternion.Identity, new BodyVelocity { Linear = new Vector3(0, 0, 0) },
                5, 0f, 1e-9f, 10, ref filter, out var t0, out var t1, out var hitLocation, out var hitNormal);

            Console.WriteLine($"Intersected: {intersected}");
        }

    }
}
