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
            camera.Position = new Vector3(0, 0, 15);
            camera.Yaw = 0;
            Simulation = Simulation.Create(BufferPool, new TestCallbacks());
            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);

            //var a = new Capsule(0.5f, 0.5f);
            //var b = new Capsule(0.5f, 0.5f);
            //var filter = new TestFilter();
            //var spherePairTask = Simulation.NarrowPhase.SweepTaskRegistry.GetTask(a.TypeId, b.TypeId);
            //var intersected = spherePairTask.Sweep(
            //    &a, a.TypeId, Quaternion.Identity, new BodyVelocity { Linear = new Vector3(1, 0, 0) },
            //    &b, b.TypeId, new Vector3(a.Radius + b.Radius + 1, a.HalfLength + b.Radius + b.HalfLength, 0), Quaternion.Identity, new BodyVelocity { Linear = new Vector3(0, 0, 0) },
            //    5, 0f, 1e-9f, 10, ref filter, out var t0, out var t1, out var hitLocation, out var hitNormal);

            //Console.WriteLine($"Intersected: {intersected}");
        }

        unsafe void DrawSweep<TShape>(TShape shape, ref RigidPose pose, in BodyVelocity velocity, int steps,
            float t, Renderer renderer, in Vector3 color)
            where TShape : struct, IShape
        {
            if (t == 0)
            {
                renderer.Shapes.AddShape(shape, Simulation.Shapes, ref pose, color);
            }
            else
            {
                var inverse = 1f / (steps - 1);
                for (int i = steps - 1; i >= 0; --i)
                {
                    var stepProgression = i * inverse;
                    var stepT = stepProgression * t;
                    PoseIntegrator.Integrate(pose, velocity, stepT, out var stepPose);
                    var stepColor = color * (0.2f + 0.8f * stepProgression);
                    renderer.Shapes.AddShape(shape, Simulation.Shapes, ref stepPose, stepColor);
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void BuildOrthnormalBasis(ref Vector3 normal, out Vector3 t1, out Vector3 t2)
        {
            //This could probably be improved.
            var sign = normal.Z < 0 ? -1 : 1;

            //This has a discontinuity at z==0. Raw frisvad has only one discontinuity, though that region is more unpredictable than the revised version.
            var scale = -1f / (sign + normal.Z);
            t1.X = normal.X * normal.Y * scale;
            t1.Y = sign + normal.Y * normal.Y * scale;
            t1.Z = -normal.Y;

            t2.X = 1f + sign * normal.X * normal.X * scale;
            t2.Y = sign * t1.X;
            t2.Z = -sign * normal.X;
        }

        unsafe void TestSweep<TShapeA, TShapeB>(
            TShapeA a, RigidPose poseA, in BodyVelocity velocityA,
            TShapeB b, RigidPose poseB, in BodyVelocity velocityB,
            Renderer renderer)
            where TShapeA : struct, IShape
            where TShapeB : struct, IShape
        {
            var filter = new TestFilter();

            var task = Simulation.NarrowPhase.SweepTaskRegistry.GetTask(a.TypeId, b.TypeId);
            var intersected = task.Sweep(
                Unsafe.AsPointer(ref a), a.TypeId, poseA.Orientation, velocityA,
                Unsafe.AsPointer(ref b), b.TypeId, poseB.Position - poseA.Position, poseB.Orientation, velocityB,
                5, 0f, 1e-9f, 10, ref filter, out var t0, out var t1, out var hitLocation, out var hitNormal);
            hitLocation += poseA.Position;

            var hitTint = intersected ? new Vector3(0.5f, 1, 0.5f) : new Vector3(1f, 0.5f, 0.5f);
            var colorA = new Vector3(0.75f, 0.75f, 1) * hitTint;
            var colorB = new Vector3(0.75f, 1f, 1) * hitTint;

            var stepCount = 4;
            DrawSweep(a, ref poseA, velocityA, stepCount, t1, renderer, colorA);
            DrawSweep(b, ref poseB, velocityB, stepCount, t1, renderer, colorB);

            if (intersected && t1 > 0)
            {
                //The normal itself will tend to be obscured by the shapes, so instead draw two lines representing the plane.
                BuildOrthnormalBasis(ref hitNormal, out var tangent1, out var tangent2);
                renderer.Lines.Allocate() = new DemoRenderer.Constraints.LineInstance(hitLocation - tangent1, hitLocation + tangent1, new Vector3(0, 1, 0), new Vector3());
                renderer.Lines.Allocate() = new DemoRenderer.Constraints.LineInstance(hitLocation - tangent2, hitLocation + tangent2, new Vector3(0, 1, 0), new Vector3());
            }
        }

        public override void Render(Renderer renderer, TextBuilder text, Font font)
        {
            base.Render(renderer, text, font);
            TestSweep(
                new Capsule(0.5f, 0.5f),
                new RigidPose { Position = new Vector3(0, 0, 0), Orientation = Quaternion.Identity },
                new BodyVelocity { Linear = new Vector3(0, 0, 0), Angular = new Vector3(1, 0, 1) },
                new Capsule(0.5f, 0.5f),
                new RigidPose { Position = new Vector3(2, 0, 0), Orientation = Quaternion.Identity },
                new BodyVelocity { Linear = new Vector3(-1, 0, 0), Angular = new Vector3(0, 0, 1) }, renderer);
        }
    }
}
