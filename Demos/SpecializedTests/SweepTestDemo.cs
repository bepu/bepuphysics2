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
            //No frisvad or friends here- just want a simple and consistent basis with only one singularity.
            t1 = Vector3.Cross(normal, new Vector3(1, -1, 1));
            var lengthSquared = t1.LengthSquared();
            if (lengthSquared < 1e-8f)
            {
                t1 = Vector3.Cross(normal, new Vector3(-1, 1, 1));
                lengthSquared = t1.LengthSquared();
            }
            t1 /= MathF.Sqrt(lengthSquared);
            t2 = Vector3.Cross(normal, t1);
        }

        unsafe void TestSweep<TShapeA, TShapeB>(
            TShapeA a, RigidPose poseA, in BodyVelocity velocityA,
            TShapeB b, RigidPose poseB, in BodyVelocity velocityB,
            float maximumT, Renderer renderer)
            where TShapeA : struct, IShape
            where TShapeB : struct, IShape
        {
            var filter = new TestFilter();

            var task = Simulation.NarrowPhase.SweepTaskRegistry.GetTask(a.TypeId, b.TypeId);
            var intersected = task.Sweep(
                Unsafe.AsPointer(ref a), a.TypeId, poseA.Orientation, velocityA,
                Unsafe.AsPointer(ref b), b.TypeId, poseB.Position - poseA.Position, poseB.Orientation, velocityB,
                maximumT, 1e-3f, 1e-7f, 25, ref filter, out var t0, out var t1, out var hitLocation, out var hitNormal);
            hitLocation += poseA.Position;

            var hitTint = intersected ? new Vector3(0.5f, 1, 0.5f) : new Vector3(1f, 0.5f, 0.5f);
            var colorA = new Vector3(0.75f, 0.75f, 1) * hitTint;
            var colorB = new Vector3(0.75f, 1f, 1) * hitTint;

            var stepCount = 250;
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

        float animationT;

        public override void Render(Renderer renderer, TextBuilder text, Font font)
        {
            animationT = (animationT + 1 / 60f) % (128);

            var x = Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), 0.02f * animationT * MathHelper.Pi);
            var y = Quaternion.CreateFromAxisAngle(new Vector3(0, 1, 0), 0.04f * animationT * MathHelper.Pi);
            var z = Quaternion.CreateFromAxisAngle(new Vector3(0, 0, 1), 0.06f * animationT * MathHelper.Pi);
            var worldA = Quaternion.Concatenate(x, Quaternion.Concatenate(y, z));
            var worldB = Quaternion.Concatenate(y, Quaternion.Concatenate(z, x));
            base.Render(renderer, text, font);
            TestSweep(
                new Capsule(.5f, 15.5f),
                new RigidPose { Position = new Vector3(0, -20, 0), Orientation = Quaternion.Concatenate(Quaternion.Identity, worldA) },
                new BodyVelocity { Linear = new Vector3(0, 1, 0), Angular = new Vector3(0.5f, 1f, -1f) },
                new Capsule(1.5f, 5.5f),
                new RigidPose { Position = new Vector3(-20, 0, 0), Orientation = Quaternion.Concatenate(Quaternion.Identity, worldB) },
                new BodyVelocity { Linear = new Vector3(1, 0, 0), Angular = new Vector3(-1f, 0, 1f) }, 50, renderer);
        }
    }
}
