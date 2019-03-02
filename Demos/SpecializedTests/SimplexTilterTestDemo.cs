using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Text;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuPhysics.CollisionDetection.SweepTasks;
using BepuUtilities;
using BepuUtilities.Memory;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.Constraints;
using DemoRenderer.UI;
using DemoUtilities;
using Quaternion = BepuUtilities.Quaternion;

namespace Demos.SpecializedTests
{
    public class SimplexTilterTestDemo : Demo
    {
        Buffer<LineInstance> shapeLines;
        List<SimplexTilterStep> steps;
        Vector3 basePosition;

        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 0, 13f);
            camera.Yaw = 0;
            camera.Pitch = 0;
            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));
            {
                var shapeA = new Cylinder(0.5f, 1f);
                var poseA = new RigidPose(new Vector3(0, 0, 0));
                var shapeB = new Cylinder(1f, 2f);
                //var positionB = new Vector3(-0.2570486f, 1.780561f, -1.033215f);
                //var localOrientationBMatrix = new Matrix3x3
                //{
                //    X = new Vector3(0.9756086f, 0.1946615f, 0.101463f),
                //    Y = new Vector3(-0.1539477f, 0.9362175f, -0.3159063f),
                //    Z = new Vector3(-0.1564862f, 0.2925809f, 0.9433496f)
                //};
                //var positionB = new Vector3(-1.437585f, 0.386236f, -1.124907f);
                var positionB = new Vector3(-0.5037585f, -0.386236f, -1.424907f);
                var localOrientationBMatrix = new Matrix3x3
                {
                    X = new Vector3(-0.7615921f, 0.001486331f, -0.648055f),
                    Y = new Vector3(0.6341797f, 0.2075436f, -0.7448099f),
                    Z = new Vector3(-0.1333926f, -0.9782246f, -0.1590062f)
                };
                //var poseB = new RigidPose(new Vector3(-0.2570486f, 1.780561f, -1.033215f), Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 1, 1)), MathF.PI * 0.35f));
                var poseB = new RigidPose(positionB, Quaternion.CreateFromRotationMatrix(localOrientationBMatrix));

                basePosition = default;
                shapeLines = MinkowskiShapeVisualizer.CreateLines<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>(
                    shapeA, shapeB, poseA, poseB, 65536,
                    0.01f, new Vector3(0.4f, 0.4f, 0),
                    0.1f, new Vector3(0, 1, 0), default, basePosition, BufferPool);

                var aWide = default(CylinderWide);
                var bWide = default(CylinderWide);
                aWide.Broadcast(shapeA);
                bWide.Broadcast(shapeB);
                var worldOffsetB = poseB.Position - poseA.Position;
                var localOrientationB = Matrix3x3.CreateFromQuaternion(Quaternion.Concatenate(poseB.Orientation, Quaternion.Conjugate(poseA.Orientation)));
                var localOffsetB = Quaternion.Transform(worldOffsetB, Quaternion.Conjugate(poseA.Orientation));
                Vector3Wide.Broadcast(localOffsetB, out var localOffsetBWide);
                Matrix3x3Wide.Broadcast(localOrientationB, out var localOrientationBWide);
                var cylinderSupportFinder = default(CylinderSupportFinder);

                var initialNormal = Vector3.Normalize(localOffsetB);
                Vector3Wide.Broadcast(initialNormal, out var initialNormalWide);
                steps = new List<SimplexTilterStep>();
                SimplexTilter<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>.FindMinimumDepth(
                    aWide, bWide, localOffsetBWide, localOrientationBWide, ref cylinderSupportFinder, ref cylinderSupportFinder, initialNormalWide, new Vector<int>(), new Vector<float>(1e-7f), new Vector<float>(-500),
                    out var depthWide2, out var localNormalWide2, steps, 1000);

                //const int iterationCount = 1000;
                //var start = Stopwatch.GetTimestamp();
                //for (int i = 0; i < iterationCount; ++i)
                //{
                //    PlaneWalker<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>.FindMinimumDepth(
                //        aWide, bWide, localOffsetBWide, localOrientationBWide, ref cylinderSupportFinder, ref cylinderSupportFinder, initialNormalWide, new Vector<int>(), new Vector<float>(1e-7f), new Vector<float>(-500), out var depthWide3, out var localNormalWide3, null, 1000);
                //}
                //var stop = Stopwatch.GetTimestamp();
                //var span = (stop - start) * 1e9f / (iterationCount * (double)Stopwatch.Frequency);
                //Console.WriteLine($"Time (ns): {span}");
            }

            //{
            //    var shapeA = new Box(1f, 1f, 1f);
            //    var poseA = new RigidPose(new Vector3(0, 0, 0));
            //    var shapeB = new Box(1f, 1f, 1f);
            //    var poseB = new RigidPose(new Vector3(0f, -1f, 0.01f), Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1,1,1)), MathHelper.PiOver2));

            //    basePosition = default;
            //    shapeLines = MinkowskiShapeVisualizer.CreateLines<Box, BoxWide, BoxSupportFinder, Box, BoxWide, BoxSupportFinder>(
            //        shapeA, shapeB, poseA, poseB, 65536,
            //        0.01f, new Vector3(0.4f, 0.4f, 0),
            //        0.1f, new Vector3(0, 1, 0), default, basePosition, BufferPool);

            //    var aWide = default(BoxWide);
            //    var bWide = default(BoxWide);
            //    aWide.Broadcast(shapeA);
            //    bWide.Broadcast(shapeB);
            //    var worldOffsetB = poseB.Position - poseA.Position;
            //    var localOrientationB = Matrix3x3.CreateFromQuaternion(Quaternion.Concatenate(poseB.Orientation, Quaternion.Conjugate(poseA.Orientation)));
            //    var localOffsetB = Quaternion.Transform(worldOffsetB, Quaternion.Conjugate(poseA.Orientation));
            //    Vector3Wide.Broadcast(localOffsetB, out var localOffsetBWide);
            //    Matrix3x3Wide.Broadcast(localOrientationB, out var localOrientationBWide);
            //    var supportFinder = default(BoxSupportFinder);

            //    var initialNormal = Vector3.Normalize(localOffsetB);
            //    Vector3Wide.Broadcast(initialNormal, out var initialNormalWide);
            //    steps = new List<SimplexTilterStep>();
            //    SimplexTilter<Box, BoxWide, BoxSupportFinder, Box, BoxWide, BoxSupportFinder>.FindMinimumDepth(
            //        aWide, bWide, localOffsetBWide, localOrientationBWide, ref supportFinder, ref supportFinder, initialNormalWide, new Vector<int>(), new Vector<float>(1e-7f), new Vector<float>(-500),
            //        out var depthWide2, out var localNormalWide2, steps, 1000);

            //    //const int iterationCount = 1000;
            //    //var start = Stopwatch.GetTimestamp();
            //    //for (int i = 0; i < iterationCount; ++i)
            //    //{
            //    //    PlaneWalker<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>.FindMinimumDepth(
            //    //        aWide, bWide, localOffsetBWide, localOrientationBWide, ref cylinderSupportFinder, ref cylinderSupportFinder, initialNormalWide, new Vector<int>(), new Vector<float>(1e-7f), new Vector<float>(-500), out var depthWide3, out var localNormalWide3, null, 1000);
            //    //}
            //    //var stop = Stopwatch.GetTimestamp();
            //    //var span = (stop - start) * 1e9f / (iterationCount * (double)Stopwatch.Frequency);
            //    //Console.WriteLine($"Time (ns): {span}");
            //}

        }

        int stepIndex;
        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            if (input.TypedCharacters.Contains('x'))
            {
                stepIndex = Math.Max(0, stepIndex - 1);
            }
            else if (input.TypedCharacters.Contains('c'))
            {
                stepIndex = Math.Min(stepIndex + 1, steps.Count - 1);
            }
            base.Update(window, camera, input, dt);
        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            MinkowskiShapeVisualizer.Draw(shapeLines, renderer);
            renderer.TextBatcher.Write(
                text.Clear().Append($"Enumerate step with X and C. Current step: ").Append(stepIndex + 1).Append(" out of ").Append(steps.Count),
                new Vector2(32, renderer.Surface.Resolution.Y - 140), 20, new Vector3(1), font);
            var step = steps[stepIndex];
            renderer.TextBatcher.Write(
               text.Clear().Append($"Next normal source: ").Append(step.NextNormalSource.ToString()),
               new Vector2(32, renderer.Surface.Resolution.Y - 120), 20, new Vector3(1), font);
            renderer.TextBatcher.Write(
               text.Clear().Append($"Best depth: ").Append(step.BestDepth, 9),
               new Vector2(32, renderer.Surface.Resolution.Y - 100), 20, new Vector3(1), font);
            renderer.TextBatcher.Write(
               text.Clear().Append($"Progression scale: ").Append(step.ProgressionScale, 9),
               new Vector2(32, renderer.Surface.Resolution.Y - 80), 20, new Vector3(1), font);

            if (!step.A.Exists && !step.B.Exists)
            {
                step.A = step.C;
            }
            else if (!step.A.Exists && step.B.Exists)
            {
                step.A = step.B;
                if (step.C.Exists)
                    step.B = step.C;
            }
            else if (step.A.Exists && !step.B.Exists)
            {
                step.B = step.C;
            }
            if (step.C.Depth < step.B.Depth && step.C.Exists && step.B.Exists)
            {
                var temp = step.B;
                step.B = step.C;
                step.C = temp;
            }
            if (step.B.Depth < step.A.Depth && step.A.Exists && step.B.Exists)
            {
                var temp = step.A;
                step.A = step.B;
                step.B = temp;
            }
            if (step.C.Depth < step.B.Depth && step.C.Exists && step.B.Exists)
            {
                var temp = step.B;
                step.B = step.C;
                step.C = temp;
            }

            if (step.A.Exists)
            {
                if (step.B.Exists)
                    renderer.Lines.Allocate() = new LineInstance(step.A.Support + basePosition, step.B.Support + basePosition, new Vector3(0, 0.6f, 0.1f), default);
                renderer.Lines.Allocate() = new LineInstance(step.A.Support + basePosition, step.A.Support + basePosition + step.A.Normal, new Vector3(0, 1f, 0), default);
                //renderer.Lines.Allocate() = new LineInstance(basePosition, step.A.Support + basePosition, new Vector3(0, 0, 0.5f), default);
            }

            if (step.B.Exists)
            {
                if (step.C.Exists)
                    renderer.Lines.Allocate() = new LineInstance(step.B.Support + basePosition, step.C.Support + basePosition, new Vector3(0, 0.6f, 0.1f), default);
                renderer.Lines.Allocate() = new LineInstance(step.B.Support + basePosition, step.B.Support + basePosition + step.B.Normal, new Vector3(0, 0.6f, 0.1f), default);
                //renderer.Lines.Allocate() = new LineInstance(step.B.Support + basePosition, basePosition, new Vector3(0, 0, 0.5f), default);
            }

            if (step.C.Exists)
            {
                if (step.A.Exists)
                    renderer.Lines.Allocate() = new LineInstance(step.C.Support + basePosition, step.A.Support + basePosition, new Vector3(0, 0.6f, 0.1f), default);
                renderer.Lines.Allocate() = new LineInstance(step.C.Support + basePosition, step.C.Support + basePosition + step.C.Normal, new Vector3(0, 0.3f, 0.2f), default);
                //renderer.Lines.Allocate() = new LineInstance(step.C.Support + basePosition, basePosition, new Vector3(0, 0, 0.5f), default);
            }

            if (step.D.Exists)
            {
                if (step.A.Exists)
                    renderer.Lines.Allocate() = new LineInstance(step.A.Support + basePosition, step.D.Support + basePosition, new Vector3(1, 0.6f, 0.1f), default);
                if (step.B.Exists)
                    renderer.Lines.Allocate() = new LineInstance(step.B.Support + basePosition, step.D.Support + basePosition, new Vector3(1, 0.6f, 0.1f), default);
                if (step.C.Exists)
                    renderer.Lines.Allocate() = new LineInstance(step.C.Support + basePosition, step.D.Support + basePosition, new Vector3(1, 0.6f, 0.1f), default);
                renderer.Lines.Allocate() = new LineInstance(step.D.Support + basePosition, step.D.Support + basePosition + step.D.Normal, new Vector3(1f, 0.8f, 0.2f), default);
                //renderer.Lines.Allocate() = new LineInstance(step.D.Support + basePosition, basePosition, new Vector3(0, 0, 0.5f), default);
            }

            renderer.Lines.Allocate() = new LineInstance(basePosition, basePosition + step.NextNormal, new Vector3(1, 0, 1), default);
            //if (step.NextNormalSource == SimplexTilterNormalSource.EdgeTilt)
            //{
            //    renderer.Lines.Allocate() = new LineInstance(step.ClosestPointOnTriangle + basePosition, step.TiltTargetPoint + basePosition, new Vector3(1, 0, 1), default);
            //    renderer.Lines.Allocate() = new LineInstance(step.ClosestPointOnTriangle + basePosition, step.ClosestPointOnTriangle + step.TiltOffset + basePosition, new Vector3(0.25f, 0, 1), default);
            //    renderer.Lines.Allocate() = new LineInstance(step.ClosestPointOnTriangle + basePosition, step.ClosestPointOnTriangle + step.TiltStart + basePosition, new Vector3(0.5f, 0, 0.5f), default);
            //}

            renderer.Lines.Allocate() = new LineInstance(basePosition, basePosition + step.BestNormal, new Vector3(1, 0, 0), default);
            base.Render(renderer, camera, input, text, font);
        }
    }
}
