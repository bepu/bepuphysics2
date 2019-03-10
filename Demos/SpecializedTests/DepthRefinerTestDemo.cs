//using System;
//using System.Collections.Generic;
//using System.Diagnostics;
//using System.Numerics;
//using System.Text;
//using BepuPhysics;
//using BepuPhysics.Collidables;
//using BepuPhysics.CollisionDetection;
//using BepuPhysics.CollisionDetection.CollisionTasks;
//using BepuPhysics.CollisionDetection.SweepTasks;
//using BepuUtilities;
//using BepuUtilities.Memory;
//using DemoContentLoader;
//using DemoRenderer;
//using DemoRenderer.Constraints;
//using DemoRenderer.UI;
//using DemoUtilities;
//using Quaternion = BepuUtilities.Quaternion;

//namespace Demos.SpecializedTests
//{
//    //This relies on the DepthRefiner being generated with the debug flag set to true in the text template.
//    public class DepthRefinerTestDemo : Demo
//    {
//        Buffer<LineInstance> shapeLines;
//        List<DepthRefinerStep> steps;
//        Vector3 basePosition;

//        public override void Initialize(ContentArchive content, Camera camera)
//        {
//            camera.Position = new Vector3(0, 0, 13f);
//            camera.Yaw = 0;
//            camera.Pitch = 0;
//            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));
//            {
//                var shapeA = new Cylinder(1f, 2f);
//                var poseA = new RigidPose(new Vector3(12, 0.5f, 12), Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), MathF.PI * 0.5f));
//                var shapeB = new Triangle(new Vector3(-2f, 0, -2f), new Vector3(2f, 0, -2f), new Vector3(-2f, 0, 2f));
//                var poseB = new RigidPose(new Vector3(12, 0, 12));

//                basePosition = default;
//                shapeLines = MinkowskiShapeVisualizer.CreateLines<Cylinder, CylinderWide, CylinderSupportFinder, Triangle, TriangleWide, TriangleSupportFinder>(
//                    shapeA, shapeB, poseA, poseB, 65536,
//                    0.01f, new Vector3(0.4f, 0.4f, 0),
//                    0.1f, new Vector3(0, 1, 0), default, basePosition, BufferPool);

//                var aWide = default(CylinderWide);
//                var bWide = default(TriangleWide);
//                aWide.Broadcast(shapeA);
//                bWide.Broadcast(shapeB);
//                var worldOffsetB = poseB.Position - poseA.Position;
//                var localOrientationB = Matrix3x3.CreateFromQuaternion(Quaternion.Concatenate(poseB.Orientation, Quaternion.Conjugate(poseA.Orientation)));
//                var localOffsetB = Quaternion.Transform(worldOffsetB, Quaternion.Conjugate(poseA.Orientation));
//                Vector3Wide.Broadcast(localOffsetB, out var localOffsetBWide);
//                Matrix3x3Wide.Broadcast(localOrientationB, out var localOrientationBWide);
//                var triangleSupportFinder = default(TriangleSupportFinder);
//                var cylinderSupportFinder = default(CylinderSupportFinder);

//                var initialNormal = Vector3.Normalize(localOffsetB);
//                Vector3Wide.Broadcast(initialNormal, out var initialNormalWide);
//                steps = new List<DepthRefinerStep>();
//                DepthRefiner<Cylinder, CylinderWide, CylinderSupportFinder, Triangle, TriangleWide, TriangleSupportFinder>.FindMinimumDepth(
//                    aWide, bWide, localOffsetBWide, localOrientationBWide, ref cylinderSupportFinder, ref triangleSupportFinder, initialNormalWide, new Vector<int>(), new Vector<float>(1e-6f), new Vector<float>(-500),
//                    out var depthWide, out var localNormalWide, steps, 50);

//            }


//            //{
//            //    var shapeA = new Cylinder(0.5f, 1f);
//            //    var poseA = new RigidPose(new Vector3(0, 0, 0));
//            //    var shapeB = new Cylinder(1f, 2f);
//            //    //var positionB = new Vector3(-0.2570486f, 0.780561f, -1.033215f);
//            //    //var positionB = new Vector3(-0.2570486f, 5.780561f, -1.033215f);
//            //    //var positionB = new Vector3(-1.0570486f, -.380561f, 1.0833215f);
//            //    var positionB = new Vector3(-1.5570486f, -.580561f, .033215f);
//            //    var localOrientationBMatrix = new Matrix3x3
//            //    {
//            //        X = new Vector3(0.9756086f, 0.1946615f, 0.101463f),
//            //        Y = new Vector3(-0.1539477f, 0.9362175f, -0.3159063f),
//            //        Z = new Vector3(-0.1564862f, 0.2925809f, 0.9433496f)
//            //    };
//            //    //var positionB = new Vector3(-1.437585f, 0.386236f, -1.124907f);
//            //    //var positionB = new Vector3(.1037585f, 1.568576f, .124907f);
//            //    //var positionB = new Vector3(1.037585f, .7568576f, 0.90424907f);
//            //    //var positionB = new Vector3(4.037585f, 2.7568576f, 2.90424907f);
//            //    //var positionB = new Vector3(3.037585f, 1.7568576f, 1.90424907f);
//            //    //var localOrientationBMatrix = new Matrix3x3
//            //    //{
//            //    //    X = new Vector3(-0.7615921f, 0.001486331f, -0.648055f),
//            //    //    Y = new Vector3(0.6341797f, 0.2075436f, -0.7448099f),
//            //    //    Z = new Vector3(-0.1333926f, -0.9782246f, -0.1590062f)
//            //    //};
//            //    //var poseB = new RigidPose(new Vector3(-0.2570486f, 1.780561f, -1.033215f), Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 1, 1)), MathF.PI * 0.35f));
//            //    var poseB = new RigidPose(positionB, Quaternion.CreateFromRotationMatrix(localOrientationBMatrix));

//            //    basePosition = default;
//            //    shapeLines = MinkowskiShapeVisualizer.CreateLines<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>(
//            //        shapeA, shapeB, poseA, poseB, 65536,
//            //        0.01f, new Vector3(0.4f, 0.4f, 0),
//            //        0.1f, new Vector3(0, 1, 0), default, basePosition, BufferPool);

//            //    var aWide = default(CylinderWide);
//            //    var bWide = default(CylinderWide);
//            //    aWide.Broadcast(shapeA);
//            //    bWide.Broadcast(shapeB);
//            //    var worldOffsetB = poseB.Position - poseA.Position;
//            //    var localOrientationB = Matrix3x3.CreateFromQuaternion(Quaternion.Concatenate(poseB.Orientation, Quaternion.Conjugate(poseA.Orientation)));
//            //    var localOffsetB = Quaternion.Transform(worldOffsetB, Quaternion.Conjugate(poseA.Orientation));
//            //    Vector3Wide.Broadcast(localOffsetB, out var localOffsetBWide);
//            //    Matrix3x3Wide.Broadcast(localOrientationB, out var localOrientationBWide);
//            //    var supportFinder = default(CylinderSupportFinder);

//            //    var initialNormal = Vector3.Normalize(localOffsetB);
//            //    Vector3Wide.Broadcast(initialNormal, out var initialNormalWide);
//            //    steps = new List<DepthRefinerStep>();
//            //    DepthRefiner<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>.FindMinimumDepth(
//            //        aWide, bWide, localOffsetBWide, localOrientationBWide, ref supportFinder, ref supportFinder, initialNormalWide, new Vector<int>(), new Vector<float>(1e-6f), new Vector<float>(-500),
//            //        out var depthWide, out var localNormalWide, steps, 50);

//            //    const int iterationCount = 100000;
//            //    double minTime = double.MaxValue;
//            //    for (int j = 0; j < 10; ++j)
//            //    {
//            //        var start = Stopwatch.GetTimestamp();
//            //        for (int i = 0; i < iterationCount; ++i)
//            //        {
//            //            DepthRefiner<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>.FindMinimumDepth(
//            //                    aWide, bWide, localOffsetBWide, localOrientationBWide, ref supportFinder, ref supportFinder, initialNormalWide, new Vector<int>(), new Vector<float>(1e-6f), new Vector<float>(-500),
//            //                    out var depthWide2, out var localNormalWide2, null, 50);
//            //        }
//            //        var stop = Stopwatch.GetTimestamp();
//            //        var span = (stop - start) * 1e9f / (iterationCount * (double)Stopwatch.Frequency);
//            //        Console.WriteLine($"Time {j} (ns): {span}");
//            //        minTime = Math.Min(span, minTime);
//            //    }
//            //    Console.WriteLine($"Best time cylinders (ns): {minTime}");
//            //}

//            //Console.WriteLine();
//            //{
//            //    var shapeA = new Box(1f, 1f, 1f);
//            //    var poseA = new RigidPose(new Vector3(0, 0, 0));
//            //    var shapeB = new Box(1f, 1f, 1f);
//            //    var poseB = new RigidPose(new Vector3(-.9f, -0.8f, 0.7f), Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 1, 1)), MathHelper.PiOver2));

//            //    basePosition = default;
//            //    shapeLines = MinkowskiShapeVisualizer.CreateLines<Box, BoxWide, BoxSupportFinder, Box, BoxWide, BoxSupportFinder>(
//            //        shapeA, shapeB, poseA, poseB, 65536,
//            //        0.01f, new Vector3(0.4f, 0.4f, 0),
//            //        0.1f, new Vector3(0, 1, 0), default, basePosition, BufferPool);

//            //    var aWide = default(BoxWide);
//            //    var bWide = default(BoxWide);
//            //    aWide.Broadcast(shapeA);
//            //    bWide.Broadcast(shapeB);
//            //    var worldOffsetB = poseB.Position - poseA.Position;
//            //    var localOrientationB = Matrix3x3.CreateFromQuaternion(Quaternion.Concatenate(poseB.Orientation, Quaternion.Conjugate(poseA.Orientation)));
//            //    var localOffsetB = Quaternion.Transform(worldOffsetB, Quaternion.Conjugate(poseA.Orientation));
//            //    Vector3Wide.Broadcast(localOffsetB, out var localOffsetBWide);
//            //    Matrix3x3Wide.Broadcast(localOrientationB, out var localOrientationBWide);
//            //    var supportFinder = default(BoxSupportFinder);

//            //    var initialNormal = Vector3.Normalize(localOffsetB);
//            //    Vector3Wide.Broadcast(initialNormal, out var initialNormalWide);
//            //    steps = new List<DepthRefinerStep>();
//            //    DepthRefiner<Box, BoxWide, BoxSupportFinder, Box, BoxWide, BoxSupportFinder>.FindMinimumDepth(
//            //        aWide, bWide, localOffsetBWide, localOrientationBWide, ref supportFinder, ref supportFinder, initialNormalWide, new Vector<int>(), new Vector<float>(1e-6f), new Vector<float>(-500),
//            //        out var depthWide, out var localNormalWide, steps, 50);

//            //    const int iterationCount = 100000;
//            //    double minTime = double.MaxValue;
//            //    for (int j = 0; j < 10; ++j)
//            //    {
//            //        var start = Stopwatch.GetTimestamp();
//            //        for (int i = 0; i < iterationCount; ++i)
//            //        {
//            //            DepthRefiner<Box, BoxWide, BoxSupportFinder, Box, BoxWide, BoxSupportFinder>.FindMinimumDepth(
//            //                aWide, bWide, localOffsetBWide, localOrientationBWide, ref supportFinder, ref supportFinder, initialNormalWide, new Vector<int>(), new Vector<float>(1e-6f), new Vector<float>(-500),
//            //                out var depthWide2, out var localNormalWide2, null, 50);
//            //        }
//            //        var stop = Stopwatch.GetTimestamp();
//            //        var span = (stop - start) * 1e9f / (iterationCount * (double)Stopwatch.Frequency);
//            //        Console.WriteLine($"Time {j} (ns): {span}");
//            //        minTime = Math.Min(span, minTime);
//            //    }
//            //    Console.WriteLine($"Best time boxes (ns): {minTime}");
//            //}

//        }

//        int stepIndex;
//        public override void Update(Window window, Camera camera, Input input, float dt)
//        {
//            if (input.TypedCharacters.Contains('x'))
//            {
//                stepIndex = Math.Max(0, stepIndex - 1);
//            }
//            else if (input.TypedCharacters.Contains('c'))
//            {
//                stepIndex = Math.Min(stepIndex + 1, steps.Count - 1);
//            }
//            base.Update(window, camera, input, dt);
//        }

//        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
//        {
//            MinkowskiShapeVisualizer.Draw(shapeLines, renderer);
//            renderer.TextBatcher.Write(
//                text.Clear().Append($"Enumerate step with X and C. Current step: ").Append(stepIndex + 1).Append(" out of ").Append(steps.Count),
//                new Vector2(32, renderer.Surface.Resolution.Y - 140), 20, new Vector3(1), font);
//            var step = steps[stepIndex];
//            renderer.TextBatcher.Write(
//               text.Clear().Append($"Next normal source: ").Append(step.NextNormalSource.ToString()),
//               new Vector2(32, renderer.Surface.Resolution.Y - 120), 20, new Vector3(1), font);
//            renderer.TextBatcher.Write(
//               text.Clear().Append($"Best depth: ").Append(step.BestDepth, 9),
//               new Vector2(32, renderer.Surface.Resolution.Y - 100), 20, new Vector3(1), font);

//            if (step.A.Exists)
//            {
//                if (step.B.Exists)
//                    renderer.Lines.Allocate() = new LineInstance(step.A.Support + basePosition, step.B.Support + basePosition, new Vector3(0, 0.6f, 0.1f), default);
//                //renderer.Lines.Allocate() = new LineInstance(basePosition, step.A.Support + basePosition, new Vector3(0, 0, 0.5f), default);
//            }

//            if (step.B.Exists)
//            {
//                if (step.C.Exists)
//                    renderer.Lines.Allocate() = new LineInstance(step.B.Support + basePosition, step.C.Support + basePosition, new Vector3(0, 0.6f, 0.1f), default);
//                //renderer.Lines.Allocate() = new LineInstance(step.B.Support + basePosition, basePosition, new Vector3(0, 0, 0.5f), default);
//            }

//            if (step.C.Exists)
//            {
//                if (step.A.Exists)
//                    renderer.Lines.Allocate() = new LineInstance(step.C.Support + basePosition, step.A.Support + basePosition, new Vector3(0, 0.6f, 0.1f), default);
//                //renderer.Lines.Allocate() = new LineInstance(step.C.Support + basePosition, basePosition, new Vector3(0, 0, 0.5f), default);
//            }

//            if (step.D.Exists)
//            {
//                if (step.A.Exists)
//                    renderer.Lines.Allocate() = new LineInstance(step.A.Support + basePosition, step.D.Support + basePosition, new Vector3(1, 0.6f, 0.1f), default);
//                if (step.B.Exists)
//                    renderer.Lines.Allocate() = new LineInstance(step.B.Support + basePosition, step.D.Support + basePosition, new Vector3(1, 0.6f, 0.1f), default);
//                if (step.C.Exists)
//                    renderer.Lines.Allocate() = new LineInstance(step.C.Support + basePosition, step.D.Support + basePosition, new Vector3(1, 0.6f, 0.1f), default);
//                //renderer.Lines.Allocate() = new LineInstance(step.D.Support + basePosition, basePosition, new Vector3(0, 0, 0.5f), default);
//            }

//            if (step.BestDepth >= 0)
//                renderer.Lines.Allocate() = new LineInstance(basePosition, basePosition + step.SearchTarget, new Vector3(1, 0, 0), default);
//            else
//            {
//                ContactLines.BuildOrthnormalBasis(step.BestNormal, out var x, out var y);

//                renderer.Lines.Allocate() = new LineInstance(basePosition, basePosition + step.BestNormal, new Vector3(1, 0, 0), default);
//                renderer.Lines.Allocate() = new LineInstance(basePosition + step.BestNormal * step.BestDepth, basePosition + x * 0.1f + step.BestNormal * step.BestDepth, new Vector3(1, 1, 0), default);
//            }
//            renderer.Lines.Allocate() = new LineInstance(basePosition + step.ClosestPointOnTriangle, basePosition + step.SearchTarget, new Vector3(0.5f, 0, 0), default);
//            renderer.Lines.Allocate() = new LineInstance(basePosition, basePosition + step.NextNormal, new Vector3(1, 0, 1), default);


//            base.Render(renderer, camera, input, text, font);
//        }
//    }
//}
