using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using BepuPhysics.Collidables;
using BepuUtilities.Collections;
using DemoContentLoader;
using DemoRenderer;
using BepuPhysics;
using DemoRenderer.UI;
using DemoUtilities;
using DemoRenderer.Constraints;
using static BepuPhysics.Collidables.ConvexHullHelper;
using System.Diagnostics;
using BepuUtilities;
using BepuPhysics.Constraints.Contact;

namespace Demos.SpecializedTests
{
    public class ConvexHullTestDemo : Demo
    {
        QuickList<Vector3> points;
        //List<DebugStep> debugSteps;
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, -2.5f, 10);
            camera.Yaw = 0;
            camera.Pitch = 0;

            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new PositionFirstTimestepper());

            //var meshContent = content.Load<MeshContent>("Content\\newt.obj");

            ////This is actually a pretty good example of how *not* to make a convex hull shape.
            ////Generating it directly from a graphical data source tends to have way more surface complexity than needed,
            ////and it tends to have a lot of near-but-not-quite-coplanar surfaces which can make the contact manifold less stable.
            ////Prefer a simpler source with more distinct features, possibly created with an automated content-time tool.
            //points = new QuickList<Vector3>(meshContent.Triangles.Length * 3, BufferPool);
            //for (int i = 0; i < meshContent.Triangles.Length; ++i)
            //{
            //    ref var triangle = ref meshContent.Triangles[i];
            //    //resisting the urge to just reinterpret the memory
            //    points.AllocateUnsafely() = triangle.A * new Vector3(1, 1.5f, 1);
            //    points.AllocateUnsafely() = triangle.B * new Vector3(1, 1.5f, 1);
            //    points.AllocateUnsafely() = triangle.C * new Vector3(1, 1.5f, 1);
            //}

            const int pointCount = 50;
            points = new QuickList<Vector3>(pointCount * 2, BufferPool);
            //points.Allocate(BufferPool) = new Vector3(0, 0, 0);
            //points.Allocate(BufferPool) = new Vector3(0, 0, 1);
            //points.Allocate(BufferPool) = new Vector3(0, 1, 0);
            //points.Allocate(BufferPool) = new Vector3(0, 1, 1);
            //points.Allocate(BufferPool) = new Vector3(1, 0, 0);
            //points.Allocate(BufferPool) = new Vector3(1, 0, 1);
            //points.Allocate(BufferPool) = new Vector3(1, 1, 0);
            //points.Allocate(BufferPool) = new Vector3(1, 1, 1);
            var random = new Random(5);
            for (int i = 0; i < pointCount; ++i)
            {
                points.AllocateUnsafely() = new Vector3(3 * random.NextSingle(), 1 * random.NextSingle(), 3 * random.NextSingle());
                //points.AllocateUnsafely() = new Vector3(0, 1, 0) + Vector3.Normalize(new Vector3(random.NextSingle() * 2 - 1, random.NextSingle() * 2 - 1, random.NextSingle() * 2 - 1)) * random.NextSingle();
            }

            var pointsBuffer = points.Span.Slice(points.Count);
            CreateShape(pointsBuffer, BufferPool, out _, out var hullShape);
            //ConvexHullHelper.ComputeHull(pointsBuffer, BufferPool, out _, out debugSteps);

            Matrix3x3.CreateScale(new Vector3(5, 0.5f, 3), out var scale);
            var transform = Matrix3x3.CreateFromAxisAngle(Vector3.Normalize(new Vector3(3, 2, 1)), 1207) * scale;
            const int transformCount = 10000;
            var transformStart = Stopwatch.GetTimestamp();
            for (int i = 0; i < transformCount; ++i)
            {
                CreateTransformedCopy(hullShape, transform, BufferPool, out var transformedHullShape);
                transformedHullShape.Dispose(BufferPool);
            }
            var transformEnd = Stopwatch.GetTimestamp();
            Console.WriteLine($"Transform hull computation time (us): {(transformEnd - transformStart) * 1e6 / (transformCount * Stopwatch.Frequency)}");

            hullShape.RayTest(RigidPose.Identity, new Vector3(0, 1, 0), -Vector3.UnitY, out var t, out var normal);

            const int rayIterationCount = 10000;
            var rayPose = RigidPose.Identity;
            var rayOrigin = new Vector3(0, 2, 0);
            var rayDirection = new Vector3(0, -1, 0);

            int hitCounter = 0;
            var start = Stopwatch.GetTimestamp();
            for (int i = 0; i < rayIterationCount; ++i)
            {
                if (hullShape.RayTest(rayPose, rayOrigin, rayDirection, out _, out _))
                {
                    ++hitCounter;
                }
            }
            var end = Stopwatch.GetTimestamp();
            Console.WriteLine($"Hit counter: {hitCounter}, computation time (us): {(end - start) * 1e6 / (rayIterationCount * Stopwatch.Frequency)}");

            const int iterationCount = 100;
            start = Stopwatch.GetTimestamp();
            for (int i = 0; i < iterationCount; ++i)
            {
                CreateShape(pointsBuffer, BufferPool, out _, out var perfTestShape);
                perfTestShape.Dispose(BufferPool);
            }
            end = Stopwatch.GetTimestamp();
            Console.WriteLine($"Hull computation time (us): {(end - start) * 1e6 / (iterationCount * Stopwatch.Frequency)}");

            var hullShapeIndex = Simulation.Shapes.Add(hullShape);
            hullShape.ComputeInertia(1, out var inertia);

            Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(0, 0, 0), inertia, new (hullShapeIndex, ContinuousDetection.Discrete(20, 20)), 0.01f));

            Simulation.Statics.Add(new StaticDescription(new Vector3(-25, -5, 0), Simulation.Shapes.Add(new Sphere(2))));
            Simulation.Statics.Add(new StaticDescription(new Vector3(-20, -5, 0), Simulation.Shapes.Add(new Capsule(0.5f, 2))));
            Simulation.Statics.Add(new StaticDescription(new Vector3(-15, -5, 0), Simulation.Shapes.Add(new Box(2f, 2f, 2f))));
            Simulation.Statics.Add(new StaticDescription(new Vector3(-10, -5, 5), Simulation.Shapes.Add(new Triangle { A = new Vector3(0, 0, -10), B = new Vector3(5, 0, -10), C = new Vector3(0, 0, -5) })));
            Simulation.Statics.Add(new StaticDescription(new Vector3(-5, -5, 0), Simulation.Shapes.Add(new Cylinder(1, 1))));
            Simulation.Statics.Add(new StaticDescription(new Vector3(-5, -5, 5), Simulation.Shapes.Add(new Cylinder(1, 1))));
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -5, 0), hullShapeIndex));

            var spacing = new Vector3(3f, 3f, 3);
            int width = 16;
            int height = 16;
            int length = 16;
            var origin = -0.5f * spacing * new Vector3(width, 0, length) + new Vector3(40, 0.2f, -40);
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    for (int k = 0; k < length; ++k)
                    {
                        Simulation.Bodies.Add(BodyDescription.CreateDynamic(
                            new RigidPose(origin + spacing * new Vector3(i, j, k), QuaternionEx.CreateFromAxisAngle(new Vector3(0, 1, 0), MathHelper.Pi * 0.05f)),
                            inertia, hullShapeIndex, 0.01f));
                    }
                }
            }
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -10, 0), Simulation.Shapes.Add(new Box(1000, 1, 1000))));
        }

        void TestConvexHullCreation()
        {
            var random = new Random(5);
            for (int iterationIndex = 0; iterationIndex < 100000; ++iterationIndex)
            {
                const int pointCount = 32;
                var points = new QuickList<Vector3>(pointCount, BufferPool);
                for (int i = 0; i < pointCount; ++i)
                {
                    points.AllocateUnsafely() = new Vector3(1 * random.NextSingle(), 2 * random.NextSingle(), 3 * random.NextSingle());
                }

                var pointsBuffer = points.Span.Slice(points.Count);
                CreateShape(pointsBuffer, BufferPool, out _, out var hullShape);

                hullShape.Dispose(BufferPool);
            }
        }

        int stepIndex = 0;

        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            //if (input.TypedCharacters.Contains('x'))
            //{
            //    stepIndex = Math.Max(stepIndex - 1, 0);
            //}
            //if (input.TypedCharacters.Contains('c'))
            //{
            //    stepIndex = Math.Min(stepIndex + 1, debugSteps.Count - 1);
            //}
            base.Update(window, camera, input, dt);
        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            //var step = debugSteps[stepIndex];
            //var scale = 10f;
            //for (int i = 0; i < points.Count; ++i)
            //{
            //    var pose = new RigidPose(points[i] * scale);
            //    renderer.Shapes.AddShape(new Box(0.1f, 0.1f, 0.1f), Simulation.Shapes, ref pose, new Vector3(0.5f, 0.5f, 0.5f));
            //    if (!step.AllowVertex[i])
            //        renderer.Shapes.AddShape(new Box(0.6f, 0.25f, 0.25f), Simulation.Shapes, ref pose, new Vector3(1, 0, 0));
            //}
            //for (int i = 0; i < step.Raw.Count; ++i)
            //{
            //    var pose = new RigidPose(points[step.Raw[i]] * scale);
            //    renderer.Shapes.AddShape(new Box(0.25f, 0.6f, 0.25f), Simulation.Shapes, ref pose, new Vector3(0, 0, 1));
            //}
            //for (int i = 0; i < step.Reduced.Count; ++i)
            //{
            //    var pose = new RigidPose(points[step.Reduced[i]] * scale);
            //    renderer.Shapes.AddShape(new Box(0.25f, 0.25f, 0.6f), Simulation.Shapes, ref pose, new Vector3(0, 1, 0));
            //}
            //for (int i = 0; i <= stepIndex; ++i)
            //{
            //    var pose = RigidPose.Identity;
            //    var oldStep = debugSteps[i];
            //    for (int j = 2; j < oldStep.Reduced.Count; ++j)
            //    {
            //        renderer.Shapes.AddShape(new Triangle
            //        {
            //            A = points[oldStep.Reduced[0]] * scale,
            //            B = points[oldStep.Reduced[j]] * scale,
            //            C = points[oldStep.Reduced[j - 1]] * scale
            //        }, Simulation.Shapes, ref pose, new Vector3(1, 0, 1));

            //    }
            //}
            //var edgeMidpoint = (points[step.SourceEdge.A] + points[step.SourceEdge.B]) * scale * 0.5f;
            //renderer.Lines.Allocate() = new LineInstance(edgeMidpoint, edgeMidpoint + step.BasisX * scale * 0.5f, new Vector3(1, 1, 0), new Vector3());
            //renderer.Lines.Allocate() = new LineInstance(edgeMidpoint, edgeMidpoint + step.BasisY * scale * 0.5f, new Vector3(0, 1, 0), new Vector3());
            //renderer.TextBatcher.Write(
            //    text.Clear().Append($"Enumerate step with X and C. Current step: ").Append(stepIndex + 1).Append(" out of ").Append(debugSteps.Count),
            //    new Vector2(32, renderer.Surface.Resolution.Y - 140), 20, new Vector3(1), font);
            base.Render(renderer, camera, input, text, font);
        }
    }
}
