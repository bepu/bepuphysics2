using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using Quaternion = BepuUtilities.Quaternion;

namespace Demos.SpecializedTests
{
    using DemoRenderer.Constraints;
    using DemoRenderer.UI;
    using DemoUtilities;
    using CylinderNelderMead = NelderMead<Cylinder, CylinderWide, Cylinder, CylinderWide, CylinderPairDepthTester>;
    public class NelderMeadTestDemo : Demo
    {
        CylinderNelderMead.ExecutionDebugData debugData;
        float[,] depths;

        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Yaw = 0;
            camera.Pitch = 0;
            camera.Position = new Vector3(0, 0, 5);

            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, 0, 0)));


            var a = new Cylinder(0.5f, 1f);
            var b = new Cylinder(1f, 0.1f);
            var localOffsetB = new Vector3(0.5f, 0.6f, 0.7f);
            var localOrientationB = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 1, 1)), MathHelper.Pi * 1.5f);
            Simulation.Bodies.Add(BodyDescription.CreateConvexKinematic(new Vector3(), Simulation.Shapes, a));
            Simulation.Bodies.Add(BodyDescription.CreateConvexKinematic(new RigidPose(localOffsetB, localOrientationB), Simulation.Shapes, b));


            CylinderWide aWide = default, bWide = default;
            aWide.Broadcast(a);
            bWide.Broadcast(b);
            Vector3Wide.Broadcast(localOffsetB, out var localOffsetBWide);
            Matrix3x3Wide.Broadcast(Matrix3x3.CreateFromQuaternion(localOrientationB), out var localOrientationBWide);
            Vector3Wide.Broadcast(Vector3.Normalize(localOffsetB), out var initialNormalGuess);
            CylinderPairDepthTester depthTester = default;
            depthTester.Test(aWide, bWide, localOffsetBWide, localOrientationBWide, initialNormalGuess, out var depth);

            CylinderNelderMead.Refine(
                aWide, bWide, localOffsetBWide, localOrientationBWide, initialNormalGuess, depth, Vector<int>.Zero, new Vector<float>(1e-4f), new Vector<float>(-float.MaxValue),
                out var refinedNormal, out var refinedDepth, out debugData);
            min = new Vector2(-2);
            max = new Vector2(2);
            CylinderNelderMead.SampleDebugDepths(aWide, bWide, localOffsetBWide, localOrientationBWide, initialNormalGuess, min, max, new Int2(256, 256), out depths);
        }

        Vector2 min, max;
        int simplexIndex;

        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            if (input.TypedCharacters.Contains('z'))
            {
                simplexIndex = Math.Max(0, simplexIndex - 1);
            }
            if (input.TypedCharacters.Contains('x'))
            {
                simplexIndex = Math.Min(debugData.Simplices.Count - 1, simplexIndex + 1);
            }
            base.Update(window, camera, input, dt);
        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            renderer.TextBatcher.Write(text.Clear().Append("Simplex index ").Append(simplexIndex + 1).Append(" out of ").Append(debugData.Simplices.Count), new Vector2(32, renderer.Surface.Resolution.Y - 32), 16, new Vector3(1), font);
            var box = new Box(2f, 2f, 0.1f);
            var minDepth = float.MaxValue;
            var maxDepth = float.MinValue;
            for (int i = 0; i < depths.GetLength(0); ++i)
            {
                for (int j = 0; j < depths.GetLength(1); ++j)
                {
                    ref var depth = ref depths[i, j];
                    if (depth < minDepth)
                        minDepth = depth;
                    else if (depth > maxDepth)
                        maxDepth = depth;
                }
            }
            //Draw boxes colored according to the sampled background depths.
            //(such a super efficient way to render a texture,,,,,,)
            var inverseDepthSpan = 1f / (maxDepth - minDepth);
            RigidPose pose;
            pose.Orientation = Quaternion.Identity;
            var gridPosition = new Vector3(-200, -200, -200);
            for (int i = 0; i < depths.GetLength(0); ++i)
            {
                for (int j = 0; j < depths.GetLength(1); ++j)
                {
                    ref var depth = ref depths[i, j];
                    var normalizedDepth = (depth - minDepth) * inverseDepthSpan;
                    pose.Position = gridPosition + new Vector3(i * box.Width, j * box.Height, 0);
                    renderer.Shapes.AddShape(box, Simulation.Shapes, ref pose, new Vector3(normalizedDepth));
                }
            }

            //Draw the current simplex over the grid.
            var simplex = debugData.Simplices[simplexIndex];
            var sampleScale = new Vector2(box.Width * depths.GetLength(0), box.Height * depths.GetLength(1)) / (max - min);
            var gridMin = new Vector2(gridPosition.X - box.HalfWidth, gridPosition.Y - box.HalfHeight);
            var z = gridPosition.Z + box.HalfLength * 1.1f;
            Vector3 ToWorldSpace(in Vector2 samplePoint)
            {
                return new Vector3((samplePoint - min) * sampleScale + gridMin, z);
            }

            var a = ToWorldSpace(simplex.A);
            var b = ToWorldSpace(simplex.B);
            var c = ToWorldSpace(simplex.C);
            renderer.Lines.Allocate() = new LineInstance(a, b, new Vector3(1, 0, 0), default);
            renderer.Lines.Allocate() = new LineInstance(b, c, new Vector3(0, 1, 0), default);
            renderer.Lines.Allocate() = new LineInstance(c, a, new Vector3(0, 0, 1), default);
            renderer.TextBatcher.Write(text.Clear().Append("Simplex source: ").Append(simplex.Step.ToString()), new Vector2(32, renderer.Surface.Resolution.Y - 48), 16, new Vector3(1), font);
            base.Render(renderer, camera, input, text, font);
        }
    }
}

