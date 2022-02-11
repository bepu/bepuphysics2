using BepuUtilities;
using DemoRenderer;
using BepuPhysics;
using BepuPhysics.Collidables;
using System.Numerics;
using System;
using DemoContentLoader;
using Demos.Demos;
using DemoUtilities;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using DemoRenderer.UI;

namespace Demos.SpecializedTests.Media
{
    /// <summary>
    /// Subjects a bunch of unfortunate ragdolls to a tumble dry cycle.
    /// </summary>
    public class RagdollTubeVideoDemo : Demo
    {
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 9, -40);
            camera.Yaw = MathHelper.Pi;
            camera.Pitch = 0;
            var filters = new CollidableProperty<SubgroupCollisionFilter>();
            //Note the lowered material stiffness compared to many of the other demos. Ragdolls aren't made of concrete.
            //Increasing the maximum recovery velocity helps keep deeper contacts strong, stopping objects from interpenetrating.
            //Higher friction helps the bodies clump and flop, rather than just sliding down the slope in the tube.
            Simulation = Simulation.Create(BufferPool, new SubgroupFilteredCallbacks(filters, new PairMaterialProperties(2, float.MaxValue, new SpringSettings(10, 1))), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(4, 1));

            int ragdollIndex = 0;
            var spacing = new Vector3(1.7f, 1.8f, 0.5f);
            int width = 4;
            int height = 4;
            int length = 120;
            var origin = -0.5f * spacing * new Vector3(width - 1, 0, length - 1) + new Vector3(0, 5f, 0);
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    for (int k = 0; k < length; ++k)
                    {
                        RagdollDemo.AddRagdoll(origin + spacing * new Vector3(i, j, k), QuaternionEx.CreateFromAxisAngle(new Vector3(0, 1, 0), MathHelper.Pi * 0.05f), ragdollIndex++, filters, Simulation);
                    }
                }
            }

            ragdollCount = ragdollIndex;
            ragdollBodyCount = Simulation.Bodies.ActiveSet.Count;
            ragdollConstraintCount = Simulation.Solver.CountConstraints();

            var tubeCenter = new Vector3(0, 8, 0);
            const int panelCount = 20;
            const float tubeRadius = 6;
            var panelShape = new Box(MathF.PI * 2 * tubeRadius / panelCount, 1, 100);
            var panelShapeIndex = Simulation.Shapes.Add(panelShape);
            var builder = new CompoundBuilder(BufferPool, Simulation.Shapes, panelCount + 1);
            for (int i = 0; i < panelCount; ++i)
            {
                var rotation = QuaternionEx.CreateFromAxisAngle(Vector3.UnitZ, i * MathHelper.TwoPi / panelCount);
                QuaternionEx.TransformUnitY(rotation, out var localUp);
                var position = localUp * tubeRadius;
                builder.AddForKinematic(panelShapeIndex, (position, rotation), 1);
            }
            builder.AddForKinematic(Simulation.Shapes.Add(new Box(1, 2, panelShape.Length)), new Vector3(0, tubeRadius - 1, 0), 0);
            builder.BuildKinematicCompound(out var children);
            var compound = new BigCompound(children, Simulation.Shapes, BufferPool);
            var tubeHandle = Simulation.Bodies.Add(BodyDescription.CreateKinematic(tubeCenter, (default, new Vector3(0, 0, .25f)), Simulation.Shapes.Add(compound), 0f));
            filters[tubeHandle] = new SubgroupCollisionFilter(int.MaxValue);
            builder.Dispose();

            var staticShape = new Box(300, 1, 300);
            var staticShapeIndex = Simulation.Shapes.Add(staticShape);
            var staticDescription = new StaticDescription(new Vector3(0, -0.5f, 0), staticShapeIndex);
            Simulation.Statics.Add(staticDescription);

            DemoMeshHelper.LoadModel(content, BufferPool, @"Content\newt.obj", new Vector3(15, 15, 15), out var newtMesh);
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, 0.5f, 80), Quaternion.CreateFromAxisAngle(Vector3.UnitY, MathF.PI), Simulation.Shapes.Add(newtMesh)));

        }
        int ragdollBodyCount;
        int ragdollConstraintCount;
        int ragdollCount;

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            var resolution = renderer.Surface.Resolution;
            renderer.TextBatcher.Write(text.Clear().Append("Ragdoll count:"), new Vector2(16, resolution.Y - 64), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Ragdoll body count:"), new Vector2(16, resolution.Y - 48), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Ragdoll constraint count:"), new Vector2(16, resolution.Y - 32), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Collision constraint count:"), new Vector2(16, resolution.Y - 16), 16, Vector3.One, font);
            const float xOffset = 192;
            renderer.TextBatcher.Write(text.Clear().Append(ragdollCount), new Vector2(xOffset, resolution.Y - 64), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append(ragdollBodyCount), new Vector2(xOffset, resolution.Y - 48), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append(ragdollConstraintCount), new Vector2(xOffset, resolution.Y - 32), 16, Vector3.One, font);
            var collisionConstraintCount = Simulation.Solver.CountConstraints() - ragdollConstraintCount;
            renderer.TextBatcher.Write(text.Clear().Append(collisionConstraintCount), new Vector2(xOffset, resolution.Y - 16), 16, Vector3.One, font);
            base.Render(renderer, camera, input, text, font);
        }
    }
}


