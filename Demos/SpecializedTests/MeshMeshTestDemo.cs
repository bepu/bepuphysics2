﻿using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;

namespace Demos.SpecializedTests
{
    /// <summary>
    /// Shows how to be mean to the physics engine by using meshes as dynamic colliders. Why would someone be so cruel? Be nice to the physics engine, save your CPU some work.
    /// </summary>
    public class MeshMeshTestDemo : Demo
    {
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 8, -10);
            camera.Yaw = MathHelper.Pi;

            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));

            var mesh = DemoMeshHelper.LoadModel(content, BufferPool, @"Content\newt.obj", Vector3.One);
            var approximateInertia = new Box(2.5f, 1, 4).ComputeInertia(1);
            var meshShapeIndex = Simulation.Shapes.Add(mesh);
            for (int meshIndex = 0; meshIndex < 3; ++meshIndex)
            {
                Simulation.Bodies.Add(
                    BodyDescription.CreateDynamic(new Vector3(0, 2 + meshIndex * 2, 0), approximateInertia, meshShapeIndex, 0.01f));
            }

            var compoundBuilder = new CompoundBuilder(BufferPool, Simulation.Shapes, 12);
            for (int i = 0; i < mesh.Triangles.Length; ++i)
            {
                compoundBuilder.Add(mesh.Triangles[i], RigidPose.Identity, 1);
            }
            compoundBuilder.BuildDynamicCompound(out var children, out var compoundInertia);
            var compound = new BigCompound(children, Simulation.Shapes, BufferPool);
            var compoundShapeIndex = Simulation.Shapes.Add(compound);
            compoundBuilder.Dispose();
            for (int i = 0; i < 3; ++i)
            {
                Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(5, 2 + i * 2, 0), compoundInertia, compoundShapeIndex, 0.01f));
            }

            var staticShape = new Box(1500, 1, 1500);
            var staticShapeIndex = Simulation.Shapes.Add(staticShape);

            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -0.5f, 0), staticShapeIndex));
        }
    }
}
