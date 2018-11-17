using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using Demos.Demos;

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

            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));

            MeshDemo.LoadModel(content, BufferPool, @"Content\newt.obj", Vector3.One, out var mesh);
            new Box(2.5f, 1, 4).ComputeInertia(1, out var approximateInertia);
            var meshShapeIndex = Simulation.Shapes.Add(mesh);
            for (int newtIndex = 0; newtIndex < 3; ++newtIndex)
            {
                Simulation.Bodies.Add(
                    BodyDescription.CreateDynamic(new Vector3(0, 2 + newtIndex * 2, 0), approximateInertia,
                    new CollidableDescription(meshShapeIndex, 0.1f), new BodyActivityDescription(0.01f)));
            }

            var staticShape = new Box(1500, 1, 1500);
            var staticShapeIndex = Simulation.Shapes.Add(staticShape);

            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -0.5f, 0), new CollidableDescription(staticShapeIndex, 0.1f)));
        }
    }
}
