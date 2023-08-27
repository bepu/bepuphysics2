﻿using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using Demos.Demos;
using DemoUtilities;
using System;
using System.Numerics;

namespace Demos.SpecializedTests.Media
{
    public class NewtVideoDemo : Demo
    {
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-5f, 5.5f, 5f);
            camera.Yaw = MathHelper.Pi / 4;
            camera.Pitch = MathHelper.Pi * 0.15f;

            var filters = new CollidableProperty<DeformableCollisionFilter>();
            Simulation = Simulation.Create(BufferPool, new DeformableCallbacks(filters), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));

            var meshContent = content.Load<MeshContent>("Content\\newt.obj");
            float cellSize = 0.1f;
            DumbTetrahedralizer.Tetrahedralize(meshContent.Triangles, cellSize, BufferPool,
                out var vertices, out var vertexSpatialIndices, out var cellVertexIndices, out var tetrahedraVertexIndices);
            var weldSpringiness = new SpringSettings(30f, 0);
            var volumeSpringiness = new SpringSettings(30f, 1);
            for (int i = 0; i < 5; ++i)
            {
                NewtDemo.CreateDeformable(Simulation, new Vector3(i * 3, 5 + i * 1.5f, 0), QuaternionEx.CreateFromAxisAngle(new Vector3(1, 0, 0), MathF.PI * (i * 0.55f)), 1f, cellSize, weldSpringiness, volumeSpringiness, i, filters, ref vertices, ref vertexSpatialIndices, ref cellVertexIndices, ref tetrahedraVertexIndices);
            }

            BufferPool.Return(ref vertices);
            vertexSpatialIndices.Dispose(BufferPool);
            BufferPool.Return(ref cellVertexIndices);
            BufferPool.Return(ref tetrahedraVertexIndices);

            Simulation.Bodies.Add(BodyDescription.CreateConvexDynamic(new Vector3(0, 100, -.5f), 10, Simulation.Shapes, new Sphere(5)));

            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -0.5f, 0), Simulation.Shapes.Add(new Box(1500, 1, 1500))));
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -1.5f, 0), Simulation.Shapes.Add(new Sphere(3))));

            var bulletShape = new Sphere(0.5f);
            bulletDescription = BodyDescription.CreateDynamic(RigidPose.Identity, bulletShape.ComputeInertia(.25f), Simulation.Shapes.Add(bulletShape), 0.01f);

            var mesh = DemoMeshHelper.LoadModel(content, BufferPool, "Content\\newt.obj", new Vector3(20));
            Simulation.Statics.Add(new StaticDescription(new Vector3(200, 0.5f, 120), QuaternionEx.CreateFromAxisAngle(Vector3.UnitY, -3 * MathHelper.PiOver4), Simulation.Shapes.Add(mesh)));
        }
        BodyDescription bulletDescription;
        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            if (input.WasPushed(OpenTK.Input.Key.Z))
            {
                bulletDescription.Pose.Position = camera.Position;
                bulletDescription.Velocity.Linear = camera.Forward * 40;
                Simulation.Bodies.Add(bulletDescription);
            }
            base.Update(window, camera, input, dt);
        }


    }
}
