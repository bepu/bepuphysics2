﻿using BepuUtilities;
using DemoRenderer;
using BepuPhysics;
using BepuPhysics.Collidables;
using System;
using System.Numerics;
using DemoContentLoader;
using BepuPhysics.Constraints;

namespace Demos.SpecializedTests
{
    public class MeshTestDemo : Demo
    {
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-10, 0, -10);
            //camera.Yaw = MathHelper.Pi ; 
            camera.Yaw = MathHelper.Pi * 3f / 4;
            //camera.Pitch = MathHelper.PiOver2 * 0.999f;
            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));

            var box = new Box(1f, 3f, 2f);
            var capsule = new Capsule(1f, 1f);
            var sphere = new Sphere(1f);
            var boxInertia = box.ComputeInertia(1);
            var capsuleInertia = capsule.ComputeInertia(1);
            var sphereInertia = sphere.ComputeInertia(1);
            var boxIndex = Simulation.Shapes.Add(box);
            var capsuleIndex = Simulation.Shapes.Add(capsule);
            var sphereIndex = Simulation.Shapes.Add(sphere);
            const int width = 16;
            const int height = 3;
            const int length = 16;
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    for (int k = 0; k < length; ++k)
                    {
                        var location = new Vector3(5, 5, 5) * new Vector3(i, j, k);// + new Vector3(-width * 1.5f, 1.5f, -length * 1.5f);
                        var bodyDescription = BodyDescription.CreateDynamic(location, default, default, 0.01f);
                        switch ((i + j) % 3)
                        {
                            case 0:
                                bodyDescription.Collidable.Shape = sphereIndex;
                                bodyDescription.LocalInertia = sphereInertia;
                                break;
                            case 1:
                                bodyDescription.Collidable.Shape = capsuleIndex;
                                bodyDescription.LocalInertia = capsuleInertia;
                                break;
                            case 2:
                                bodyDescription.Collidable.Shape = boxIndex;
                                bodyDescription.LocalInertia = boxInertia;
                                break;
                        }
                        Simulation.Bodies.Add(bodyDescription);

                    }
                }
            }

            //var testShape = new Box(50, 2, 0.2f);
            //testShape.ComputeInertia(1, out var testInertia);
            //Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(10, 10, 10), testInertia, new CollidableDescription(Simulation.Shapes.Add(testShape), 10.1f), new BodyActivityDescription(-0.01f)));


            var newtMesh = DemoMeshHelper.LoadModel(content, BufferPool, @"Content\newt.obj", new Vector3(5, 5, 5));
            Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(30, 20, 30), newtMesh.ComputeClosedInertia(10), Simulation.Shapes.Add(newtMesh), 0.01f));

            Simulation.Statics.Add(new StaticDescription(new Vector3(30, 15, 30), Simulation.Shapes.Add(new Box(15, 1, 15))));

            var boxMesh = DemoMeshHelper.LoadModel(content, BufferPool, @"Content\box.obj", new Vector3(5, 1, 5));
            Simulation.Statics.Add(new StaticDescription(new Vector3(10, 5, -20), Simulation.Shapes.Add(boxMesh)));

            var fanMesh = DemoMeshHelper.CreateFan(64, 16, new Vector3(1, 1, 1), BufferPool);
            Simulation.Statics.Add(new StaticDescription(new Vector3(-10, 0, -20), Simulation.Shapes.Add(fanMesh)));

            const int planeWidth = 128;
            const int planeHeight = 128;
            var planeMesh = DemoMeshHelper.CreateDeformedPlane(planeWidth, planeHeight,
                (int x, int y) =>
                {
                    return new Vector3(x - planeWidth / 2, 1 * MathF.Cos(x / 2f) * MathF.Sin(y / 2f), y - planeHeight / 2);
                }, new Vector3(2, 1, 2), BufferPool);
            Simulation.Statics.Add(new StaticDescription(new Vector3(64, -10, 64), BepuUtilities.QuaternionEx.CreateFromAxisAngle(new Vector3(0, 1, 0), MathF.PI / 2), Simulation.Shapes.Add(planeMesh)));
        }



    }
}


