﻿using BepuUtilities;
using DemoRenderer;
using BepuPhysics;
using BepuPhysics.Collidables;
using System;
using System.Numerics;
using DemoContentLoader;
using BepuPhysics.Constraints;
using DemoRenderer.UI;
using DemoUtilities;

namespace Demos.Demos
{
    public class CompoundDemo : Demo
    {
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-13f, 6, -13f);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = MathHelper.Pi * 0.05f;

            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10f, 0)), new SolveDescription(8, 1));

            using (var compoundBuilder = new CompoundBuilder(BufferPool, Simulation.Shapes, 8))
            {
                {
                    //Note that, in bepuphysics v2, there is no 'recentering' when constructing a shape. The pose you pass in for a child is exactly the pose that the compound will use, 
                    //even if the 'true' center of mass isn't at the local origin.
                    //Instead, if recentering is desired, it should performed ahead of time. The CompoundBuilder can help with this.
                    //We'll construct this compound using shapes far from the origin, and then use the CompoundBuilder overload that recenters the children and outputs the computed center.
                    var capsuleChildShape = new Capsule(0.5f, 0.5f);
                    var capsuleLocalPose = new RigidPose { Position = new Vector3(-0.5f, 4, 4), Orientation = Quaternion.Identity };
                    var boxChildShape = new Box(0.5f, 1f, 1.5f);
                    var boxLocalPose = new RigidPose { Position = new Vector3(0.5f, 4, 4), Orientation = Quaternion.Identity };

                    //All allocations from the buffer pool used for the final compound shape will be disposed when the demo is disposed. Don't have to worry about leaks in these demos.
                    compoundBuilder.Add(capsuleChildShape, capsuleLocalPose, 1);
                    compoundBuilder.Add(boxChildShape, boxLocalPose, 1);
                    compoundBuilder.BuildDynamicCompound(out var compoundChildren, out var compoundInertia, out var compoundCenter);
                    compoundBuilder.Reset();
                    Simulation.Bodies.Add(BodyDescription.CreateDynamic(compoundCenter, compoundInertia, Simulation.Shapes.Add(new Compound(compoundChildren)), 0.01f));
                }

                //Build a stack of sphere grids to stress manifold reduction heuristics in a convex-ish situation.
                {
                    var gridShape = new Sphere(0.5f);
                    const float gridSpacing = 1.5f;
                    const int gridWidth = 3;
                    var gridShapeIndex = Simulation.Shapes.Add(gridShape);
                    var gridBoxInertia = gridShape.ComputeInertia(1);
                    float localPoseOffset = -0.5f * gridSpacing * (gridWidth - 1);
                    for (int i = 0; i < gridWidth; ++i)
                    {
                        for (int j = 0; j < gridWidth; ++j)
                        {
                            compoundBuilder.Add(gridShapeIndex, new Vector3(localPoseOffset, 0, localPoseOffset) + new Vector3(gridSpacing) * new Vector3(i, 0, j), gridBoxInertia.InverseInertiaTensor, 1);
                        }
                    }
                    compoundBuilder.BuildDynamicCompound(out var gridChildren, out var gridInertia, out var center);
                    compoundBuilder.Reset();
                    var gridCompound = new Compound(gridChildren);
                    var bodyDescription = BodyDescription.CreateDynamic(RigidPose.Identity, gridInertia, Simulation.Shapes.Add(gridCompound), 0.01f);
                    for (int i = 0; i < 4; ++i)
                    {
                        bodyDescription.Pose.Position = new Vector3(0, 2 + i * 3, 0);
                        Simulation.Bodies.Add(bodyDescription);
                    }
                }

                //Build a table and use it for a couple of different tests. 
                {
                    var legShape = new Box(0.2f, 1, 0.2f);
                    var legInverseInertia = legShape.ComputeInertia(1f);
                    var legShapeIndex = Simulation.Shapes.Add(legShape);
                    var legPose0 = new RigidPose { Position = new Vector3(-1.5f, 0, -1.5f), Orientation = Quaternion.Identity };
                    var legPose1 = new RigidPose { Position = new Vector3(-1.5f, 0, 1.5f), Orientation = Quaternion.Identity };
                    var legPose2 = new RigidPose { Position = new Vector3(1.5f, 0, -1.5f), Orientation = Quaternion.Identity };
                    var legPose3 = new RigidPose { Position = new Vector3(1.5f, 0, 1.5f), Orientation = Quaternion.Identity };
                    compoundBuilder.Add(legShapeIndex, legPose0, legInverseInertia.InverseInertiaTensor, 1);
                    compoundBuilder.Add(legShapeIndex, legPose1, legInverseInertia.InverseInertiaTensor, 1);
                    compoundBuilder.Add(legShapeIndex, legPose2, legInverseInertia.InverseInertiaTensor, 1);
                    compoundBuilder.Add(legShapeIndex, legPose3, legInverseInertia.InverseInertiaTensor, 1);
                    var tableTopPose = new RigidPose { Position = new Vector3(0, 0.6f, 0), Orientation = Quaternion.Identity };
                    var tableTopShape = new Box(3.2f, 0.2f, 3.2f);
                    compoundBuilder.Add(tableTopShape, tableTopPose, 3);

                    compoundBuilder.BuildDynamicCompound(out var tableChildren, out var tableInertia, out var tableCenter);
                    compoundBuilder.Reset();
                    var table = new Compound(tableChildren);
                    var tableDescription = BodyDescription.CreateDynamic(RigidPose.Identity, tableInertia, Simulation.Shapes.Add(table), 0.01f);

                    //Stack some tables.
                    {
                        for (int i = 0; i < 10; ++i)
                        {
                            tableDescription.Pose.Position = new Vector3(10, 3 + i * 1.4f, 10);
                            Simulation.Bodies.Add(tableDescription);
                        }
                    }
                    {
                        for (int k = 0; k < 5; ++k)
                        {
                            tableDescription.Pose.Position = new Vector3(64 + k * 3, 6 + k * 1.4f, 32);
                            Simulation.Bodies.Add(tableDescription);
                        }
                        //for (int i = 0; i < 10; ++i)
                        //{
                        //    for (int j = 0; j < 20; ++j)
                        //    {
                        //        for (int k = 0; k < 10; ++k)
                        //        {
                        //            tableDescription.Pose.Position = new Vector3(32 + i * 6, 6 + j * 1.4f, 16 + k * 6);
                        //            Simulation.Bodies.Add(tableDescription);
                        //        }
                        //    }
                        //}
                    }

                    //Put a table on top of a sphere to stress out nonconvex reduction for divergent normals.
                    {
                        tableDescription.Pose.Position = new Vector3(10, 6, 0);
                        Simulation.Bodies.Add(tableDescription);

                        var sphereShape = new Sphere(3);
                        var sphereIndex = Simulation.Shapes.Add(sphereShape);
                        var sphereDescription = new StaticDescription(new Vector3(10, 2, 0), sphereIndex);
                        Simulation.Statics.Add(sphereDescription);
                    }

                    //Put another table on the ground, but with a clamp-ish thing on it that generates opposing normals.
                    {
                        tableDescription.Pose.Position = new Vector3(10, 3, -10);
                        Simulation.Bodies.Add(tableDescription);

                        var clampPieceShape = new Box(2f, 0.1f, 0.3f);
                        var clampPieceInverseInertia = clampPieceShape.ComputeInertia(1f);
                        var clampPieceShapeIndex = Simulation.Shapes.Add(clampPieceShape);
                        var clamp0 = new RigidPose { Position = new Vector3(0, -0.2f, -1.1f), Orientation = Quaternion.Identity };
                        var clamp1 = new RigidPose { Position = new Vector3(0, 0.2f, -1.1f), Orientation = Quaternion.Identity };
                        var clamp2 = new RigidPose { Position = new Vector3(0, -0.2f, 0), Orientation = Quaternion.Identity };
                        var clamp3 = new RigidPose { Position = new Vector3(0, 0.2f, 0), Orientation = Quaternion.Identity };
                        var clamp4 = new RigidPose { Position = new Vector3(0, -0.2f, 1.1f), Orientation = Quaternion.Identity };
                        var clamp5 = new RigidPose { Position = new Vector3(0, 0.2f, 1.1f), Orientation = Quaternion.Identity };
                        compoundBuilder.Add(clampPieceShapeIndex, clamp0, clampPieceInverseInertia.InverseInertiaTensor, 1);
                        compoundBuilder.Add(clampPieceShapeIndex, clamp1, clampPieceInverseInertia.InverseInertiaTensor, 1);
                        compoundBuilder.Add(clampPieceShapeIndex, clamp2, clampPieceInverseInertia.InverseInertiaTensor, 1);
                        compoundBuilder.Add(clampPieceShapeIndex, clamp3, clampPieceInverseInertia.InverseInertiaTensor, 1);
                        compoundBuilder.Add(clampPieceShapeIndex, clamp4, clampPieceInverseInertia.InverseInertiaTensor, 1);
                        compoundBuilder.Add(clampPieceShapeIndex, clamp5, clampPieceInverseInertia.InverseInertiaTensor, 1);

                        compoundBuilder.BuildDynamicCompound(out var clampChildren, out var clampInertia, out var clampCenter);
                        compoundBuilder.Reset();
                        var clamp = new Compound(clampChildren);
                        var clampDescription = BodyDescription.CreateDynamic(
                            tableDescription.Pose.Position + new Vector3(2f, 0.3f, 0), clampInertia, Simulation.Shapes.Add(clamp), 0.01f);
                        Simulation.Bodies.Add(clampDescription);
                    }

                }

                //Create a tree-accelerated big compound.
                {
                    var random = new Random(5);
                    var treeCompoundBoxShape = new Box(0.5f, 1.5f, 1f);
                    var treeCompoundBoxShapeIndex = Simulation.Shapes.Add(treeCompoundBoxShape);
                    var childInertia = treeCompoundBoxShape.ComputeInertia(1);
                    for (int i = 0; i < 128; ++i)
                    {
                        RigidPose localPose;
                        localPose.Position = new Vector3(12, 6, 12) * (0.5f * new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle()) - Vector3.One);
                        float orientationLengthSquared;
                        do
                        {
                            localPose.Orientation = new Quaternion(random.NextSingle(), random.NextSingle(), random.NextSingle(), random.NextSingle());
                            orientationLengthSquared = QuaternionEx.LengthSquared(ref localPose.Orientation);
                        }
                        while (orientationLengthSquared < 1e-9f);
                        QuaternionEx.Scale(localPose.Orientation, 1f / MathF.Sqrt(orientationLengthSquared), out localPose.Orientation);
                        //Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), MathF.PI, out localPose.Orientation);

                        compoundBuilder.Add(treeCompoundBoxShapeIndex, localPose, childInertia.InverseInertiaTensor, 1);
                    }
                    compoundBuilder.BuildDynamicCompound(out var children, out var inertia, out var center);
                    compoundBuilder.Reset();

                    var compound = new BigCompound(children, Simulation.Shapes, BufferPool);
                    //var compound = new Compound(children);
                    var compoundIndex = Simulation.Shapes.Add(compound);
                    for (int i = 0; i < 8; ++i)
                    {
                        Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(0, 4 + 5 * i, 32), inertia, compoundIndex, 0.01f));
                    }
                }
            }

            //Prevent stuff from falling into the infinite void.
            {
                var boxShape = new Box(256, 1, 256);
                var groundShapeIndex = Simulation.Shapes.Add(boxShape);
                var groundDescription = new StaticDescription(RigidPose.Identity, groundShapeIndex);
                Simulation.Statics.Add(groundDescription);
            }
            const int planeWidth = 48;
            const int planeHeight = 48;
            var planeMesh = DemoMeshHelper.CreateDeformedPlane(planeWidth, planeHeight,
                (int x, int y) =>
                        {
                            Vector2 offsetFromCenter = new Vector2(x - planeWidth / 2, y - planeHeight / 2);
                            return new Vector3(offsetFromCenter.X, MathF.Cos(x / 4f) * MathF.Sin(y / 4f) - 0.01f * offsetFromCenter.LengthSquared(), offsetFromCenter.Y);
                        }, new Vector3(2, 1, 2), BufferPool);
            Simulation.Statics.Add(new StaticDescription(new Vector3(64, 4, 32), QuaternionEx.CreateFromAxisAngle(new Vector3(0, 1, 0), MathF.PI / 2), Simulation.Shapes.Add(planeMesh)));
        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            var bottomY = renderer.Surface.Resolution.Y;
            renderer.TextBatcher.Write(text.Clear().Append("There are two type of compounds built in: Compound and BigCompound."), new Vector2(16, bottomY - 80), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Compounds lack an acceleration structure, so they're good for low overhead compounds with few children."), new Vector2(16, bottomY - 64), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("BigCompounds contain an acceleration structure to keep more complex shapes fast."), new Vector2(16, bottomY - 48), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("This demo throws some different compounds together and stress tests their contact generation behavior."), new Vector2(16, bottomY - 16), 16, Vector3.One, font);
            base.Render(renderer, camera, input, text, font);
        }
    }
}
