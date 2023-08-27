﻿using BepuUtilities;
using DemoRenderer;
using BepuPhysics;
using BepuPhysics.Collidables;
using System.Numerics;
using BepuPhysics.Constraints;
using DemoContentLoader;

namespace Demos.SpecializedTests
{
    public class ClothLatticeDemo : Demo
    {
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-120, 30, -120);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = 0.1f;
            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));

            //Build a grid of shapes to be connected.
            var clothNodeShape = new Sphere(0.5f);
            var clothNodeInertia = clothNodeShape.ComputeInertia(1);
            var clothNodeShapeIndex = Simulation.Shapes.Add(clothNodeShape);
            const int width = 128;
            const int length = 128;
            const float spacing = 1.75f;
            BodyHandle[][] nodeHandles = new BodyHandle[width][];
            for (int i = 0; i < width; ++i)
            {
                nodeHandles[i] = new BodyHandle[length];
                for (int j = 0; j < length; ++j)
                {
                    var location = new Vector3(0, 30, 0) + new Vector3(spacing, 0, spacing) * (new Vector3(i, 0, j) + new Vector3(-width * 0.5f, 0, -length * 0.5f));
                    var bodyDescription = BodyDescription.CreateDynamic(location, clothNodeInertia, new(clothNodeShapeIndex, 0.1f), 0.01f);
                    nodeHandles[i][j] = Simulation.Bodies.Add(bodyDescription);

                }
            }
            //Construct some joints between the nodes.
            var left = new BallSocket
            {
                LocalOffsetA = new Vector3(-spacing * 0.5f, 0, 0),
                LocalOffsetB = new Vector3(spacing * 0.5f, 0, 0),
                SpringSettings = new SpringSettings(10, 1)
            };
            var up = new BallSocket
            {
                LocalOffsetA = new Vector3(0, 0, -spacing * 0.5f),
                LocalOffsetB = new Vector3(0, 0, spacing * 0.5f),
                SpringSettings = new SpringSettings(10, 1)
            };
            var leftUp = new BallSocket
            {
                LocalOffsetA = new Vector3(-spacing * 0.5f, 0, -spacing * 0.5f),
                LocalOffsetB = new Vector3(spacing * 0.5f, 0, spacing * 0.5f),
                SpringSettings = new SpringSettings(10, 1)
            };
            var rightUp = new BallSocket
            {
                LocalOffsetA = new Vector3(spacing * 0.5f, 0, -spacing * 0.5f),
                LocalOffsetB = new Vector3(-spacing * 0.5f, 0, spacing * 0.5f),
                SpringSettings = new SpringSettings(10, 1)
            };
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < length; ++j)
                {
                    if (i >= 1)
                        Simulation.Solver.Add(nodeHandles[i][j], nodeHandles[i - 1][j], left);
                    if (j >= 1)
                        Simulation.Solver.Add(nodeHandles[i][j], nodeHandles[i][j - 1], up);
                    if (i >= 1 && j >= 1)
                        Simulation.Solver.Add(nodeHandles[i][j], nodeHandles[i - 1][j - 1], leftUp);
                    if (i < width - 1 && j >= 1)
                        Simulation.Solver.Add(nodeHandles[i][j], nodeHandles[i + 1][j - 1], rightUp);
                }
            }
            var bigBallShape = new Sphere(25);
            var bigBallShapeIndex = Simulation.Shapes.Add(bigBallShape);

            var bigBallDescription = new StaticDescription(new Vector3(-10, -15, 0), bigBallShapeIndex);
            Simulation.Statics.Add(bigBallDescription);

            var groundShape = new Box(200, 1, 200);
            var groundShapeIndex = Simulation.Shapes.Add(groundShape);

            var groundDescription = new StaticDescription(new Vector3(0, -10, 0), groundShapeIndex);
            Simulation.Statics.Add(groundDescription);
        }

    }
}


