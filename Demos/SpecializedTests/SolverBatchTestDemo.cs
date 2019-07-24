using BepuUtilities;
using DemoRenderer;
using DemoUtilities;
using BepuPhysics;
using BepuPhysics.Collidables;
using System;
using System.Numerics;
using System.Diagnostics;
using BepuUtilities.Memory;
using BepuUtilities.Collections;
using BepuPhysics.Constraints;
using DemoContentLoader;

namespace Demos.Demos
{
    public class SolverBatchTestDemo : Demo
    {
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-120, 30, -120);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = 0.1f;
            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));
            Simulation.Solver.IterationCount = 8;

            //Build a grid of shapes to be connected.
            var clothNodeShape = new Sphere(0.5f);
            clothNodeShape.ComputeInertia(1, out var clothNodeInertia);
            var clothNodeShapeIndex = Simulation.Shapes.Add(clothNodeShape);
            const int width = 128;
            const int length = 128;
            const float spacing = 1.75f;
            int[][] nodeHandles = new int[width][];
            for (int i = 0; i < width; ++i)
            {
                nodeHandles[i] = new int[length];
                for (int j = 0; j < length; ++j)
                {
                    var location = new Vector3(0, 30, 0) + new Vector3(spacing, 0, spacing) * (new Vector3(i, 0, j) + new Vector3(-width * 0.5f, 0, -length * 0.5f));
                    var bodyDescription = new BodyDescription
                    {
                        Activity = new BodyActivityDescription { MinimumTimestepCountUnderThreshold = 32, SleepThreshold = 0.01f },
                        Pose = new RigidPose
                        {
                            Orientation = BepuUtilities.Quaternion.Identity,
                            Position = location
                        },
                        Collidable = new CollidableDescription
                        {
                            Shape = clothNodeShapeIndex,
                            Continuity = new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Discrete },
                            SpeculativeMargin = 0.1f
                        },
                        LocalInertia = clothNodeInertia
                    };
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
                        Simulation.Solver.Add(nodeHandles[i][j], nodeHandles[i - 1][j], ref left);
                    if (j >= 1)
                        Simulation.Solver.Add(nodeHandles[i][j], nodeHandles[i][j - 1], ref up);
                    if (i >= 1 && j >= 1)
                        Simulation.Solver.Add(nodeHandles[i][j], nodeHandles[i - 1][j - 1], ref leftUp);
                    if (i < width - 1 && j >= 1)
                        Simulation.Solver.Add(nodeHandles[i][j], nodeHandles[i + 1][j - 1], ref rightUp);
                }
            }
            var bigBallShape = new Sphere(45);
            var bigBallShapeIndex = Simulation.Shapes.Add(bigBallShape);

            var bigBallDescription = new BodyDescription
            {
                Collidable = new CollidableDescription
                {
                    Continuity = new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Discrete },
                    Shape = bigBallShapeIndex,
                    SpeculativeMargin = 0.1f
                },
                Activity = new BodyActivityDescription(0),
                Pose = new RigidPose
                {
                    Position = new Vector3(-10, -15, 0),
                    Orientation = BepuUtilities.Quaternion.Identity
                }
            };
            bigBallHandle = Simulation.Bodies.Add(bigBallDescription);

            var groundShape = new Box(200, 1, 200);
            var groundShapeIndex = Simulation.Shapes.Add(groundShape);

            var groundDescription = new BodyDescription
            {
                Collidable = new CollidableDescription
                {
                    Continuity = new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Discrete },
                    Shape = groundShapeIndex,
                    SpeculativeMargin = 0.1f
                },
                Activity = new BodyActivityDescription(0),
                Pose = new RigidPose
                {
                    Position = new Vector3(0, -10, 0),
                    Orientation = BepuUtilities.Quaternion.Identity
                }
            };
            Simulation.Bodies.Add(groundDescription);
        }
        int bigBallHandle;
        float timeAccumulator;
        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            var bigBall = new BodyReference(bigBallHandle, Simulation.Bodies);
            timeAccumulator += 1 / 60f;
            if (timeAccumulator > MathF.PI * 128)
                timeAccumulator -= MathF.PI * 128;
            if (!bigBall.Awake)
                Simulation.Awakener.AwakenBody(bigBallHandle);
            bigBall.Velocity.Linear = new Vector3(0, 3f * MathF.Sin(timeAccumulator * 5), 0);
            base.Update(window, camera, input, dt);
        }
    }
}


