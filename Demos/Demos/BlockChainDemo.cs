using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using Quaternion = BepuUtilities.Quaternion;

namespace Demos.Demos
{
    /// <summary>
    /// Shows how to build a decentralized ledger of transactions out of a sequence of connected blocks which could be used to revolutionize the backbone of finance while offering
    /// a new form of trust management that can be applied to a wide range of industry problems to achieve a more #secure and sustainable network of verifiable #future connections
    /// on which the accelerated development of #blockchain #solutions on the #cloud to magnify your business impact to imagine what could be achieve more by taking advantage of 
    /// truly #BigData with modern AI analytics augmented by deep-learned #blockchain #technology #to make some boxes act like a multipendulum.
    /// </summary>
    public class BlockChainDemo : Demo
    {
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-30, 8, -60);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = 0;

            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));

            var boxShape = new Box(1, 1, 1);
            boxShape.ComputeInertia(1, out var boxInertia);
            var boxIndex = Simulation.Shapes.Add(boxShape);
            const int forkCount = 20;
            const int blocksPerChain = 20;
            int[] blockHandles = new int[blocksPerChain];
            for (int forkIndex = 0; forkIndex < forkCount; ++forkIndex)
            {
                //Build the blocks.
                for (int blockIndex = 0; blockIndex < blocksPerChain; ++blockIndex)
                {
                    var bodyDescription = new BodyDescription
                    {
                        //Make the uppermost block kinematic to hold up the rest of the chain.
                        LocalInertia = blockIndex == blocksPerChain - 1 ? new BodyInertia() : boxInertia,
                        Pose = new RigidPose
                        {
                            Position = new Vector3(0,
                                5 + blockIndex * (boxShape.Height + 1),
                                (forkIndex - forkCount * 0.5f) * (boxShape.Length + 4)),
                            Orientation = Quaternion.Identity
                        },
                        Activity = new BodyActivityDescription { MinimumTimestepCountUnderThreshold = 32, SleepThreshold = .01f },
                        Collidable = new CollidableDescription { Shape = boxIndex, SpeculativeMargin = .1f },
                    };
                    blockHandles[blockIndex] = Simulation.Bodies.Add(bodyDescription);
                }
                //Build the chains.
                for (int i = 1; i < blocksPerChain; ++i)
                {
                    var ballSocket = new BallSocket
                    {
                        LocalOffsetA = new Vector3(0, 1f, 0),
                        LocalOffsetB = new Vector3(0, -1f, 0),
                        SpringSettings = new SpringSettings(30, 5)
                    };
                    Simulation.Solver.Add(blockHandles[i - 1], blockHandles[i], ref ballSocket);
                }
            }

            var staticShape = new Box(200, 1, 200);
            var staticShapeIndex = Simulation.Shapes.Add(staticShape);

            var staticDescription = new StaticDescription
            {
                Collidable = new CollidableDescription
                {
                    Continuity = new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Discrete },
                    Shape = staticShapeIndex,
                    SpeculativeMargin = 0.1f
                },
                Pose = new RigidPose
                {
                    Position = new Vector3(1, -0.5f, 1),
                    Orientation = Quaternion.Identity
                }
            };
            Simulation.Statics.Add(staticDescription);

            //Build the coin description for the ponz-I mean ICO.
            var coinShape = new Cylinder(1.5f, 0.2f);
            coinShape.ComputeInertia(1, out var coinInertia);
            coinDescription = BodyDescription.CreateDynamic(RigidPose.Identity, coinInertia, new CollidableDescription(Simulation.Shapes.Add(coinShape), 0.1f), new BodyActivityDescription(0.01f));
        }

        BodyDescription coinDescription;
        Random random = new Random(5);
        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            if (input.WasPushed(OpenTK.Input.Key.Z))
            {
                //INVEST TODAY FOR INCREDIBLE RETURNS DON'T MISS OUT LOOK AT THE COINS THERE ARE A LOT OF THEM AND THEY COULD BE YOURS
                var origin = new Vector3(-30, 5, -30) + new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) * new Vector3(60, 30, 60);
                for (int i = 0; i < 128; ++i)
                {
                    var direction = new Vector3(-1) + 2 * new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble());
                    var length = direction.Length();
                    if (length > 1e-7f)
                        direction /= length;
                    else
                        direction = new Vector3(0, 1, 0);

                    coinDescription.Pose.Position = origin + direction * 10 * (float)random.NextDouble();
                    coinDescription.Pose.Orientation = Quaternion.Normalize(new Quaternion(0.01f + (float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()));
                    coinDescription.Velocity.Linear = direction * (5 + 30 * (float)random.NextDouble());
                    Simulation.Bodies.Add(coinDescription);
                }
            }            
            base.Update(window, camera, input, dt);
        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            text.Clear().Append("Press Z to create an ICO.");
            renderer.TextBatcher.Write(text, new Vector2(20, renderer.Surface.Resolution.Y - 20), 16, new Vector3(1, 1, 1), font);
            base.Render(renderer, camera, input, text, font);
        }

    }
}
