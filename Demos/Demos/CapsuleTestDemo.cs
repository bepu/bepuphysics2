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

namespace Demos.Demos
{
    public class CapsuleTestDemo : Demo
    {
        public unsafe override void Initialize(Camera camera)
        {
            camera.Position = new Vector3(-10, 5, -10);
            //camera.Yaw = MathHelper.Pi ; 
            camera.Yaw = MathHelper.Pi * 3f / 4;
            //camera.Pitch = MathHelper.Pi * 0.1f;
            Simulation = Simulation.Create(BufferPool, new TestCallbacks());
            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);

            var shape = new Capsule(.5f, 3.5f);
            shape.ComputeInertia(1, out var localInertia);
            //capsuleInertia.InverseInertiaTensor = new Triangular3x3();
            var shapeIndex = Simulation.Shapes.Add(shape);
            const int width = 4;
            const int height = 10;
            const int length = 4;
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    for (int k = 0; k < length; ++k)
                    {
                        var location = new Vector3(1.5f, 1.5f, 4.4f) * new Vector3(i, j, k) + new Vector3(-width * 1.5f, 1.5f, -length * 1.5f);
                        var bodyDescription = new BodyDescription
                        {
                            Activity = new BodyActivityDescription { MinimumTimestepCountUnderThreshold = 32, SleepThreshold = 0.01f },
                            LocalInertia = localInertia,
                            Pose = new RigidPose
                            {
                                Orientation = BepuUtilities.Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), MathF.PI / 2),
                                Position = location
                            },
                            Collidable = new CollidableDescription { SpeculativeMargin = 0.1f, Shape = shapeIndex }
                        };
                        Simulation.Bodies.Add(bodyDescription);

                    }
                }
            }

            //var bodyDescription = new BodyDescription
            //{
            //    Pose = new RigidPose
            //    {
            //        Position = new Vector3(0, 5, -0.5f),
            //        //Orientation = BepuUtilities.Quaternion.Identity
            //        Orientation = BepuUtilities.Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), MathHelper.PiOver2)
            //    },
            //    LocalInertia = new BodyInertia(),
            //    Collidable = new CollidableDescription
            //    {
            //        Continuity = new ContinuousDetectionSettings(),
            //        SpeculativeMargin = 0.1f,
            //        Shape = shapeIndex
            //    },
            //    Activity = new BodyActivityDescription
            //    {
            //        SleepThreshold = -.1f,
            //        MinimumTimestepCountUnderThreshold = 32
            //    },
            //    //Velocity = new BodyVelocity { Angular = new Vector3(0, (rowIndex % 2 - 0.5f) * 20, 0) }
            //};
            //Simulation.Bodies.Add(ref bodyDescription);

            //var staticShape = new Sphere(4f);
            //var staticShapeIndex = Simulation.Shapes.Add(staticShape);
            //const int staticGridWidth = 64;
            //const float staticSpacing = 6;
            //var gridOffset = -0.5f * staticGridWidth * staticSpacing;
            //for (int i = 0; i < staticGridWidth; ++i)
            //{
            //    for (int j = 0; j < staticGridWidth; ++j)
            //    {
            //        var staticDescription = new StaticDescription
            //        {
            //            Collidable = new CollidableDescription
            //            {
            //                Continuity = new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Discrete },
            //                Shape = staticShapeIndex,
            //                SpeculativeMargin = 0.1f
            //            },
            //            Pose = new RigidPose
            //            {
            //                Position = new Vector3(
            //                gridOffset + i * staticSpacing,
            //                -4,
            //                gridOffset + j * staticSpacing),
            //                Orientation = BepuUtilities.Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 0, 1)), MathHelper.PiOver4)
            //            }
            //        };
            //        Simulation.Statics.Add(ref staticDescription);
            //    }
            //}
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -3, 0), new CollidableDescription { SpeculativeMargin = 0.1f, Shape = Simulation.Shapes.Add(new Box(100, 1, 100)) }));

        }


    }
}
