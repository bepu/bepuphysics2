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
    public class ShapePileDemo : Demo
    {
        public unsafe override void Initialize(Camera camera)
        {
            camera.Position = new Vector3(-30, 10, -30);
            //camera.Yaw = MathHelper.Pi ; 
            camera.Yaw = MathHelper.Pi * 3f / 4;
            //camera.Pitch = MathHelper.PiOver2 * 0.999f;
            Simulation = Simulation.Create(BufferPool, new TestCallbacks());

            var box = new Box(1f, 3f, 2f);
            var capsule = new Capsule(1f, 1f);
            var sphere = new Sphere(1.5f);
            box.ComputeInertia(1, out var boxInertia);
            capsule.ComputeInertia(1, out var capsuleInertia);
            sphere.ComputeInertia(1, out var sphereInertia);
            //capsuleInertia.InverseInertiaTensor = new Triangular3x3();
            var boxIndex = Simulation.Shapes.Add(box);
            var capsuleIndex = Simulation.Shapes.Add(capsule);
            var sphereIndex = Simulation.Shapes.Add(sphere);
            const int width = 8;
            const int height = 16;
            const int length = 8;
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    for (int k = 0; k < length; ++k)
                    {
                        var location = new Vector3(3, 3, 3) * new Vector3(i, j, k) + new Vector3(-width * 1.5f, 1.5f, -length * 1.5f);
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
                                Continuity = new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Discrete },
                                SpeculativeMargin = 0.1f
                            }
                        };
                        switch (j % 3)
                        {
                            case 0:
                                bodyDescription.Collidable.Shape = boxIndex;
                                bodyDescription.LocalInertia = boxInertia;
                                break;
                            case 1:
                                bodyDescription.Collidable.Shape = capsuleIndex;
                                bodyDescription.LocalInertia = capsuleInertia;
                                break;
                            case 2:
                                bodyDescription.Collidable.Shape = sphereIndex;
                                bodyDescription.LocalInertia = sphereInertia;
                                break;
                        }
                        Simulation.Bodies.Add(ref bodyDescription);

                    }
                }
            }
            
            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);
            Simulation.Deterministic = false;
            
            var staticShape = new Box(1, 1, 1);
            var staticShapeIndex = Simulation.Shapes.Add(staticShape);
            const int staticGridWidth = 100;
            const float staticSpacing = 1.2f;
            var gridOffset = -0.5f * staticGridWidth * staticSpacing;
            for (int i = 0; i < staticGridWidth; ++i)
            {
                for (int j = 0; j < staticGridWidth; ++j)
                {
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
                            Position = new Vector3(
                                1 + gridOffset + i * staticSpacing,
                                -0.707f,
                                0.5f + gridOffset + j * staticSpacing),
                            //Orientation = BepuUtilities.Quaternion.Identity
                            Orientation = BepuUtilities.Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1 + i, i * j % 10, -10 + -j)), (i ^ j) * 0.5f * (MathHelper.PiOver4))
                            //Orientation = BepuUtilities.Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(0, 0, 1)), MathHelper.Pi)
                        }
                    };
                    Simulation.Statics.Add(ref staticDescription);
                }
            }

        }

    }
}


