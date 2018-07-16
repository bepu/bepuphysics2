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
using DemoContentLoader;

namespace Demos.Demos
{
    public class MeshDemo : Demo
    {
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-10, 0, -10);
            //camera.Yaw = MathHelper.Pi ; 
            camera.Yaw = MathHelper.Pi * 3f / 4;
            //camera.Pitch = MathHelper.PiOver2 * 0.999f;
            Simulation = Simulation.Create(BufferPool, new TestCallbacks());
            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);

            var box = new Box(1f, 3f, 2f);
            var capsule = new Capsule(1f, 1f);
            var sphere = new Sphere(.5f);
            box.ComputeInertia(1, out var boxInertia);
            capsule.ComputeInertia(1, out var capsuleInertia);
            sphere.ComputeInertia(1, out var sphereInertia);
            var boxIndex = Simulation.Shapes.Add(box);
            var capsuleIndex = Simulation.Shapes.Add(capsule);
            var sphereIndex = Simulation.Shapes.Add(sphere);
            const int width = 1;
            const int height = 3;
            const int length = 1;
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    for (int k = 0; k < length; ++k)
                    {
                        var location = new Vector3(3, 3, 3) * new Vector3(i, j, k);// + new Vector3(-width * 1.5f, 1.5f, -length * 1.5f);
                        var bodyDescription = new BodyDescription
                        {
                            Activity = new BodyActivityDescription { MinimumTimestepCountUnderThreshold = 32, SleepThreshold = -0.01f },
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
                        //Simulation.Bodies.Add(bodyDescription);

                    }
                }
            }
            Simulation.Bodies.Add(new BodyDescription
            {
                Activity = new BodyActivityDescription(-1),
                Pose = new RigidPose(new Vector3(1, 9, 0), BepuUtilities.Quaternion.CreateFromYawPitchRoll(0, 0, -0.00001f)),
                Collidable = new CollidableDescription(sphereIndex, 1),
                LocalInertia = capsuleInertia
            });
            Simulation.Bodies.Add(new BodyDescription
            {
                Activity = new BodyActivityDescription(-1),
                Pose = new RigidPose(new Vector3(1, 6, 0), BepuUtilities.Quaternion.CreateFromYawPitchRoll(0, 0, -0.00001f)),
                Collidable = new CollidableDescription(capsuleIndex, 1),
                LocalInertia = capsuleInertia
            });
            Simulation.Bodies.Add(new BodyDescription
            {
                Activity = new BodyActivityDescription(-1),
                Pose = new RigidPose(new Vector3(0, 0, 0), BepuUtilities.Quaternion.Identity),
                Collidable = new CollidableDescription(boxIndex, .1f),
                LocalInertia = boxInertia
            });
            Simulation.Bodies.Add(new BodyDescription
            {
                Activity = new BodyActivityDescription(-1),
                Pose = new RigidPose(new Vector3(1, 3, 0), BepuUtilities.Quaternion.Identity),
                Collidable = new CollidableDescription(boxIndex, .1f),
                LocalInertia = new BodyInertia()
            });

            var meshContent = content.Load<MeshContent>(@"Content\box.obj");
            BufferPool.Take<Triangle>(meshContent.Triangles.Length, out var triangles);
            for (int i = 0; i < meshContent.Triangles.Length; ++i)
            {
                triangles[i] = new Triangle(meshContent.Triangles[i].A, meshContent.Triangles[i].B, meshContent.Triangles[i].C);
            }
            var meshShape = new Mesh(triangles.Slice(0, meshContent.Triangles.Length), new Vector3(5, 1, 5), BufferPool);
            //BufferPool.Take<Triangle>(1, out var triangles);
            //for (int i = 3; i < 4; ++i)
            //{
            //    triangles[i - 3] = new Triangle(meshContent.Triangles[i].A, meshContent.Triangles[i].B, meshContent.Triangles[i].C);
            //}
            //var meshShape = new Mesh(triangles.Slice(0, 1), new Vector3(5, 1, 5), BufferPool);
            var staticShapeIndex = Simulation.Shapes.Add(meshShape);

            for (int i = 0; i < 1; ++i)
            {
                var staticDescription = new StaticDescription
                {
                    Collidable = new CollidableDescription(staticShapeIndex, 0.1f),
                    Pose = new RigidPose
                    {
                        Position = new Vector3(i * 10, -10, 0),
                        Orientation = BepuUtilities.Quaternion.Identity
                        //Orientation = BepuUtilities.Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1 + i, i * j % 10, -10 + -j)), (i ^ j) * 0.5f * (MathHelper.PiOver4))
                        //Orientation = BepuUtilities.Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(0, 0, 1)), MathHelper.Pi)
                    }
                };
                Simulation.Statics.Add(staticDescription);
            }


        }

        public override void Update(Input input, float dt)
        {
            if (input.IsDown(OpenTK.Input.Key.P))
                Console.Write("SDF");
            base.Update(input, dt);
        }

    }
}


