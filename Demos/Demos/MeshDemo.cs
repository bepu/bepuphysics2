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
            //Simulation.Bodies.Add(new BodyDescription
            //{
            //    Activity = new BodyActivityDescription(-1),
            //    Pose = new RigidPose(new Vector3(1, 9, 0), BepuUtilities.Quaternion.CreateFromYawPitchRoll(0, 0, -0.00001f)),
            //    Collidable = new CollidableDescription(sphereIndex, 1),
            //    LocalInertia = capsuleInertia
            //});
            //Simulation.Bodies.Add(new BodyDescription
            //{
            //    Activity = new BodyActivityDescription(-1),
            //    Pose = new RigidPose(new Vector3(1, 6, 0), BepuUtilities.Quaternion.CreateFromYawPitchRoll(0, 0, -0.00001f)),
            //    Collidable = new CollidableDescription(capsuleIndex, 1),
            //    LocalInertia = capsuleInertia
            //});
            Simulation.Bodies.Add(new BodyDescription
            {
                Activity = new BodyActivityDescription(-1),
                Pose = new RigidPose(new Vector3(0, 0, 0), BepuUtilities.Quaternion.Identity),
                Collidable = new CollidableDescription(boxIndex, 10000f),
                LocalInertia = boxInertia
            });
            //Simulation.Bodies.Add(new BodyDescription
            //{
            //    Activity = new BodyActivityDescription(-1),
            //    Pose = new RigidPose(new Vector3(1, 3, 0), BepuUtilities.Quaternion.Identity),
            //    Collidable = new CollidableDescription(boxIndex, .1f),
            //    LocalInertia = new BodyInertia()
            //});


            LoadModel(content, BufferPool, @"Content\box.obj", new Vector3(5, 1, 5), out var boxMesh);
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -5, 0), new CollidableDescription(Simulation.Shapes.Add(boxMesh), 0.1f)));

            CreateFan(64, 64, new Vector3(1, 1, 1), BufferPool, out var fanMesh);
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -10, 0), new CollidableDescription(Simulation.Shapes.Add(fanMesh), 0.1f)));


        }

        static void LoadModel(ContentArchive content, BufferPool pool, string contentName, in Vector3 scaling, out Mesh mesh)
        {
            var meshContent = content.Load<MeshContent>(contentName);
            pool.Take<Triangle>(meshContent.Triangles.Length, out var triangles);
            for (int i = 0; i < meshContent.Triangles.Length; ++i)
            {
                triangles[i] = new Triangle(meshContent.Triangles[i].A, meshContent.Triangles[i].B, meshContent.Triangles[i].C);
            }
            mesh = new Mesh(triangles.Slice(0, meshContent.Triangles.Length), scaling, pool);
        }

        static void CreateFan(int triangleCount, float radius, in Vector3 scaling, BufferPool pool, out Mesh mesh)
        {
            var anglePerTriangle = 2 * MathF.PI / triangleCount;
            pool.Take<Triangle>(triangleCount, out var triangles);
            triangles = triangles.Slice(0, triangleCount);

            for (int i = 0; i < triangleCount; ++i)
            {
                var firstAngle = i * anglePerTriangle;
                var secondAngle = ((i + 1) % triangleCount) * anglePerTriangle;

                ref var triangle = ref triangles[i];
                triangle.A = new Vector3(radius * MathF.Cos(firstAngle), 0, radius * MathF.Sin(firstAngle));
                triangle.B = new Vector3(radius * MathF.Cos(secondAngle), 0, radius * MathF.Sin(secondAngle));
                triangle.C = new Vector3();
            }
            mesh = new Mesh(triangles, scaling, pool);
        }

        public override void Update(Input input, float dt)
        {
            if (input.IsDown(OpenTK.Input.Key.P))
                Console.Write("SDF");
            base.Update(input, dt);
        }

    }
}


