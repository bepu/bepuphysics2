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
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace Demos.Demos
{
    public class TriangleTestDemo : Demo
    {
        public unsafe override void Initialize(Camera camera)
        {
            {
                SphereTriangleTester tester;
                SphereWide sphere;
                sphere.Broadcast(new Sphere(0.5f));
                TriangleWide triangle;
                var a = new Vector3(0, 0, 0);
                var b = new Vector3(1, 0, 0);
                var c = new Vector3(0, 0, 1);
                //var center = (a + b + c) / 3f;
                //a -= center;
                //b -= center;
                //c -= center;
                triangle.Broadcast(new Triangle(a, b, c));
                var margin = new Vector<float>(1f);
                Vector3Wide.Broadcast(new Vector3(1, -1, 0), out var offsetB);
                QuaternionWide.Broadcast(BepuUtilities.Quaternion.CreateFromAxisAngle(new Vector3(0, 1, 0), MathF.PI / 2), out var orientationB);
                tester.Test(ref sphere, ref triangle, ref margin, ref offsetB, ref orientationB, out var manifold);
            }
            {
                CapsuleTriangleTester tester;
                CapsuleWide capsule;
                capsule.Broadcast(new Capsule(0.5f, 0.5f));
                TriangleWide triangle;
                var a = new Vector3(0, 0, 0);
                var b = new Vector3(1, 0, 0);
                var c = new Vector3(0, 0, 1);
                //var center = (a + b + c) / 3f;
                //a -= center;
                //b -= center;
                //c -= center;
                triangle.Broadcast(new Triangle(a, b, c));
                var margin = new Vector<float>(2f);
                Vector3Wide.Broadcast(new Vector3(-1f, -0.5f, -1f), out var offsetB);
                QuaternionWide.Broadcast(BepuUtilities.Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(-1, 0, 1)), MathHelper.PiOver2), out var orientationA);
                QuaternionWide.Broadcast(BepuUtilities.Quaternion.CreateFromAxisAngle(new Vector3(0, 1, 0), 0), out var orientationB);
                tester.Test(ref capsule, ref triangle, ref margin, ref offsetB, ref orientationA, ref orientationB, out var manifold);
            }
            {
                BoxTriangleTester tester;
                BoxWide shape;
                shape.Broadcast(new Box(1f, 1f, 1f));
                TriangleWide triangle;
                var a = new Vector3(0, 0, 0);
                var b = new Vector3(1, 0, 0);
                var c = new Vector3(0, 0, 1);
                //var center = (a + b + c) / 3f;
                //a -= center;
                //b -= center;
                //c -= center;
                triangle.Broadcast(new Triangle(a, b, c));
                var margin = new Vector<float>(2f);
                Vector3Wide.Broadcast(new Vector3(-1f, -0.5f, -1f), out var offsetB);
                QuaternionWide.Broadcast(BepuUtilities.Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(-1, 0, 1)), MathHelper.PiOver2), out var orientationA);
                QuaternionWide.Broadcast(BepuUtilities.Quaternion.CreateFromAxisAngle(new Vector3(0, 1, 0), 0), out var orientationB);
                tester.Test(ref shape, ref triangle, ref margin, ref offsetB, ref orientationA, ref orientationB, out var manifold);
            }
            {
                camera.Position = new Vector3(0, 3, 10);
                camera.Yaw = 0;

                Simulation = Simulation.Create(BufferPool, new TestCallbacks());
                Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);

                var triangleDescription = new StaticDescription
                {
                    Pose = new RigidPose
                    {
                        Position = new Vector3(2, 0, 2),
                        Orientation = BepuUtilities.Quaternion.CreateFromAxisAngle(new Vector3(0, 1, 0), MathF.PI / 3.2345f)
                    },
                    Collidable = new CollidableDescription
                    {
                        Shape = Simulation.Shapes.Add(new Triangle(
                            new Vector3(-3, -0.5f, -3),
                            new Vector3(3, 0, -3),
                            new Vector3(-3, 0, 3))),
                        SpeculativeMargin = 0.1f
                    }
                };
                Simulation.Statics.Add(triangleDescription);

                var shape = new Box(1f, 1f, 1f);
                var bodydescription = new BodyDescription
                {
                    Collidable = new CollidableDescription { Shape = Simulation.Shapes.Add(shape), SpeculativeMargin = 0.1f },
                    Activity = new BodyActivityDescription { SleepThreshold = -1 },
                    Pose = new RigidPose
                    {
                        Position = new Vector3(1, 2, 1),
                        Orientation = BepuUtilities.Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), 0)
                        //Orientation = BepuUtilities.Quaternion.Identity
                    }
                };
                shape.ComputeInertia(1, out bodydescription.LocalInertia);
                Simulation.Bodies.Add(bodydescription);
            }
        }

        public override void Update(Input input, float dt)
        {
            if (input.IsDown(OpenTK.Input.Key.P))
                Console.WriteLine("ASDF");
            base.Update(input, dt);
        }


    }
}
