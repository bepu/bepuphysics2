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
using DemoContentLoader;

namespace Demos.Demos
{
    public class TriangleTestDemo : Demo
    {
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            {
                SphereTriangleTester tester;
                SphereWide sphere = default;
                sphere.Broadcast(new Sphere(0.5f));
                TriangleWide triangle = default;
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
                tester.Test(ref sphere, ref triangle, ref margin, ref offsetB, ref orientationB, Vector<float>.Count, out var manifold);
            }
            {
                CapsuleTriangleTester tester;
                CapsuleWide capsule = default;
                capsule.Broadcast(new Capsule(0.5f, 0.5f));
                TriangleWide triangle = default;
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
                tester.Test(ref capsule, ref triangle, ref margin, ref offsetB, ref orientationA, ref orientationB, Vector<float>.Count, out var manifold);
            }
            {
                BoxTriangleTester tester;
                BoxWide shape = default; 
                shape.Broadcast(new Box(1f, 1f, 1f));
                TriangleWide triangle = default;
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
                tester.Test(ref shape, ref triangle, ref margin, ref offsetB, ref orientationA, ref orientationB, Vector<float>.Count, out var manifold);
            }
            {
                TrianglePairTester tester;
                TriangleWide a = default, b = default;
                a.Broadcast(new Triangle(new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(0, 0, 1)));
                b.Broadcast(new Triangle(new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(0, 0, 1)));

                var margin = new Vector<float>(2f);
                Vector3Wide.Broadcast(new Vector3(0, -1, 0), out var offsetB);
                QuaternionWide.Broadcast(BepuUtilities.Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(-1, 0, 1)), 0), out var orientationA);
                QuaternionWide.Broadcast(BepuUtilities.Quaternion.CreateFromAxisAngle(new Vector3(0, 1, 0), 0), out var orientationB);
                tester.Test(ref a, ref b, ref margin, ref offsetB, ref orientationA, ref orientationB, Vector<float>.Count, out var manifold);
            }
            {
                camera.Position = new Vector3(0, 3, 10);
                camera.Yaw = 0;

                Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));

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
                        SpeculativeMargin = 10.1f
                    }
                };
                Simulation.Statics.Add(triangleDescription);

                var shape = new Triangle(new Vector3(0, 0, 3), new Vector3(0, 0, 0), new Vector3(-3, 3, 0));
                var bodyDescription = new BodyDescription
                {
                    Collidable = new CollidableDescription { Shape = Simulation.Shapes.Add(shape), SpeculativeMargin = 0.1f },
                    Activity = new BodyActivityDescription { SleepThreshold = -1 },
                    Pose = new RigidPose
                    {
                        Position = new Vector3(1, -0.01f, 1),
                        Orientation = BepuUtilities.Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 0, 1)), 0)
                        //Orientation = BepuUtilities.Quaternion.Identity
                    }
                };
                shape.ComputeInertia(1, out bodyDescription.LocalInertia);
                //bodyDescription.LocalInertia.InverseInertiaTensor = new Triangular3x3();
                Simulation.Bodies.Add(bodyDescription);
            }
        }

        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            if (input.IsDown(OpenTK.Input.Key.P))
                Console.WriteLine("ASDF");
            base.Update(window, camera, input, dt);
        }


    }
}
