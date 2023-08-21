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
using BepuPhysics.Constraints;

namespace Demos.SpecializedTests
{
    public class TriangleTestDemo : Demo
    {
        public override void Initialize(ContentArchive content, Camera camera)
        {
            {
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
                QuaternionWide.Broadcast(QuaternionEx.CreateFromAxisAngle(new Vector3(0, 1, 0), MathF.PI / 2), out var orientationB);
                SphereTriangleTester.Test(ref sphere, ref triangle, ref margin, ref offsetB, ref orientationB, Vector<float>.Count, out var manifold);
            }
            {
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
                QuaternionWide.Broadcast(QuaternionEx.CreateFromAxisAngle(Vector3.Normalize(new Vector3(-1, 0, 1)), MathHelper.PiOver2), out var orientationA);
                QuaternionWide.Broadcast(QuaternionEx.CreateFromAxisAngle(new Vector3(0, 1, 0), 0), out var orientationB);
                CapsuleTriangleTester.Test(ref capsule, ref triangle, ref margin, ref offsetB, ref orientationA, ref orientationB, Vector<float>.Count, out var manifold);
            }
            {
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
                QuaternionWide.Broadcast(QuaternionEx.CreateFromAxisAngle(Vector3.Normalize(new Vector3(-1, 0, 1)), MathHelper.PiOver2), out var orientationA);
                QuaternionWide.Broadcast(QuaternionEx.CreateFromAxisAngle(new Vector3(0, 1, 0), 0), out var orientationB);
                BoxTriangleTester.Test(ref shape, ref triangle, ref margin, ref offsetB, ref orientationA, ref orientationB, Vector<float>.Count, out var manifold);
            }
            {
                TriangleWide a = default, b = default;
                a.Broadcast(new Triangle(new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(0, 0, 1)));
                b.Broadcast(new Triangle(new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(0, 0, 1)));

                var margin = new Vector<float>(2f);
                Vector3Wide.Broadcast(new Vector3(0, -1, 0), out var offsetB);
                QuaternionWide.Broadcast(QuaternionEx.CreateFromAxisAngle(Vector3.Normalize(new Vector3(-1, 0, 1)), 0), out var orientationA);
                QuaternionWide.Broadcast(QuaternionEx.CreateFromAxisAngle(new Vector3(0, 1, 0), 0), out var orientationB);
                TrianglePairTester.Test(ref a, ref b, ref margin, ref offsetB, ref orientationA, ref orientationB, Vector<float>.Count, out var manifold);
            }
            {
                camera.Position = new Vector3(0, 3, -10);
                camera.Yaw = MathF.PI;
                camera.Pitch = 0;

                Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1), 5, 1), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));

                //var triangleDescription = new StaticDescription
                //{
                //    Pose = new RigidPose
                //    {
                //        Position = new Vector3(2 - 10, 0, 2),
                //        Orientation = QuaternionEx.CreateFromAxisAngle(new Vector3(0, 1, 0), MathF.PI / 3.2345f)
                //    },
                //    Collidable = new CollidableDescription
                //    {
                //        Shape = Simulation.Shapes.Add(new Triangle(
                //            new Vector3(-3, -0.5f, -3),
                //            new Vector3(3, 0, -3),
                //            new Vector3(-3, 0, 3))),
                //        SpeculativeMargin = 10.1f
                //    }
                //};
                //Simulation.Statics.Add(triangleDescription);

                //var shape = new Triangle(new Vector3(0, 0, 3), new Vector3(0, 0, 0), new Vector3(-3, 3, 0));
                //var bodyDescription = new BodyDescription
                //{
                //    Collidable = new CollidableDescription { Shape = Simulation.Shapes.Add(shape), SpeculativeMargin = 0.1f },
                //    Activity = new BodyActivityDescription { SleepThreshold = -1 },
                //    Pose = new RigidPose
                //    {
                //        Position = new Vector3(1 - 10, -0.01f, 1),
                //        Orientation = QuaternionEx.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 0, 1)), 0)
                //        //Orientation = BepuUtilities.Quaternion.Identity
                //    }
                //};
                //shape.ComputeInertia(1, out bodyDescription.LocalInertia);
                ////bodyDescription.LocalInertia.InverseInertiaTensor = new Triangular3x3();
                //Simulation.Bodies.Add(bodyDescription);

                Simulation.Statics.Add(new StaticDescription(new Vector3(0, -5, 0), Simulation.Shapes.Add(new Box(200, 5, 200))));
                Simulation.Statics.Add(new StaticDescription(new Vector3(10, -2, 30), Simulation.Shapes.Add(new Box(10, 5, 10))));

                Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(20, 2, 0), new BodyInertia { InverseMass = 1 }, new(Simulation.Shapes.Add(new Sphere(1.75f)), 0.1f, 0.1f), -1));
                var capsule = new Capsule(2, 2);
                Simulation.Bodies.Add(BodyDescription.CreateDynamic((new Vector3(20, 2, 3), Quaternion.CreateFromYawPitchRoll(0f, 1.745329E-05f, 0f)), capsule.ComputeInertia(1), new(Simulation.Shapes.Add(capsule), 0.1f, 0.1f), -1));
                var testBox = new Box(2, 3, 2);
                var testBoxInertia = testBox.ComputeInertia(1);
                Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(20, 2, 6), testBoxInertia, new(Simulation.Shapes.Add(testBox), 10.1f, 10.1f), -1));

                var cylinder = new Cylinder(1.75f, 0.5f);
                var cylinderInertia = cylinder.ComputeInertia(1);
                //cylinderInertia.InverseInertiaTensor = default;
                Simulation.Bodies.Add(BodyDescription.CreateDynamic((new Vector3(20, 2, 9), Quaternion.CreateFromAxisAngle(new Vector3(0, 0, 1), MathF.PI / 2f)), cylinderInertia, new(Simulation.Shapes.Add(cylinder), 5, 5), -1));

                var cylinder2 = new Cylinder(.5f, 0.5f);
                var cylinder2Inertia = cylinder2.ComputeInertia(1);
                Simulation.Bodies.Add(BodyDescription.CreateDynamic((new Vector3(23, 2, 9), Quaternion.CreateFromAxisAngle(new Vector3(0, 0, 1), 0)), cylinder2Inertia, new(Simulation.Shapes.Add(cylinder2), 5, 5), -1));
                var points = new QuickList<Vector3>(8, BufferPool);
                points.AllocateUnsafely() = new Vector3(0, 0, 0);
                points.AllocateUnsafely() = new Vector3(0, 0, 2);
                points.AllocateUnsafely() = new Vector3(2, 0, 0);
                points.AllocateUnsafely() = new Vector3(2, 0, 2);
                points.AllocateUnsafely() = new Vector3(0, 2, 0);
                points.AllocateUnsafely() = new Vector3(0, 2, 2);
                points.AllocateUnsafely() = new Vector3(2, 2, 0);
                points.AllocateUnsafely() = new Vector3(2, 2, 2);
                var convexHull = new ConvexHull(points, BufferPool, out _);
                var convexHullInertia = convexHull.ComputeInertia(1);
                Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(20, 2, 12), convexHullInertia, new(Simulation.Shapes.Add(convexHull), 0.1f, 0.1f), -1));
                Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(23, 2, 12), convexHullInertia, new(Simulation.Shapes.Add(convexHull), 0.1f, 0.1f), -1));

                CompoundBuilder builder = new CompoundBuilder(BufferPool, Simulation.Shapes, 2);
                builder.Add(new Box(1, 1, 1), RigidPose.Identity, 1);
                builder.Add(new Triangle { A = new(-0.5f, 1, 0), B = new(0.5f, 1, 0), C = new Vector3(0f, 3, -1) }, RigidPose.Identity, 1);
                builder.BuildDynamicCompound(out var children, out var compoundInertia);
                //compoundInertia.InverseInertiaTensor = default;
                var compound = new Compound(children);
                Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(20, 3, 14), compoundInertia, new(Simulation.Shapes.Add(compound), 10.1f, 10.1f), -1));

                {
                    var triangles = new QuickList<Triangle>(4, BufferPool);
                    var v0 = new Vector3(0, 1.75f, 0);
                    var v1 = new Vector3(8, 1.75f, 0);
                    var v2 = new Vector3(0, 1.75f, 50);
                    var v3 = new Vector3(8, 1.75f, 50);
                    triangles.AllocateUnsafely() = new Triangle { A = v2, B = v0, C = v1 };
                    triangles.AllocateUnsafely() = new Triangle { A = v2, B = v1, C = v3 };
                    triangles.AllocateUnsafely() = new Triangle { A = v0, B = v2, C = v1 };
                    triangles.AllocateUnsafely() = new Triangle { A = v1, B = v2, C = v3 };
                    var testMesh = new Mesh(triangles, Vector3.One, BufferPool);
                    Simulation.Statics.Add(new StaticDescription(new Vector3(30, -2.5f, 0), Simulation.Shapes.Add(testMesh)));

                    Simulation.Statics.Add(new StaticDescription(new Vector3(0, -2.5f, 0), Simulation.Shapes.Add(new Triangle(v2, v0, v1))));
                }


                var mesh = DemoMeshHelper.LoadModel(content, BufferPool, "Content\\newt.obj", new Vector3(3));
                var collidable = new CollidableDescription(Simulation.Shapes.Add(mesh), 2f, 2f, ContinuousDetection.Discrete);
                var newtInertia = mesh.ComputeClosedInertia(1);
                for (int i = 0; i < 5; ++i)
                {
                    Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(-20, 5 + i * 5, 0), newtInertia, collidable, -1e-2f));
                }

                {
                    var triangles = new QuickList<Triangle>(4, BufferPool);
                    var v0 = new Vector3(3, 1f, 0);
                    var v1 = new Vector3(3, 0, 2);
                    var v2 = new Vector3(0, 1f, 2);
                    var v3 = new Vector3(2, 2, 1);
                    triangles.AllocateUnsafely() = new Triangle { A = v0, B = v2, C = v1 };
                    triangles.AllocateUnsafely() = new Triangle { A = v1, B = v2, C = v3 };
                    var testMesh = new Mesh(triangles, Vector3.One, BufferPool);
                    Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(22, -2.5f, 0), new BodyInertia { InverseMass = 1 }, new(Simulation.Shapes.Add(testMesh), 10.1f, 10.1f), -1f));
                }
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
