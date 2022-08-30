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
using BepuPhysics.Constraints;

namespace Demos.SpecializedTests
{
    public class ShapePileTestDemo : Demo
    {
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-30, 10, -30);
            //camera.Yaw = MathHelper.Pi ; 
            camera.Yaw = MathHelper.Pi * 3f / 4;
            //camera.Pitch = MathHelper.PiOver2 * 0.999f;
            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(4, 1));
            Simulation.Deterministic = true;
            //Simulation.Deterministic = true;

            var sphere = new Sphere(1.5f);
            var capsule = new Capsule(1f, 1f);
            var box = new Box(1f, 3f, 2f);
            var cylinder = new Cylinder(1.5f, 0.3f);
            var points = new QuickList<Vector3>(32, BufferPool);
            //Boxlike point cloud.
            //points.Allocate(BufferPool) = new Vector3(0, 0, 0);
            //points.Allocate(BufferPool) = new Vector3(0, 0, 1);
            //points.Allocate(BufferPool) = new Vector3(0, 1, 0);
            //points.Allocate(BufferPool) = new Vector3(0, 1, 1);
            //points.Allocate(BufferPool) = new Vector3(1, 0, 0);
            //points.Allocate(BufferPool) = new Vector3(1, 0, 1);
            //points.Allocate(BufferPool) = new Vector3(1, 1, 0);
            //points.Allocate(BufferPool) = new Vector3(1, 1, 1);

            //Rando pointcloud.
            //var random = new Random(5);
            //for (int i = 0; i < 32; ++i)
            //{
            //    points.Allocate(BufferPool) = new Vector3(3 * random.NextSingle(), 1 * random.NextSingle(), 3 * random.NextSingle());
            //}

            //Dodecahedron pointcloud.
            points.Allocate(BufferPool) = new Vector3(-1, -1, -1);
            points.Allocate(BufferPool) = new Vector3(-1, -1, 1);
            points.Allocate(BufferPool) = new Vector3(-1, 1, -1);
            points.Allocate(BufferPool) = new Vector3(-1, 1, 1);
            points.Allocate(BufferPool) = new Vector3(1, -1, -1);
            points.Allocate(BufferPool) = new Vector3(1, -1, 1);
            points.Allocate(BufferPool) = new Vector3(1, 1, -1);
            points.Allocate(BufferPool) = new Vector3(1, 1, 1);

            const float goldenRatio = 1.618033988749f;
            const float oogr = 1f / goldenRatio;

            points.Allocate(BufferPool) = new Vector3(0, goldenRatio, oogr);
            points.Allocate(BufferPool) = new Vector3(0, -goldenRatio, oogr);
            points.Allocate(BufferPool) = new Vector3(0, goldenRatio, -oogr);
            points.Allocate(BufferPool) = new Vector3(0, -goldenRatio, -oogr);

            points.Allocate(BufferPool) = new Vector3(oogr, 0, goldenRatio);
            points.Allocate(BufferPool) = new Vector3(oogr, 0, -goldenRatio);
            points.Allocate(BufferPool) = new Vector3(-oogr, 0, goldenRatio);
            points.Allocate(BufferPool) = new Vector3(-oogr, 0, -goldenRatio);

            points.Allocate(BufferPool) = new Vector3(goldenRatio, oogr, 0);
            points.Allocate(BufferPool) = new Vector3(goldenRatio, -oogr, 0);
            points.Allocate(BufferPool) = new Vector3(-goldenRatio, oogr, 0);
            points.Allocate(BufferPool) = new Vector3(-goldenRatio, -oogr, 0);

            var convexHull = new ConvexHull(points.Span.Slice(points.Count), BufferPool, out _);
            var boxInertia = box.ComputeInertia(1);
            var capsuleInertia = capsule.ComputeInertia(1);
            var sphereInertia = sphere.ComputeInertia(1);
            var cylinderInertia = cylinder.ComputeInertia(1);
            var hullInertia = convexHull.ComputeInertia(1);
            var boxIndex = Simulation.Shapes.Add(box);
            var capsuleIndex = Simulation.Shapes.Add(capsule);
            var sphereIndex = Simulation.Shapes.Add(sphere);
            var cylinderIndex = Simulation.Shapes.Add(cylinder);
            var hullIndex = Simulation.Shapes.Add(convexHull);
            const int width = 16;
            const int height = 16;
            const int length = 16;
            var shapeCount = 0;
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    for (int k = 0; k < length; ++k)
                    {
                        var location = new Vector3(6, 3, 6) * new Vector3(i, j, k) + new Vector3(-width * 3, 5.5f, -length * 3);
                        var bodyDescription = BodyDescription.CreateKinematic(location, new(default, ContinuousDetection.Passive), -0.01f);
                        var index = shapeCount++;
                        switch (index % 5)
                        {
                            //case 0:
                            //    bodyDescription.Collidable.Shape = sphereIndex;
                            //    bodyDescription.LocalInertia = sphereInertia;
                            //    break;
                            //case 1:
                            //    bodyDescription.Collidable.Shape = capsuleIndex;
                            //    bodyDescription.LocalInertia = capsuleInertia;
                            //    break;
                            //case 2:
                            //    bodyDescription.Collidable.Shape = boxIndex;
                            //    bodyDescription.LocalInertia = boxInertia;
                            //    break;
                            //case 3:
                            //    bodyDescription.Collidable.Shape = cylinderIndex;
                            //    bodyDescription.LocalInertia = cylinderInertia;
                            //    break;
                            //case 4:
                            default:
                                bodyDescription.Collidable.Shape = hullIndex;
                                bodyDescription.LocalInertia = hullInertia;
                                break;
                        }
                        Simulation.Bodies.Add(bodyDescription);

                    }
                }
            }

            //Simulation.Statics.Add(new StaticDescription(new Vector3(), Simulation.Shapes.Add(new Box(500, 1, 500))));
            DemoMeshHelper.CreateDeformedPlane(128, 128, (x, y) => new Vector3(x - 64, 2f * (float)(Math.Sin(x * 0.5f) * Math.Sin(y * 0.5f)), y - 64), new Vector3(4, 1, 4), BufferPool, out var mesh);
            Simulation.Statics.Add(new StaticDescription(new Vector3(), Simulation.Shapes.Add(mesh)));
        }

    }
}


