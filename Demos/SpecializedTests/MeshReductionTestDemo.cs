using System;
using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using OpenTK.Input;

namespace Demos.SpecializedTests
{
    public class MeshReductionTestDemo : Demo
    {

        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 5, 10);
            camera.Yaw = 0;
            camera.Pitch = 0;

            //The PositionFirstTimestepper is the simplest timestepping mode, but since it integrates velocity into position at the start of the frame, directly modified velocities outside of the timestep
            //will be integrated before collision detection or the solver has a chance to intervene. That's fine in this demo. Other built-in options include the PositionLastTimestepper and the SubsteppingTimestepper.
            //Note that the timestepper also has callbacks that you can use for executing logic between processing stages, like BeforeCollisionDetection.
            Simulation = Simulation.Create(BufferPool,
                new DemoNarrowPhaseCallbacks() { ContactSpringiness = new(30, 1), MaximumRecoveryVelocity = 2, FrictionCoefficient = 0 },
                new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new PositionFirstTimestepper());

            var builder = new CompoundBuilder(BufferPool, Simulation.Shapes, 2);
            builder.Add(new Box(1.85f, 0.7f, 4.73f), RigidPose.Identity, 10);
            builder.Add(new Box(1.85f, 0.6f, 2.5f), new RigidPose(new Vector3(0, 0.65f, -0.35f)), 0.5f);
            builder.BuildDynamicCompound(out var children, out var bodyInertia, out _);
            builder.Dispose();
            var bodyShape = new Compound(children);
            var bodyShapeIndex = Simulation.Shapes.Add(bodyShape);
            var wheelShape = new Cylinder(0.4f, .18f);
            wheelShape.ComputeInertia(0.25f, out var wheelInertia);
            var wheelShapeIndex = Simulation.Shapes.Add(wheelShape);



            const int planeWidth = 257;
            const float scale = 3;
            Vector2 terrainPosition = new Vector2(1 - planeWidth, 1 - planeWidth) * scale * 0.5f;


            Vector3 min = new Vector3(-planeWidth * scale * 0.45f, 10, -planeWidth * scale * 0.45f);
            Vector3 span = new Vector3(planeWidth * scale * 0.9f, 15, planeWidth * scale * 0.9f);


            DemoMeshHelper.CreateDeformedPlane(planeWidth, planeWidth,
                (int vX, int vY) =>
                {
                    var octave0 = (MathF.Sin((vX + 5f) * 0.05f) + MathF.Sin((vY + 11) * 0.05f)) * 1.8f;
                    var octave1 = (MathF.Sin((vX + 17) * 0.15f) + MathF.Sin((vY + 19) * 0.15f)) * 0.9f;
                    var octave2 = (MathF.Sin((vX + 37) * 0.35f) + MathF.Sin((vY + 93) * 0.35f)) * 0.4f;
                    var octave3 = (MathF.Sin((vX + 53) * 0.65f) + MathF.Sin((vY + 47) * 0.65f)) * 0.2f;
                    var octave4 = (MathF.Sin((vX + 67) * 1.50f) + MathF.Sin((vY + 13) * 1.5f)) * 0.125f;
                    var distanceToEdge = planeWidth / 2 - Math.Max(Math.Abs(vX - planeWidth / 2), Math.Abs(vY - planeWidth / 2));
                    var edgeRamp = 25f / (distanceToEdge + 1);
                    var terrainHeight = octave0 + octave1 + octave2 + octave3 + octave4;
                    var vertexPosition = new Vector2(vX * scale, vY * scale) + terrainPosition;
                    return new Vector3(vertexPosition.X, terrainHeight + edgeRamp, vertexPosition.Y);

                }, new Vector3(1, 1, 1), BufferPool, out var planeMesh);
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -15, 0), QuaternionEx.CreateFromAxisAngle(new Vector3(0, 1, 0), MathF.PI / 2),
                new CollidableDescription(Simulation.Shapes.Add(planeMesh), 0.1f)));

            var testBox = new Box(3, 3, 3);
            testBox.ComputeInertia(1, out var testBoxInertia);
            Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(0, 10, 0), testBoxInertia, new CollidableDescription(Simulation.Shapes.Add(testBox), 10f), new BodyActivityDescription(-1)));
            var testSphere = new Sphere(.1f);
            testSphere.ComputeInertia(1, out var testSphereInertia);
            //testSphereInertia.InverseInertiaTensor = default;
            Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(10, 10, 0), testSphereInertia, new CollidableDescription(Simulation.Shapes.Add(testSphere), 10f), new BodyActivityDescription(-1)));
            var testCylinder = new Cylinder(1.5f, 2f);
            testCylinder.ComputeInertia(1, out var testCylinderInertia);
            //testCylinderInertia.InverseInertiaTensor = default;
            Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(15, 10, 0), testCylinderInertia, new CollidableDescription(Simulation.Shapes.Add(testCylinder), 10f), new BodyActivityDescription(-1)));
            var testCapsule = new Capsule(.1f, 2f);
            testCapsule.ComputeInertia(1, out var testCapsuleInertia);
            //testCapsuleInertia.InverseInertiaTensor = default;
            Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(18, 10, 0), testCapsuleInertia, new CollidableDescription(Simulation.Shapes.Add(testCapsule), 10f), new BodyActivityDescription(-1)));

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
            convexHull.ComputeInertia(1, out var convexHullInertia);
            Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(23, 10, 0), convexHullInertia, new CollidableDescription(Simulation.Shapes.Add(convexHull), 10f), new BodyActivityDescription(-1)));

            //var sphere = new Sphere(1.5f);
            //var capsule = new Capsule(1f, 1f);
            //var box = new Box(32f, 32f, 32f);
            //var cylinder = new Cylinder(1.5f, 0.3f);
            //const int pointCount = 32;
            //var points = new QuickList<Vector3>(pointCount, BufferPool);
            ////points.Allocate(BufferPool) = new Vector3(0, 0, 0);
            ////points.Allocate(BufferPool) = new Vector3(0, 0, 1);
            ////points.Allocate(BufferPool) = new Vector3(0, 1, 0);
            ////points.Allocate(BufferPool) = new Vector3(0, 1, 1);
            ////points.Allocate(BufferPool) = new Vector3(1, 0, 0);
            ////points.Allocate(BufferPool) = new Vector3(1, 0, 1);
            ////points.Allocate(BufferPool) = new Vector3(1, 1, 0);
            ////points.Allocate(BufferPool) = new Vector3(1, 1, 1);
            //var random = new Random(5);
            //for (int i = 0; i < pointCount; ++i)
            //{
            //    points.AllocateUnsafely() = new Vector3(3 * (float)random.NextDouble(), 1 * (float)random.NextDouble(), 3 * (float)random.NextDouble());
            //    //points.AllocateUnsafely() = new Vector3(0, 1, 0) + Vector3.Normalize(new Vector3((float)random.NextDouble() * 2 - 1, (float)random.NextDouble() * 2 - 1, (float)random.NextDouble() * 2 - 1)) * (float)random.NextDouble();
            //}
            //var convexHull = new ConvexHull(points.Span.Slice(points.Count), BufferPool, out _);
            //box.ComputeInertia(1, out var boxInertia);
            //capsule.ComputeInertia(1, out var capsuleInertia);
            //sphere.ComputeInertia(1, out var sphereInertia);
            //cylinder.ComputeInertia(1, out var cylinderInertia);
            //convexHull.ComputeInertia(1, out var hullInertia);
            //var boxIndex = Simulation.Shapes.Add(box);
            //var capsuleIndex = Simulation.Shapes.Add(capsule);
            //var sphereIndex = Simulation.Shapes.Add(sphere);
            //var cylinderIndex = Simulation.Shapes.Add(cylinder);
            //var hullIndex = Simulation.Shapes.Add(convexHull);

            const int width = 12;
            const int height = 1;
            const int length = 12;
            var shapeCount = 0;
            var random = new Random(5);
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    for (int k = 0; k < length; ++k)
                    {
                        var location = new Vector3(70, 35, 70) * new Vector3(i, j, k) + new Vector3(-width * 70 / 2f, 5f, -length * 70 / 2f);
                        var bodyDescription = new BodyDescription
                        {
                            Activity = new BodyActivityDescription(0.01f),
                            Pose = new RigidPose
                            {
                                Orientation = Quaternion.Identity,
                                Position = location
                            },
                            Collidable = new CollidableDescription
                            {
                                Continuity = new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Discrete },
                                SpeculativeMargin = 0.1f
                            }
                        };
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
                            case 2:
                            default:
                                var box = new Box(1 + 128 * (float)random.NextDouble(), 1 + 128 * (float)random.NextDouble(), 1 + 128 * (float)random.NextDouble());
                                box.ComputeInertia(1, out var boxInertia);
                                bodyDescription.Collidable.Shape = Simulation.Shapes.Add(box);
                                bodyDescription.LocalInertia = boxInertia;
                                break;
                                //case 3:
                                //    bodyDescription.Collidable.Shape = cylinderIndex;
                                //    bodyDescription.LocalInertia = cylinderInertia;
                                //    break;
                                //case 4:
                                //    bodyDescription.Collidable.Shape = hullIndex;
                                //    bodyDescription.LocalInertia = hullInertia;
                                //    break;
                        }
                        var bodyHandle = Simulation.Bodies.Add(bodyDescription);
                    }
                }
            }


        }

    }
}