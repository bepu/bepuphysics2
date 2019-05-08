using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Collections;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using Quaternion = BepuUtilities.Quaternion;

namespace Demos.Demos
{
    public class SweepDemo : Demo
    {
        struct Filter : ISweepFilter
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowTest(int childA, int childB)
            {
                return true;
            }
        }

        ConvexHull hull;

        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 10, 40);
            camera.Yaw = 0;
            camera.Pitch = 0;
            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));

            var box = new Box(2f, 2f, 2f);
            var capsule = new Capsule(1f, 1f);
            var sphere = new Sphere(1.5f);
            box.ComputeInertia(1, out var boxInertia);
            capsule.ComputeInertia(1, out var capsuleInertia);
            sphere.ComputeInertia(1, out var sphereInertia);
            var boxIndex = Simulation.Shapes.Add(box);
            var capsuleIndex = Simulation.Shapes.Add(capsule);
            var sphereIndex = Simulation.Shapes.Add(sphere);
            const int width = 12;
            const int height = 3;
            const int length = 12;
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    for (int k = 0; k < length; ++k)
                    {
                        var location = new Vector3(5, 5, 5) * new Vector3(i, j, k) + new Vector3(-width * 2.5f, 2.5f, -length * 2.5f);
                        var bodyDescription = new BodyDescription
                        {
                            Activity = new BodyActivityDescription { MinimumTimestepCountUnderThreshold = 32, SleepThreshold = 0.1f },
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
                        Simulation.Bodies.Add(bodyDescription);

                    }
                }
            }

            //Don't really want to regenerate a convex hull every frame; just cache one out.
            const int pointCount = 32;
            var points = new QuickList<Vector3>(pointCount, BufferPool);
            var random = new Random(5);
            for (int i = 0; i < pointCount; ++i)
            {
                points.AllocateUnsafely() = new Vector3((float)random.NextDouble() - 0.5f,  (float)random.NextDouble() - 0.5f, (float)random.NextDouble() - 0.5f);
            }
            ConvexHullHelper.CreateShape(points.Span.Slice(points.Count), BufferPool, out _, out hull);
            points.Dispose(BufferPool);

            //var staticShapeIndex = Simulation.Shapes.Add(new Box(100, 1, 100));
            //var staticDescription = new StaticDescription
            //{
            //    Collidable = new CollidableDescription
            //    {
            //        Continuity = new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Discrete },
            //        Shape = Simulation.Shapes.Add(new Box(100, 1, 100)),
            //        SpeculativeMargin = 0.1f
            //    },
            //    Pose = new RigidPose { Position = new Vector3(0, -1, 0), Orientation = Quaternion.Identity }
            //};
            //Simulation.Statics.Add(staticDescription);

            const int planeWidth = 64;
            const int planeHeight = 64;
            DemoMeshHelper.CreateDeformedPlane(planeWidth, planeHeight,
                (int x, int y) =>
                {
                    return new Vector3(x, 1 * MathF.Cos(x / 4f) * MathF.Sin(y / 4f), y);
                }, new Vector3(2, 3, 2), BufferPool, out var planeMesh);
            Simulation.Statics.Add(new StaticDescription(new Vector3(-64, -10, -64), new CollidableDescription(Simulation.Shapes.Add(planeMesh), 0.1f)));

        }

        void DrawShape<TShape>(ref TShape shape, ref RigidPose pose, in Vector3 color, Shapes shapes, Renderer renderer)
            where TShape : struct, IShape
        {
            if (typeof(TShape) == typeof(Triangle))
            {
                //For the sake of visualization in this demo, give the triangles a backface. Collisions don't have backfaces, but sweeps do, and it's nice to be able to see the shape.
                //A little bit hacky, but hey, it works.
                ref var triangle = ref Unsafe.As<TShape, Triangle>(ref shape);
                Triangle flippedTriangle;
                flippedTriangle.C = triangle.C;
                flippedTriangle.A = triangle.B;
                flippedTriangle.B = triangle.A;
                renderer.Shapes.AddShape(triangle, shapes, ref pose, color);
                renderer.Shapes.AddShape(flippedTriangle, shapes, ref pose, color);
            }
            else
            {
                renderer.Shapes.AddShape(shape, shapes, ref pose, color);
            }
        }

        unsafe void DrawSweep<TShape>(TShape shape, ref RigidPose pose, in BodyVelocity velocity, int steps,
            float t, Renderer renderer, in Vector3 color)
            where TShape : struct, IShape
        {
            if (steps == 1)
            {
                DrawShape(ref shape, ref pose, color, Simulation.Shapes, renderer);
            }
            else
            {
                var inverse = 1f / (steps - 1);
                for (int i = steps - 1; i >= 0; --i)
                {
                    var stepProgression = i * inverse;
                    var stepT = stepProgression * t;
                    PoseIntegration.Integrate(pose, velocity, stepT, out var stepPose);
                    var stepColor = color * (0.2f + 0.8f * stepProgression);
                    DrawShape(ref shape, ref stepPose, stepColor, Simulation.Shapes, renderer);
                }
            }
        }

        unsafe void DrawImpact(Renderer renderer, ref Vector3 hitLocation, ref Vector3 hitNormal)
        {
            //The normal itself will tend to be obscured by the shapes, so instead draw two lines representing the plane.
            DemoRenderer.Constraints.ContactLines.BuildOrthnormalBasis(hitNormal, out var tangent1, out var tangent2);
            renderer.Lines.Allocate() = new DemoRenderer.Constraints.LineInstance(hitLocation - tangent1, hitLocation + tangent1, new Vector3(0, 1, 0), new Vector3());
            renderer.Lines.Allocate() = new DemoRenderer.Constraints.LineInstance(hitLocation - tangent2, hitLocation + tangent2, new Vector3(0, 1, 0), new Vector3());
        }
        unsafe void TestSweep<TShapeA, TShapeB>(
            TShapeA a, RigidPose poseA, in BodyVelocity velocityA,
            TShapeB b, RigidPose poseB, in BodyVelocity velocityB,
            float maximumT, Renderer renderer)
            where TShapeA : struct, IShape
            where TShapeB : struct, IShape
        {
            var filter = new Filter();

            var task = Simulation.NarrowPhase.SweepTaskRegistry.GetTask(a.TypeId, b.TypeId);
            if (task == null)
                return;
            var intersected = task.Sweep(
                Unsafe.AsPointer(ref a), a.TypeId, poseA.Orientation, velocityA,
                Unsafe.AsPointer(ref b), b.TypeId, poseB.Position - poseA.Position, poseB.Orientation, velocityB,
                maximumT, 1e-2f, 1e-5f, 25, ref filter, Simulation.Shapes, Simulation.NarrowPhase.SweepTaskRegistry, BufferPool,
                out var t0, out var t1, out var hitLocation, out var hitNormal);
            hitLocation += poseA.Position;

            var hitTint = intersected ? new Vector3(0.5f, 1, 0.5f) : new Vector3(1f, 0.5f, 0.5f);
            var colorA = new Vector3(0.75f, 0.75f, 1) * hitTint;
            var colorB = new Vector3(0.75f, 1f, 1) * hitTint;

            var stepCount = (intersected && t1 > 0) || !intersected ? 100 : 1;
            var visualizedT = intersected ? t1 : maximumT;
            DrawSweep(a, ref poseA, velocityA, stepCount, visualizedT, renderer, colorA);
            DrawSweep(b, ref poseB, velocityB, stepCount, visualizedT, renderer, colorB);

            if (intersected && t1 > 0)
            {
                DrawImpact(renderer, ref hitLocation, ref hitNormal);
            }
        }

        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            base.Update(window, camera, input, dt);

            if (!input.WasDown(OpenTK.Input.Key.P))
                animationT = (animationT + 1 / 60f) % (128);
        }

        float animationT;


        struct SceneSweepHitHandler : ISweepHitHandler
        {
            public Vector3 HitLocation;
            public Vector3 HitNormal;
            public float T;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowTest(CollidableReference collidable)
            {
                return true;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowTest(CollidableReference collidable, int child)
            {
                return true;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void OnHit(ref float maximumT, float t, in Vector3 hitLocation, in Vector3 hitNormal, CollidableReference collidable)
            {
                //Changing the maximum T value prevents the traversal from visiting any leaf nodes more distant than that later in the traversal.
                //It is effectively an optimization that you can use if you only care about the time of first impact.
                if (t < maximumT)
                    maximumT = t;
                if (t < T)
                {
                    T = t;
                    HitLocation = hitLocation;
                    HitNormal = hitNormal;
                }
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void OnHitAtZeroT(ref float maximumT, CollidableReference collidable)
            {
                maximumT = 0;
                T = 0;
                HitLocation = new Vector3();
                HitNormal = new Vector3();
            }
        }

        void StandardTestSweep<TA, TB>(in TA a, in TB b, ref Vector3 position, in Quaternion initialOrientationA, in Quaternion initialOrientationB, Renderer renderer)
            where TA : struct, IShape where TB : struct, IShape
        {
            TestSweep(
                a,
                new RigidPose { Position = new Vector3(-10, 0, 0) + position, Orientation = initialOrientationA },
                new BodyVelocity { Linear = new Vector3(1, -1, 0), Angular = new Vector3(1, 0, 1) },
                b,
                new RigidPose { Position = new Vector3(10, 0, 0) + position, Orientation = initialOrientationB },
                new BodyVelocity { Linear = new Vector3(-1, -1, 0), Angular = new Vector3(0, 1, 0) },
                50f, renderer);
            position.Y -= 5;
        }
        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            var xRotation = Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), 0.02f * animationT * MathHelper.Pi);
            var yRotation = Quaternion.CreateFromAxisAngle(new Vector3(0, 1, 0), 0.04f * animationT * MathHelper.Pi);
            var zRotation = Quaternion.CreateFromAxisAngle(new Vector3(0, 0, 1), 0.06f * animationT * MathHelper.Pi);
            var worldA = Quaternion.Concatenate(xRotation, Quaternion.Concatenate(yRotation, zRotation));
            var worldB = Quaternion.Concatenate(yRotation, Quaternion.Concatenate(zRotation, xRotation));
            base.Render(renderer, camera, input, text, font);

            var compoundBuilder = new CompoundBuilder(BufferPool, Simulation.Shapes, 8);
            compoundBuilder.Add(new Box(1f, 0.5f, 0.75f), new RigidPose { Orientation = Quaternion.Identity, Position = new Vector3(-0.5f, 0, 0) }, 1);
            compoundBuilder.Add(new Sphere(0.5f), new RigidPose { Orientation = Quaternion.Identity, Position = new Vector3(0.5f, 0, 0) }, 1);
            compoundBuilder.BuildKinematicCompound(out var compoundChildren);
            var compound = new Compound(compoundChildren);
            var bigCompound = new BigCompound(compoundChildren, Simulation.Shapes, BufferPool);
            compoundBuilder.Dispose();

            const int planeWidth = 3;
            const int planeHeight = 3;
            DemoMeshHelper.CreateDeformedPlane(planeWidth, planeHeight,
                (int x, int y) =>
                {
                    return new Vector3(x - 1.5f, 0.1f * MathF.Cos(x) * MathF.Sin(y), y - 1.5f);
                }, new Vector3(1, 2, 1), BufferPool, out var mesh);


            var triangle = new Triangle(new Vector3(0, 0, 0), new Vector3(2, 0, -1), new Vector3(-1, 0, 1.5f));
            var triangleCenter = (triangle.A + triangle.B + triangle.C) / 3f;
            triangle.A -= triangleCenter;
            triangle.B -= triangleCenter;
            triangle.C -= triangleCenter;
            var position = new Vector3(-90, 60, -75);
            StandardTestSweep(new Sphere(0.5f), new Sphere(.25f), ref position, worldA, worldB, renderer);
            StandardTestSweep(new Sphere(0.5f), new Capsule(.25f, 1f), ref position, worldA, worldB, renderer);
            StandardTestSweep(new Sphere(0.5f), new Box(.5f, 1f, 1.5f), ref position, worldA, worldB, renderer);
            StandardTestSweep(new Sphere(0.5f), triangle, ref position, worldA, worldB, renderer);
            StandardTestSweep(new Sphere(0.5f), new Cylinder(.25f, 1f), ref position, worldA, worldB, renderer);
            StandardTestSweep(new Sphere(0.5f), hull, ref position, worldA, worldB, renderer);
            StandardTestSweep(new Sphere(0.5f), compound, ref position, worldA, worldB, renderer);
            StandardTestSweep(new Sphere(0.5f), bigCompound, ref position, worldA, worldB, renderer);
            StandardTestSweep(new Sphere(0.5f), mesh, ref position, worldA, worldB, renderer);

            position = new Vector3(-60, 60, -75);
            StandardTestSweep(new Capsule(0.5f, 1), new Capsule(.25f, 1.5f), ref position, worldA, worldB, renderer);
            StandardTestSweep(new Capsule(0.5f, 1), new Box(.5f, 1f, 1.5f), ref position, worldA, worldB, renderer);
            StandardTestSweep(new Capsule(0.5f, 1), triangle, ref position, worldA, worldB, renderer);
            StandardTestSweep(new Capsule(0.5f, 1), new Cylinder(.25f, 1.5f), ref position, worldA, worldB, renderer);
            StandardTestSweep(new Capsule(0.5f, 1), hull, ref position, worldA, worldB, renderer);
            StandardTestSweep(new Capsule(0.5f, 1), compound, ref position, worldA, worldB, renderer);
            StandardTestSweep(new Capsule(0.5f, 1), bigCompound, ref position, worldA, worldB, renderer);
            StandardTestSweep(new Capsule(0.5f, 1), mesh, ref position, worldA, worldB, renderer);

            position = new Vector3(-30, 60, -75);
            StandardTestSweep(new Box(0.5f, 0.5f, 0.5f), new Box(.5f, 1f, 1.5f), ref position, worldA, worldB, renderer);
            StandardTestSweep(new Box(0.5f, 0.5f, 0.5f), triangle, ref position, worldA, worldB, renderer);
            StandardTestSweep(new Box(0.5f, 0.5f, 0.5f), new Cylinder(.25f, 1.5f), ref position, worldA, worldB, renderer);
            StandardTestSweep(new Box(0.5f, 0.5f, 0.5f), hull, ref position, worldA, worldB, renderer);
            StandardTestSweep(new Box(0.5f, 0.5f, 0.5f), compound, ref position, worldA, worldB, renderer);
            StandardTestSweep(new Box(0.5f, 0.5f, 0.5f), bigCompound, ref position, worldA, worldB, renderer);
            StandardTestSweep(new Box(0.5f, 0.5f, 0.5f), mesh, ref position, worldA, worldB, renderer);

            position = new Vector3(0, 60, -75);
            StandardTestSweep(triangle, triangle, ref position, worldA, worldB, renderer);
            StandardTestSweep(triangle, new Cylinder(0.5f, 1), ref position, worldA, worldB, renderer);
            StandardTestSweep(triangle, hull, ref position, worldA, worldB, renderer);
            StandardTestSweep(triangle, compound, ref position, worldA, worldB, renderer);
            StandardTestSweep(triangle, bigCompound, ref position, worldA, worldB, renderer);
            StandardTestSweep(triangle, mesh, ref position, worldA, worldB, renderer);

            position = new Vector3(30, 60, -75);
            StandardTestSweep(new Cylinder(0.5f, 1), new Cylinder(.25f, 1.5f), ref position, worldA, worldB, renderer);
            StandardTestSweep(new Cylinder(0.5f, 1), hull, ref position, worldA, worldB, renderer);
            StandardTestSweep(new Cylinder(0.5f, 1), compound, ref position, worldA, worldB, renderer);
            StandardTestSweep(new Cylinder(0.5f, 1), bigCompound, ref position, worldA, worldB, renderer);
            StandardTestSweep(new Cylinder(0.5f, 1), mesh, ref position, worldA, worldB, renderer);

            position = new Vector3(60, 60, -75);
            StandardTestSweep(hull, hull, ref position, worldA, worldB, renderer);
            StandardTestSweep(hull, compound, ref position, worldA, worldB, renderer);
            StandardTestSweep(hull, bigCompound, ref position, worldA, worldB, renderer);
            StandardTestSweep(hull, mesh, ref position, worldA, worldB, renderer);

            position = new Vector3(90, 60, -75);
            StandardTestSweep(compound, compound, ref position, worldA, worldB, renderer);
            StandardTestSweep(compound, bigCompound, ref position, worldA, worldB, renderer);
            StandardTestSweep(compound, mesh, ref position, worldA, worldB, renderer);

            position = new Vector3(120, 60, -75);
            StandardTestSweep(bigCompound, bigCompound, ref position, worldA, worldB, renderer);
            StandardTestSweep(bigCompound, mesh, ref position, worldA, worldB, renderer);


            //Get rid of the compound children registries so that we don't spam allocations.
            for (int i = 0; i < compound.Children.Length; ++i)
            {
                Simulation.Shapes.Remove(compound.Children[i].ShapeIndex);
            }
            //The resources of compound are a subset of those used by the bigCompoound, so we just dispose the bigCompound.
            bigCompound.Dispose(BufferPool);
            mesh.Dispose(BufferPool);

            //Perform simulation-wide queries against the other collidables in the scene.
            var localOrigin = new Vector3(-25, 15, 0);
            var localDirection = new Vector3(7, -10, 0);
            var sweepCount = 16;
            for (int i = 0; i < sweepCount; ++i)
            {
                Matrix3x3.CreateFromAxisAngle(new Vector3(0, 1, 0), i * MathHelper.TwoPi / sweepCount, out var rotation);
                Matrix3x3.Transform(localOrigin, rotation, out var sweepOrigin);
                Matrix3x3.Transform(localDirection, rotation, out var sweepDirection);

                SceneSweepHitHandler hitHandler = default;
                hitHandler.T = float.MaxValue;
                var shape = new Box(1, 2, 1.5f);
                var initialPose = new RigidPose { Position = sweepOrigin, Orientation = Quaternion.Identity };
                var sweepVelocity = new BodyVelocity { Linear = sweepDirection };
                Simulation.Sweep(shape, initialPose, sweepVelocity, 10, BufferPool, ref hitHandler);
                DrawSweep(shape, ref initialPose, sweepVelocity, 20, hitHandler.T, renderer,
                    hitHandler.T < float.MaxValue ? new Vector3(0.25f, 1, 0.25f) : new Vector3(1, 0.25f, 0.25f));

                if (hitHandler.T < float.MaxValue && hitHandler.T > 0)
                {
                    DrawImpact(renderer, ref hitHandler.HitLocation, ref hitHandler.HitNormal);
                }
            }
        }



    }
}
