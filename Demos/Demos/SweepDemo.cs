using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuUtilities;
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

        public unsafe override void Initialize(Camera camera)
        {
            camera.Position = new Vector3(0, 10, 40);
            camera.Yaw = 0;
            camera.Pitch = 0;
            Simulation = Simulation.Create(BufferPool, new TestCallbacks());
            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);

            var box = new Box(2f, 2f, 2f);
            var capsule = new Capsule(1f, 1f);
            var sphere = new Sphere(1.5f);
            box.ComputeInertia(1, out var boxInertia);
            capsule.ComputeInertia(1, out var capsuleInertia);
            sphere.ComputeInertia(1, out var sphereInertia);
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
                            Activity = new BodyActivityDescription { MinimumTimestepCountUnderThreshold = 32, SleepThreshold = 0.3f },
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


            var staticShapeIndex = Simulation.Shapes.Add(new Box(100, 1, 100));
            var staticDescription = new StaticDescription
            {
                Collidable = new CollidableDescription
                {
                    Continuity = new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Discrete },
                    Shape = Simulation.Shapes.Add(new Box(100, 1, 100)),
                    SpeculativeMargin = 0.1f
                },
                Pose = new RigidPose { Position = new Vector3(0, -1, 0), Orientation = Quaternion.Identity }
            };
            Simulation.Statics.Add(ref staticDescription);


        }

        unsafe void DrawSweep<TShape>(TShape shape, ref RigidPose pose, in BodyVelocity velocity, int steps,
            float t, Renderer renderer, in Vector3 color)
            where TShape : struct, IShape
        {
            if (steps == 1)
            {
                renderer.Shapes.AddShape(shape, Simulation.Shapes, ref pose, color);
            }
            else
            {
                var inverse = 1f / (steps - 1);
                for (int i = steps - 1; i >= 0; --i)
                {
                    var stepProgression = i * inverse;
                    var stepT = stepProgression * t;
                    PoseIntegrator.Integrate(pose, velocity, stepT, out var stepPose);
                    var stepColor = color * (0.2f + 0.8f * stepProgression);
                    renderer.Shapes.AddShape(shape, Simulation.Shapes, ref stepPose, stepColor);
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
            var intersected = task.Sweep(
                Unsafe.AsPointer(ref a), a.TypeId, poseA.Orientation, velocityA,
                Unsafe.AsPointer(ref b), b.TypeId, poseB.Position - poseA.Position, poseB.Orientation, velocityB,
                maximumT, 1e-2f, 1e-5f, 25, ref filter, Simulation.Shapes, Simulation.NarrowPhase.SweepTaskRegistry,
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

        public override void Update(Input input, float dt)
        {
            base.Update(input, dt);

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

        public override void Render(Renderer renderer, TextBuilder text, Font font)
        {
            var xRotation = Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), 0.02f * animationT * MathHelper.Pi);
            var yRotation = Quaternion.CreateFromAxisAngle(new Vector3(0, 1, 0), 0.04f * animationT * MathHelper.Pi);
            var zRotation = Quaternion.CreateFromAxisAngle(new Vector3(0, 0, 1), 0.06f * animationT * MathHelper.Pi);
            var worldA = Quaternion.Concatenate(xRotation, Quaternion.Concatenate(yRotation, zRotation));
            var worldB = Quaternion.Concatenate(yRotation, Quaternion.Concatenate(zRotation, xRotation));
            base.Render(renderer, text, font);
            var basePosition = new Vector3(0, 30, -50);
            TestSweep(
                new Sphere(0.5f),
                new RigidPose { Position = new Vector3(-10, 30, 0) + basePosition, Orientation = Quaternion.Concatenate(Quaternion.Identity, worldA) },
                new BodyVelocity { Linear = new Vector3(1, -1, 0), Angular = new Vector3(1, 0, 1) },
                new Sphere(.25f),
                new RigidPose { Position = new Vector3(10, 30, 0) + basePosition, Orientation = Quaternion.Concatenate(Quaternion.Identity, worldB) },
                new BodyVelocity { Linear = new Vector3(-1, -1, 0), Angular = new Vector3(0, 1, 0) }, 50f, renderer);
            TestSweep(
                new Sphere(0.5f),
                new RigidPose { Position = new Vector3(-10, 25, 0) + basePosition, Orientation = Quaternion.Concatenate(Quaternion.Identity, worldA) },
                new BodyVelocity { Linear = new Vector3(1, -1, 0), Angular = new Vector3(1, 0, 1) },
                new Capsule(.25f, 1f),
                new RigidPose { Position = new Vector3(10, 25, 0) + basePosition, Orientation = Quaternion.Concatenate(Quaternion.Identity, worldB) },
                new BodyVelocity { Linear = new Vector3(-1, -1, 0), Angular = new Vector3(0, 1, 0) }, 50f, renderer);
            TestSweep(
                new Sphere(0.5f),
                new RigidPose { Position = new Vector3(-10, 20, 0) + basePosition, Orientation = Quaternion.Concatenate(Quaternion.Identity, worldA) },
                new BodyVelocity { Linear = new Vector3(1, -1, 0), Angular = new Vector3(1, 0, 1) },
                new Box(.5f, 1f, 1.5f),
                new RigidPose { Position = new Vector3(10, 20, 0) + basePosition, Orientation = Quaternion.Concatenate(Quaternion.Identity, worldB) },
                new BodyVelocity { Linear = new Vector3(-1, -1, 0), Angular = new Vector3(0, 1, 0) }, 50f, renderer);
            TestSweep(
                new Capsule(0.5f, 0.5f),
                new RigidPose { Position = new Vector3(-10, 15, 0) + basePosition, Orientation = Quaternion.Concatenate(Quaternion.Identity, worldA) },
                new BodyVelocity { Linear = new Vector3(1, -1, 0), Angular = new Vector3(1, 0, 1) },
                new Capsule(.5f, 1.5f),
                new RigidPose { Position = new Vector3(10, 15, 0) + basePosition, Orientation = Quaternion.Concatenate(Quaternion.Identity, worldB) },
                new BodyVelocity { Linear = new Vector3(-1, -1, 0), Angular = new Vector3(0, 1, 0) }, 50f, renderer);
            TestSweep(
                new Capsule(0.5f, 1f),
                new RigidPose { Position = new Vector3(-10, 10, 0) + basePosition, Orientation = Quaternion.Concatenate(Quaternion.Identity, worldA) },
                new BodyVelocity { Linear = new Vector3(1, -1, 0), Angular = new Vector3(1, 0, 1) },
                new Box(1, 1, 1),
                new RigidPose { Position = new Vector3(10, 10, 0) + basePosition, Orientation = Quaternion.Concatenate(Quaternion.Identity, worldB) },
                new BodyVelocity { Linear = new Vector3(-1, -1, 0), Angular = new Vector3(0, 1, 0) }, 50f, renderer);
            TestSweep(
                new Box(0.5f, 0.5f, 0.5f),
                new RigidPose { Position = new Vector3(-10, 5, 0) + basePosition, Orientation = Quaternion.Concatenate(Quaternion.Identity, worldA) },
                new BodyVelocity { Linear = new Vector3(1, -1, 0), Angular = new Vector3(1, 0, 1) },
                new Box(.25f, .5f, 1.5f),
                new RigidPose { Position = new Vector3(10, 5, 0) + basePosition, Orientation = Quaternion.Concatenate(Quaternion.Identity, worldB) },
                new BodyVelocity { Linear = new Vector3(-1, -1, 0), Angular = new Vector3(0, 1, 0) }, 50f, renderer);

            var compoundBuilder = new CompoundBuilder(BufferPool, Simulation.Shapes, 8);
            compoundBuilder.Add(new Box(1f, 0.5f, 0.75f), new RigidPose { Orientation = Quaternion.Identity, Position = new Vector3(-0.5f, 0, 0) }, 1);
            compoundBuilder.Add(new Sphere(0.5f), new RigidPose { Orientation = Quaternion.Identity, Position = new Vector3(0.5f, 0, 0) }, 1);
            compoundBuilder.BuildKinematicCompound(out var compoundChildren);
            var compound = new Compound(compoundChildren);
            compoundBuilder.Dispose();

            TestSweep(
                compound,
                new RigidPose { Position = new Vector3(-10, 0, 0) + basePosition, Orientation = Quaternion.Concatenate(Quaternion.Identity, worldA) },
                new BodyVelocity { Linear = new Vector3(1, -1, 0), Angular = new Vector3(1, 0, 1) },
                new Sphere(0.5f),
                new RigidPose { Position = new Vector3(10, 0, 0) + basePosition, Orientation = Quaternion.Concatenate(Quaternion.Identity, worldB) },
                new BodyVelocity { Linear = new Vector3(-1, -1, 0), Angular = new Vector3(0, 1, 0) }, 50f, renderer);
            TestSweep(
                compound,
                new RigidPose { Position = new Vector3(-10, -5, 0) + basePosition, Orientation = Quaternion.Concatenate(Quaternion.Identity, worldA) },
                new BodyVelocity { Linear = new Vector3(1, -1, 0), Angular = new Vector3(1, 0, 1) },
                new Capsule(.5f, 1.5f),
                new RigidPose { Position = new Vector3(10, -5, 0) + basePosition, Orientation = Quaternion.Concatenate(Quaternion.Identity, worldB) },
                new BodyVelocity { Linear = new Vector3(-1, -1, 0), Angular = new Vector3(0, 1, 0) }, 50f, renderer);
            TestSweep(
                compound,
                new RigidPose { Position = new Vector3(-10, -10, 0) + basePosition, Orientation = Quaternion.Concatenate(Quaternion.Identity, worldA) },
                new BodyVelocity { Linear = new Vector3(1, -1, 0), Angular = new Vector3(1, 0, 1) },
                new Box(1, 1.5f, 2f),
                new RigidPose { Position = new Vector3(10, -10, 0) + basePosition, Orientation = Quaternion.Concatenate(Quaternion.Identity, worldB) },
                new BodyVelocity { Linear = new Vector3(-1, -1, 0), Angular = new Vector3(0, 1, 0) }, 50f, renderer);

            TestSweep(
                compound,
                new RigidPose { Position = new Vector3(-10, -15, 0) + basePosition, Orientation = Quaternion.Concatenate(Quaternion.Identity, worldA) },
                new BodyVelocity { Linear = new Vector3(1, -1, 0), Angular = new Vector3(1, 0, 1) },
                compound,
                new RigidPose { Position = new Vector3(10, -15, 0) + basePosition, Orientation = Quaternion.Concatenate(Quaternion.Identity, worldB) },
                new BodyVelocity { Linear = new Vector3(-1, -1, 0), Angular = new Vector3(0, 1, 0) }, 50f, renderer);

            //Get rid of the compound children registries so that we don't spam allocations.
            for (int i = 0; i < compound.Children.Length; ++i)
            {
                Simulation.Shapes.Remove(compound.Children[i].ShapeIndex);
            }

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
                Simulation.Sweep(shape, initialPose, sweepVelocity, 10, ref hitHandler);
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
