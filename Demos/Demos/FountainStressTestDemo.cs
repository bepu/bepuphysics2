using BepuUtilities;
using DemoRenderer;
using DemoUtilities;
using BepuPhysics;
using BepuPhysics.Collidables;
using System;
using System.Numerics;
using BepuUtilities.Memory;
using BepuUtilities.Collections;
using System.Diagnostics;
using DemoContentLoader;

namespace Demos.Demos
{
    public class FountainStressTestDemo : Demo
    {
        QuickQueue<StaticDescription> removedStatics;
        QuickQueue<BodyHandle> dynamicHandles;
        Random random;
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-15f, 20, -15f);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = MathHelper.Pi * 0.1f;
            //Using minimum sized allocations forces as many resizes as possible.
            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new PositionFirstTimestepper(), initialAllocationSizes:
            new SimulationAllocationSizes
            {
                Bodies = 1,
                ConstraintCountPerBodyEstimate = 1,
                Constraints = 1,
                ConstraintsPerTypeBatch = 1,
                Islands = 1,
                ShapesPerType = 1,
                Statics = 1
            });

            Simulation.Deterministic = true;

            const int planeWidth = 8;
            const int planeHeight = 8;
            DemoMeshHelper.CreateDeformedPlane(planeWidth, planeHeight,
                (int x, int y) =>
                {
                    Vector2 offsetFromCenter = new Vector2(x - planeWidth / 2, y - planeHeight / 2);
                    return new Vector3(offsetFromCenter.X, MathF.Cos(x / 4f) * MathF.Sin(y / 4f) - 0.2f * offsetFromCenter.LengthSquared(), offsetFromCenter.Y);
                }, new Vector3(2, 1, 2), BufferPool, out var staticShape);
            var staticShapeIndex = Simulation.Shapes.Add(staticShape);
            const int staticGridWidthInInstances = 128;
            const float staticSpacing = 8;
            for (int i = 0; i < staticGridWidthInInstances; ++i)
            {
                for (int j = 0; j < staticGridWidthInInstances; ++j)
                {
                    var staticDescription = new StaticDescription(new Vector3(
                            -staticGridWidthInInstances * staticSpacing * 0.5f + i * staticSpacing,
                            -4 + 4 * (float)Math.Cos(i * 0.3) + 4 * (float)Math.Cos(j * 0.3),
                            -staticGridWidthInInstances * staticSpacing * 0.5f + j * staticSpacing),
                            new CollidableDescription(staticShapeIndex, 0.1f));
                    Simulation.Statics.Add(staticDescription);
                }
            }

            //A bunch of kinematic balls do acrobatics as an extra stressor.
            var kinematicShape = new Sphere(8);
            var kinematicShapeIndex = Simulation.Shapes.Add(kinematicShape);
            var kinematicCount = 64;
            var anglePerKinematic = MathHelper.TwoPi / kinematicCount;
            var startingRadius = 256;
            kinematicHandles = new BodyHandle[kinematicCount];
            for (int i = 0; i < kinematicCount; ++i)
            {
                var angle = anglePerKinematic * i;
                var description = BodyDescription.CreateKinematic(new Vector3(
                            startingRadius * (float)Math.Cos(angle),
                            0,
                            startingRadius * (float)Math.Sin(angle)),
                            new CollidableDescription(kinematicShapeIndex, 0.1f),
                            new BodyActivityDescription(0, 4));
                kinematicHandles[i] = Simulation.Bodies.Add(description);
            }

            dynamicHandles = new QuickQueue<BodyHandle>(65536, BufferPool);
            removedStatics = new QuickQueue<StaticDescription>(512, BufferPool);
            random = new Random(5);
        }

        double time;
        double t;
        BodyHandle[] kinematicHandles;

        void AddConvexShape<TConvex>(in TConvex convex, out TypedIndex shapeIndex, out BodyInertia inertia) where TConvex : unmanaged, IConvexShape
        {
            shapeIndex = Simulation.Shapes.Add(convex);
            convex.ComputeInertia(1, out inertia);
        }

        ConvexHull CreateRandomHull()
        {
            const int pointCount = 32;
            var points = new QuickList<Vector3>(pointCount, BufferPool);
            for (int i = 0; i < pointCount; ++i)
            {
                points.AllocateUnsafely() = new Vector3(1 * (float)random.NextDouble(), 0.5f * (float)random.NextDouble(), 1.5f * (float)random.NextDouble());
            }
            var hull = new ConvexHull(points.Span.Slice(points.Count), BufferPool, out _);
            points.Dispose(BufferPool);
            return hull;
        }

        void CreateRandomCompound(out Buffer<CompoundChild> children, out BodyInertia inertia)
        {
            using (var compoundBuilder = new CompoundBuilder(BufferPool, Simulation.Shapes, 6))
            {
                var childCount = random.Next(2, 6);
                for (int i = 0; i < childCount; ++i)
                {
                    TypedIndex shapeIndex;
                    BodyInertia childInertia;
                    switch (random.Next(0, 5))
                    {
                        default:
                            AddConvexShape(new Sphere(0.35f + 0.35f * (float)random.NextDouble()), out shapeIndex, out childInertia);
                            break;
                        case 1:
                            AddConvexShape(new Capsule(
                                0.35f + 0.35f * (float)random.NextDouble(),
                                0.35f + 0.35f * (float)random.NextDouble()), out shapeIndex, out childInertia);
                            break;
                        case 2:
                            AddConvexShape(new Box(
                                0.35f + 0.35f * (float)random.NextDouble(),
                                0.35f + 0.35f * (float)random.NextDouble(),
                                0.35f + 0.35f * (float)random.NextDouble()), out shapeIndex, out childInertia);
                            break;
                        case 3:
                            AddConvexShape(new Cylinder(0.1f + (float)random.NextDouble(), 0.2f + (float)random.NextDouble()), out shapeIndex, out childInertia);
                            break;
                        case 4:
                            AddConvexShape(CreateRandomHull(), out shapeIndex, out childInertia);
                            break;
                    }
                    RigidPose localPose;
                    localPose.Position = new Vector3(2, 2, 2) * (0.5f * new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) - Vector3.One);
                    float orientationLengthSquared;
                    do
                    {
                        localPose.Orientation = new Quaternion((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble());
                    }
                    while ((orientationLengthSquared = localPose.Orientation.LengthSquared()) < 1e-9f);
                    QuaternionEx.Scale(localPose.Orientation, 1f / MathF.Sqrt(orientationLengthSquared), out localPose.Orientation);
                    compoundBuilder.Add(shapeIndex, localPose, childInertia.InverseInertiaTensor, 1);
                }
                compoundBuilder.BuildDynamicCompound(out children, out inertia, out var center);
            }
        }

        public void CreateBodyDescription(Random random, in RigidPose pose, in BodyVelocity velocity, out BodyDescription description)
        {
            //For the sake of the stress test, every single body has its own shape that gets removed when the body is removed.
            TypedIndex shapeIndex;
            BodyInertia inertia;
            switch (random.Next(0, 7))
            {
                default:
                    {
                        AddConvexShape(new Sphere(0.35f + 0.35f * (float)random.NextDouble()), out shapeIndex, out inertia);
                    }
                    break;
                case 1:
                    {
                        AddConvexShape(new Capsule(
                            0.35f + 0.35f * (float)random.NextDouble(),
                            0.35f + 0.35f * (float)random.NextDouble()), out shapeIndex, out inertia);
                    }
                    break;
                case 2:
                    {
                        AddConvexShape(new Box(
                            0.35f + 0.6f * (float)random.NextDouble(),
                            0.35f + 0.6f * (float)random.NextDouble(),
                            0.35f + 0.6f * (float)random.NextDouble()), out shapeIndex, out inertia);
                    }
                    break;
                case 3:
                    {
                        AddConvexShape(new Cylinder(0.1f + 0.5f * (float)random.NextDouble(), 0.2f + (float)random.NextDouble()), out shapeIndex, out inertia);
                    }
                    break;
                case 4:
                    {
                        AddConvexShape(CreateRandomHull(), out shapeIndex, out inertia);
                    }
                    break;
                case 5:
                    {
                        CreateRandomCompound(out var children, out inertia);
                        shapeIndex = Simulation.Shapes.Add(new Compound(children));
                    }
                    break;
                case 6:
                    {
                        CreateRandomCompound(out var children, out inertia);
                        shapeIndex = Simulation.Shapes.Add(new BigCompound(children, Simulation.Shapes, BufferPool));
                    }
                    break;
            }

            description = new BodyDescription
            {
                Pose = pose,
                LocalInertia = inertia,
                Collidable = new CollidableDescription(shapeIndex, 5f),
                Activity = new BodyActivityDescription(0.1f),
                Velocity = velocity
            };
            switch (random.Next(3))
            {
                case 0: description.Collidable.Continuity = ContinuousDetectionSettings.Discrete; break;
                case 1: description.Collidable.Continuity = ContinuousDetectionSettings.Passive; break;
                case 2: description.Collidable.Continuity = ContinuousDetectionSettings.Continuous(1e-3f, 1e-3f); break;
            }
        }

        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            var timestepDuration = 1f / 60f;
            time += timestepDuration;

            //Occasionally, the animation stops completely. The resulting velocities will be zero, so the kinematics will have a chance to rest (testing kinematic rest states).
            var dip = 0.1;
            var progressionMultiplier = 0.5 - dip + (1 + dip) * 0.5 * Math.Cos(time * 0.25);
            if (progressionMultiplier < 0)
                progressionMultiplier = 0;
            t += timestepDuration * progressionMultiplier;

            var baseAngle = (float)(t * 0.015);
            var anglePerKinematic = MathHelper.TwoPi / kinematicHandles.Length;
            var maxDisplacement = 50 * timestepDuration;
            var inverseDt = 1f / timestepDuration;
            for (int i = 0; i < kinematicHandles.Length; ++i)
            {
                ref var bodyLocation = ref Simulation.Bodies.HandleToLocation[kinematicHandles[i].Value];

                ref var set = ref Simulation.Bodies.Sets[bodyLocation.SetIndex];
                var angle = anglePerKinematic * i;
                var positionAngle = baseAngle + angle;
                var radius = 128 + 32 * (float)Math.Cos(3 * (angle + t * (1f / 3f))) + 32 * (float)Math.Cos(t * (1f / 3f));
                var targetLocation = new Vector3(
                    radius * (float)Math.Cos(positionAngle),
                    16 + 16 * (float)Math.Cos(4 * (angle + t * 0.5)),
                    radius * (float)Math.Sin(positionAngle));

                var correction = targetLocation - set.Poses[bodyLocation.Index].Position;
                var distance = correction.Length();
                if (distance > 1e-4)
                {
                    if (bodyLocation.SetIndex > 0)
                    {
                        //We're requesting a nonzero velocity, so it must be active.
                        Simulation.Awakener.AwakenSet(bodyLocation.SetIndex);
                    }
                    if (distance > maxDisplacement)
                    {
                        correction *= maxDisplacement / distance;
                    }
                    Debug.Assert(bodyLocation.SetIndex == 0);
                    Simulation.Bodies.ActiveSet.Velocities[bodyLocation.Index].Linear = correction * inverseDt;
                }
                else
                {
                    if (bodyLocation.SetIndex == 0)
                    {
                        Simulation.Bodies.ActiveSet.Velocities[bodyLocation.Index].Linear = new Vector3();
                    }
                }
            }

            //Remove some statics from the simulation.
            var missingStaticsAsymptote = 512;
            var staticRemovalsPerFrame = 8;
            for (int i = 0; i < staticRemovalsPerFrame; ++i)
            {
                var indexToRemove = random.Next(Simulation.Statics.Count);
                Simulation.Statics.GetDescription(Simulation.Statics.IndexToHandle[indexToRemove], out var staticDescription);
                Simulation.Statics.RemoveAt(indexToRemove);
                removedStatics.Enqueue(staticDescription, BufferPool);
            }

            var staticApplyDescriptionsPerFrame = 8;
            for (int i = 0; i < staticApplyDescriptionsPerFrame; ++i)
            {
                var indexToReapply = random.Next(Simulation.Statics.Count);
                var handleToReapply = Simulation.Statics.IndexToHandle[indexToReapply];
                Simulation.Statics.GetDescription(handleToReapply, out var staticDescription);
                //Statics don't have as much in the way of transitions. They can't be shapeless, and going from one shape to another doesn't anything that a pose change doesn't. For now, we'll just test the application of descriptions with different poses.
                var mutatedDescription = staticDescription;
                mutatedDescription.Pose.Position.Y += 50;
                QuaternionEx.Concatenate(mutatedDescription.Pose.Orientation, QuaternionEx.CreateFromAxisAngle(new Vector3(0, 1, 0), (float)random.NextDouble() * MathF.PI), out mutatedDescription.Pose.Orientation);
                Simulation.Statics.ApplyDescription(handleToReapply, mutatedDescription);
                Simulation.Statics.ApplyDescription(handleToReapply, staticDescription);
            }


            //Add some of the missing static bodies back into the simulation.
            var staticAddCount = removedStatics.Count * (staticRemovalsPerFrame / (float)missingStaticsAsymptote);
            for (int i = 0; i < staticAddCount; ++i)
            {
                Debug.Assert(removedStatics.Count > 0);
                var staticDescription = removedStatics.Dequeue();
                Simulation.Statics.Add(staticDescription);
            }


            //Spray some shapes!
            int newShapeCount = 8;
            var spawnPose = new RigidPose(new Vector3(0, 10, 0));
            for (int i = 0; i < newShapeCount; ++i)
            {
                CreateBodyDescription(random, spawnPose, new BodyVelocity(new Vector3(-30 + 60 * (float)random.NextDouble(), 75, -30 + 60 * (float)random.NextDouble()), default), out var bodyDescription);
                dynamicHandles.Enqueue(Simulation.Bodies.Add(bodyDescription), BufferPool);
            }
            int targetAsymptote = 65536;
            var removalCount = (int)(dynamicHandles.Count * (newShapeCount / (float)targetAsymptote));
            for (int i = 0; i < removalCount; ++i)
            {
                if (dynamicHandles.TryDequeue(out var handle))
                {
                    ref var bodyLocation = ref Simulation.Bodies.HandleToLocation[handle.Value];
                    //Every body has a unique shape, so we need to remove shapes with bodies.
                    var shapeIndex = Simulation.Bodies.Sets[bodyLocation.SetIndex].Collidables[bodyLocation.Index].Shape;
                    Simulation.Bodies.Remove(handle);
                    Simulation.Shapes.RecursivelyRemoveAndDispose(shapeIndex, BufferPool);
                }
                else
                {
                    break;
                }
            }

            //Change some dynamic objects without adding/removing them to make sure all the state transition stuff works reasonably well.
            var dynamicApplyDescriptionsPerFrame = 8;
            for (int i = 0; i < dynamicApplyDescriptionsPerFrame; ++i)
            {
                var handle = dynamicHandles[random.Next(dynamicHandles.Count)];
                Simulation.Bodies.GetDescription(handle, out var description);
                Simulation.Shapes.RecursivelyRemoveAndDispose(description.Collidable.Shape, BufferPool);
                CreateBodyDescription(random, description.Pose, description.Velocity, out var newDescription);
                if (random.NextDouble() < 0.1f)
                {
                    //Occasionally make a dynamic kinematic.
                    newDescription.LocalInertia = default;
                }
                Simulation.Bodies.ApplyDescription(handle, newDescription);
                Simulation.Bodies.UpdateBounds(handle);
            }

            base.Update(window, camera, input, dt);

            if (input != null && input.WasPushed(OpenTK.Input.Key.P))
                GC.Collect(int.MaxValue, GCCollectionMode.Forced, true, true);

        }

    }
}
