using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using SharpDX.Direct3D11;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace Demos.Demos
{
    /// <summary>
    /// Shows a bundle of ropes being tangled up by spinning weights.
    /// </summary>
    public class RopeTwistDemo : Demo
    {
        struct Filter
        {
            public short RopeIndex;
            public short IndexInRope;
        }

        unsafe struct RopeNarrowPhaseCallbacks : INarrowPhaseCallbacks
        {
            public CollidableProperty<Filter> Filters;
            public SpringSettings ContactSpringiness;

            public void Initialize(Simulation simulation)
            {
                Filters.Initialize(simulation);
                //Use a default if the springiness value wasn't initialized.
                if (ContactSpringiness.AngularFrequency == 0 && ContactSpringiness.TwiceDampingRatio == 0)
                    ContactSpringiness = new SpringSettings(30, 1);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b)
            {
                var aFilter = Filters[a];
                var bFilter = Filters[b];
                return (aFilter.RopeIndex != bFilter.RopeIndex || Math.Abs(aFilter.IndexInRope - bFilter.IndexInRope) > 3) && (a.Mobility == CollidableMobility.Dynamic || b.Mobility == CollidableMobility.Dynamic);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
            {
                return true;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial) where TManifold : unmanaged, IContactManifold<TManifold>
            {
                pairMaterial.FrictionCoefficient = 0f;
                pairMaterial.MaximumRecoveryVelocity = 200f;
                pairMaterial.SpringSettings = ContactSpringiness;
                return true;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold)
            {
                return true;
            }

            public void Dispose()
            {
            }
        }

        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 25, 45);
            camera.Yaw = 0;
            camera.Pitch = 0;

            var filters = new CollidableProperty<Filter>();
            Simulation = Simulation.Create(BufferPool,
                new RopeNarrowPhaseCallbacks { ContactSpringiness = new SpringSettings(1200, 1), Filters = filters },
                new DemoPoseIntegratorCallbacks(new Vector3(0, 0, 0)), new EmbeddedSubsteppingTimestepper2(30), solverIterationCount: 1);
            //Simulation = Simulation.Create(BufferPool,
            //    new RopeNarrowPhaseCallbacks { ContactSpringiness = new SpringSettings(1200, 1), Filters = filters },
            //    new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SubsteppingTimestepper2(30), solverIterationCount: 1);


            for (int twistIndex = 0; twistIndex < 5; ++twistIndex)
            {
                const int ropeCount = 4;
                var startLocation = new Vector3(-30 + twistIndex * 15, 50, 0);

                var bigWreckingBall = new Sphere(3);
                //This wrecking ball is much, much heavier.
                bigWreckingBall.ComputeInertia(10000, out var bigWreckingBallInertia);
                var bigWreckingBallIndex = Simulation.Shapes.Add(bigWreckingBall);
                const float ropeBodySpacing = -0.1f;
                const float ropeBodyRadius = 0.2f;
                const int ropeBodyCount = 130;
                var wreckingBallPosition = startLocation - new Vector3(0, ropeBodyRadius + (ropeBodyRadius * 2 + ropeBodySpacing) * ropeBodyCount + bigWreckingBall.Radius, 0);
                var description = BodyDescription.CreateDynamic(wreckingBallPosition, bigWreckingBallInertia, new CollidableDescription(bigWreckingBallIndex, 25f), new BodyActivityDescription(-0.01f));
                //Give it a little bump.
                //description.Velocity = new BodyVelocity(new Vector3(-10, 0, 0), default);
                var wreckingBallBodyHandle = Simulation.Bodies.Add(description);
                var wreckingBallBody = Simulation.Bodies.GetBodyReference(wreckingBallBodyHandle);
                wreckingBallBody.Velocity.Angular = new Vector3(0, 0, 0);
                filters.Allocate(wreckingBallBodyHandle) = new Filter { RopeIndex = (short)(16384 + twistIndex), IndexInRope = ropeBodyCount };

                for (int ropeIndex = 0; ropeIndex < ropeCount; ++ropeIndex)
                {
                    var angle = ropeIndex * MathF.PI * 2 / ropeCount;
                    const float ropeDistributionRadius = 1f;
                    var horizontalOffset = ropeDistributionRadius * new Vector3(MathF.Sin(angle), 0, MathF.Cos(angle));
                    var ropeStartLocation = startLocation + horizontalOffset;

                    var springSettings = new SpringSettings(300, 1);
                    var bodyHandles = RopeStabilityDemo.BuildRopeBodies(Simulation, ropeStartLocation, ropeBodyCount, ropeBodyRadius, ropeBodySpacing, 1f, 0, 25f);
                    for (int i = 0; i < bodyHandles.Length; ++i)
                    {
                        filters.Allocate(bodyHandles[i]) = new Filter { RopeIndex = (short)ropeIndex, IndexInRope = (short)i };
                    }

                    bool TryCreateConstraint(int handleIndexA, int handleIndexB)
                    {
                        if (handleIndexA >= bodyHandles.Length || handleIndexB >= bodyHandles.Length)
                            return false;
                        var maximumDistance = Vector3.Distance(
                            new BodyReference(bodyHandles[handleIndexA], Simulation.Bodies).Pose.Position,
                            new BodyReference(bodyHandles[handleIndexB], Simulation.Bodies).Pose.Position);
                        Simulation.Solver.Add(bodyHandles[handleIndexA], bodyHandles[handleIndexB], new DistanceLimit(default, default, .01f, maximumDistance, springSettings));
                        return true;
                    }
                    const int constraintsPerBody = 1;
                    for (int i = 0; i < bodyHandles.Length - 1; ++i)
                    {
                        //Note that you could also create constraints which span even more links. For example, connect i and i+1, i+2, i+4, i+8 and i+16 rather than just the nearest bodies.
                        //That would make it behave a bit more like the previous cheat constraint, but it can be useful.
                        for (int j = 1; j <= constraintsPerBody; ++j)
                        {
                            if (!TryCreateConstraint(i, i + j))
                                break;
                        }
                    }

                    var wreckingBallConnectionOffset = horizontalOffset + new Vector3(0, bigWreckingBall.Radius, 0);
                    var ropeConnectionToBall = wreckingBallBody.Pose.Position + wreckingBallConnectionOffset;
                    for (int i = 1; i <= constraintsPerBody; ++i)
                    {
                        var targetBodyHandleIndex = bodyHandles.Length - i;
                        if (targetBodyHandleIndex < 0)
                            break;
                        var maximumDistance = Vector3.Distance(
                            new BodyReference(bodyHandles[targetBodyHandleIndex], Simulation.Bodies).Pose.Position,
                            ropeConnectionToBall);
                        Simulation.Solver.Add(bodyHandles[targetBodyHandleIndex], wreckingBallBodyHandle, new DistanceLimit(default, wreckingBallConnectionOffset, 0.01f, maximumDistance, springSettings));
                    }

                }
            }


            Simulation.Statics.Add(new StaticDescription(new Vector3(0, 0, 0), new CollidableDescription(Simulation.Shapes.Add(new Box(200, 1, 200)), 0.1f)));
            //Simulation.Statics.Add(new StaticDescription(
            //    new Vector3(100, 70, 0), BepuUtilities.QuaternionEx.CreateFromAxisAngle(new Vector3(1, 0, 0), MathF.PI * 0.5f),
            //    new CollidableDescription(Simulation.Shapes.Add(new Capsule(8, 64)), 0.1f)));

        }

    }
}
