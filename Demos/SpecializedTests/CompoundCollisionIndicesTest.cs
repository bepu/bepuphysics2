﻿using System;
using System.Numerics;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using BepuPhysics;
using BepuPhysics.Collidables;
using System.Runtime.CompilerServices;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;

namespace Demos.SpecializedTests
{
    public struct IndexReportingNarrowPhaseCallbacks : INarrowPhaseCallbacks
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin)
        {
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
        {
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial) where TManifold : unmanaged, IContactManifold<TManifold>
        {
            if (manifold.Count > 0)
            {
                if (manifold.Convex)
                {
                    Console.WriteLine($"CONVEX PAIR: {pair.A} versus {pair.B}");
                }
                else
                {
                    Console.WriteLine($"NONCONVEX PAIR: {pair.A} versus {pair.B}");
                }
            }
            pairMaterial.FrictionCoefficient = 1f;
            pairMaterial.MaximumRecoveryVelocity = 2f;
            pairMaterial.SpringSettings = new SpringSettings(30, 1);
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold)
        {
            if (manifold.Count > 0)
                Console.WriteLine($"SUBPAIR: {pair.A} child {childIndexA} versus {pair.B} child {childIndexB}");
            return true;
        }

        public void Initialize(Simulation simulation)
        {
        }

        public void Dispose()
        {
        }

    }

    public class CompoundCollisionIndicesTest : Demo
    {
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 4, -6);
            camera.Yaw = MathHelper.Pi;

            Simulation = Simulation.Create(BufferPool, new IndexReportingNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, 0f, 0)), new SolveDescription(8, 1));

            var builder = new CompoundBuilder(BufferPool, Simulation.Shapes, 4);
            builder.Add(new Sphere(0.5f), new Vector3(-1, 0, 0), 1);
            builder.Add(new Capsule(0.5f, 1f), new Vector3(0, 0, 0), 1);
            builder.Add(new Box(1f, 1f, 1f), new Vector3(1, 0, 0), 1);
            builder.BuildDynamicCompound(out var children, out var inertia, out var center);

            var compoundShapeIndex = Simulation.Shapes.Add(new Compound(children));

            Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(0, 2, 0), inertia, compoundShapeIndex, 0.01f));
            Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(0, 4, 0), inertia, compoundShapeIndex, 0.01f));

            Simulation.Statics.Add(new StaticDescription(new Vector3(), Simulation.Shapes.Add(new Box(100, 1, 100))));
        }
    }
}
