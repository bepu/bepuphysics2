using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
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
    public unsafe struct IndexReportingNarrowPhaseCallbacks : INarrowPhaseCallbacks
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b)
        {
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
        {
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void ConfigureMaterial(out PairMaterialProperties pairMaterial)
        {
            pairMaterial.FrictionCoefficient = 1f;
            pairMaterial.MaximumRecoveryVelocity = 2f;
            pairMaterial.SpringSettings = new SpringSettings(30, 1);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, NonconvexContactManifold* manifold, out PairMaterialProperties pairMaterial)
        {
            if (manifold->Count > 0)
                Console.WriteLine($"NONCONVEX PAIR: {pair.A} versus {pair.B}");
            ConfigureMaterial(out pairMaterial);
            return true;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, ConvexContactManifold* manifold, out PairMaterialProperties pairMaterial)
        {
            if (manifold->Count > 0)
                Console.WriteLine($"CONVEX PAIR: {pair.A} versus {pair.B}");
            ConfigureMaterial(out pairMaterial);
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ConvexContactManifold* manifold)
        {
            if (manifold->Count > 0)
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

            Simulation = Simulation.Create(BufferPool, new IndexReportingNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, 0f, 0)));

            var builder = new CompoundBuilder(BufferPool, Simulation.Shapes, 4);
            builder.Add(new Sphere(0.5f), new RigidPose(new Vector3(-1, 0, 0)), 1);
            builder.Add(new Capsule(0.5f, 1f), new RigidPose(new Vector3(0, 0, 0)), 1);
            builder.Add(new Box(1f, 1f, 1f), new RigidPose(new Vector3(1, 0, 0)), 1);
            builder.BuildDynamicCompound(out var children, out var inertia, out var center);

            var compoundCollidable = new CollidableDescription(Simulation.Shapes.Add(new Compound(children)), 0.1f);

            Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(0, 2, 0), inertia, compoundCollidable, new BodyActivityDescription(0.01f)));
            Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(0, 4, 0), inertia, compoundCollidable, new BodyActivityDescription(0.01f)));

            Simulation.Statics.Add(new StaticDescription(new Vector3(), new CollidableDescription(Simulation.Shapes.Add(new Box(100, 1, 100)), 0.1f)));
        }
    }
}
