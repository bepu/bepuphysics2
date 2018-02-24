using BepuUtilities;
using BepuUtilities.Memory;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using System;
using System.Runtime.CompilerServices;

namespace Demos
{
    public unsafe struct TestCallbacks : INarrowPhaseCallbacks
    {
        public void Initialize(Simulation simulation)
        {
        }

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
        public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, ContactManifold* manifold, out PairMaterialProperties pairMaterial)
        {
            pairMaterial.FrictionCoefficient = 1;
            pairMaterial.MaximumRecoveryVelocity = 2f;// float.MaxValue;
            pairMaterial.SpringSettings.NaturalFrequency = MathHelper.Pi * 60;
            pairMaterial.SpringSettings.DampingRatio = 1f;
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ContactManifold* manifold)
        {
            return true;
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Flush(IThreadDispatcher threadDispatcher)
        {
        }

        public void Dispose()
        {
        }
    }

}
