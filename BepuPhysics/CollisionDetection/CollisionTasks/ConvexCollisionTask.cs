﻿using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Memory;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using static BepuUtilities.GatherScatter;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public interface IPairTester<TShapeWideA, TShapeWideB, TManifoldWideType>
    {
        /// <summary>
        /// Gets the nubmer of pairs which would ideally be gathered together before executing a wide test.
        /// </summary>
        static abstract int BatchSize { get; }

        //Note that, while the interface requires all three of these implementations, concrete implementers will only ever have one defined or called.
        //Including the other unused functions is just here to simplify its use in the batch execution loop.
        static abstract void Test(ref TShapeWideA a, ref TShapeWideB b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount, out TManifoldWideType manifold);
        static abstract void Test(ref TShapeWideA a, ref TShapeWideB b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out TManifoldWideType manifold);
        static abstract void Test(ref TShapeWideA a, ref TShapeWideB b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out TManifoldWideType manifold);
    }

    public interface IContactManifoldWide
    {
        void ApplyFlipMask(ref Vector3Wide offsetB, in Vector<int> flipMask);
        void ReadFirst(in Vector3Wide offsetB, ref ConvexContactManifold target);
    }

    public class ConvexCollisionTask<TShapeA, TShapeWideA, TShapeB, TShapeWideB, TPair, TPairWide, TManifoldWide, TPairTester> : CollisionTask
            where TShapeA : unmanaged, IShape where TShapeB : unmanaged, IShape
            where TShapeWideA : unmanaged, IShapeWide<TShapeA> where TShapeWideB : unmanaged, IShapeWide<TShapeB>
            where TPair : unmanaged, ICollisionPair<TPair>
            where TPairWide : unmanaged, ICollisionPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB, TPair, TPairWide>
            where TPairTester : struct, IPairTester<TShapeWideA, TShapeWideB, TManifoldWide>
            where TManifoldWide : unmanaged, IContactManifoldWide
    {
        public ConvexCollisionTask()
        {
            BatchSize = TPairTester.BatchSize;
            ShapeTypeIndexA = TShapeA.TypeId;
            ShapeTypeIndexB = TShapeB.TypeId;
            PairType = TPair.PairType;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override unsafe void ExecuteBatch<TCallbacks>(ref UntypedList batch, ref CollisionBatcher<TCallbacks> batcher)
        {
            ref var start = ref Unsafe.As<byte, TPair>(ref batch.Buffer[0]);
            //With any luck, the compiler will eventually get rid of these unnecessary zero inits. 
            //Might be able to get rid of manifoldWide and defaultPairTester with some megahacks, but it comes with significant forward danger and questionable benefit.
            var pairWide = default(TPairWide);
            ref var aWide = ref TPairWide.GetShapeA(ref pairWide);
            ref var bWide = ref TPairWide.GetShapeB(ref pairWide);
            if (aWide.InternalAllocationSize > 0)
            {
                var memory = stackalloc byte[aWide.InternalAllocationSize];
                aWide.Initialize(new Buffer<byte>(memory, aWide.InternalAllocationSize));
            }
            if (bWide.InternalAllocationSize > 0)
            {
                var memory = stackalloc byte[bWide.InternalAllocationSize];
                bWide.Initialize(new Buffer<byte>(memory, bWide.InternalAllocationSize));
            }
            TManifoldWide manifoldWide;
            var manifold = default(ConvexContactManifold);

            for (int i = 0; i < batch.Count; i += Vector<float>.Count)
            {
                ref var bundleStart = ref Unsafe.Add(ref start, i);
                int countInBundle = batch.Count - i;
                if (countInBundle > Vector<float>.Count)
                    countInBundle = Vector<float>.Count;

                //TODO: If we have gather intrinsics, they might be a win.
                for (int j = 0; j < countInBundle; ++j)
                {
                    pairWide.WriteSlot(j, Unsafe.Add(ref bundleStart, j));
                }

                if (TPairWide.OrientationCount == 2)
                {
                    TPairTester.Test(
                        ref aWide,
                        ref bWide,
                        ref TPairWide.GetSpeculativeMargin(ref pairWide),
                        ref TPairWide.GetOffsetB(ref pairWide),
                        ref TPairWide.GetOrientationA(ref pairWide),
                        ref TPairWide.GetOrientationB(ref pairWide),
                        countInBundle,
                        out manifoldWide);
                }
                else if (TPairWide.OrientationCount == 1)
                {
                    //Note that, in the event that there is only one orientation, it belongs to the second shape.
                    //The only shape that doesn't need orientation is a sphere, and it will be in slot A by convention.
                    Debug.Assert(typeof(TShapeWideA) == typeof(SphereWide));
                    TPairTester.Test(
                        ref aWide,
                        ref bWide,
                        ref TPairWide.GetSpeculativeMargin(ref pairWide),
                        ref TPairWide.GetOffsetB(ref pairWide),
                        ref TPairWide.GetOrientationB(ref pairWide),
                        countInBundle,
                        out manifoldWide);
                }
                else
                {
                    Debug.Assert(TPairWide.OrientationCount == 0);
                    Debug.Assert(typeof(TShapeWideA) == typeof(SphereWide) && typeof(TShapeWideB) == typeof(SphereWide), "No orientation implies a special case involving two spheres.");
                    //Really, this could be made into a direct special case, but eh.
                    TPairTester.Test(
                        ref aWide,
                        ref bWide,
                        ref TPairWide.GetSpeculativeMargin(ref pairWide),
                        ref TPairWide.GetOffsetB(ref pairWide),
                        countInBundle,
                        out manifoldWide);
                }

                //Flip back any contacts associated with pairs which had to be flipped for shape order.
                if (TPairWide.HasFlipMask)
                {
                    manifoldWide.ApplyFlipMask(ref TPairWide.GetOffsetB(ref pairWide), TPairWide.GetFlipMask(ref pairWide));
                }

                for (int j = 0; j < countInBundle; ++j)
                {
                    ref var manifoldSource = ref GetOffsetInstance(ref manifoldWide, j);
                    ref var offsetSource = ref GetOffsetInstance(ref TPairWide.GetOffsetB(ref pairWide), j);
                    manifoldSource.ReadFirst(offsetSource, ref manifold);
                    ref var pair = ref Unsafe.Add(ref bundleStart, j);
                    batcher.ProcessConvexResult(ref manifold, ref TPair.GetContinuation(ref pair));
                }
            }

        }

    }
}
