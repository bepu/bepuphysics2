using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using static BepuUtilities.GatherScatter;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public interface IPairTester<TShapeWideA, TShapeWideB, TManifoldWideType>
    {
        /// <summary>
        /// Gets the nubmer of pairs which would ideally be gathered together before executing a wide test.
        /// </summary>
        int BatchSize { get; }

        //Note that, while the interface requires all three of these implementations, concrete implementers will only ever have one defined or called.
        //Including the other unused functions is just here to simplify its use in the batch execution loop.
        void Test(ref TShapeWideA a, ref TShapeWideB b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, out TManifoldWideType manifold);
        void Test(ref TShapeWideA a, ref TShapeWideB b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, out TManifoldWideType manifold);
        void Test(ref TShapeWideA a, ref TShapeWideB b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, out TManifoldWideType manifold);
    }

    public interface IContactManifoldWide
    {
        void ApplyFlipMask(ref Vector3Wide offsetB, in Vector<int> flipMask);
        void ReadFirst(in Vector3Wide offsetB, ref ConvexContactManifold target);
    }

    public class ConvexCollisionTask<TShapeA, TShapeWideA, TShapeB, TShapeWideB, TPair, TPairWide, TManifoldWide, TPairTester> : CollisionTask
            where TShapeA : struct, IShape where TShapeB : struct, IShape
            where TShapeWideA : struct, IShapeWide<TShapeA> where TShapeWideB : struct, IShapeWide<TShapeB>
            where TPair : struct, ICollisionPair<TPair>
            where TPairWide : struct, ICollisionPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB, TPair, TPairWide>
            where TPairTester : struct, IPairTester<TShapeWideA, TShapeWideB, TManifoldWide>
            where TManifoldWide : IContactManifoldWide
    {
        public ConvexCollisionTask()
        {
            BatchSize = default(TPairTester).BatchSize;
            ShapeTypeIndexA = default(TShapeA).TypeId;
            ShapeTypeIndexB = default(TShapeB).TypeId;
            PairType = default(TPair).PairType;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override unsafe void ExecuteBatch<TCallbacks>(ref UntypedList batch, ref CollisionBatcher<TCallbacks> batcher)
        {
            ref var start = ref Unsafe.As<byte, TPair>(ref batch.Buffer[0]);
            //With any luck, the compiler will eventually get rid of these unnecessary zero inits. 
            //Might be able to get rid of manifoldWide and defaultPairTester with some megahacks, but it comes with significant forward danger and questionable benefit.
            var pairWide = default(TPairWide);
            var manifoldWide = default(TManifoldWide);
            var defaultPairTester = default(TPairTester);
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
                    //Reposition the wide reference so that the first lane of the alias matches the target lane in the underlying memory.
                    ref var offsetFloatStart = ref Unsafe.Add(ref Unsafe.As<TPairWide, float>(ref pairWide), j);
                    ref var target = ref Unsafe.As<float, TPairWide>(ref offsetFloatStart);
                    target.WriteFirst(ref Unsafe.Add(ref bundleStart, j));
                }

                if (pairWide.OrientationCount == 2)
                {
                    defaultPairTester.Test(
                        ref pairWide.GetShapeA(ref pairWide),
                        ref pairWide.GetShapeB(ref pairWide),
                        ref pairWide.GetSpeculativeMargin(ref pairWide),
                        ref pairWide.GetOffsetB(ref pairWide),
                        ref pairWide.GetOrientationA(ref pairWide),
                        ref pairWide.GetOrientationB(ref pairWide),
                        out manifoldWide);
                }
                else if (pairWide.OrientationCount == 1)
                {
                    //Note that, in the event that there is only one orientation, it belongs to the second shape.
                    //The only shape that doesn't need orientation is a sphere, and it will be in slot A by convention.
                    Debug.Assert(typeof(TShapeWideA) == typeof(SphereWide));
                    defaultPairTester.Test(
                        ref pairWide.GetShapeA(ref pairWide),
                        ref pairWide.GetShapeB(ref pairWide),
                        ref pairWide.GetSpeculativeMargin(ref pairWide),
                        ref pairWide.GetOffsetB(ref pairWide),
                        ref pairWide.GetOrientationB(ref pairWide),
                        out manifoldWide);
                }
                else
                {
                    Debug.Assert(pairWide.OrientationCount == 0);
                    Debug.Assert(typeof(TShapeWideA) == typeof(SphereWide) && typeof(TShapeWideB) == typeof(SphereWide), "No orientation implies a special case involving two spheres.");
                    //Really, this could be made into a direct special case, but eh.
                    defaultPairTester.Test(
                        ref pairWide.GetShapeA(ref pairWide),
                        ref pairWide.GetShapeB(ref pairWide),
                        ref pairWide.GetSpeculativeMargin(ref pairWide),
                        ref pairWide.GetOffsetB(ref pairWide),
                        out manifoldWide);
                }

                //Flip back any contacts associated with pairs which had to be flipped for shape order.
                if (pairWide.HasFlipMask)
                {
                    manifoldWide.ApplyFlipMask(ref pairWide.GetOffsetB(ref pairWide), pairWide.GetFlipMask(ref pairWide));
                }

                for (int j = 0; j < countInBundle; ++j)
                {
                    //TODO: You could just use the vector indexer here. Far more likely to work in the long run; use it if the performance is comparable.
                    ref var manifoldAsFloat = ref Unsafe.Add(ref Unsafe.As<TManifoldWide, float>(ref manifoldWide), j);
                    ref var manifoldSource = ref Unsafe.As<float, TManifoldWide>(ref manifoldAsFloat);
                    ref var offsetAsFloat = ref Unsafe.Add(ref Unsafe.As<Vector3Wide, float>(ref pairWide.GetOffsetB(ref pairWide)), j);
                    ref var offsetSource = ref Unsafe.As<float, Vector3Wide>(ref offsetAsFloat);
                    manifoldSource.ReadFirst(offsetSource, ref manifold);
                    ref var pair = ref Unsafe.Add(ref bundleStart, j);
                    batcher.ProcessConvexResult(&manifold, ref pair.GetContinuation(ref pair));
                }
            }

        }

    }
}
