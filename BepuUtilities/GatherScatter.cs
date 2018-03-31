using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuUtilities
{
    public static class GatherScatter
    {
        //TODO: A lot of stuff in here has grown stale. Other stuff needs to be moved into more appropriate locations. Revisit this in the future once things are baked a little more.

        /// <summary>
        /// Gets a reference to an element from a vector without using pointers, bypassing direct vector access for codegen reasons. This performs no bounds testing!
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe ref T Get<T>(ref Vector<T> vector, int index) where T : struct
        {
            //TODO: This is a very compiler specific implementation which should be revisited as time goes on. Good chance it will become unnecessary, suboptimal, or counterproductive.
            return ref Unsafe.Add(ref Unsafe.As<Vector<T>, T>(ref vector), index);
        }

        /// <summary>
        /// Copies from one bundle lane to another. The bundle must be a contiguous block of Vector types.
        /// </summary>
        /// <typeparam name="T">Type of the copied bundles.</typeparam>
        /// <param name="sourceBundle">Source bundle of the data to copy.</param>
        /// <param name="sourceInnerIndex">Index of the lane within the source bundle.</param>
        /// <param name="targetBundle">Target bundle of the data to copy.</param>
        /// <param name="targetInnerIndex">Index of the lane within the target bundle.</param>
        /// <remarks>
        /// For performance critical operations, a specialized implementation should be used. This uses a loop with stride equal to a Vector that isn't yet unrolled.
        /// </remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CopyLane<T>(ref T sourceBundle, int sourceInnerIndex, ref T targetBundle, int targetInnerIndex)
        {
            //Note the truncation. Currently used for some types that don't have a size evenly divisible by the Vector<int>.Count * sizeof(int).
            var sizeInInts = (Unsafe.SizeOf<T>() >> 2) & ~BundleIndexing.VectorMask;

            ref var sourceBase = ref Unsafe.Add(ref Unsafe.As<T, int>(ref sourceBundle), sourceInnerIndex);
            ref var targetBase = ref Unsafe.Add(ref Unsafe.As<T, int>(ref targetBundle), targetInnerIndex);

            targetBase = sourceBase;
            //Would be nice if this just auto-unrolled based on the size, considering the jit considers all the relevant bits to be constants!
            //Unfortunately, as of this writing, the jit doesn't.
            //for (int i = Vector<int>.Count; i < sizeInInts; i += Vector<int>.Count)
            //{
            //    Unsafe.Add(ref targetBase, i) = Unsafe.Add(ref sourceBase, i);
            //}

            //To compensate for the compiler, here we go:
            int offset = Vector<int>.Count;
            //8 wide unroll empirically chosen.
            while (offset + Vector<int>.Count * 8 <= sizeInInts)
            {
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset); offset += Vector<int>.Count;
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset); offset += Vector<int>.Count;
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset); offset += Vector<int>.Count;
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset); offset += Vector<int>.Count;
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset); offset += Vector<int>.Count;
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset); offset += Vector<int>.Count;
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset); offset += Vector<int>.Count;
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset); offset += Vector<int>.Count;
            }
            if (offset + 4 * Vector<int>.Count <= sizeInInts)
            {
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset); offset += Vector<int>.Count;
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset); offset += Vector<int>.Count;
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset); offset += Vector<int>.Count;
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset); offset += Vector<int>.Count;
            }
            if (offset + 2 * Vector<int>.Count <= sizeInInts)
            {
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset); offset += Vector<int>.Count;
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset); offset += Vector<int>.Count;
            }
            if (offset + Vector<int>.Count <= sizeInInts)
            {
                Unsafe.Add(ref targetBase, offset) = Unsafe.Add(ref sourceBase, offset);
            }
        }

        
        /// <summary>
        /// Clears a bundle lane using the default value of the specified type. The bundle must be a contiguous block of Vector types, all sharing the same type,
        /// and the first vector must start at the address pointed to by the bundle reference.
        /// </summary>
        /// <typeparam name="TOuter">Type containing one or more Vectors.</typeparam>
        /// <typeparam name="TVector">Type of the vectors to clear.</typeparam>
        /// <param name="bundle">Target bundle to clear a lane in.</param>
        /// <param name="innerIndex">Index of the lane within the target bundle to clear.</param>
        /// <remarks>
        /// For performance critical operations, a specialized implementation should be used. This uses a loop with stride equal to a Vector.
        /// </remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ClearLane<TOuter, TVector>(ref TOuter bundle, int innerIndex) where TVector : struct
        {
            //This should be folded into a single constant by the jit.
            var vectorCount = Unsafe.SizeOf<TOuter>() / Unsafe.SizeOf<Vector<TVector>>();
            ref var laneBase = ref Unsafe.Add(ref Unsafe.As<TOuter, TVector>(ref bundle), innerIndex);
            for (int i = 0; i < vectorCount; ++i)
            {
                Unsafe.Add(ref laneBase, i * Vector<TVector>.Count) = default;
            }
        }

        /// <summary>
        /// Gets a reference to a shifted bundle container such that the first slot of each bundle covers the given inner index of the original bundle reference.
        /// </summary>
        /// <typeparam name="T">Type of the bundle container.</typeparam>
        /// <param name="bundleContainer">Bundle container whose reference acts as the base for the shifted reference.</param>
        /// <param name="innerIndex">Index within the bundle to access with the shifted reference.</param>
        /// <returns>Shifted bundle container reference covering the inner index of the original bundle reference.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref T GetOffsetInstance<T>(ref T bundleContainer, int innerIndex) where T : struct
        {
            return ref Unsafe.As<float, T>(ref Unsafe.Add(ref Unsafe.As<T, float>(ref bundleContainer), innerIndex));
        }

        /// <summary>
        /// Gets a reference to the first element in the vector reference.
        /// </summary>
        /// <typeparam name="T">Type of value held by the vector.</typeparam>
        /// <param name="vector">Vector to pull the first slot value from.</param>
        /// <returns>Reference to the value in the given vector's first slot.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref T GetFirst<T>(ref Vector<T> vector) where T : struct
        {
            return ref Unsafe.As<Vector<T>, T>(ref vector);
        }
    }
}

