using BepuUtilities;
using BepuUtilities.Memory;
using BepuPhysics.Constraints;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics
{
    public class GatherScatter
    {

        //IMPLEMENTATION NOTES:

        //UNSAFE CASTS FOR VECTOR MEMORY ACCESS
        //Because there is no exposed 'gather' API, and because the vector constructor only takes managed arrays, and because there is no way to set vector indices,
        //we do a gross hack where we manually stuff the memory backing of a bunch of vectors.
        //This logic is coupled with the layout of the EntityVelocities and BodyReferences structs and makes assumptions about the memory layout of the types.
        //This assumption SHOULD hold on all current runtimes, but don't be too surprised if it breaks later.
        //With any luck, it will be later enough that a proper solution exists.

        //'NULL' CONSTRAINT CONNECTIONS
        //Any 'null' connections should simply redirect to a reserved velocities slot containing zeroes.
        //Attempting to include a branch to special case null connections slows it down quite a bit (~20% total with zero null connections).
        //The benefit of not having to read/write data is extremely weak, and often introducing null connections actually slows things down further
        //until ~70% of constraints have null connections (overheads presumably caused by branch misprediction).
        //In any simulation with a nontrivial number of null connections, the reserved velocity slot will tend to end up in cache anyway, so the loads should be cheap.
        //During scatters, there is a risk of false sharing on the reserved slot when running with multiple threads. 
        //However, we should take measures to avoid false sharing for all constraints. More measurement should be done to check the impact later.

        //MULTIBODY CONSTRAINTS AND MANUAL INLINING
        //Unfortunately, as of this writing, there still seems to be a very small value in manually inlining all involved bodies.
        //This is going to get pretty annoying if there are a variety of different constraint body counts. For runtime-defined N-body constraints,
        //we will likely end up just having a per-body gather, and that will be fine. I'm not gonna make 128 variants of these functions!

        //TODO: A lot of stuff in here has grown stale. Other stuff needs to be moved into more appropriate locations. Revisit this in the future once things are baked a little more.

        /// <summary>
        /// Gets a reference to an element from a vector without using pointers, bypassing direct vector access for codegen reasons.
        /// This appears to produce identical assembly to taking the pointer and applying an offset. You can do slightly better for batched accesses
        /// by taking the pointer or reference only once, though the performance difference is small.
        /// This performs no bounds testing!
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe ref T Get<T>(ref Vector<T> vector, int index) where T : struct
        {
            return ref Unsafe.Add(ref Unsafe.As<Vector<T>, T>(ref vector), index);

            //For comparison, an implementation like this:
            //return ref *((float*)Unsafe.AsPointer(ref vector) + index);
            //doesn't inline (sometimes?). 
            //The good news is that, in addition to inlining and producing decent assembly, the pointerless approach doesn't open the door
            //for GC related problems and the user doesn't need to pin memory.
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
        /// Swaps lanes between two bundles. The bundle type must be a contiguous block of Vector types.
        /// </summary>
        /// <typeparam name="T">Type of the swapped bundles.</typeparam>
        /// <param name="bundleA">Source bundle of the data to copy.</param>
        /// <param name="innerIndexA">Index of the lane within the source bundle.</param>
        /// <param name="bundleB">Target bundle of the data to copy.</param>
        /// <param name="innerIndexB">Index of the lane within the target bundle.</param>
        /// <remarks>
        /// For performance critical operations, a specialized implementation should be used. This uses a loop with stride equal to a Vector.
        /// </remarks>
        public static void SwapLanes<T>(ref T bundleA, int innerIndexA, ref T bundleB, int innerIndexB)
        {
            Debug.Assert((Unsafe.SizeOf<T>() & BundleIndexing.VectorMask) == 0,
                "This implementation doesn't truncate the count under the assumption that the type is evenly divisible by the bundle size." +
                "If you later use SwapLanes with a type that breaks this assumption, introduce a truncation as in CopyLanes.");
            var sizeInInts = Unsafe.SizeOf<T>() >> 2;
            ref var aBase = ref Unsafe.Add(ref Unsafe.As<T, int>(ref bundleA), innerIndexA);
            ref var bBase = ref Unsafe.Add(ref Unsafe.As<T, int>(ref bundleB), innerIndexB);
            for (int i = 0; i < sizeInInts; i += Vector<int>.Count)
            {
                var oldA = Unsafe.Add(ref aBase, i);
                ref var b = ref Unsafe.Add(ref bBase, i);
                Unsafe.Add(ref aBase, i) = b;
                b = oldA;
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
            //Note the truncation. This is used on some types that aren't evenly divisible.
            //This should be folded into a single constant by the jit.
            var sizeInElements = (Unsafe.SizeOf<TOuter>() / (Vector<TVector>.Count * Unsafe.SizeOf<TVector>())) * Unsafe.SizeOf<TVector>();
            ref var laneBase = ref Unsafe.Add(ref Unsafe.As<TOuter, TVector>(ref bundle), innerIndex);
            for (int i = 0; i < sizeInElements; i += Vector<int>.Count)
            {
                Unsafe.Add(ref laneBase, i) = default(TVector);
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
        /// <param name="count">Number of elements in the lane to clear.</param>
        /// <remarks>
        /// For performance critical operations, a specialized implementation should be used. This uses a loop with stride equal to a Vector.
        /// </remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ClearLane<TOuter, TVector>(ref TOuter bundle, int innerIndex, int count) where TVector : struct
        {
            ref var laneBase = ref Unsafe.Add(ref Unsafe.As<TOuter, TVector>(ref bundle), innerIndex);
            for (int i = 0; i < count; ++i)
            {
                Unsafe.Add(ref laneBase, i * Vector<TVector>.Count) = default(TVector);
            }
        }

        //TODO: Given that this is body-specialized, it seems likely we should move this into the Bodies alongside similar stuff.

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void GatherVelocities(ref Buffer<BodyVelocity> velocities, ref TwoBodyReferences references, int count, out BodyVelocities velocitiesA, out BodyVelocities velocitiesB)
        {
            ref var targetLinearAX = ref Unsafe.As<Vector<float>, float>(ref velocitiesA.LinearVelocity.X);
            ref var targetLinearBX = ref Unsafe.As<Vector<float>, float>(ref velocitiesB.LinearVelocity.X);

            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);

            for (int i = 0; i < count; ++i)
            {
                ref var indexA = ref Unsafe.Add(ref baseIndexA, i);
                {
                    ref var sourceVelocities = ref velocities[indexA];
                    ref var linearX = ref Unsafe.Add(ref targetLinearAX, i);
                    linearX = sourceVelocities.Linear.X;
                    Unsafe.Add(ref linearX, Vector<float>.Count) = sourceVelocities.Linear.Y;
                    Unsafe.Add(ref linearX, 2 * Vector<float>.Count) = sourceVelocities.Linear.Z;
                    Unsafe.Add(ref linearX, 3 * Vector<float>.Count) = sourceVelocities.Angular.X;
                    Unsafe.Add(ref linearX, 4 * Vector<float>.Count) = sourceVelocities.Angular.Y;
                    Unsafe.Add(ref linearX, 5 * Vector<float>.Count) = sourceVelocities.Angular.Z;
                }

                {
                    ref var indexB = ref Unsafe.Add(ref indexA, Vector<float>.Count);
                    ref var sourceVelocities = ref velocities[indexB];
                    ref var linearX = ref Unsafe.Add(ref targetLinearBX, i);
                    linearX = sourceVelocities.Linear.X;
                    Unsafe.Add(ref linearX, Vector<float>.Count) = sourceVelocities.Linear.Y;
                    Unsafe.Add(ref linearX, 2 * Vector<float>.Count) = sourceVelocities.Linear.Z;
                    Unsafe.Add(ref linearX, 3 * Vector<float>.Count) = sourceVelocities.Angular.X;
                    Unsafe.Add(ref linearX, 4 * Vector<float>.Count) = sourceVelocities.Angular.Y;
                    Unsafe.Add(ref linearX, 5 * Vector<float>.Count) = sourceVelocities.Angular.Z;
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void ScatterVelocities(ref Buffer<BodyVelocity> velocities, ref TwoBodyReferences references, int count, ref BodyVelocities velocitiesA, ref BodyVelocities velocitiesB)
        {
            ref var baseTargetIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
            ref var baseSourceLinearAX = ref Unsafe.As<Vector<float>, float>(ref velocitiesA.LinearVelocity.X);

            ref var baseTargetIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);
            ref var baseSourceLinearBX = ref Unsafe.As<Vector<float>, float>(ref velocitiesB.LinearVelocity.X);

            for (int i = 0; i < count; ++i)
            {
                //We'll use the memory layout of the BodyVelocities struct. 
                //Grab the pointer to the row within the velocities bundle, and use a stride of Vector<float>.Count to reach the next velocity entry.
                ref var sourceLinearAX = ref Unsafe.Add(ref baseSourceLinearAX, i);
                var indexA = Unsafe.Add(ref baseTargetIndexA, i);
                ref var targetA = ref velocities[indexA];
                //TODO: There are some codegen variants we could try here. Field/unsafe access likely not any better, but given the frequency of this path's use, it might be worth checking.
                targetA.Linear = new Vector3(
                    sourceLinearAX,
                    Unsafe.Add(ref sourceLinearAX, Vector<float>.Count),
                    Unsafe.Add(ref sourceLinearAX, 2 * Vector<float>.Count));
                targetA.Angular = new Vector3(
                    Unsafe.Add(ref sourceLinearAX, 3 * Vector<float>.Count),
                    Unsafe.Add(ref sourceLinearAX, 4 * Vector<float>.Count),
                    Unsafe.Add(ref sourceLinearAX, 5 * Vector<float>.Count));

                ref var sourceLinearBX = ref Unsafe.Add(ref baseSourceLinearBX, i);
                var indexB = Unsafe.Add(ref baseTargetIndexB, i);
                ref var targetB = ref velocities[indexB];
                targetB.Linear = new Vector3(
                    sourceLinearBX,
                    Unsafe.Add(ref sourceLinearBX, Vector<float>.Count),
                    Unsafe.Add(ref sourceLinearBX, 2 * Vector<float>.Count));
                targetB.Angular = new Vector3(
                    Unsafe.Add(ref sourceLinearBX, 3 * Vector<float>.Count),
                    Unsafe.Add(ref sourceLinearBX, 4 * Vector<float>.Count),
                    Unsafe.Add(ref sourceLinearBX, 5 * Vector<float>.Count));

                //Note that no attempt is made to avoid writing to kinematic or null velocities.
                //Loading the necessary data for the condition and branching to avoid the write takes longer than just writing it.
                //No constraint should actually result in a change to the velocity of a kinematic or null body since they have infinite inertia.

                //TODO: CHECK THE ABOVE ASSUMPTION. With more recent changes, it may be that doing a test could be worth it. Most likely it would require the body indices
                //to directly contain a flag regarding the kinematic-ness of the body... Loading more data to avoid writing data is kinda silly.
                //Doing such a thing would require keeping the body references list in sync with any changes to the body's kinematic state. Not impossible- such transition management
                //is already required for some systems. Still not wonderful.
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void GatherVelocities(ref Buffer<BodyVelocity> velocities, ref Vector<int> references, int count, out BodyVelocities velocitiesA)
        {
            ref var targetLinearAX = ref Unsafe.As<Vector<float>, float>(ref velocitiesA.LinearVelocity.X);

            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references);

            for (int i = 0; i < count; ++i)
            {
                ref var indexA = ref Unsafe.Add(ref baseIndexA, i);
                {
                    ref var sourceVelocities = ref velocities[indexA];
                    ref var linearX = ref Unsafe.Add(ref targetLinearAX, i);
                    linearX = sourceVelocities.Linear.X;
                    Unsafe.Add(ref linearX, Vector<float>.Count) = sourceVelocities.Linear.Y;
                    Unsafe.Add(ref linearX, 2 * Vector<float>.Count) = sourceVelocities.Linear.Z;
                    Unsafe.Add(ref linearX, 3 * Vector<float>.Count) = sourceVelocities.Angular.X;
                    Unsafe.Add(ref linearX, 4 * Vector<float>.Count) = sourceVelocities.Angular.Y;
                    Unsafe.Add(ref linearX, 5 * Vector<float>.Count) = sourceVelocities.Angular.Z;
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void ScatterVelocities(ref Buffer<BodyVelocity> velocities, ref Vector<int> references, int count, ref BodyVelocities velocitiesA)
        {
            ref var baseTargetIndexA = ref Unsafe.As<Vector<int>, int>(ref references);
            ref var baseSourceLinearAX = ref Unsafe.As<Vector<float>, float>(ref velocitiesA.LinearVelocity.X);            

            for (int i = 0; i < count; ++i)
            {
                //We'll use the memory layout of the BodyVelocities struct. 
                //Grab the pointer to the row within the velocities bundle, and use a stride of Vector<float>.Count to reach the next velocity entry.
                ref var sourceLinearAX = ref Unsafe.Add(ref baseSourceLinearAX, i);
                var indexA = Unsafe.Add(ref baseTargetIndexA, i);
                ref var targetA = ref velocities[indexA];
                //TODO: There are some codegen variants we could try here. Field/unsafe access likely not any better, but given the frequency of this path's use, it might be worth checking.
                targetA.Linear = new Vector3(
                    sourceLinearAX,
                    Unsafe.Add(ref sourceLinearAX, Vector<float>.Count),
                    Unsafe.Add(ref sourceLinearAX, 2 * Vector<float>.Count));
                targetA.Angular = new Vector3(
                    Unsafe.Add(ref sourceLinearAX, 3 * Vector<float>.Count),
                    Unsafe.Add(ref sourceLinearAX, 4 * Vector<float>.Count),
                    Unsafe.Add(ref sourceLinearAX, 5 * Vector<float>.Count));

                //Note that no attempt is made to avoid writing to kinematic or null velocities.
                //Loading the necessary data for the condition and branching to avoid the write takes longer than just writing it.
                //No constraint should actually result in a change to the velocity of a kinematic or null body since they have infinite inertia.

                //TODO: CHECK THE ABOVE ASSUMPTION. With more recent changes, it may be that doing a test could be worth it. Most likely it would require the body indices
                //to directly contain a flag regarding the kinematic-ness of the body... Loading more data to avoid writing data is kinda silly.
                //Doing such a thing would require keeping the body references list in sync with any changes to the body's kinematic state. Not impossible- such transition management
                //is already required for some systems. Still not wonderful.
            }
        }

        //AOSOA->AOS
        /// <summary>
        /// Gets a lane of a container of vectors, assuming that the vectors are contiguous.
        /// </summary>
        /// <typeparam name="T">Type of the values to copy out of the container lane.</typeparam>
        /// <param name="startVector">First vector of the contiguous vector region to get a lane within.</param>
        /// <param name="innerIndex">Index of the lane within the vectors.</param>
        /// <param name="values">Reference to a contiguous set of values to hold the values copied out of the vector lane slots.</param>
        /// <param name="valueCount">Number of values to iterate over.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetLane<T>(ref Vector<T> startVector, int innerIndex, ref T values, int valueCount) where T : struct
        {
            ref var lane = ref Get(ref startVector, innerIndex);
            values = lane;
            //Even if the jit recognizes the count as constant, it doesn't unroll anything. Could do it manually, like we did in CopyLane.
            for (int vectorIndex = 1; vectorIndex < valueCount; ++vectorIndex)
            {
                //The multiplication should become a shift; the jit recognizes the count as constant.
                Unsafe.Add(ref values, vectorIndex) = Unsafe.Add(ref lane, vectorIndex * Vector<T>.Count);
            }
        }

        //AOS->AOSOA
        /// <summary>
        /// Sets a lane of a container of vectors, assuming that the vectors are contiguous.
        /// </summary>
        /// <typeparam name="T">Type of the values to copy into the container lane.</typeparam>
        /// <param name="startVector">First vector of the contiguous vector region to set a lane within.</param>
        /// <param name="innerIndex">Index of the lane within the vectors.</param>
        /// <param name="values">Reference to a contiguous set of values to copy into the vector lane slots.</param>
        /// <param name="valueCount">Number of values to iterate over.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void SetLane<T>(ref Vector<T> startVector, int innerIndex, ref T values, int valueCount) where T : struct
        {
            ref var lane = ref Get(ref startVector, innerIndex);
            lane = values;
            //Even if the jit recognizes the count as constant, it doesn't unroll anything. Could do it manually, like we did in CopyLane.
            for (int vectorIndex = 1; vectorIndex < valueCount; ++vectorIndex)
            {
                //The multiplication should become a shift; the jit recognizes the count as constant.
                Unsafe.Add(ref lane, vectorIndex * Vector<T>.Count) = Unsafe.Add(ref values, vectorIndex);
            }
        }


    }
}

