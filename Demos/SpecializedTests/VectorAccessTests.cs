using BepuPhysics;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace Demos.SpecializedTests
{
    class VectorAccessTests
    {        
        //The following tests show different methods of accessing an element within a vector.
        //(The fact that it is a vector is basically irrelevant in all of them except for the direct index.)
        //Direct vector indexing introduces bounds tests which the alternate methods avoid, making it slightly slower.
        [MethodImpl(MethodImplOptions.NoInlining)]
        unsafe static float[] TestVectorPointerAccess(int iterationCount)
        {
            var vector = new Vector<float>();
            float[] accumulators = new float[Vector<float>.Count];
            int[] indices = new int[] { 0, 1, 2, 3 };
            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
            {
                var pVector = (float*)Unsafe.AsPointer(ref vector);
                accumulators[0] += pVector[indices[0]];
                accumulators[1] += pVector[indices[1]];
                accumulators[2] += pVector[indices[2]];
                accumulators[3] += pVector[indices[3]];
            }

            return accumulators;
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        unsafe static float[] TestRedundantVectorPointerAccess(int iterationCount)
        {
            var vector = new Vector<float>();
            float[] accumulators = new float[Vector<float>.Count];
            int[] indices = new int[] { 0, 1, 2, 3 };
            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
            {
                //While this is reasonably fast, it's not faster than pointerless approaches, which is nice- pointers are vulnerable to GC.
                accumulators[0] += *((float*)Unsafe.AsPointer(ref vector) + indices[0]);
                accumulators[1] += *((float*)Unsafe.AsPointer(ref vector) + indices[1]);
                accumulators[2] += *((float*)Unsafe.AsPointer(ref vector) + indices[2]);
                accumulators[3] += *((float*)Unsafe.AsPointer(ref vector) + indices[3]);
            }

            return accumulators;
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        unsafe static float[] TestWrappedVectorAccess(int iterationCount)
        {
            var vector = new Vector<float>();
            float[] accumulators = new float[Vector<float>.Count];
            int[] indices = new int[] { 0, 1, 2, 3 };
            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
            {
                accumulators[0] += GatherScatter.Get(ref vector, indices[0]);
                accumulators[1] += GatherScatter.Get(ref vector, indices[1]);
                accumulators[2] += GatherScatter.Get(ref vector, indices[2]);
                accumulators[3] += GatherScatter.Get(ref vector, indices[3]);
            }

            return accumulators;
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        unsafe static float[] TestVectorRefAccess(int iterationCount)
        {
            var vector = new Vector<float>();
            float[] accumulators = new float[Vector<float>.Count];
            int[] indices = new int[] { 0, 1, 2, 3 };
            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
            {
                //codegen note: you lose a lot of inlining and performance if you don't have a ref local.
                //Interestingly, this approach is often marginally faster than the pure pointer approach despite having an extra movss.
                ref var rVector = ref Unsafe.As<Vector<float>, float>(ref vector);
                accumulators[0] += Unsafe.Add(ref rVector, indices[0]);
                accumulators[1] += Unsafe.Add(ref rVector, indices[1]);
                accumulators[2] += Unsafe.Add(ref rVector, indices[2]);
                accumulators[3] += Unsafe.Add(ref rVector, indices[3]);
            }

            return accumulators;
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        static float[] TestVectorAccess(int iterationCount)
        {
            var vector = new Vector<float>();
            float[] accumulators = new float[Vector<float>.Count];
            int[] indices = new int[] { 0, 1, 2, 3 };
            for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
            {
                accumulators[0] += vector[indices[0]];
                accumulators[1] += vector[indices[1]];
                accumulators[2] += vector[indices[2]];
                accumulators[3] += vector[indices[3]];
            }

            return accumulators;
        }

        public static void Test()
        {
            const int iterationCount = 1000000000;
            var vectorAcc = TestVectorAccess(1);
            var vectorPointerAcc = TestVectorPointerAccess(1);
            var vectorRedundantPointerAcc = TestRedundantVectorPointerAccess(1);
            var vectorRefAcc = TestVectorRefAccess(1);
            var vectorWrappedAcc = TestWrappedVectorAccess(1);

            var t0 = Stopwatch.GetTimestamp();
            vectorAcc = TestVectorAccess(iterationCount);
            var t1 = Stopwatch.GetTimestamp();
            vectorPointerAcc = TestVectorPointerAccess(iterationCount);
            var t2 = Stopwatch.GetTimestamp();
            vectorRedundantPointerAcc = TestRedundantVectorPointerAccess(iterationCount);
            var t3 = Stopwatch.GetTimestamp();
            vectorRefAcc = TestVectorRefAccess(iterationCount);
            var t4 = Stopwatch.GetTimestamp();
            vectorWrappedAcc = TestWrappedVectorAccess(iterationCount);
            var t5 = Stopwatch.GetTimestamp();

            var nanosecondScalingFactor = (1e9 / iterationCount) / Stopwatch.Frequency;
            var vectorTime = (t1 - t0) * nanosecondScalingFactor;
            var vectorPointerTime = (t2 - t1) * nanosecondScalingFactor;
            var vectorRedundantPointerTime = (t3 - t2) * nanosecondScalingFactor;
            var vectorRefTime = (t4 - t3) * nanosecondScalingFactor;
            var vectorWrappedTime = (t5 - t4) * nanosecondScalingFactor;

            Console.WriteLine($"Vector time (ns): {vectorTime}, vector pointer time (ns): {vectorPointerTime}, vector redundant pointer time (ns): {vectorRedundantPointerTime}," +
                $" vector ref time (ns): {vectorRefTime},  vector wrapped time (ns): {vectorWrappedTime}");
            Console.WriteLine($"accs: {vectorAcc}{vectorPointerAcc}{vectorRefAcc}{vectorRedundantPointerAcc}{vectorWrappedAcc}");

            Console.WriteLine($"Bounds test: {BoundsTest(2)}");
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        static float BoundsTest(int index)
        {
            var vector = new Vector<float>();
            if (index < 0)
                index = 0;
            if (index >= Vector<float>.Count)
                index = Vector<float>.Count - 1;
            return vector[index];
        }
    }
}
