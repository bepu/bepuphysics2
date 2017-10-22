using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace Demos.SpecializedTests
{
    public static class CacheBlaster
    {
        const int byteCount = (1 << 24); //16.7MB is bigger than most desktop last level caches. You'll want to pick something higher if you're running this on some ginormo xeon.
        const int intCount = byteCount / 4;
        static int vectorCount = intCount / Vector<int>.Count;
        static int vectorMask = vectorCount - 1;
        static int[] readblob = new int[intCount];
        static int[] writeblob = new int[intCount];

        /// <summary>
        /// Attempts to evict most or all of the cache levels to simulate a cold start.
        /// Doesn't do a whole lot for simulations so large that they significantly exceed the cache size.
        /// </summary>
        [MethodImpl(MethodImplOptions.NoOptimization)]
        public static void Blast()
        {
            //We don't have a guarantee that the processor is using pure LRU replacement. Some modern processors are a little trickier.
            //Scrambling the accesses should make it harder for the CPU to keep stuff cached.
            const int vectorsPerJob = 32;
            int intsPerJob = vectorsPerJob * Vector<int>.Count;
            Parallel.For(0, intCount / intsPerJob, jobIndex =>
            {
                var baseIndex = jobIndex * intsPerJob;
                ref Vector<int> read = ref Unsafe.As<int, Vector<int>>(ref readblob[0]);
                ref Vector<int> write = ref Unsafe.As<int, Vector<int>>(ref writeblob[baseIndex]);

                for (int i = 0; i < vectorsPerJob; ++i)
                {
                    Unsafe.Add(ref write, i) = Unsafe.Add(ref read, (i * 104395303) & vectorMask);
                }
            });
        }
    }
}