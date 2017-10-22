using BepuPhysics;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace Demos.SpecializedTests
{
    class LayoutTests
    {
        struct Timer
        {
            long begin;

            public static Timer Start()
            {
                return new Timer { begin = Stopwatch.GetTimestamp() };
            }
            public double Stop()
            {
                return (Stopwatch.GetTimestamp() - begin) / (double)Stopwatch.Frequency;
            }
        }

        [MethodImpl(MethodImplOptions.NoOptimization)]
        static double Time<T>(Action<T> action, Func<int, int, T> contextBuilder, Action<T> verifier, int laneCount, int laneWidth, int iterationCount)
        {
            //Note lack of optimizations; the pre-jit seems to get poofed when optimizations are enabled.
            action(contextBuilder(laneCount, laneWidth));
            long tickSum = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                //Each iteration destroys the cache coherence.
                GC.Collect(int.MaxValue, GCCollectionMode.Forced, true, true);
                var context = contextBuilder(laneCount, laneWidth);
                var start = Stopwatch.GetTimestamp();
                action(context);
                var end = Stopwatch.GetTimestamp();
                tickSum += end - start;
                verifier(context);
            }
            return (double)tickSum / (iterationCount * laneCount * Stopwatch.Frequency);
        }

        //AOS TESTS
        //AOS is massively hamstrung in this test due to the horizontal nature of the operations- it's forced to use scalar operations.
        //But that's actually a realistic representation of the kind of work being done in the engine- rarely do you get to use a bunch of 4-wide operations in sequence 
        //when working on 3 dimensional geometry. And it simply can't take advantage of AVX2+.
        //So it's basically here as a 'whoa that's a lot worse' baseline.
        [StructLayout(LayoutKind.Explicit, Size = 4 * 4)]
        struct AOSData4
        {
            [FieldOffset(0)]
            public float Array;
        }
        [StructLayout(LayoutKind.Explicit, Size = 8 * 4)]
        struct AOSData8
        {
            [FieldOffset(0)]
            public float Array;
        }
        [StructLayout(LayoutKind.Explicit, Size = 16 * 4)]
        struct AOSData16
        {
            [FieldOffset(0)]
            public float Array;
        }
        [StructLayout(LayoutKind.Explicit, Size = 32 * 4)]
        struct AOSData32
        {
            [FieldOffset(0)]
            public float Array;
        }
        [StructLayout(LayoutKind.Explicit, Size = 64 * 4)]
        struct AOSData64
        {
            [FieldOffset(0)]
            public float Array;
        }
        [StructLayout(LayoutKind.Explicit, Size = 128 * 4)]
        struct AOSData128
        {
            [FieldOffset(0)]
            public float Array;
        }
        struct AOSContext<T>
        {
            public int[] AccessIndices;
            public T[] Input;
            public float[] Result;
            public float[] ExpectedResult;
            public int LaneCount;
            public int LaneWidth;
        }
        static AOSContext<T> GetAOSContext<T>(int laneCount, int laneWidth)
        {
            AOSContext<T> context;
            context.Input = new T[laneCount];
            context.Result = new float[laneCount];
            context.ExpectedResult = new float[laneCount];
            context.AccessIndices = new int[laneCount];
            for (int i = 0; i < laneCount; ++i)
            {
                context.AccessIndices[i] = i;
                ref var baseRef = ref Unsafe.As<T, float>(ref context.Input[i]);
                for (int j = 0; j < laneWidth; ++j)
                {
                    Unsafe.Add(ref baseRef, j) = j;
                }
                context.ExpectedResult[i] = (laneWidth - 1) * (laneWidth) / 2;
            }
            context.LaneWidth = Unsafe.SizeOf<T>() >> 2;
            Debug.Assert(laneWidth == context.LaneWidth, "didn't exactly use a super generic implementation here oh well,");
            context.LaneCount = laneCount;
            return context;
        }
        static AOSContext<T> GetScrambledAOSContext<T>(int laneCount, int laneWidth)
        {
            var context = GetAOSContext<T>(laneCount, laneWidth);
            Random random = new Random(5);
            //Scramble the access, but only at the resolution of Vector<float> because our other scrambles don't scramble within the vectors.
            //Scrambling within the vectors would require a gather in the other tests (a relative strength of the pure scalar approach, I suppose).
            //This is a bit of a nonrepresentative test, but it gets the point across reasonably well.
            //We're also being a little unfair because the amount of data being loaded is 4x larger for these ints, but... realistically we just want to compare SOA with AOSOA.
            for (int swapSource = laneCount - Vector<float>.Count; swapSource >= 0; swapSource -= Vector<float>.Count)
            {
                var swapTarget = Vector<float>.Count * random.Next(swapSource / Vector<float>.Count);
                for (int i = 0; i < Vector<float>.Count; ++i)
                {
                    var temp = context.AccessIndices[swapTarget + i];
                    context.AccessIndices[swapTarget + i] = context.AccessIndices[swapSource + i];
                    context.AccessIndices[swapSource + i] = temp;
                }
            }
            return context;
        }
        static void AOS<T>(AOSContext<T> context) where T : struct
        {
            ref var baseRef = ref Unsafe.As<T, float>(ref context.Input[0]);
            ref var baseResultRef = ref context.Result[0];
            for (int accessIndex = 0; accessIndex < context.LaneCount; ++accessIndex)
            {
                var laneIndex = context.AccessIndices[accessIndex];
                ref var laneBaseRef = ref Unsafe.Add(ref baseRef, laneIndex * context.LaneWidth);
                ref var resultRef = ref Unsafe.Add(ref baseResultRef, laneIndex);
                for (int elementIndex = 0; elementIndex < context.LaneWidth; ++elementIndex)
                {
                    resultRef += Unsafe.Add(ref laneBaseRef, elementIndex);
                }
            }
        }
        static void VerifyAOS<T>(AOSContext<T> context)
        {
            for (int i = 0; i < context.LaneCount; ++i)
            {
                if (context.ExpectedResult[i] != context.Result[i])
                    Console.WriteLine("Bad AOS.");
            }
        }

        //SOA TESTS
        struct SOAContext
        {
            public int[] AccessIndices;
            public Vector<float>[][] Input;
            public Vector<float>[] Result;
            public Vector<float>[] ExpectedResult;
            public int VectorLaneCount;

        }
        static SOAContext GetSOAContext(int laneCount, int laneWidth)
        {
            SOAContext context;
            context.VectorLaneCount = laneCount >> BundleIndexing.VectorShift;
            context.Input = new Vector<float>[context.VectorLaneCount][];
            context.Result = new Vector<float>[context.VectorLaneCount];
            context.ExpectedResult = new Vector<float>[context.VectorLaneCount];
            context.AccessIndices = new int[context.VectorLaneCount];
            for (int i = 0; i < context.Input.Length; ++i)
            {
                context.AccessIndices[i] = i;
                context.Input[i] = new Vector<float>[laneWidth];
                for (int columnIndex = 0; columnIndex < laneWidth; ++columnIndex)
                {
                    for (int innerIndex = 0; innerIndex < Vector<float>.Count; ++innerIndex)
                    {
                        GatherScatter.Get(ref context.Input[i][columnIndex], innerIndex) = columnIndex;
                    }
                }
                for (int innerIndex = 0; innerIndex < Vector<float>.Count; ++innerIndex)
                {
                    GatherScatter.Get(ref context.ExpectedResult[i], innerIndex) = (laneWidth - 1) * (laneWidth) / 2;
                }
            }
            return context;
        }
        static SOAContext GetScrambledSOAContext(int laneCount, int laneWidth)
        {
            var context = GetSOAContext(laneCount, laneWidth);
            Random random = new Random(5);
            //Scramble the access, but only at the resolution of Vector<float>.
            for (int swapSource = context.VectorLaneCount - 1; swapSource >= 0; --swapSource)
            {
                var swapTarget = random.Next(swapSource);

                var temp = context.AccessIndices[swapTarget];
                context.AccessIndices[swapTarget] = context.AccessIndices[swapSource];
                context.AccessIndices[swapSource] = temp;

            }
            return context;
        }
        static void SOA(SOAContext context)
        {
            for (int accessIndex = 0; accessIndex < context.VectorLaneCount; ++accessIndex)
            {
                var vectorLaneIndex = context.AccessIndices[accessIndex];
                var vectorLane = context.Input[vectorLaneIndex];
                for (int elementIndex = 0; elementIndex < vectorLane.Length; ++elementIndex)
                {
                    context.Result[vectorLaneIndex] += vectorLane[elementIndex];
                }
            }
        }
        static void VerifySOA(SOAContext context)
        {
            for (int i = 0; i < context.VectorLaneCount; ++i)
            {
                for (int innerIndex = 0; innerIndex < Vector<float>.Count; ++innerIndex)
                {
                    if (GatherScatter.Get(ref context.ExpectedResult[i], innerIndex) != GatherScatter.Get(ref context.Result[i], innerIndex))
                        Console.WriteLine("Bad SOA.");
                }
            }
        }

        //AOSOA TESTS
        [StructLayout(LayoutKind.Explicit, Size = 64)]
        struct AOSOAData64Bytes
        {
            [FieldOffset(0)]
            public Vector<float> Array;
        }
        [StructLayout(LayoutKind.Explicit, Size = 128)]
        struct AOSOAData128Bytes
        {
            [FieldOffset(0)]
            public Vector<float> Array;
        }
        [StructLayout(LayoutKind.Explicit, Size = 256)]
        struct AOSOAData256Bytes
        {
            [FieldOffset(0)]
            public Vector<float> Array;
        }
        [StructLayout(LayoutKind.Explicit, Size = 512)]
        struct AOSOAData512Bytes
        {
            [FieldOffset(0)]
            public Vector<float> Array;
        }
        [StructLayout(LayoutKind.Explicit, Size = 1024)]
        struct AOSOAData1024Bytes
        {
            [FieldOffset(0)]
            public Vector<float> Array;
        }
        [StructLayout(LayoutKind.Explicit, Size = 2048)]
        struct AOSOAData2048Bytes
        {
            [FieldOffset(0)]
            public Vector<float> Array;
        }
        [StructLayout(LayoutKind.Explicit, Size = 4096)]
        struct AOSOAData4096Bytes
        {
            [FieldOffset(0)]
            public Vector<float> Array;
        }
        [StructLayout(LayoutKind.Explicit, Size = 8192)]
        struct AOSOAData8192Bytes
        {
            [FieldOffset(0)]
            public Vector<float> Array;
        }
        struct AOSOAContext<T>
        {
            public int[] AccessIndices;
            public T[] Input;
            public Vector<float>[] Result;
            public Vector<float>[] ExpectedResult;
            public int VectorLaneCount;
            public int LaneWidth;
        }
        static AOSOAContext<T> GetAOSOAContext<T>(int laneCount, int laneWidth)
        {
            AOSOAContext<T> context;
            context.LaneWidth = laneWidth;
            context.VectorLaneCount = laneCount >> BundleIndexing.VectorShift;
            context.Input = new T[context.VectorLaneCount];
            context.Result = new Vector<float>[context.VectorLaneCount];
            context.ExpectedResult = new Vector<float>[context.VectorLaneCount];
            context.AccessIndices = new int[context.VectorLaneCount];

            for (int i = 0; i < context.VectorLaneCount; ++i)
            {
                context.AccessIndices[i] = i;
                ref var baseRef = ref Unsafe.As<T, Vector<float>>(ref context.Input[i]);
                for (int elementIndex = 0; elementIndex < laneWidth; ++elementIndex)
                {
                    for (int innerIndex = 0; innerIndex < Vector<float>.Count; ++innerIndex)
                    {
                        GatherScatter.Get(ref Unsafe.Add(ref baseRef, elementIndex), innerIndex) = elementIndex;
                    }
                }
                for (int innerIndex = 0; innerIndex < Vector<float>.Count; ++innerIndex)
                {
                    GatherScatter.Get(ref context.ExpectedResult[i], innerIndex) = (laneWidth - 1) * (laneWidth) / 2;
                }

            }
            return context;
        }
        static AOSOAContext<T> GetScrambledAOSOAContext<T>(int laneCount, int laneWidth)
        {
            var context = GetAOSOAContext<T>(laneCount, laneWidth);
            Random random = new Random(5);
            //Scramble the access, but only at the resolution of Vector<float>.
            for (int swapSource = context.VectorLaneCount - 1; swapSource >= 0; --swapSource)
            {
                var swapTarget = random.Next(swapSource);

                var temp = context.AccessIndices[swapTarget];
                context.AccessIndices[swapTarget] = context.AccessIndices[swapSource];
                context.AccessIndices[swapSource] = temp;

            }
            return context;
        }
        static void AOSOA<T>(AOSOAContext<T> context) where T : struct
        {
            ref var baseInputRef = ref Unsafe.As<T, Vector<float>>(ref context.Input[0]);
            ref var baseResultRef = ref context.Result[0];
            for (int accessIndex = 0; accessIndex < context.VectorLaneCount; ++accessIndex)
            {
                var vectorLaneIndex = context.AccessIndices[accessIndex];
                ref var laneBaseRef = ref Unsafe.Add(ref baseInputRef, vectorLaneIndex * context.LaneWidth);
                ref var resultRef = ref Unsafe.Add(ref baseResultRef, vectorLaneIndex);
                for (int elementIndex = 0; elementIndex < context.LaneWidth; ++elementIndex)
                {
                    resultRef += Unsafe.Add(ref laneBaseRef, elementIndex);
                }
            }
        }
        static void VerifyAOSOA<T>(AOSOAContext<T> context)
        {
            for (int i = 0; i < context.VectorLaneCount; ++i)
            {
                for (int innerIndex = 0; innerIndex < Vector<float>.Count; ++innerIndex)
                {
                    if (GatherScatter.Get(ref context.ExpectedResult[i], innerIndex) != GatherScatter.Get(ref context.Result[i], innerIndex))
                        Console.WriteLine("Bad AOSOA.");
                }
            }
        }

        public static void Test()
        {
            const double timeScaling = 1e9;
            const int laneCount = 1 << 18;
            const int iterationCount = 10;
            double[] aosTimes = new double[6];
            aosTimes[0] = Time(AOS, GetAOSContext<AOSData4>, VerifyAOS, laneCount, 4, iterationCount);
            aosTimes[1] = Time(AOS, GetAOSContext<AOSData8>, VerifyAOS, laneCount, 8, iterationCount);
            aosTimes[2] = Time(AOS, GetAOSContext<AOSData16>, VerifyAOS, laneCount, 16, iterationCount);
            aosTimes[3] = Time(AOS, GetAOSContext<AOSData32>, VerifyAOS, laneCount, 32, iterationCount);
            aosTimes[4] = Time(AOS, GetAOSContext<AOSData64>, VerifyAOS, laneCount, 64, iterationCount);
            aosTimes[5] = Time(AOS, GetAOSContext<AOSData128>, VerifyAOS, laneCount, 128, iterationCount);

            Console.WriteLine("AOS Times, by lane width:");
            for (int i = 0; i < aosTimes.Length; ++i)
            {
                Console.WriteLine($"{4 << i}: {aosTimes[i] * timeScaling} ns");
            }


            double[] aosScrambledTimes = new double[6];
            aosScrambledTimes[0] = Time(AOS, GetScrambledAOSContext<AOSData4>, VerifyAOS, laneCount, 4, iterationCount);
            aosScrambledTimes[1] = Time(AOS, GetScrambledAOSContext<AOSData8>, VerifyAOS, laneCount, 8, iterationCount);
            aosScrambledTimes[2] = Time(AOS, GetScrambledAOSContext<AOSData16>, VerifyAOS, laneCount, 16, iterationCount);
            aosScrambledTimes[3] = Time(AOS, GetScrambledAOSContext<AOSData32>, VerifyAOS, laneCount, 32, iterationCount);
            aosScrambledTimes[4] = Time(AOS, GetScrambledAOSContext<AOSData64>, VerifyAOS, laneCount, 64, iterationCount);
            aosScrambledTimes[5] = Time(AOS, GetScrambledAOSContext<AOSData128>, VerifyAOS, laneCount, 128, iterationCount);

            Console.WriteLine("AOS Scrambled Times, by lane width:");
            for (int i = 0; i < aosTimes.Length; ++i)
            {
                Console.WriteLine($"{4 << i}: {aosScrambledTimes[i] * timeScaling} ns");
            }

            double[] soaTimes = new double[6];
            soaTimes[0] = Time(SOA, GetSOAContext, VerifySOA, laneCount, 4, iterationCount);
            soaTimes[1] = Time(SOA, GetSOAContext, VerifySOA, laneCount, 8, iterationCount);
            soaTimes[2] = Time(SOA, GetSOAContext, VerifySOA, laneCount, 16, iterationCount);
            soaTimes[3] = Time(SOA, GetSOAContext, VerifySOA, laneCount, 32, iterationCount);
            soaTimes[4] = Time(SOA, GetSOAContext, VerifySOA, laneCount, 64, iterationCount);
            soaTimes[5] = Time(SOA, GetSOAContext, VerifySOA, laneCount, 128, iterationCount);

            Console.WriteLine("SOA Times, by lane width:");
            for (int i = 0; i < aosTimes.Length; ++i)
            {
                Console.WriteLine($"{4 << i}: {soaTimes[i] * timeScaling} ns");
            }

            double[] soaScrambledTimes = new double[6];
            soaScrambledTimes[0] = Time(SOA, GetScrambledSOAContext, VerifySOA, laneCount, 4, iterationCount);
            soaScrambledTimes[1] = Time(SOA, GetScrambledSOAContext, VerifySOA, laneCount, 8, iterationCount);
            soaScrambledTimes[2] = Time(SOA, GetScrambledSOAContext, VerifySOA, laneCount, 16, iterationCount);
            soaScrambledTimes[3] = Time(SOA, GetScrambledSOAContext, VerifySOA, laneCount, 32, iterationCount);
            soaScrambledTimes[4] = Time(SOA, GetScrambledSOAContext, VerifySOA, laneCount, 64, iterationCount);
            soaScrambledTimes[5] = Time(SOA, GetScrambledSOAContext, VerifySOA, laneCount, 128, iterationCount);

            Console.WriteLine("SOA Scrambled Times, by lane width:");
            for (int i = 0; i < aosTimes.Length; ++i)
            {
                Console.WriteLine($"{4 << i}: {soaScrambledTimes[i] * timeScaling} ns");
            }

            double[] aosoaTimes = new double[6];
            //Each AOSOA tile contains laneWidth * Vector<float>.Count * 4 bytes.
            //Since creating variable sized structs at compile time is tricky, just switch.
            switch (Vector<float>.Count)
            {
                case 4:
                    aosoaTimes[0] = Time(AOSOA, GetAOSOAContext<AOSOAData64Bytes>, VerifyAOSOA, laneCount, 4, iterationCount);
                    aosoaTimes[1] = Time(AOSOA, GetAOSOAContext<AOSOAData128Bytes>, VerifyAOSOA, laneCount, 8, iterationCount);
                    aosoaTimes[2] = Time(AOSOA, GetAOSOAContext<AOSOAData256Bytes>, VerifyAOSOA, laneCount, 16, iterationCount);
                    aosoaTimes[3] = Time(AOSOA, GetAOSOAContext<AOSOAData512Bytes>, VerifyAOSOA, laneCount, 32, iterationCount);
                    aosoaTimes[4] = Time(AOSOA, GetAOSOAContext<AOSOAData1024Bytes>, VerifyAOSOA, laneCount, 64, iterationCount);
                    aosoaTimes[5] = Time(AOSOA, GetAOSOAContext<AOSOAData2048Bytes>, VerifyAOSOA, laneCount, 128, iterationCount);
                    break;
                case 8:
                    aosoaTimes[0] = Time(AOSOA, GetAOSOAContext<AOSOAData128Bytes>, VerifyAOSOA, laneCount, 4, iterationCount);
                    aosoaTimes[1] = Time(AOSOA, GetAOSOAContext<AOSOAData256Bytes>, VerifyAOSOA, laneCount, 8, iterationCount);
                    aosoaTimes[2] = Time(AOSOA, GetAOSOAContext<AOSOAData512Bytes>, VerifyAOSOA, laneCount, 16, iterationCount);
                    aosoaTimes[3] = Time(AOSOA, GetAOSOAContext<AOSOAData1024Bytes>, VerifyAOSOA, laneCount, 32, iterationCount);
                    aosoaTimes[4] = Time(AOSOA, GetAOSOAContext<AOSOAData2048Bytes>, VerifyAOSOA, laneCount, 64, iterationCount);
                    aosoaTimes[5] = Time(AOSOA, GetAOSOAContext<AOSOAData4096Bytes>, VerifyAOSOA, laneCount, 128, iterationCount);
                    break;
                case 16:
                    aosoaTimes[0] = Time(AOSOA, GetAOSOAContext<AOSOAData256Bytes>, VerifyAOSOA, laneCount, 4, iterationCount);
                    aosoaTimes[1] = Time(AOSOA, GetAOSOAContext<AOSOAData512Bytes>, VerifyAOSOA, laneCount, 8, iterationCount);
                    aosoaTimes[2] = Time(AOSOA, GetAOSOAContext<AOSOAData1024Bytes>, VerifyAOSOA, laneCount, 16, iterationCount);
                    aosoaTimes[3] = Time(AOSOA, GetAOSOAContext<AOSOAData2048Bytes>, VerifyAOSOA, laneCount, 32, iterationCount);
                    aosoaTimes[4] = Time(AOSOA, GetAOSOAContext<AOSOAData4096Bytes>, VerifyAOSOA, laneCount, 64, iterationCount);
                    aosoaTimes[5] = Time(AOSOA, GetAOSOAContext<AOSOAData8192Bytes>, VerifyAOSOA, laneCount, 128, iterationCount);
                    break;
            }

            Console.WriteLine("AOSOA Times, by lane width:");
            for (int i = 0; i < aosoaTimes.Length; ++i)
            {
                Console.WriteLine($"{4 << i}: {aosoaTimes[i] * timeScaling} ns");
            }

            double[] aosoaScrambledTimes = new double[6];
            //Each AOSOA tile contains laneWidth * Vector<float>.Count * 4 bytes.
            //Since creating variable sized structs at compile time is tricky, just switch.
            switch (Vector<float>.Count)
            {
                case 4:
                    aosoaScrambledTimes[0] = Time(AOSOA, GetScrambledAOSOAContext<AOSOAData64Bytes>, VerifyAOSOA, laneCount, 4, iterationCount);
                    aosoaScrambledTimes[1] = Time(AOSOA, GetScrambledAOSOAContext<AOSOAData128Bytes>, VerifyAOSOA, laneCount, 8, iterationCount);
                    aosoaScrambledTimes[2] = Time(AOSOA, GetScrambledAOSOAContext<AOSOAData256Bytes>, VerifyAOSOA, laneCount, 16, iterationCount);
                    aosoaScrambledTimes[3] = Time(AOSOA, GetScrambledAOSOAContext<AOSOAData512Bytes>, VerifyAOSOA, laneCount, 32, iterationCount);
                    aosoaScrambledTimes[4] = Time(AOSOA, GetScrambledAOSOAContext<AOSOAData1024Bytes>, VerifyAOSOA, laneCount, 64, iterationCount);
                    aosoaScrambledTimes[5] = Time(AOSOA, GetScrambledAOSOAContext<AOSOAData2048Bytes>, VerifyAOSOA, laneCount, 128, iterationCount);
                    break;                              
                case 8:                                 
                    aosoaScrambledTimes[0] = Time(AOSOA, GetScrambledAOSOAContext<AOSOAData128Bytes>, VerifyAOSOA, laneCount, 4, iterationCount);
                    aosoaScrambledTimes[1] = Time(AOSOA, GetScrambledAOSOAContext<AOSOAData256Bytes>, VerifyAOSOA, laneCount, 8, iterationCount);
                    aosoaScrambledTimes[2] = Time(AOSOA, GetScrambledAOSOAContext<AOSOAData512Bytes>, VerifyAOSOA, laneCount, 16, iterationCount);
                    aosoaScrambledTimes[3] = Time(AOSOA, GetScrambledAOSOAContext<AOSOAData1024Bytes>, VerifyAOSOA, laneCount, 32, iterationCount);
                    aosoaScrambledTimes[4] = Time(AOSOA, GetScrambledAOSOAContext<AOSOAData2048Bytes>, VerifyAOSOA, laneCount, 64, iterationCount);
                    aosoaScrambledTimes[5] = Time(AOSOA, GetScrambledAOSOAContext<AOSOAData4096Bytes>, VerifyAOSOA, laneCount, 128, iterationCount);
                    break;                                 
                case 16:                                     
                    aosoaScrambledTimes[0] = Time(AOSOA, GetScrambledAOSOAContext<AOSOAData256Bytes>, VerifyAOSOA, laneCount, 4, iterationCount);
                    aosoaScrambledTimes[1] = Time(AOSOA, GetScrambledAOSOAContext<AOSOAData512Bytes>, VerifyAOSOA, laneCount, 8, iterationCount);
                    aosoaScrambledTimes[2] = Time(AOSOA, GetScrambledAOSOAContext<AOSOAData1024Bytes>, VerifyAOSOA, laneCount, 16, iterationCount);
                    aosoaScrambledTimes[3] = Time(AOSOA, GetScrambledAOSOAContext<AOSOAData2048Bytes>, VerifyAOSOA, laneCount, 32, iterationCount);
                    aosoaScrambledTimes[4] = Time(AOSOA, GetScrambledAOSOAContext<AOSOAData4096Bytes>, VerifyAOSOA, laneCount, 64, iterationCount);
                    aosoaScrambledTimes[5] = Time(AOSOA, GetScrambledAOSOAContext<AOSOAData8192Bytes>, VerifyAOSOA, laneCount, 128, iterationCount);
                    break;
            }

            Console.WriteLine("AOSOA Scrambled Times, by lane width:");
            for (int i = 0; i < aosoaTimes.Length; ++i)
            {
                Console.WriteLine($"{4 << i}: {aosoaScrambledTimes[i] * timeScaling} ns");
            }
        }
    }
}
