using BepuUtilities.Memory;
using BepuPhysics;
using BepuPhysics.Constraints;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace Demos.SpecializedTests
{
    class GatherScatterTests
    {
        struct Context
        {
            public Buffer<BodyVelocity> BodyVelocities;
            public TwoBodyReferences[] BodyReferences;
            public int ConstraintBundleCount;
        }
        struct ConstraintBodies
        {
            public int A, B;
        }
        static Context GetFreshContext(int constraintBundleCount, int bodyCount, Comparison<ConstraintBodies> sortComparison = null)
        {
            Context context;
            context.ConstraintBundleCount = constraintBundleCount;
            var pool = new BufferPool();
            pool.SpecializeFor<BodyVelocity>().Take(bodyCount, out context.BodyVelocities);
            var connections = new ConstraintBodies[constraintBundleCount * Vector<int>.Count];
            var random = new Random(5);

            for (int i = 0; i < connections.Length; ++i)
            {
                connections[i] = new ConstraintBodies { A = random.Next(bodyCount), B = random.Next(bodyCount) };
            }
            if (sortComparison != null)
            {
                //Sorting the connections should increase the probability that at least one of the two bodies associated with a constraint will be in the cache already from earlier prefetches.
                Array.Sort(connections, sortComparison);
            }
            context.BodyReferences = new TwoBodyReferences[constraintBundleCount];
            for (int iterationIndex = 0; iterationIndex < constraintBundleCount; ++iterationIndex)
            {
                var baseSourceIndex = iterationIndex << BundleIndexing.VectorShift;
                for (int i = 0; i < Vector<int>.Count; ++i)
                {
                    var c = connections[baseSourceIndex + i];
                    GatherScatter.Get(ref context.BodyReferences[iterationIndex].IndexA, i) = c.A;
                    GatherScatter.Get(ref context.BodyReferences[iterationIndex].IndexB, i) = c.B;
                }
            }
            return context;
        }

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
        static double Time(Action<Context> action, int iterationCount, int constraintCount, int bodyBundleCount, Comparison<ConstraintBodies> constraintSort = null)
        {
            //Note lack of optimizations; the pre-jit seems to get poofed when optimizations are enabled.
            action(GetFreshContext(1, 1));
            GC.Collect(3, GCCollectionMode.Forced, true, true);
            var context = GetFreshContext(constraintCount, bodyBundleCount, constraintSort);
            var timer = Timer.Start();
            for (int i = 0; i < iterationCount; ++i)
            {
                action(context);
            }
            return timer.Stop() / (constraintCount * iterationCount);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        static void TestGather(Context context)
        {
            for (int i = 0; i < context.ConstraintBundleCount; ++i)
            {
                BodyVelocities.GatherVelocities(ref context.BodyVelocities, ref context.BodyReferences[i], Vector<float>.Count, out var a, out var b);
            }
        }


        [MethodImpl(MethodImplOptions.NoInlining)]
        static void TestScatter(Context context)
        {
            var a = new BodyVelocities();
            var b = new BodyVelocities();
            for (int i = 0; i < context.ConstraintBundleCount; ++i)
            {
                BodyVelocities.ScatterVelocities(ref a, ref b, ref context.BodyVelocities, ref context.BodyReferences[i], Vector<float>.Count);
            }
        }




        public static void Test()
        {
            const int iterationCount = 1000;
            const int constraintCount = 4096 << 3;
            const int bodyBundleCount = 1024 << 2;
            Comparison<ConstraintBodies> smallestBiggestSort = (x, y) =>
            {
                ulong Encode(ConstraintBodies bodies)
                {
                    if (bodies.A < bodies.B)
                        return (ulong)(bodies.A << 32) | (ulong)bodies.B;
                    return (ulong)(bodies.B << 32) | (ulong)bodies.A;
                }
                return Encode(x).CompareTo(Encode(y));
            };

            var gatherTime = Time(TestGather, iterationCount, constraintCount, bodyBundleCount);
            var sortedGatherTime = Time(TestGather, iterationCount, constraintCount, bodyBundleCount, smallestBiggestSort);
            var scatterTime = Time(TestScatter, iterationCount, constraintCount, bodyBundleCount);
            var sortedScatterTime = Time(TestScatter, iterationCount, constraintCount, bodyBundleCount, smallestBiggestSort);

            const double scaling = 1e9;

            Console.WriteLine(
                $"gather time (ns): {gatherTime * scaling},\n" +
                $"sorted gather time (ns): {sortedGatherTime * scaling},\n" +
                $"scatter time (ns): {scatterTime * scaling},\n" +
                $"sorted scatter time (ns): {sortedScatterTime * scaling}");

        }

    }
}
