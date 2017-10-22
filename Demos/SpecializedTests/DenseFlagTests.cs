using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading;

namespace Demos.SpecializedTests
{
    public static class DenseFlagTests
    {
        public struct Job
        {
            public int Start;
            public int End;
        }
        static Buffer<int> accessIndices;
        static int globalJobCounter = 0;
        static Job[] jobs;

        
        public interface IDataLayout
        {
            void Initialize();
            void InitializeIteration(int flagCount);
            void Execute(Job job);
            void Dispose();
            void Validate(int flagCount);
        }
        public class Shorts : IDataLayout
        {
            BufferPool pool;
            Buffer<short> buffer;

            public void Initialize()
            {
                pool = new BufferPool();
            }

            public void InitializeIteration(int flagCount)
            {
                pool.SpecializeFor<short>().Take(flagCount, out buffer);
            }

            public void Execute(Job job)
            {
                for (int i = job.Start; i < job.End; ++i)
                {
                    buffer[accessIndices[i]] = unchecked((short)0xFFFF);
                }
            }

            public void Dispose()
            {
                pool.Clear();
            }

            public void Validate(int flagCount)
            {
                for (int i = 0; i < flagCount; ++i)
                {
                    Debug.Assert(buffer[i] == unchecked((short)0xFFFF));
                }
            }

        }
        public class Ints : IDataLayout
        {
            BufferPool pool;
            Buffer<int> buffer;

            public void Initialize()
            {
                pool = new BufferPool();
            }

            public void InitializeIteration(int flagCount)
            {
                pool.SpecializeFor<int>().Take(flagCount, out buffer);
            }

            public void Execute(Job job)
            {
                for (int i = job.Start; i < job.End; ++i)
                {
                    buffer[accessIndices[i]] = unchecked((int)0xFFFFFFFF);
                }
            }

            public void Dispose()
            {
                pool.Clear();
            }

            public void Validate(int flagCount)
            {
                for (int i = 0; i < flagCount; ++i)
                {
                    Debug.Assert(buffer[i] == unchecked((int)0xFFFF_FFFF));
                }
            }

        }


        public class Longs : IDataLayout
        {
            BufferPool pool;
            Buffer<long> buffer;

            public void Initialize()
            {
                pool = new BufferPool();
            }

            public void InitializeIteration(int flagCount)
            {
                pool.SpecializeFor<long>().Take(flagCount, out buffer);
            }

            public void Execute(Job job)
            {
                for (int i = job.Start; i < job.End; ++i)
                {
                    buffer[accessIndices[i]] = unchecked((long)0xFFFF_FFFF_FFFF_FFFF);
                }
            }

            public void Dispose()
            {
                pool.Clear();
            }

            public void Validate(int flagCount)
            {
                for (int i = 0; i < flagCount; ++i)
                {
                    Debug.Assert(buffer[i] == unchecked((long)0xFFFF_FFFF_FFFF_FFFF));
                }
            }
        }

        public class Bytes : IDataLayout
        {
            BufferPool pool;
            RawBuffer buffer;

            public void Initialize()
            {
                pool = new BufferPool();
                buffer = new RawBuffer();
            }

            public void InitializeIteration(int flagCount)
            {
                pool.Take(flagCount, out buffer);
            }

            public void Execute(Job job)
            {
                for (int i = job.Start; i < job.End; ++i)
                {
                    buffer[accessIndices[i]] = 0xFF;
                }
            }

            public void Dispose()
            {
                pool.Clear();
            }

            public void Validate(int flagCount)
            {
                for (int i = 0; i < flagCount; ++i)
                {
                    Debug.Assert(buffer[i] == 0xFF);
                }
            }

        }

        public class Bits : IDataLayout
        {
            BufferPool pool;
            Buffer<int> buffer;

            public void Initialize()
            {
                pool = new BufferPool();
            }

            public void InitializeIteration(int flagCount)
            {
                pool.SpecializeFor<int>().Take((int)Math.Ceiling(flagCount / 32.0), out buffer);
            }

            public void Execute(Job job)
            {
                for (int i = job.Start; i < job.End; ++i)
                {
                    var index = accessIndices[i];
                    var intIndex = index >> 5;
                    var flag = 1 << (index & ((1 << 5) - 1));
                    Interlocked.Add(ref buffer[intIndex], flag);
                }
            }

            public void Dispose()
            {
                pool.Clear();
            }

            public void Validate(int flagCount)
            {
                for (int i = 0; i < flagCount; ++i)
                {
                    var value = buffer[i / 32];
                    var flag = 1 << (i & 31);
                    Debug.Assert((value & flag) == flag);
                }
            }

        }

        public static double Time<TDataLayout>(int iterationCount, int flagCount, IThreadDispatcher dispatcher) where TDataLayout : IDataLayout, new()
        {
            CacheBlaster.Blast();
            var dataLayout = new TDataLayout();
            dataLayout.Initialize();
            dataLayout.InitializeIteration(flagCount);
            Action<int> executeFunction = workerIndex =>
            {
                int jobIndex;
                while ((jobIndex = Interlocked.Increment(ref globalJobCounter) - 1) < jobs.Length)
                {
                    dataLayout.Execute(jobs[jobIndex]);
                }
            };
            globalJobCounter = 0;
            dispatcher.DispatchWorkers(executeFunction); //jit warmup
            dataLayout.Validate(flagCount);
            long time = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                //Note that individual executions of each approach do not reuse the same memory. The goal is to force cache misses.
                dataLayout.InitializeIteration(flagCount);
                globalJobCounter = 0;
                var start = Stopwatch.GetTimestamp();
                dispatcher.DispatchWorkers(executeFunction);
                var end = Stopwatch.GetTimestamp();
                time += end - start;
                dataLayout.Validate(flagCount);
            }
            dataLayout.Dispose();
            GC.Collect(3, GCCollectionMode.Forced, true);
            return time / (iterationCount * (double)Stopwatch.Frequency);
        }

        public static void Test()
        {
            var threadDispatcher = new SimpleThreadDispatcher(Environment.ProcessorCount);
            const int flagCount = 65536;

            const int jobsPerThread = 4;
            var jobCount = threadDispatcher.ThreadCount * jobsPerThread;
            jobs = new Job[jobCount];
            var previousEnd = 0;
            var baseJobIndexCount = flagCount / jobCount;
            var remainder = flagCount - baseJobIndexCount * jobCount;
            for (int i = 0; i < jobCount; ++i)
            {
                var jobIndexCount = i < remainder ? baseJobIndexCount + 1 : baseJobIndexCount;
                jobs[i] = new Job { Start = previousEnd, End = previousEnd += jobIndexCount };
            }

            var pool = new BufferPool();
            pool.SpecializeFor<int>().Take(flagCount, out accessIndices);
            for (int i = 0; i < flagCount; ++i)
            {
                accessIndices[i] = i;
            }
            var random = new Random(5);
            for (int i = 0; i < flagCount - 1; ++i)
            {
                ref var a = ref accessIndices[i];
                ref var b = ref accessIndices[random.Next(i + 1, flagCount)];
                var temp = a;
                a = b;
                b = temp;
            }
            long sumScrambled = 0, sumOrdered = 0;
            for (int i = 0; i < flagCount; ++i)
            {
                sumScrambled += accessIndices[i];
                sumOrdered += i;
            }
            Debug.Assert(sumScrambled == sumOrdered);



            const int iterationCount = 10000;
            for (int i = 0; i < 10; ++i)
            {
                Console.WriteLine($"Bits time (us): {1e6 * Time<Bits>(iterationCount, flagCount, threadDispatcher)}");
                Console.WriteLine($"Bytes time (us): {1e6 * Time<Bytes>(iterationCount, flagCount, threadDispatcher)}");
                Console.WriteLine($"Shorts time (us): {1e6 * Time<Shorts>(iterationCount, flagCount, threadDispatcher)}");
                Console.WriteLine($"Ints time (us): {1e6 * Time<Ints>(iterationCount, flagCount, threadDispatcher)}");
                Console.WriteLine($"Longs time (us): {1e6 * Time<Longs>(iterationCount, flagCount, threadDispatcher)}");
            }
            pool.Clear();

        }
    }
}
