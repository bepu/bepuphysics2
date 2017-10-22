using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Text;

namespace Demos.SpecializedTests
{
    public static class VirtualOverheadTest
    {
        abstract class Superclass
        {
            public abstract void Do(ref int i);
        }
        class Increment : Superclass
        {
            public override void Do(ref int i)
            {
                ++i;
            }
        }
        class Decrement : Superclass
        {
            public override void Do(ref int i)
            {
                --i;
            }
        }
        class LShift : Superclass
        {
            public override void Do(ref int i)
            {
                i <<= 1;
            }
        }
        class RShift : Superclass
        {
            public override void Do(ref int i)
            {
                i >>= 1;
            }
        }
        class Scramble1 : Superclass
        {
            public override void Do(ref int i)
            {
                i = i ^ (i >> 4);
            }
        }
        class Scramble2 : Superclass
        {
            public override void Do(ref int i)
            {
                i = i ^ (i << 4);
            }
        }
        class Increment2 : Superclass
        {
            public override void Do(ref int i)
            {
                ++i;
            }
        }
        class Decrement2 : Superclass
        {
            public override void Do(ref int i)
            {
                --i;
            }
        }
        class LShift2 : Superclass
        {
            public override void Do(ref int i)
            {
                i <<= 1;
            }
        }
        class RShift2 : Superclass
        {
            public override void Do(ref int i)
            {
                i >>= 1;
            }
        }
        class Scramble12 : Superclass
        {
            public override void Do(ref int i)
            {
                i = i ^ (i >> 4);
            }
        }
        class Scramble22 : Superclass
        {
            public override void Do(ref int i)
            {
                i = i ^ (i << 4);
            }
        }

        enum ExecutionType
        {
            Increment = 0, Decrement = 1, LShift = 2, RShift = 3, Scramble1 = 4, Scramble2 = 5,
            Increment2 = 6, Decrement2 = 7, LShift2 = 8, RShift2 = 9, Scramble12 = 10, Scramble22 = 11
        }
        struct Switch
        {
            public ExecutionType Type;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Do(ref int i)
            {
                switch (Type)
                {
                    case ExecutionType.Increment:
                        Increment(ref i);
                        break;
                    case ExecutionType.Decrement:
                        Decrement(ref i);
                        break;
                    case ExecutionType.LShift:
                        LShift(ref i);
                        break;
                    case ExecutionType.RShift:
                        RShift(ref i);
                        break;
                    case ExecutionType.Scramble1:
                        Scramble1(ref i);
                        break;
                    case ExecutionType.Scramble2:
                        Scramble2(ref i);
                        break;
                    case ExecutionType.Increment2:
                        Increment2(ref i);
                        break;
                    case ExecutionType.Decrement2:
                        Decrement2(ref i);
                        break;
                    case ExecutionType.LShift2:
                        LShift2(ref i);
                        break;
                    case ExecutionType.RShift2:
                        RShift2(ref i);
                        break;
                    case ExecutionType.Scramble12:
                        Scramble12(ref i);
                        break;
                    case ExecutionType.Scramble22:
                        Scramble22(ref i);
                        break;
                }
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void Increment(ref int i)
            {
                ++i;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void Decrement(ref int i)
            {
                --i;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void LShift(ref int i)
            {
                i <<= 1;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void RShift(ref int i)
            {
                i >>= 1;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void Scramble1(ref int i)
            {
                i = i ^ (i >> 4);
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void Scramble2(ref int i)
            {
                i = i ^ (i << 4);
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void Increment2(ref int i)
            {
                ++i;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void Decrement2(ref int i)
            {
                --i;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void LShift2(ref int i)
            {
                i <<= 1;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void RShift2(ref int i)
            {
                i >>= 1;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void Scramble12(ref int i)
            {
                i = i ^ (i >> 4);
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void Scramble22(ref int i)
            {
                i = i ^ (i << 4);
            }
        }

        public static void Test()
        {
            const int invocationTestCount = 256;
            const int iterations = 10000;
            var switches = new Switch[invocationTestCount];
            var virtuals = new Superclass[invocationTestCount];
            Random random = new Random(5);
            for (int i = 0; i < invocationTestCount; ++i)
            {
                var executionType = (ExecutionType)random.Next(12);
                switches[i].Type = executionType;
                switch (executionType)
                {
                    case ExecutionType.Increment:
                        virtuals[i] = new Increment();
                        break;
                    case ExecutionType.Decrement:
                        virtuals[i] = new Decrement();
                        break;
                    case ExecutionType.LShift:
                        virtuals[i] = new LShift();
                        break;
                    case ExecutionType.RShift:
                        virtuals[i] = new RShift();
                        break;
                    case ExecutionType.Scramble1:
                        virtuals[i] = new Scramble1();
                        break;
                    case ExecutionType.Scramble2:
                        virtuals[i] = new Scramble2();
                        break;
                    case ExecutionType.Increment2:
                        virtuals[i] = new Increment2();
                        break;
                    case ExecutionType.Decrement2:
                        virtuals[i] = new Decrement2();
                        break;
                    case ExecutionType.LShift2:
                        virtuals[i] = new LShift2();
                        break;
                    case ExecutionType.RShift2:
                        virtuals[i] = new RShift2();
                        break;
                    case ExecutionType.Scramble12:
                        virtuals[i] = new Scramble12();
                        break;
                    case ExecutionType.Scramble22:
                        virtuals[i] = new Scramble22();
                        break;

                }
            }
            int switchValue = 0;
            int virtualValue = 0;
            //Warmup.
            for (int i = 0; i < invocationTestCount; ++i)
            {
                switches[i].Do(ref switchValue);
                virtuals[i].Do(ref virtualValue);
            }
            var switchStart = Stopwatch.GetTimestamp();
            for (int i = 0; i < iterations; ++i)
            {
                for (int j = 0; j < invocationTestCount; ++j)
                {
                    switches[j].Do(ref switchValue);
                    switches[j].Do(ref switchValue);
                    switches[j].Do(ref switchValue);
                    switches[j].Do(ref switchValue);
                    switches[j].Do(ref switchValue);

                    switches[j].Do(ref switchValue);
                    switches[j].Do(ref switchValue);
                    switches[j].Do(ref switchValue);
                    switches[j].Do(ref switchValue);
                    switches[j].Do(ref switchValue);
                }
            }
            var virtualStart = Stopwatch.GetTimestamp();
            for (int i = 0; i < iterations; ++i)
            {
                for (int j = 0; j < invocationTestCount; ++j)
                {
                    virtuals[j].Do(ref virtualValue);
                    virtuals[j].Do(ref virtualValue);
                    virtuals[j].Do(ref virtualValue);
                    virtuals[j].Do(ref virtualValue);
                    virtuals[j].Do(ref virtualValue);

                    virtuals[j].Do(ref virtualValue);
                    virtuals[j].Do(ref virtualValue);
                    virtuals[j].Do(ref virtualValue);
                    virtuals[j].Do(ref virtualValue);
                    virtuals[j].Do(ref virtualValue);
                }
            }
            var virtualEnd = Stopwatch.GetTimestamp();
            Console.WriteLine($"Switch time (ns): {1e9 * (virtualStart - switchStart) / (Stopwatch.Frequency * invocationTestCount * iterations * 10)}");
            Console.WriteLine($"Virtual time (ns): {1e9 * (virtualEnd - virtualStart) / (Stopwatch.Frequency * invocationTestCount * iterations * 10)}");
            Console.WriteLine($"Switch accumulator: {switchValue}, virtual accumulator: {virtualValue}");
        }
    }
}
