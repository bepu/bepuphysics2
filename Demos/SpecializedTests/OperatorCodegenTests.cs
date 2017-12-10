using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace Demos.SpecializedTests
{
    public static class OperatorCodegenTests
    {
        struct TestVector3Wide
        {
            public TestVector3Wide(in Vector<float> s)
            {
                X = s;
                Y = s;
                Z = s;
            }
            public Vector<float> X;
            public Vector<float> Y;
            public Vector<float> Z;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void AddRef(ref TestVector3Wide a, ref TestVector3Wide b, out TestVector3Wide result)
            {
                result.X = a.X + b.X;
                result.Y = a.Y + b.Y;
                result.Z = a.Z + b.Z;
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void AddIn(in TestVector3Wide a, in TestVector3Wide b, out TestVector3Wide result)
            {
                result.X = a.X + b.X;
                result.Y = a.Y + b.Y;
                result.Z = a.Z + b.Z;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static TestVector3Wide operator +(in TestVector3Wide a, in TestVector3Wide b)
            {
                AddIn(a, b, out var result);
                return result;
            }
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        static void RefTest(out TestVector3Wide accumulator)
        {
            accumulator = new TestVector3Wide();
            var a = new TestVector3Wide(new Vector<float>(2f));
            TestVector3Wide.AddRef(ref accumulator, ref a, out accumulator);
            TestVector3Wide.AddRef(ref accumulator, ref a, out accumulator);
            TestVector3Wide.AddRef(ref accumulator, ref a, out accumulator);
            TestVector3Wide.AddRef(ref accumulator, ref a, out accumulator);
            TestVector3Wide.AddRef(ref accumulator, ref a, out accumulator);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        static void InTest(out TestVector3Wide accumulator)
        {
            accumulator = new TestVector3Wide();
            var a = new TestVector3Wide(new Vector<float>(2f));
            TestVector3Wide.AddIn(accumulator, a, out accumulator);
            TestVector3Wide.AddIn(accumulator, a, out accumulator);
            TestVector3Wide.AddIn(accumulator, a, out accumulator);
            TestVector3Wide.AddIn(accumulator, a, out accumulator);
            TestVector3Wide.AddIn(accumulator, a, out accumulator);
        }
        [MethodImpl(MethodImplOptions.NoInlining)]
        static void OpTest(out TestVector3Wide accumulator)
        {
            accumulator = new TestVector3Wide();
            var a = new TestVector3Wide(new Vector<float>(2f));
            accumulator += a;
            accumulator += a;
            accumulator += a;
            accumulator += a;
            accumulator += a;
        }


        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void Test()
        {
            OpTest(out var opTest);
            InTest(out var inTest);
            RefTest(out var refTest);

            Console.WriteLine($"Op {opTest}, In {inTest}, Ref {refTest}");
        }
    }
}
