using BepuPhysics;
using BepuPhysics.Constraints;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace Demos.SpecializedTests
{
    public static class InCodegenTests
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
            public void InstanceTest1(out Vector<float> result)
            {
                result = X * X + Y * Y + Z * Z;
            }
            [MethodImpl(MethodImplOptions.NoInlining)]
            public void InstanceTest2(out Vector<float> result)
            {
                result = X * X + Y * Y + Z * Z;
            }

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
            var a = new TestVector3Wide(new Vector<float>(2f));
            TestVector3Wide.AddRef(ref a, ref a, out var s0);
            TestVector3Wide.AddRef(ref a, ref a, out var s1);
            TestVector3Wide.AddRef(ref a, ref a, out var s2);
            TestVector3Wide.AddRef(ref a, ref a, out var s3);
            TestVector3Wide.AddRef(ref s0, ref s1, out var s4);
            TestVector3Wide.AddRef(ref s2, ref s3, out var s5);
            TestVector3Wide.AddRef(ref s4, ref s5, out accumulator);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        static void InTest(out TestVector3Wide accumulator)
        {
            var a = new TestVector3Wide(new Vector<float>(2f));
            TestVector3Wide.AddIn(a, a, out var s0);
            TestVector3Wide.AddIn(a, a, out var s1);
            TestVector3Wide.AddIn(a, a, out var s2);
            TestVector3Wide.AddIn(a, a, out var s3);
            TestVector3Wide.AddIn(s0, s1, out var s4);
            TestVector3Wide.AddIn(s2, s3, out var s5);
            TestVector3Wide.AddIn(s4, s5, out accumulator);
        }
        [MethodImpl(MethodImplOptions.NoInlining)]
        static void OpTest(out TestVector3Wide accumulator)
        {
            accumulator = new TestVector3Wide();
            var a = new TestVector3Wide(new Vector<float>(2f));
            var s0 = a + a;
            var s1 = a + a;
            var s2 = a + a;
            var s3 = a + a;
            var s4 = s0 + s1;
            var s5 = s2 + s3;
            accumulator = s4 + s5;
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        static void InstanceMethodRefTest1(ref TestVector3Wide input, out Vector<float> result)
        {
            input.InstanceTest1(out result);
        }
        [MethodImpl(MethodImplOptions.NoInlining)]
        static void InstanceMethodRefTest2(ref TestVector3Wide input, out Vector<float> result)
        {
            input.InstanceTest2(out result);
        }
        [MethodImpl(MethodImplOptions.NoInlining)]
        static void InstanceMethodInTest1(in TestVector3Wide input, out Vector<float> result)
        {
            input.InstanceTest1(out result);
        }
        [MethodImpl(MethodImplOptions.NoInlining)]
        static void InstanceMethodInTest2(in TestVector3Wide input, out Vector<float> result)
        {
            input.InstanceTest2(out result);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void Test()
        {
            OpTest(out var opTest);
            InTest(out var inTest);
            RefTest(out var refTest);

            InstanceMethodInTest1(opTest, out inTest.X);
            InstanceMethodInTest2(opTest, out inTest.X);
            InstanceMethodRefTest1(ref opTest, out inTest.X);
            InstanceMethodRefTest2(ref opTest, out inTest.X);

            var simulation = Simulation.Create(new BepuUtilities.Memory.BufferPool(), new TestCallbacks());
            var bodyHandle = simulation.Bodies.Add(new BodyDescription());
            var grabber = new OneBodyLinearServo();
            simulation.Solver.Add(0, ref grabber);

            Console.WriteLine($"Op {opTest}, In {inTest}, Ref {refTest}");
        }
    }
}
