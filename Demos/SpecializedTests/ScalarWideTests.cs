using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;

namespace Demos.SpecializedTests
{
    //TODO: This is just a bit of experimental stuff...
    //For now, this is type specialized for single precision floats. That is by far the most common datatype required by the engine.
    //Later, we could consider a Vector<T> style implementation.

    //Note that this implementation supposes a pretty low upper bound on the size of hardware intrinsics.
    //While intel will probably stay at AVX512 for an architecture or two since it's the size of a cache line, there exist other architectures
    //with variable widths of up to 256 bytes. We do not have a clean way to handle those; there's no guarantee that they'd even be pow2.
    //We can't use jit-constant stuff to define the size of structs, and if we have a runtime variable length width, we lose the ability to access lanes within a wide struct
    //by simply reinterpreting a float offset. The stride would change.
    /// <summary>
    /// Provides a wide group of single precision floats for use with SIMD.
    /// </summary>
    [StructLayout(LayoutKind.Explicit, Size = size)]
    public struct ScalarWideUnsafeAdds
    {
        const int size = 64;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(ref ScalarWideUnsafeAdds a, ref ScalarWideUnsafeAdds b, out ScalarWideUnsafeAdds result)
        {
            int bundleCount = size / (4 * Vector<float>.Count);
            if (bundleCount <= 4 && Vector.IsHardwareAccelerated)
            {
                ref var startA = ref Unsafe.As<ScalarWideUnsafeAdds, Vector<float>>(ref a);
                ref var startB = ref Unsafe.As<ScalarWideUnsafeAdds, Vector<float>>(ref b);
                ref var startResult = ref Unsafe.As<ScalarWideUnsafeAdds, Vector<float>>(ref result);
                if (bundleCount >= 1)
                    startResult = startA + startB;
                if (bundleCount >= 2)
                    Unsafe.Add(ref startResult, 1) = Unsafe.Add(ref startA, 1) + Unsafe.Add(ref startB, 1);
                if (bundleCount >= 3)
                    Unsafe.Add(ref startResult, 2) = Unsafe.Add(ref startA, 2) + Unsafe.Add(ref startB, 2);
                if (bundleCount >= 4)
                    Unsafe.Add(ref startResult, 3) = Unsafe.Add(ref startA, 3) + Unsafe.Add(ref startB, 3);
            }
            else
            {
                ref var startA = ref Unsafe.As<ScalarWideUnsafeAdds, float>(ref a);
                ref var startB = ref Unsafe.As<ScalarWideUnsafeAdds, float>(ref b);
                ref var startResult = ref Unsafe.As<ScalarWideUnsafeAdds, float>(ref result);
                for (int i = 0; i < (size / sizeof(float)); ++i)
                {
                    Unsafe.Add(ref startResult, i) = Unsafe.Add(ref startA, i) + Unsafe.Add(ref startB, i);
                }
            }

        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ScalarWideUnsafeAdds operator +(ScalarWideUnsafeAdds a, ScalarWideUnsafeAdds b)
        {
            Add(ref a, ref b, out var result);
            return result;
        }
    }
    [StructLayout(LayoutKind.Explicit, Size = size)]
    public struct ScalarWideCast
    {
        const int size = 64;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(ref ScalarWideCast a, ref ScalarWideCast b, out ScalarWideCast result)
        {
            if (Vector.IsHardwareAccelerated)
            {
                int bundleCount = size / (4 * Vector<float>.Count);
                if (bundleCount == 4)
                {
                    ref var castA = ref Unsafe.As<ScalarWideCast, Scalar4V>(ref a);
                    ref var castB = ref Unsafe.As<ScalarWideCast, Scalar4V>(ref b);
                    ref var castResult = ref Unsafe.As<ScalarWideCast, Scalar4V>(ref result);
                    castResult.V0 = castA.V0 + castB.V0;
                    castResult.V1 = castA.V1 + castB.V1;
                    castResult.V2 = castA.V2 + castB.V2;
                    castResult.V3 = castA.V3 + castB.V3;
                }
                else if (bundleCount == 2)
                {
                    ref var castA = ref Unsafe.As<ScalarWideCast, Scalar2V>(ref a);
                    ref var castB = ref Unsafe.As<ScalarWideCast, Scalar2V>(ref b);
                    ref var castResult = ref Unsafe.As<ScalarWideCast, Scalar2V>(ref result);
                    castResult.V0 = castA.V0 + castB.V0;
                    castResult.V1 = castA.V1 + castB.V1;
                }
                else 
                {
                    ref var castA = ref Unsafe.As<ScalarWideCast, Scalar1V>(ref a);
                    ref var castB = ref Unsafe.As<ScalarWideCast, Scalar1V>(ref b);
                    ref var castResult = ref Unsafe.As<ScalarWideCast, Scalar1V>(ref result);
                    castResult.V0 = castA.V0 + castB.V0;
                }
            }
            else
            {
                ref var startA = ref Unsafe.As<ScalarWideCast, float>(ref a);
                ref var startB = ref Unsafe.As<ScalarWideCast, float>(ref b);
                ref var startResult = ref Unsafe.As<ScalarWideCast, float>(ref result);
                for (int i = 0; i < (size / sizeof(float)); ++i)
                {
                    Unsafe.Add(ref startResult, i) = Unsafe.Add(ref startA, i) + Unsafe.Add(ref startB, i);
                }
            }

        }
    }
    public struct Scalar4V
    {
        public Vector<float> V0;
        public Vector<float> V1;
        public Vector<float> V2;
        public Vector<float> V3;
    }
    public struct Scalar2V
    {
        public Vector<float> V0;
        public Vector<float> V1;
    }
    public struct Scalar1V
    {
        public Vector<float> V0;
    }


    static class ScalarWideTests
    {
        [MethodImpl(MethodImplOptions.NoInlining)]
        static void RefAdd(ref ScalarWideUnsafeAdds a, ref ScalarWideUnsafeAdds b, out ScalarWideUnsafeAdds result)
        {
            ScalarWideUnsafeAdds.Add(ref a, ref b, out result);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        static void OpAdd(ref ScalarWideUnsafeAdds a, ref ScalarWideUnsafeAdds b, out ScalarWideUnsafeAdds result)
        {
            result = a + b;
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        static void CastAdd(ref ScalarWideCast a, ref ScalarWideCast b, out ScalarWideCast result)
        {
            ScalarWideCast.Add(ref a, ref b, out result);
        }

        struct ExplicitStruct
        {
            public Vector<float> V0;
            public Vector<float> V1;
            public Vector<float> V2;
            public Vector<float> V3;
        }


        [MethodImpl(MethodImplOptions.NoInlining)]
        static void ExplicitAdd(ref ExplicitStruct a, ref ExplicitStruct b, out ExplicitStruct result)
        {
            result.V0 = a.V0 + b.V0;
            result.V1 = a.V1 + b.V1;
            result.V2 = a.V2 + b.V2;
            result.V3 = a.V3 + b.V3;
        }

        public static void Test()
        {
            var wideA = new ScalarWideUnsafeAdds();
            var wideB = new ScalarWideUnsafeAdds();
            RefAdd(ref wideA, ref wideB, out var result1);
            OpAdd(ref wideA, ref wideB, out var result2);
            var explicitA = new ExplicitStruct();
            var explicitB = new ExplicitStruct();
            var castWideA = new ScalarWideCast();
            var castWideB = new ScalarWideCast();
            CastAdd(ref castWideA, ref castWideB, out var result3);

            ExplicitAdd(ref explicitA, ref explicitB, out var result4);
        }
    }
}
