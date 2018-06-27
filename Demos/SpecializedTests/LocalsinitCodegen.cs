using BepuPhysics;
using BepuPhysics.Constraints;
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
    public static class LocalsinitCodegen
    {
        public struct OuterStruct8
        {
            public struct ScalarInnerStruct
            {
                public float X;
            }
            public ScalarInnerStruct X;
            public ScalarInnerStruct Y;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void Scale(ref OuterStruct8 m, float s, out OuterStruct8 result)
            {
                result.X.X = m.X.X * s;
                result.Y.X = m.Y.X * s;
            }

            [MethodImpl(MethodImplOptions.NoInlining)]
            public static void Test(ref OuterStruct8 m, float s, out OuterStruct8 result)
            {
                //8 byte mov clear
                Scale(ref m, s, out var intermediate);
                Scale(ref intermediate, s, out result);

                //Notably, if we use flattened temps, no locals initialization is emitted:
                //var tempXX = m.X.X * s;
                //var tempYX = m.Y.X * s;
                //result.X.X = tempXX * s;
                //result.Y.X = tempYX * s;
                //This holds for the later tests too.
            }
        }

        public struct OuterStruct16
        {
            public struct ScalarInnerStruct
            {
                public float X;
                public float Y;
            }
            public ScalarInnerStruct X;
            public ScalarInnerStruct Y;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void Scale(ref OuterStruct16 m, float s, out OuterStruct16 result)
            {
                result.X.X = m.X.X * s;
                result.X.Y = m.X.Y * s;
                result.Y.X = m.Y.X * s;
                result.Y.Y = m.Y.Y * s;
            }

            [MethodImpl(MethodImplOptions.NoInlining)]
            public static void Test(ref OuterStruct16 m, float s, out OuterStruct16 result)
            {
                //rep stos, ecx 4 (16 bytes)
                Scale(ref m, s, out var intermediate);
                Scale(ref intermediate, s, out result);
            }
        }

        public struct DummyOuterStruct
        {
            public struct InnerStruct
            {
                public float XX;
                public float XY;
                public float YX;
                public float YY;
            }

            public InnerStruct Inner;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Scale(ref DummyOuterStruct m, float s, out DummyOuterStruct result)
            {
                result.Inner.XX = m.Inner.XX * s;
                result.Inner.XY = m.Inner.XY * s;
                result.Inner.YX = m.Inner.YX * s;
                result.Inner.YY = m.Inner.YY * s;
            }

            [MethodImpl(MethodImplOptions.NoInlining)]
            public static void Test(ref DummyOuterStruct m, float s, out DummyOuterStruct result)
            {
                //rep stos, ecx 4h (16 bytes)
                Scale(ref m, s, out var intermediate);
                Scale(ref intermediate, s, out result);
            }
        }

        public struct ScalarFlattenedStruct
        {
            public float XX;
            public float XY;
            public float YX;
            public float YY;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Scale(ref ScalarFlattenedStruct m, float s, out ScalarFlattenedStruct result)
            {
                result.XX = m.XX * s;
                result.XY = m.XY * s;
                result.YX = m.YX * s;
                result.YY = m.YY * s;
            }
            [MethodImpl(MethodImplOptions.NoInlining)]
            public static void Test(ref ScalarFlattenedStruct m, float s, out ScalarFlattenedStruct result)
            {
                //no locals initialization
                Scale(ref m, s, out var intermediate);
                Scale(ref intermediate, s, out result);
            }
        }
        public struct Int4
        {
            public int A;
            public int B;
            public int C;
            public int D;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Scale(ref Int4 m, int s, out Int4 result)
            {
                result.A = m.A * s;
                result.B = m.B * s;
                result.C = m.C * s;
                result.D = m.D * s;
            }

            [MethodImpl(MethodImplOptions.NoInlining)]
            public static void Test(ref Int4 m, int s, out Int4 result)
            {
                //no locals initialized
                Scale(ref m, s, out var temp);
                Scale(ref temp, s, out result);
            }
        }

        public struct Int5
        {
            public int A;
            public int B;
            public int C;
            public int D;
            public int E;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Scale(ref Int5 m, int s, out Int5 result)
            {
                result.A = m.A * s;
                result.B = m.B * s;
                result.C = m.C * s;
                result.D = m.D * s;
                result.E = m.E * s;
            }

            [MethodImpl(MethodImplOptions.NoInlining)]
            public static void Test(ref Int5 m, int s, out Int5 result)
            {
                //rep stos, ecx 6 (24 bytes)
                Scale(ref m, s, out var temp);
                Scale(ref temp, s, out result);
            }
        }
        public struct Long4
        {
            public long A;
            public long B;
            public long C;
            public long D;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Scale(ref Long4 m, long s, out Long4 result)
            {
                result.A = m.A * s;
                result.B = m.B * s;
                result.C = m.C * s;
                result.D = m.D * s;
            }

            [MethodImpl(MethodImplOptions.NoInlining)]
            public static void Test(ref Long4 m, long s, out Long4 result)
            {
                //no locals initialized
                Scale(ref m, s, out var temp);
                Scale(ref temp, s, out result);
            }
        }

        public struct Long5
        {
            public long A;
            public long B;
            public long C;
            public long D;
            public long E;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Scale(ref Long5 m, long s, out Long5 result)
            {
                result.A = m.A * s;
                result.B = m.B * s;
                result.C = m.C * s;
                result.D = m.D * s;
                result.E = m.E * s;
            }

            [MethodImpl(MethodImplOptions.NoInlining)]
            public static void Test(ref Long5 m, long s, out Long5 result)
            {
                //rep stos, ecx Ah (40 bytes)
                Scale(ref m, s, out var temp);
                Scale(ref temp, s, out result);
            }
        }
        public struct VectorFloat4
        {
            public Vector<float> A;
            public Vector<float> B;
            public Vector<float> C;
            public Vector<float> D;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Scale(ref VectorFloat4 m, ref Vector<float> s, out VectorFloat4 result)
            {
                result.A = m.A * s;
                result.B = m.B * s;
                result.C = m.C * s;
                result.D = m.D * s;

            }

            [MethodImpl(MethodImplOptions.NoInlining)]
            public static void Test(ref VectorFloat4 m, ref Vector<float> s, out VectorFloat4 result)
            {
                //no locals initialized
                Scale(ref m, ref s, out var temp);
                Scale(ref temp, ref s, out result);
            }
        }

        public struct VectorFloat5
        {
            public Vector<float> A;
            public Vector<float> B;
            public Vector<float> C;
            public Vector<float> D;
            public Vector<float> E;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Scale(ref VectorFloat5 m, ref Vector<float> s, out VectorFloat5 result)
            {
                result.A = m.A * s;
                result.B = m.B * s;
                result.C = m.C * s;
                result.D = m.D * s;
                result.E = m.E * s;
            }

            [MethodImpl(MethodImplOptions.NoInlining)]
            public static void Test(ref VectorFloat5 m, ref Vector<float> s, out VectorFloat5 result)
            {
                //rep stos, ecx 14h (80 bytes) when Vector<float>.Count == 4
                Scale(ref m, ref s, out var temp);
                Scale(ref temp, ref s, out result);
            }
        }

        [StructLayout(LayoutKind.Sequential, Size = 16)]
        struct DummyStruct16
        {
        }
        [StructLayout(LayoutKind.Sequential, Size = 32)]
        struct DummyStruct32
        {
        }
        [StructLayout(LayoutKind.Sequential, Size = 48)]
        struct DummyStruct48
        {
        }
        [StructLayout(LayoutKind.Sequential, Size = 64)]
        struct DummyStruct64
        {
        }
        [StructLayout(LayoutKind.Sequential, Size = 80)]
        struct DummyStruct80
        {
        }


        [MethodImpl(MethodImplOptions.NoInlining)]
        static void DoDummies<T>(ref T a, out T result)
        {
            //init across size equal to T size. (for 16 byte case, uses two movs)
            var temp = a;
            result = temp;
        }


        struct ScalarStructType
        {
            public float A;
            public float B;
            public float C;
            public float D;
            public float E;
            public float F;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void DoSomeWorkWithAScalarStruct(ref float source, out float result)
        {
            ScalarStructType u;
            u.A = 2 * source;
            u.B = 3 * source;
            u.C = 4 * source;
            u.D = 5 * source;
            u.E = 6 * source;
            u.F = 7 * source;
            result = u.A + u.B + u.C + u.D + u.E + u.F;
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        static void TestScalarStruct()
        {
            float f = 0;
            for (int i = 0; i < 100; ++i)
            {
                DoSomeWorkWithAScalarStruct(ref f, out f);
            }
        }

        struct StructType
        {
            public Vector<float> A;
            public Vector<float> B;
            public Vector<float> C;
            public Vector<float> D;
            public Vector<float> E;
            public Vector<float> F;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void DoSomeWorkWithAStruct(ref Vector<float> source, out Vector<float> result)
        {
            StructType u;
            u.A = new Vector<float>(2) * source;
            u.B = new Vector<float>(3) * source;
            u.C = new Vector<float>(4) * source;
            u.D = new Vector<float>(5) * source;
            u.E = new Vector<float>(6) * source;
            u.F = new Vector<float>(7) * source;
            result = u.A + u.B + u.C + u.D + u.E + u.F;
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        static void TestStruct()
        {
            Vector<float> f = default;
            for (int i = 0; i < 100; ++i)
            {
                DoSomeWorkWithAStruct(ref f, out f);
            }
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        static void TestStructManuallyInlined()
        {
            Vector<float> f = default;
            for (int i = 0; i < 100; ++i)
            {
                StructType u;
                u.A = new Vector<float>(2) * f;
                u.B = new Vector<float>(3) * f;
                u.C = new Vector<float>(4) * f;
                u.D = new Vector<float>(5) * f;
                u.E = new Vector<float>(6) * f;
                u.F = new Vector<float>(7) * f;
                f = u.A + u.B + u.C + u.D + u.E + u.F;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void DoSomeWorkStructless(ref Vector<float> source, out Vector<float> result)
        {
            var a = new Vector<float>(2) * source;
            var b = new Vector<float>(3) * source;
            var c = new Vector<float>(4) * source;
            var d = new Vector<float>(5) * source;
            var e = new Vector<float>(6) * source;
            var f = new Vector<float>(7) * source;
            result = d + e + f + a + b + c;
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        static void TestStructless()
        {
            Vector<float> f = default;
            for (int i = 0; i < 100; ++i)
            {
                DoSomeWorkStructless(ref f, out f);
            }
        }


        public static void Test()
        {
            {
                //Nested versus flattened tests
                var s8 = new OuterStruct8();
                var s16 = new OuterStruct16();
                var s = 0f;
                OuterStruct8.Test(ref s8, s, out var result8);
                OuterStruct16.Test(ref s16, s, out var result16);

                var d = new DummyOuterStruct();
                DummyOuterStruct.Test(ref d, 0f, out var dResult);

                var f = new ScalarFlattenedStruct();
                ScalarFlattenedStruct.Test(ref f, s, out var flattenedResult);

            }

            {
                //Field count tests
                var i4 = new Int4();
                var i5 = new Int5();
                var l4 = new Long4();
                var l5 = new Long5();
                var vf4 = new VectorFloat4();
                var vf5 = new VectorFloat5();
                var s = new Vector<float>();
                Int4.Test(ref i4, 0, out var i4result);
                Int5.Test(ref i5, 0, out var i5result);
                Long4.Test(ref l4, 0, out var l4result);
                Long5.Test(ref l5, 0, out var l5result);
                VectorFloat4.Test(ref vf4, ref s, out var vf4Result);
                VectorFloat5.Test(ref vf5, ref s, out var vf5Result);
            }

            {
                //Assignment-only tests
                var s16 = new DummyStruct16();
                var s32 = new DummyStruct32();
                var s48 = new DummyStruct48();
                var s64 = new DummyStruct64();
                var s80 = new DummyStruct80();
                DoDummies(ref s16, out var result16);
                DoDummies(ref s32, out var result32);
                DoDummies(ref s48, out var result48);
                DoDummies(ref s64, out var result64);
                DoDummies(ref s80, out var result80);
            }

            {
                TestScalarStruct();
                TestStruct();
                TestStructManuallyInlined();
                TestStructless();
            }
        }






    }
}
