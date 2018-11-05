using BepuUtilities;
using System;
using System.Numerics;

//using bMatrix3x3 = BEPUutilities.Matrix3x3;
//using bVector3 = BEPUutilities.Vector3;

namespace BEPUutilitiesTests
{
    public static class Matrix3x3Tests
    {

        //public static float TestTransformScalar(int iterationCount)
        //{
        //    bVector3 v = new bVector3(1, 2, 3);
        //    bMatrix3x3 m = bMatrix3x3.Identity;
        //    float accumulator = 0;
        //    for (int i = 0; i < iterationCount; ++i)
        //    {
        //        bVector3 r0, r1;
        //        bMatrix3x3.Transform(ref v, ref m, out r0);
        //        bMatrix3x3.Transform(ref r0, ref m, out r1);
        //        bMatrix3x3.Transform(ref r1, ref m, out r0);
        //        bMatrix3x3.Transform(ref r0, ref m, out r1);
        //        bMatrix3x3.Transform(ref r1, ref m, out r0);
        //        bMatrix3x3.Transform(ref r0, ref m, out r1);
        //        bMatrix3x3.Transform(ref r1, ref m, out r0);
        //        bMatrix3x3.Transform(ref r0, ref m, out r1);
        //        bMatrix3x3.Transform(ref r1, ref m, out r0);
        //        bMatrix3x3.Transform(ref r0, ref m, out r1);
        //        accumulator += 0.000001f * r1.X;
        //    }
        //    return accumulator;
        //}
        public static float TestSIMDTransformTranspose(int iterationCount)
        {
            Vector3 v = new Vector3(1, 2, 3);
            Matrix3x3 m = Matrix3x3.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Vector3 r0, r1;
                Matrix3x3.TransformTranspose(v, m, out r0);
                Matrix3x3.TransformTranspose(r0, m, out r1);
                Matrix3x3.TransformTranspose(r1, m, out r0);
                Matrix3x3.TransformTranspose(r0, m, out r1);
                Matrix3x3.TransformTranspose(r1, m, out r0);
                Matrix3x3.TransformTranspose(r0, m, out r1);
                Matrix3x3.TransformTranspose(r1, m, out r0);
                Matrix3x3.TransformTranspose(r0, m, out r1);
                Matrix3x3.TransformTranspose(r1, m, out r0);
                Matrix3x3.TransformTranspose(r0, m, out r1);
                accumulator += 0.000001f * r1.X;
            }
            return accumulator;
        }

        public static float TestSIMDTransform(int iterationCount)
        {
            Vector3 v = new Vector3(1, 2, 3);
            Matrix3x3 m = Matrix3x3.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Vector3 r0, r1;
                Matrix3x3.Transform(v, m, out r0);
                Matrix3x3.Transform(r0, m, out r1);
                Matrix3x3.Transform(r1, m, out r0);
                Matrix3x3.Transform(r0, m, out r1);
                Matrix3x3.Transform(r1, m, out r0);
                Matrix3x3.Transform(r0, m, out r1);
                Matrix3x3.Transform(r1, m, out r0);
                Matrix3x3.Transform(r0, m, out r1);
                Matrix3x3.Transform(r1, m, out r0);
                Matrix3x3.Transform(r0, m, out r1);
                accumulator += 0.000001f * r1.X;
            }
            return accumulator;
        }

        //public static float TestScalarMultiply(int iterationCount)
        //{
        //    bMatrix3x3 m1 = bMatrix3x3.Identity;
        //    bMatrix3x3 m2 = bMatrix3x3.Identity;
        //    float accumulator = 0;
        //    for (int i = 0; i < iterationCount; ++i)
        //    {
        //        bMatrix3x3 r0, r1;
        //        bMatrix3x3.Multiply(ref m1, ref m2, out r0);
        //        bMatrix3x3.Multiply(ref r0, ref m2, out r1);
        //        bMatrix3x3.Multiply(ref r1, ref m2, out r0);
        //        bMatrix3x3.Multiply(ref r0, ref m2, out r1);
        //        bMatrix3x3.Multiply(ref r1, ref m2, out r0);
        //        bMatrix3x3.Multiply(ref r0, ref m2, out r1);
        //        bMatrix3x3.Multiply(ref r1, ref m2, out r0);
        //        bMatrix3x3.Multiply(ref r0, ref m2, out r1);
        //        bMatrix3x3.Multiply(ref r1, ref m2, out r0);
        //        bMatrix3x3.Multiply(ref r0, ref m2, out r1);
        //        accumulator += 0.000001f * r1.M11;
        //    }
        //    return accumulator;
        //}

        public static float TestSIMDMultiply(int iterationCount)
        {
            Matrix3x3 m1 = Matrix3x3.Identity;
            Matrix3x3 m2 = Matrix3x3.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Matrix3x3 r0, r1;
                Matrix3x3.Multiply(m1, m2, out r0);
                Matrix3x3.Multiply(r0, m2, out r1);
                Matrix3x3.Multiply(r1, m2, out r0);
                Matrix3x3.Multiply(r0, m2, out r1);
                Matrix3x3.Multiply(r1, m2, out r0);
                Matrix3x3.Multiply(r0, m2, out r1);
                Matrix3x3.Multiply(r1, m2, out r0);
                Matrix3x3.Multiply(r0, m2, out r1);
                Matrix3x3.Multiply(r1, m2, out r0);
                Matrix3x3.Multiply(r0, m2, out r1);
                accumulator += 0.000001f * r1.X.X;
            }
            return accumulator;
        }



        //public unsafe static void TestMultiplyCorrectness()
        //{
        //    const int iterationCount = 100000;
        //    Random random = new Random(5);
        //    for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
        //    {
        //        Matrix3x3 simdA, simdB;
        //        bMatrix3x3 scalarA, scalarB;
        //        var simdPointerA = (float*)&simdA;
        //        var scalarPointerA = (float*)&scalarA;
        //        var simdPointerB = (float*)&simdB;
        //        var scalarPointerB = (float*)&scalarB;
        //        for (int i = 0; i < 9; ++i)
        //        {
        //            scalarPointerA[i] = simdPointerA[i] = (float)(random.NextDouble() * 4 - 2);
        //            scalarPointerB[i] = simdPointerB[i] = (float)(random.NextDouble() * 4 - 2);
        //        }

        //        Matrix3x3 simdResult;
        //        Matrix3x3.Multiply(ref simdA, ref simdB, out simdResult);
        //        bMatrix3x3 scalarResult;
        //        bMatrix3x3.Multiply(ref scalarA, ref scalarB, out scalarResult);
        //        var simdPointerResult = (float*)&simdResult;
        //        var scalarPointerResult = (float*)&scalarResult;

        //        for (int i = 0; i < 9; ++i)
        //        {
        //            const float threshold = 1e-5f;
        //            var simdScalarError = Math.Abs(simdPointerResult[i] - scalarPointerResult[i]);
        //            if (simdScalarError > threshold)
        //            {
        //                Console.WriteLine($"Excess error for {i}");
        //            }
        //        }
        //    }

        //    for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
        //    {
        //        Matrix3x3 simdA, simdB;
        //        bMatrix3x3 scalarA, scalarB;
        //        var simdPointerA = (float*)&simdA;
        //        var scalarPointerA = (float*)&scalarA;
        //        var simdPointerB = (float*)&simdB;
        //        var scalarPointerB = (float*)&scalarB;
        //        for (int i = 0; i < 9; ++i)
        //        {
        //            scalarPointerA[i] = simdPointerA[i] = (float)(random.NextDouble() * 4 - 2);
        //            scalarPointerB[i] = simdPointerB[i] = (float)(random.NextDouble() * 4 - 2);
        //        }

        //        Matrix3x3.Multiply(ref simdA, ref simdB, out simdA);
        //        bMatrix3x3.Multiply(ref scalarA, ref scalarB, out scalarA);


        //        for (int i = 0; i < 9; ++i)
        //        {
        //            const float threshold = 1e-5f;
        //            var simdScalarError = Math.Abs(simdPointerA[i] - scalarPointerA[i]);
        //            if (simdScalarError > threshold)
        //            {
        //                Console.WriteLine($"Excess error for {i}");
        //            }
        //        }
        //    }

        //    for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
        //    {
        //        Matrix3x3 simdA, simdB;
        //        bMatrix3x3 scalarA, scalarB;
        //        var simdPointerA = (float*)&simdA;
        //        var scalarPointerA = (float*)&scalarA;
        //        var simdPointerB = (float*)&simdB;
        //        var scalarPointerB = (float*)&scalarB;
        //        for (int i = 0; i < 9; ++i)
        //        {
        //            scalarPointerA[i] = simdPointerA[i] = (float)(random.NextDouble() * 4 - 2);
        //            scalarPointerB[i] = simdPointerB[i] = (float)(random.NextDouble() * 4 - 2);
        //        }

        //        Matrix3x3.Multiply(ref simdA, ref simdB, out simdB);
        //        bMatrix3x3.Multiply(ref scalarA, ref scalarB, out scalarB);


        //        for (int i = 0; i < 9; ++i)
        //        {
        //            const float threshold = 1e-5f;
        //            var simdScalarError = Math.Abs(simdPointerB[i] - scalarPointerB[i]);
        //            if (simdScalarError > threshold)
        //            {
        //                Console.WriteLine($"Excess error for {i}");
        //            }
        //        }
        //    }


        //    for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
        //    {
        //        Matrix3x3 simd;
        //        bMatrix3x3 scalar;
        //        var simdPointer = (float*)&simd;
        //        var scalarPointer = (float*)&scalar;
        //        for (int i = 0; i < 9; ++i)
        //        {
        //            scalarPointer[i] = simdPointer[i] = (float)(random.NextDouble() * 4 - 2);
        //        }

        //        Matrix3x3.Multiply(ref simd, ref simd, out simd);
        //        bMatrix3x3.Multiply(ref scalar, ref scalar, out scalar);

        //        for (int i = 0; i < 9; ++i)
        //        {
        //            const float threshold = 1e-5f;
        //            var simdScalarError = Math.Abs(simdPointer[i] - scalarPointer[i]);
        //            if (simdScalarError > threshold)
        //            {
        //                Console.WriteLine($"Excess error for {i}");
        //            }
        //        }
        //    }
        //}

        public static float TestSIMDTranspose(int iterationCount)
        {
            Matrix3x3 m = Matrix3x3.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Matrix3x3 r0, r1;
                Matrix3x3.Transpose(m, out r0);
                Matrix3x3.Transpose(r0, out r1);
                Matrix3x3.Transpose(r1, out r0);
                Matrix3x3.Transpose(r0, out r1);
                Matrix3x3.Transpose(r1, out r0);
                Matrix3x3.Transpose(r0, out r1);
                Matrix3x3.Transpose(r1, out r0);
                Matrix3x3.Transpose(r0, out r1);
                Matrix3x3.Transpose(r1, out r0);
                Matrix3x3.Transpose(r0, out r1);
                accumulator += r1.X.X;

            }
            return accumulator;
        }

        public unsafe static float TestSIMDScalarPointerTranspose(int iterationCount)
        {
            Matrix3x3 m = Matrix3x3.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Matrix3x3 r0, r1;
                Matrix3x3.Transpose(&m, &r0);
                Matrix3x3.Transpose(&r0, &r1);
                Matrix3x3.Transpose(&r1, &r0);
                Matrix3x3.Transpose(&r0, &r1);
                Matrix3x3.Transpose(&r1, &r0);
                Matrix3x3.Transpose(&r0, &r1);
                Matrix3x3.Transpose(&r1, &r0);
                Matrix3x3.Transpose(&r0, &r1);
                Matrix3x3.Transpose(&r1, &r0);
                Matrix3x3.Transpose(&r0, &r1);
                accumulator += r1.X.X;

            }
            return accumulator;
        }

        //public static float TestScalarTranspose(int iterationCount)
        //{
        //    bMatrix3x3 m = bMatrix3x3.Identity;
        //    float accumulator = 0;
        //    for (int i = 0; i < iterationCount; ++i)
        //    {
        //        bMatrix3x3 r0, r1;
        //        bMatrix3x3.Transpose(ref m, out r0);
        //        bMatrix3x3.Transpose(ref r0, out r1);
        //        bMatrix3x3.Transpose(ref r1, out r0);
        //        bMatrix3x3.Transpose(ref r0, out r1);
        //        bMatrix3x3.Transpose(ref r1, out r0);
        //        bMatrix3x3.Transpose(ref r0, out r1);
        //        bMatrix3x3.Transpose(ref r1, out r0);
        //        bMatrix3x3.Transpose(ref r0, out r1);
        //        bMatrix3x3.Transpose(ref r1, out r0);
        //        bMatrix3x3.Transpose(ref r0, out r1);
        //        accumulator += r1.M11;

        //    }
        //    return accumulator;
        //}


        //public static float TestScalarInvert(int iterationCount)
        //{
        //    bMatrix3x3 m = bMatrix3x3.Identity;
        //    float accumulator = 0;
        //    for (int i = 0; i < iterationCount; ++i)
        //    {
        //        bMatrix3x3 r0, r1;
        //        bMatrix3x3.Invert(ref m, out r0);
        //        bMatrix3x3.Invert(ref r0, out r1);
        //        bMatrix3x3.Invert(ref r1, out r0);
        //        bMatrix3x3.Invert(ref r0, out r1);
        //        bMatrix3x3.Invert(ref r1, out r0);
        //        bMatrix3x3.Invert(ref r0, out r1);
        //        bMatrix3x3.Invert(ref r1, out r0);
        //        bMatrix3x3.Invert(ref r0, out r1);
        //        bMatrix3x3.Invert(ref r1, out r0);
        //        bMatrix3x3.Invert(ref r0, out r1);
        //        accumulator += r1.M11;

        //    }
        //    return accumulator;
        //}
        public static float TestSIMDInvert(int iterationCount)
        {
            Matrix3x3 m = Matrix3x3.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Matrix3x3 r0, r1;
                Matrix3x3.Invert(m, out r0);
                Matrix3x3.Invert(r0, out r1);
                Matrix3x3.Invert(r1, out r0);
                Matrix3x3.Invert(r0, out r1);
                Matrix3x3.Invert(r1, out r0);
                Matrix3x3.Invert(r0, out r1);
                Matrix3x3.Invert(r1, out r0);
                Matrix3x3.Invert(r0, out r1);
                Matrix3x3.Invert(r1, out r0);
                Matrix3x3.Invert(r0, out r1);
                accumulator += r1.X.X;

            }
            return accumulator;
        }
        public unsafe static float TestSIMDScalarInvert(int iterationCount)
        {
            Matrix3x3 m = Matrix3x3.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Matrix3x3 r0, r1;
                Matrix3x3.Invert(&m, &r0);
                Matrix3x3.Invert(&r0, &r1);
                Matrix3x3.Invert(&r1, &r0);
                Matrix3x3.Invert(&r0, &r1);
                Matrix3x3.Invert(&r1, &r0);
                Matrix3x3.Invert(&r0, &r1);
                Matrix3x3.Invert(&r1, &r0);
                Matrix3x3.Invert(&r0, &r1);
                Matrix3x3.Invert(&r1, &r0);
                Matrix3x3.Invert(&r0, &r1);
                accumulator += r1.X.X;

            }
            return accumulator;
        }

        //public unsafe static void TestInversionCorrectness()
        //{
        //    Random random = new Random(5);
        //    for (int iterationIndex = 0; iterationIndex < 1000; ++iterationIndex)
        //    {
        //        bMatrix3x3 scalar;
        //        Matrix3x3 simd;
        //        Matrix3x3 simdScalar;
        //        var scalarPointer = (float*)&scalar;
        //        var simdPointer = (float*)&simd;
        //        var simdScalarPointer = (float*)&simdScalar;

        //        //Create a guaranteed invertible matrix.
        //        scalar = bMatrix3x3.CreateFromAxisAngle(
        //            bVector3.Normalize(new bVector3(
        //                0.1f + (float)random.NextDouble(),
        //                0.1f + (float)random.NextDouble(),
        //                0.1f + (float)random.NextDouble())),
        //            (float)random.NextDouble());

        //        for (int i = 0; i < 9; ++i)
        //        {
        //            simdScalarPointer[i] = simdPointer[i] = scalarPointer[i];
        //        }


        //        bMatrix3x3.Invert(ref scalar, out scalar);
        //        Matrix3x3.Invert(ref simd, out simd);
        //        Matrix3x3.Invert(&simdScalar, &simdScalar);

        //        for (int i = 0; i < 9; ++i)
        //        {
        //            var errorSimd = Math.Abs(simdPointer[i] - scalarPointer[i]);
        //            var errorSimdScalar = Math.Abs(simdScalarPointer[i] - scalarPointer[i]);
        //            Assert.IsTrue(errorSimd < 1e-5f);
        //            Assert.IsTrue(errorSimdScalar < 1e-5f);
        //        }
        //    }
        //}


        public unsafe static void Test()
        {
            Console.WriteLine("MATRIX3x3 RESULTS:");
            Console.WriteLine($"Size: {sizeof(Matrix3x3)}");
            //TestMultiplyCorrectness();
            //TestInversionCorrectness();
            const int iterationCount = 10000000;

            Helper.Test("Invert SIMD", TestSIMDInvert, iterationCount);
            Helper.Test("Invert SIMDScalar", TestSIMDScalarInvert, iterationCount);
            //Helper.Test("Invert Scalar", TestScalarInvert, iterationCount);

            Helper.Test("Transpose SIMD", TestSIMDTranspose, iterationCount);
            Helper.Test("Transpose SIMDscalarpointer", TestSIMDScalarPointerTranspose, iterationCount);
            //Helper.Test("Transpose Scalar", TestScalarTranspose, iterationCount);

            Helper.Test("Multiply SIMD", TestSIMDMultiply, iterationCount);
            //Helper.Test("Multiply Scalar", TestScalarMultiply, iterationCount);

            Helper.Test("Transform SIMD", TestSIMDTransform, iterationCount);
            Helper.Test("TransformTranspose SIMD", TestSIMDTransformTranspose, iterationCount);
            //Helper.Test("Transform Scalar", TestTransformScalar, iterationCount);

        }
    }
}
