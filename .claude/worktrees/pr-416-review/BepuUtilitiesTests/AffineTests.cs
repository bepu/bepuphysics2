using BepuUtilities;
using System;
using System.Numerics;

//using bAffineTransform = BEPUutilities.AffineTransform;
//using bMatrix3x3 = BEPUutilities.Matrix3x3;
//using bVector3 = BEPUutilities.Vector3;

namespace BEPUutilitiesTests
{
    public static class AffineTests
    {

        //public static float TestTransformScalar(int iterationCount)
        //{
        //    bVector3 v = new bVector3(1, 2, 3);
        //    bAffineTransform m = bAffineTransform.Identity;
        //    float accumulator = 0;
        //    for (int i = 0; i < iterationCount; ++i)
        //    {
        //        bVector3 r0, r1;
        //        bAffineTransform.Transform(ref v, ref m, out r0);
        //        bAffineTransform.Transform(ref r0, ref m, out r1);
        //        bAffineTransform.Transform(ref r1, ref m, out r0);
        //        bAffineTransform.Transform(ref r0, ref m, out r1);
        //        bAffineTransform.Transform(ref r1, ref m, out r0);
        //        bAffineTransform.Transform(ref r0, ref m, out r1);
        //        bAffineTransform.Transform(ref r1, ref m, out r0);
        //        bAffineTransform.Transform(ref r0, ref m, out r1);
        //        bAffineTransform.Transform(ref r1, ref m, out r0);
        //        bAffineTransform.Transform(ref r0, ref m, out r1);
        //        accumulator += 0.000001f * r1.X;
        //    }
        //    return accumulator;
        //}


        public static float TestSIMDTransform(int iterationCount)
        {
            Vector3 v = new Vector3(1, 2, 3);
            AffineTransform m = AffineTransform.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Vector3 r0, r1;
                AffineTransform.Transform(v, m, out r0);
                AffineTransform.Transform(r0, m, out r1);
                AffineTransform.Transform(r1, m, out r0);
                AffineTransform.Transform(r0, m, out r1);
                AffineTransform.Transform(r1, m, out r0);
                AffineTransform.Transform(r0, m, out r1);
                AffineTransform.Transform(r1, m, out r0);
                AffineTransform.Transform(r0, m, out r1);
                AffineTransform.Transform(r1, m, out r0);
                AffineTransform.Transform(r0, m, out r1);
                accumulator += 0.000001f * r1.X;
            }
            return accumulator;
        }

        //public static float TestScalarMultiply(int iterationCount)
        //{
        //    bAffineTransform m1 = bAffineTransform.Identity;
        //    bAffineTransform m2 = bAffineTransform.Identity;
        //    float accumulator = 0;
        //    for (int i = 0; i < iterationCount; ++i)
        //    {
        //        bAffineTransform r0, r1;
        //        bAffineTransform.Multiply(ref m1, ref m2, out r0);
        //        bAffineTransform.Multiply(ref r0, ref m2, out r1);
        //        bAffineTransform.Multiply(ref r1, ref m2, out r0);
        //        bAffineTransform.Multiply(ref r0, ref m2, out r1);
        //        bAffineTransform.Multiply(ref r1, ref m2, out r0);
        //        bAffineTransform.Multiply(ref r0, ref m2, out r1);
        //        bAffineTransform.Multiply(ref r1, ref m2, out r0);
        //        bAffineTransform.Multiply(ref r0, ref m2, out r1);
        //        bAffineTransform.Multiply(ref r1, ref m2, out r0);
        //        bAffineTransform.Multiply(ref r0, ref m2, out r1);
        //        accumulator += 0.000001f * r1.Translation.X;
        //    }
        //    return accumulator;
        //}

        public static float TestSIMDMultiply(int iterationCount)
        {
            AffineTransform m1 = AffineTransform.Identity;
            AffineTransform m2 = AffineTransform.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                AffineTransform r0, r1;
                AffineTransform.Multiply(m1, m2, out r0);
                AffineTransform.Multiply(r0, m2, out r1);
                AffineTransform.Multiply(r1, m2, out r0);
                AffineTransform.Multiply(r0, m2, out r1);
                AffineTransform.Multiply(r1, m2, out r0);
                AffineTransform.Multiply(r0, m2, out r1);
                AffineTransform.Multiply(r1, m2, out r0);
                AffineTransform.Multiply(r0, m2, out r1);
                AffineTransform.Multiply(r1, m2, out r0);
                AffineTransform.Multiply(r0, m2, out r1);
                accumulator += 0.000001f * r1.Translation.X;
            }
            return accumulator;
        }



        //public unsafe static void TestMultiplyCorrectness()
        //{
        //    const int iterationCount = 100000;
        //    Random random = new Random(5);
        //    for (int iterationIndex = 0; iterationIndex < iterationCount; ++iterationIndex)
        //    {
        //        AffineTransform simdA, simdB;
        //        bAffineTransform scalarA, scalarB;
        //        var simdPointerA = (float*)&simdA;
        //        var scalarPointerA = (float*)&scalarA;
        //        var simdPointerB = (float*)&simdB;
        //        var scalarPointerB = (float*)&scalarB;
        //        for (int i = 0; i < 12; ++i)
        //        {
        //            scalarPointerA[i] = simdPointerA[i] = (float)(random.NextDouble() * 4 - 2);
        //            scalarPointerB[i] = simdPointerB[i] = (float)(random.NextDouble() * 4 - 2);
        //        }

        //        AffineTransform simdResult;
        //        AffineTransform.Multiply(ref simdA, ref simdB, out simdResult);
        //        bAffineTransform scalarResult;
        //        bAffineTransform.Multiply(ref scalarA, ref scalarB, out scalarResult);
        //        var simdPointerResult = (float*)&simdResult;
        //        var scalarPointerResult = (float*)&scalarResult;

        //        for (int i = 0; i < 12; ++i)
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
        //        AffineTransform simdA, simdB;
        //        bAffineTransform scalarA, scalarB;
        //        var simdPointerA = (float*)&simdA;
        //        var scalarPointerA = (float*)&scalarA;
        //        var simdPointerB = (float*)&simdB;
        //        var scalarPointerB = (float*)&scalarB;
        //        for (int i = 0; i < 12; ++i)
        //        {
        //            scalarPointerA[i] = simdPointerA[i] = (float)(random.NextDouble() * 4 - 2);
        //            scalarPointerB[i] = simdPointerB[i] = (float)(random.NextDouble() * 4 - 2);
        //        }

        //        AffineTransform.Multiply(ref simdA, ref simdB, out simdA);
        //        bAffineTransform.Multiply(ref scalarA, ref scalarB, out scalarA);


        //        for (int i = 0; i < 12; ++i)
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
        //        AffineTransform simdA, simdB;
        //        bAffineTransform scalarA, scalarB;
        //        var simdPointerA = (float*)&simdA;
        //        var scalarPointerA = (float*)&scalarA;
        //        var simdPointerB = (float*)&simdB;
        //        var scalarPointerB = (float*)&scalarB;
        //        for (int i = 0; i < 12; ++i)
        //        {
        //            scalarPointerA[i] = simdPointerA[i] = (float)(random.NextDouble() * 4 - 2);
        //            scalarPointerB[i] = simdPointerB[i] = (float)(random.NextDouble() * 4 - 2);
        //        }

        //        AffineTransform.Multiply(ref simdA, ref simdB, out simdB);
        //        bAffineTransform.Multiply(ref scalarA, ref scalarB, out scalarB);


        //        for (int i = 0; i < 12; ++i)
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
        //        AffineTransform simd;
        //        bAffineTransform scalar;
        //        var simdPointer = (float*)&simd;
        //        var scalarPointer = (float*)&scalar;
        //        for (int i = 0; i < 12; ++i)
        //        {
        //            scalarPointer[i] = simdPointer[i] = (float)(random.NextDouble() * 4 - 2);
        //        }

        //        AffineTransform.Multiply(ref simd, ref simd, out simd);
        //        bAffineTransform.Multiply(ref scalar, ref scalar, out scalar);

        //        for (int i = 0; i < 12; ++i)
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



        //public static float TestScalarInvert(int iterationCount)
        //{
        //    bAffineTransform m = bAffineTransform.Identity;
        //    float accumulator = 0;
        //    for (int i = 0; i < iterationCount; ++i)
        //    {
        //        bAffineTransform r0, r1;
        //        bAffineTransform.Invert(ref m, out r0);
        //        bAffineTransform.Invert(ref r0, out r1);
        //        bAffineTransform.Invert(ref r1, out r0);
        //        bAffineTransform.Invert(ref r0, out r1);
        //        bAffineTransform.Invert(ref r1, out r0);
        //        bAffineTransform.Invert(ref r0, out r1);
        //        bAffineTransform.Invert(ref r1, out r0);
        //        bAffineTransform.Invert(ref r0, out r1);
        //        bAffineTransform.Invert(ref r1, out r0);
        //        bAffineTransform.Invert(ref r0, out r1);
        //        accumulator += r1.Translation.X;

        //    }
        //    return accumulator;
        //}
        public static float TestSIMDInvert(int iterationCount)
        {
            AffineTransform m = AffineTransform.Identity;
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                AffineTransform r0, r1;
                AffineTransform.Invert(m, out r0);
                AffineTransform.Invert(r0, out r1);
                AffineTransform.Invert(r1, out r0);
                AffineTransform.Invert(r0, out r1);
                AffineTransform.Invert(r1, out r0);
                AffineTransform.Invert(r0, out r1);
                AffineTransform.Invert(r1, out r0);
                AffineTransform.Invert(r0, out r1);
                AffineTransform.Invert(r1, out r0);
                AffineTransform.Invert(r0, out r1);
                accumulator += r1.Translation.X;

            }
            return accumulator;
        }

        //public unsafe static void TestInversionCorrectness()
        //{
        //    Random random = new Random(5);
        //    for (int iterationIndex = 0; iterationIndex < 1000; ++iterationIndex)
        //    {
        //        bAffineTransform scalar;
        //        AffineTransform simd;
        //        var scalarPointer = (float*)&scalar;
        //        var simdPointer = (float*)&simd;

        //        //Create a guaranteed invertible transform.
        //        scalar.LinearTransform = bMatrix3x3.CreateFromAxisAngle(
        //            bVector3.Normalize(new bVector3(
        //                0.1f + (float)random.NextDouble(),
        //                0.1f + (float)random.NextDouble(),
        //                0.1f + (float)random.NextDouble())),
        //            (float)random.NextDouble());
        //        scalar.Translation = new bVector3((float)random.NextDouble() * 10, (float)random.NextDouble() * 10, (float)random.NextDouble() * 10);

        //        for (int i = 0; i < 12; ++i)
        //        {
        //            simdPointer[i] = scalarPointer[i];
        //        }


        //        bAffineTransform.Invert(ref scalar, out scalar);
        //        AffineTransform.Invert(ref simd, out simd);

        //        for (int i = 0; i < 12; ++i)
        //        {
        //            var errorSimd = Math.Abs(simdPointer[i] - scalarPointer[i]);
        //            Assert.IsTrue(errorSimd < 1e-5f);
        //        }
        //    }
        //}


        public unsafe static void Test()
        {
            Console.WriteLine("AFFINETRANSFORM RESULTS:");
            Console.WriteLine($"Size: {sizeof(AffineTransform)}");
            //TestMultiplyCorrectness();
            //TestInversionCorrectness();
            const int iterationCount = 10000000;

            Helper.Test("Invert SIMD", TestSIMDInvert, iterationCount);
            //Helper.Test("Invert Scalar", TestScalarInvert, iterationCount);

            Helper.Test("Multiply SIMD", TestSIMDMultiply, iterationCount);
            //Helper.Test("Multiply Scalar", TestScalarMultiply, iterationCount);

            Helper.Test("Transform SIMD", TestSIMDTransform, iterationCount);
            //Helper.Test("Transform Scalar", TestTransformScalar, iterationCount);

        }
    }
}
