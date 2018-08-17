using BepuUtilities;
using BepuPhysics;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using Quaternion = BepuUtilities.Quaternion;

namespace Demos.SpecializedTests
{
    public static class MathTests
    {
        public interface IRandomizedTest
        {
            void Test(Random random, int innerIterations);
        }


        [MethodImpl(MethodImplOptions.NoInlining)]
        public static double Test<T>(int iterations, int innerIterations) where T : struct, IRandomizedTest
        {
            var random = new Random(5);
            var test = default(T);
            var start = Stopwatch.GetTimestamp();
            for (int i = 0; i < iterations; ++i)
            {
                test.Test(random, innerIterations);
            }
            var end = Stopwatch.GetTimestamp();
            return (end - start) / (double)Stopwatch.Frequency;
        }


        struct QuaternionToMatrixTest : IRandomizedTest
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Test(Random random, int innerIterations)
            {
                Quaternion q;
                q.X = (float)random.NextDouble() * 2 - 1;
                q.Y = (float)random.NextDouble() * 2 - 1;
                q.Z = (float)random.NextDouble() * 2 - 1;
                q.W = (float)random.NextDouble() * 2 - 1;

                Quaternion.Normalize(ref q);

                for (int i = 0; i < innerIterations; ++i)
                {
                    Matrix3x3.CreateFromQuaternion(q, out var r);
                    Quaternion.CreateFromRotationMatrix(r, out var qTest);

#if DEBUG
                    const float epsilon = 1e-6f;
                    var lengthX = r.X.Length();
                    var lengthY = r.Y.Length();
                    var lengthZ = r.Z.Length();
                    Debug.Assert(
                        Math.Abs(1 - lengthX) < epsilon &&
                        Math.Abs(1 - lengthY) < epsilon &&
                        Math.Abs(1 - lengthZ) < epsilon);


                    if (qTest.X * q.X < 0)
                    {
                        Quaternion.Negate(qTest, out qTest);
                    }
                    Debug.Assert(
                        Math.Abs(qTest.X - q.X) < epsilon &&
                        Math.Abs(qTest.Y - q.Y) < epsilon &&
                        Math.Abs(qTest.Z - q.Z) < epsilon &&
                        Math.Abs(qTest.W - q.W) < epsilon);
#endif
                }
            }
        }

        struct QuaternionToMatrixTestSIMD : IRandomizedTest
        {

            [MethodImpl(MethodImplOptions.NoInlining)]
            public void Test(Random random, int innerIterations)
            {
                QuaternionWide q;
                q.X = new Vector<float>((float)random.NextDouble() * 2 - 1);
                q.Y = new Vector<float>((float)random.NextDouble() * 2 - 1);
                q.Z = new Vector<float>((float)random.NextDouble() * 2 - 1);
                q.W = new Vector<float>((float)random.NextDouble() * 2 - 1);

                QuaternionWide.Normalize(q, out q);

                for (int i = 0; i < innerIterations; ++i)
                {
                    Matrix3x3Wide.CreateFromQuaternion(q, out var r);
                    QuaternionWide.CreateFromRotationMatrix(r, out var qTest);
#if DEBUG
                    const float epsilon = 1e-6f;
                    Vector3Wide.Length(r.X, out var lengthX);
                    Vector3Wide.Length(r.Y, out var lengthY);
                    Vector3Wide.Length(r.Z, out var lengthZ);
                    Debug.Assert(
                        Vector.LessThanAll(Vector.Abs(Vector<float>.One - lengthX), new Vector<float>(epsilon)) &&
                        Vector.LessThanAll(Vector.Abs(Vector<float>.One - lengthY), new Vector<float>(epsilon)) &&
                        Vector.LessThanAll(Vector.Abs(Vector<float>.One - lengthZ), new Vector<float>(epsilon)));


                    if (qTest.X[0] * q.X[0] < 0)
                    {
                        QuaternionWide.Negate(qTest, out qTest);
                    }
                    Debug.Assert(
                        Vector.LessThanAll(Vector.Abs(qTest.X - q.X), new Vector<float>(epsilon)) &&
                        Vector.LessThanAll(Vector.Abs(qTest.Y - q.Y), new Vector<float>(epsilon)) &&
                        Vector.LessThanAll(Vector.Abs(qTest.Z - q.Z), new Vector<float>(epsilon)) &&
                        Vector.LessThanAll(Vector.Abs(qTest.W - q.W), new Vector<float>(epsilon)));
#endif
                }
            }
        }


        struct RotationBetweenNormalsTestSIMD : IRandomizedTest
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void GenerateRandomVectorCandidate(Random random, out Vector3Wide candidate)
            {
                candidate.X = new Vector<float>((float)random.NextDouble());
                candidate.Y = new Vector<float>((float)random.NextDouble());
                candidate.Z = new Vector<float>((float)random.NextDouble());
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void GenerateRandomNormalizedVector(Random random, out Vector3Wide normal)
            {
                GenerateRandomVectorCandidate(random, out normal);
                while (true)
                {
                    Vector3Wide.LengthSquared(normal, out var lengthSquared);
                    if (Vector.LessThanAny(lengthSquared, new Vector<float>(0.0001f)))
                    {
                        GenerateRandomVectorCandidate(random, out normal);
                    }
                    else
                        break;
                }
                Vector3Wide.Normalize(normal, out normal);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Test(Random random, int innerIterations)
            {
                GenerateRandomNormalizedVector(random, out var v1);
                GenerateRandomNormalizedVector(random, out var v2);

                for (int i = 0; i < innerIterations; ++i)
                {
                    QuaternionWide.GetQuaternionBetweenNormalizedVectors(v1, v2, out var v1ToV2);
                    QuaternionWide.GetQuaternionBetweenNormalizedVectors(v2, v1, out var v2ToV1);
#if DEBUG
                    QuaternionWide.ConcatenateWithoutOverlap(v1ToV2, v2ToV1, out var concatenated);
                    QuaternionWide.TransformWithoutOverlap(v1, v1ToV2, out var v1TransformedToV2);
                    QuaternionWide.TransformWithoutOverlap(v2, v2ToV1, out var v2TransformedToV1);
                    QuaternionWide.TransformWithoutOverlap(v1, concatenated, out var v1TransformedToV1);

                    Vector3Wide.Subtract(v1TransformedToV2, v2, out var v1ToV2Error);
                    Vector3Wide.LengthSquared(v1ToV2Error, out var v1ToV2ErrorLength);
                    Vector3Wide.Subtract(v2TransformedToV1, v1, out var v2ToV1Error);
                    Vector3Wide.LengthSquared(v2ToV1Error, out var v2ToV1ErrorLength);
                    Vector3Wide.Subtract(v1TransformedToV1, v1, out var v1ToV1Error);
                    Vector3Wide.LengthSquared(v1ToV1Error, out var v1ToV1ErrorLength);
                    const float epsilon = 1e-6f;
                    Debug.Assert(
                        Vector.LessThanAll(v1ToV2ErrorLength, new Vector<float>(epsilon)) &&
                        Vector.LessThanAll(v2ToV1ErrorLength, new Vector<float>(epsilon)) &&
                        Vector.LessThanAll(v1ToV1ErrorLength, new Vector<float>(epsilon)));
#endif
                }


            }
        }

        struct RotationBetweenNormalsCornerTestSIMD : IRandomizedTest
        {

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Test(Random random, int innerIterations)
            {
                RotationBetweenNormalsTestSIMD.GenerateRandomNormalizedVector(random, out var v1);
                Vector3Wide.Negate(v1, out var v2);

                for (int i = 0; i < innerIterations; ++i)
                {
                    QuaternionWide.GetQuaternionBetweenNormalizedVectors(v1, v2, out var v1ToV2);
                    QuaternionWide.GetQuaternionBetweenNormalizedVectors(v2, v1, out var v2ToV1);
#if DEBUG
                    QuaternionWide.ConcatenateWithoutOverlap(v1ToV2, v2ToV1, out var concatenated);
                    QuaternionWide.TransformWithoutOverlap(v1, v1ToV2, out var v1TransformedToV2);
                    QuaternionWide.TransformWithoutOverlap(v2, v2ToV1, out var v2TransformedToV1);
                    QuaternionWide.TransformWithoutOverlap(v1, concatenated, out var v1TransformedToV1);

                    Vector3Wide.Subtract(v1TransformedToV2, v2, out var v1ToV2Error);
                    Vector3Wide.LengthSquared(v1ToV2Error, out var v1ToV2ErrorLength);
                    Vector3Wide.Subtract(v2TransformedToV1, v1, out var v2ToV1Error);
                    Vector3Wide.LengthSquared(v2ToV1Error, out var v2ToV1ErrorLength);
                    Vector3Wide.Subtract(v1TransformedToV1, v1, out var v1ToV1Error);
                    Vector3Wide.LengthSquared(v1ToV1Error, out var v1ToV1ErrorLength);
                    const float epsilon = 1e-6f;
                    Debug.Assert(
                        Vector.LessThanAll(v1ToV2ErrorLength, new Vector<float>(epsilon)) &&
                        Vector.LessThanAll(v2ToV1ErrorLength, new Vector<float>(epsilon)) &&
                        Vector.LessThanAll(v1ToV1ErrorLength, new Vector<float>(epsilon)));
#endif

                }


            }
        }

        struct RotationBetweenNormalsTest : IRandomizedTest
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void GenerateRandomVectorCandidate(Random random, out Vector3 candidate)
            {
                candidate.X = (float)random.NextDouble();
                candidate.Y = (float)random.NextDouble();
                candidate.Z = (float)random.NextDouble();
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void GenerateRandomNormalizedVector(Random random, out Vector3 normal)
            {
                GenerateRandomVectorCandidate(random, out normal);
                while (true)
                {
                    if (normal.LengthSquared() < 0.0001f)
                    {
                        GenerateRandomVectorCandidate(random, out normal);
                    }
                    else
                        break;
                }
                normal = Vector3.Normalize(normal);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Test(Random random, int innerIterations)
            {
                GenerateRandomNormalizedVector(random, out var v1);
                GenerateRandomNormalizedVector(random, out var v2);

                for (int i = 0; i < innerIterations; ++i)
                {
                    Quaternion.GetQuaternionBetweenNormalizedVectors(v1, v2, out var v1ToV2);
                    Quaternion.GetQuaternionBetweenNormalizedVectors(v2, v1, out var v2ToV1);

#if DEBUG
                    Quaternion.ConcatenateWithoutOverlap(v1ToV2, v2ToV1, out var concatenated);
                    Quaternion.Transform(v1, v1ToV2, out var v1TransformedToV2);
                    Quaternion.Transform(v2, v2ToV1, out var v2TransformedToV1);
                    Quaternion.Transform(v1, concatenated, out var v1TransformedToV1);


                    var v1ToV2ErrorLength = (v1TransformedToV2 - v2).LengthSquared();
                    var v2ToV1ErrorLength = (v2TransformedToV1 - v1).LengthSquared();
                    var v1ToV1ErrorLength = (v1TransformedToV1 - v1).LengthSquared();
                    const float epsilon = 1e-6f;
                    Debug.Assert(
                        v1ToV2ErrorLength < epsilon &&
                        v2ToV1ErrorLength < epsilon &&
                        v1ToV1ErrorLength < epsilon);
#endif
                }


            }
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void Test()
        {
            const int iterations = 10000;
            const int innerIterations = 1000;
            Console.WriteLine($"Q<->M Scalar: {Test<QuaternionToMatrixTest>(iterations, innerIterations)}");
            Console.WriteLine($"Q<->M SIMD: {Test<QuaternionToMatrixTestSIMD>(iterations, innerIterations)}");
            Console.WriteLine($"Rotation between normals SIMD: {Test<RotationBetweenNormalsTestSIMD>(iterations, innerIterations)}");
            Console.WriteLine($"Rotation between normals cornercase SIMD: {Test<RotationBetweenNormalsCornerTestSIMD>(iterations, innerIterations)}");
            Console.WriteLine($"Rotation between normals scalar: {Test<RotationBetweenNormalsTest>(iterations, innerIterations)}");
        }
    }
}
