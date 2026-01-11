using System;
using System.Numerics;
using BepuUtilities;
using Xunit;

namespace SimdTests
{
    /// <summary>
    /// Correctness tests that verify SIMD implementations match scalar reference implementations.
    /// </summary>
    public class CorrectnessTests
    {
        private const float Epsilon = 1e-5f; // Tolerance for floating-point comparisons

        /// <summary>
        /// Helper to compare two vectors element-wise within epsilon tolerance.
        /// </summary>
        private static void AssertVectorsEqual(Vector<float> expected, Vector<float> actual, string message)
        {
            for (int i = 0; i < Vector<float>.Count; i++)
            {
                Assert.True(
                    MathF.Abs(expected[i] - actual[i]) < Epsilon,
                    $"{message} - Mismatch at index {i}: expected {expected[i]}, got {actual[i]}"
                );
            }
        }

        /// <summary>
        /// Helper to compare two integer vectors element-wise.
        /// </summary>
        private static void AssertIntVectorsEqual(Vector<int> expected, Vector<int> actual, string message)
        {
            for (int i = 0; i < Vector<int>.Count; i++)
            {
                Assert.Equal(expected[i], actual[i]);
            }
        }

        [Fact]
        public void TestFastReciprocal_RandomValues()
        {
            var random = new Random(12345);

            for (int test = 0; test < 100; test++)
            {
                var values = new float[Vector<float>.Count];
                for (int i = 0; i < Vector<float>.Count; i++)
                {
                    // Generate random values between 0.1 and 100 to avoid division by near-zero
                    values[i] = (float)(random.NextDouble() * 99.9 + 0.1);
                }

                var input = new Vector<float>(values);
                var expected = ReferenceImplementations.FastReciprocal_Reference(input);
                var actual = MathHelper.FastReciprocal(input);

                // Note: Hardware reciprocal estimates may have lower precision, so we use a relaxed epsilon
                for (int i = 0; i < Vector<float>.Count; i++)
                {
                    var relativeError = MathF.Abs((expected[i] - actual[i]) / expected[i]);
                    Assert.True(
                        relativeError < 0.001f, // 0.1% relative error tolerance
                        $"FastReciprocal mismatch at test {test}, index {i}: expected {expected[i]}, got {actual[i]}, relative error {relativeError}"
                    );
                }
            }
        }

        [Fact]
        public void TestFastReciprocal_SpecialValues()
        {
            // Test with specific values
            var testValues = new[]
            {
                new Vector<float>(1.0f),
                new Vector<float>(2.0f),
                new Vector<float>(0.5f),
                new Vector<float>(100.0f),
            };

            foreach (var input in testValues)
            {
                var expected = ReferenceImplementations.FastReciprocal_Reference(input);
                var actual = MathHelper.FastReciprocal(input);

                for (int i = 0; i < Vector<float>.Count; i++)
                {
                    var relativeError = MathF.Abs((expected[i] - actual[i]) / expected[i]);
                    Assert.True(relativeError < 0.001f);
                }
            }
        }

        [Fact]
        public void TestFastReciprocalSquareRoot_RandomValues()
        {
            var random = new Random(12345);

            for (int test = 0; test < 100; test++)
            {
                var values = new float[Vector<float>.Count];
                for (int i = 0; i < Vector<float>.Count; i++)
                {
                    // Generate random positive values
                    values[i] = (float)(random.NextDouble() * 99.9 + 0.1);
                }

                var input = new Vector<float>(values);
                var expected = ReferenceImplementations.FastReciprocalSquareRoot_Reference(input);
                var actual = MathHelper.FastReciprocalSquareRoot(input);

                for (int i = 0; i < Vector<float>.Count; i++)
                {
                    var relativeError = MathF.Abs((expected[i] - actual[i]) / expected[i]);
                    Assert.True(
                        relativeError < 0.001f,
                        $"FastReciprocalSquareRoot mismatch at test {test}, index {i}: expected {expected[i]}, got {actual[i]}, relative error {relativeError}"
                    );
                }
            }
        }

        [Fact]
        public void TestCreateTrailingMaskForCountInBundle()
        {
            // Test all possible count values
            for (int count = 0; count <= Vector<int>.Count; count++)
            {
                var expected = ReferenceImplementations.CreateTrailingMaskForCountInBundle_Reference(count);
                var actual = BundleIndexing.CreateTrailingMaskForCountInBundle(count);

                AssertIntVectorsEqual(expected, actual, $"CreateTrailingMaskForCountInBundle with count={count}");
            }
        }

        [Fact]
        public void TestCreateMaskForCountInBundle()
        {
            // Test all possible count values
            for (int count = 0; count <= Vector<int>.Count; count++)
            {
                var expected = ReferenceImplementations.CreateMaskForCountInBundle_Reference(count);
                var actual = BundleIndexing.CreateMaskForCountInBundle(count);

                AssertIntVectorsEqual(expected, actual, $"CreateMaskForCountInBundle with count={count}");
            }
        }

        [Fact]
        public void TestGetFirstSetLaneIndex()
        {
            // Test various mask patterns
            var testCases = new[]
            {
                (CreateMask(new[] { -1, 0, 0, 0, 0, 0, 0, 0 }), 0), // First lane set
                (CreateMask(new[] { 0, -1, 0, 0, 0, 0, 0, 0 }), 1), // Second lane set
                (CreateMask(new[] { 0, 0, 0, -1, 0, 0, 0, 0 }), 3), // Middle lane set
                (CreateMask(new[] { 0, 0, 0, 0, 0, 0, 0, 0 }), -1), // No lanes set
                (CreateMask(new[] { -1, -1, -1, -1, -1, -1, -1, -1 }), 0), // All lanes set
            };

            foreach (var (mask, expectedIndex) in testCases)
            {
                var expected = ReferenceImplementations.GetFirstSetLaneIndex_Reference(mask);
                var actual = BundleIndexing.GetFirstSetLaneIndex(mask);

                Assert.Equal(expectedIndex, expected);
                Assert.Equal(expected, actual);
            }
        }

        [Fact]
        public void TestGetLastSetLaneCount()
        {
            // Test various mask patterns
            var testCases = new[]
            {
                (CreateMask(new[] { 0, 0, 0, 0, 0, 0, 0, 0 }), 0), // No lanes set
                (CreateMask(new[] { 0, 0, 0, 0, 0, 0, 0, -1 }), 1), // Last lane set
                (CreateMask(new[] { 0, 0, 0, 0, 0, 0, -1, -1 }), 2), // Last two lanes set
                (CreateMask(new[] { -1, -1, -1, -1, -1, -1, -1, -1 }), 8), // All lanes set
                (CreateMask(new[] { -1, -1, 0, 0, 0, 0, -1, -1 }), 2), // Only last two consecutive
            };

            foreach (var (mask, expectedCount) in testCases)
            {
                var expected = ReferenceImplementations.GetLastSetLaneCount_Reference(mask);
                var actual = BundleIndexing.GetLastSetLaneCount(mask);

                Assert.Equal(expectedCount, expected);
                Assert.Equal(expected, actual);
            }
        }

        /// <summary>
        /// Helper to create a mask vector from an array (handles both Vector4 and Vector8).
        /// </summary>
        private static Vector<int> CreateMask(int[] values)
        {
            var result = new int[Vector<int>.Count];
            for (int i = 0; i < Math.Min(values.Length, Vector<int>.Count); i++)
            {
                result[i] = values[i];
            }
            return new Vector<int>(result);
        }
    }
}
