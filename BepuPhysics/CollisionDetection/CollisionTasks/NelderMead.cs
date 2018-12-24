using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public interface IDepthTester<TShapeA, TShapeWideA, TShapeB, TShapeWideB>
        where TShapeA : IConvexShape
        where TShapeWideA : IShapeWide<TShapeA>
        where TShapeB : IConvexShape
        where TShapeWideB : IShapeWide<TShapeB>
    {
        /// <summary>
        /// Computes the interval overlap along the given normal.
        /// </summary>
        /// <param name="normal">Normal to test the depth along.</param>
        /// <param name="scaledDepth">Depth multiplied by the squared normal length.</param>
        void Test(in TShapeWideA a, in TShapeWideA b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB, in Vector3Wide normal, out Vector<float> scaledDepth);
    }

    public static class NelderMead<TShapeA, TShapeWideA, TShapeB, TShapeWideB, TDepthTester>
        where TShapeA : IConvexShape
        where TShapeWideA : IShapeWide<TShapeA>
        where TShapeB : IConvexShape
        where TShapeWideB : IShapeWide<TShapeB>
        where TDepthTester : struct, IDepthTester<TShapeA, TShapeWideA, TShapeB, TShapeWideB>
    {
        internal struct SimplexEntry
        {
            public Vector2Wide Point;
            //To avoid divisions and square roots, the depth will be tracked without normalizing the normals at each step:
            //depth(N) = dot(N / ||N||, extremePoint) 
            //depth(N)^2 = dot(N, extremePoint)^2 / ||N||^2
            //sign(dot(N, extremePoint) * depth(N)^2 = sign(dot(N, extremePoint)) * dot(N, extremePoint)^2 / ||N||^2
            //So the squared normal length and the signed squared unnormalized depth are both tracked. 
            public Vector<float> DepthNumerator;
            public Vector<float> DepthDenominator;
        }

        internal struct Simplex
        {
            /// <summary>
            /// Point on the search space representing the best normal.
            /// </summary>
            public SimplexEntry A;
            /// <summary>
            /// Point on the search space representing the second best normal.
            /// </summary>
            public SimplexEntry B;
            /// <summary>
            /// Point on the search space representing the worst normal.
            /// </summary>
            public SimplexEntry C;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Sample(in Vector2Wide samplePoint, in TShapeWideA a, in TShapeWideA b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB,
               in Vector3Wide x, in Vector3Wide y, in Vector3Wide initialNormal, ref TDepthTester depthTester, out Vector<float> depthNumerator, out Vector<float> depthDenominator)
        {
            Vector3Wide.Scale(x, samplePoint.X, out var xContribution);
            Vector3Wide.Scale(y, samplePoint.Y, out var yContribution);
            Vector3Wide.Add(initialNormal, xContribution, out var sampleNormal);
            Vector3Wide.Add(sampleNormal, yContribution, out sampleNormal);
            depthTester.Test(a, b, localOffsetB, localOrientationB, sampleNormal, out var scaledDepth);
            depthNumerator = scaledDepth * scaledDepth;
            depthNumerator = Vector.ConditionalSelect(Vector.LessThan(scaledDepth, Vector<float>.Zero), -depthNumerator, depthNumerator);
            Vector3Wide.LengthSquared(sampleNormal, out depthDenominator);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void LessThan(in SimplexEntry a, in SimplexEntry b, out Vector<int> aSmallerThanB)
        {
            //Since we have depths stored in the format (sign(dot(N, extremePoint)) * dot(N, extremePoint)) / ||N||^2, 
            //we can compare depths without dividing by multiplying both sides by the denominators.
            aSmallerThanB = Vector.LessThan(a.DepthNumerator * b.DepthDenominator, b.DepthNumerator * a.DepthDenominator);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void ConditionalSelect(in Vector<int> useA, in SimplexEntry a, in SimplexEntry b, out SimplexEntry result)
        {
            result.Point.X = Vector.ConditionalSelect(useA, a.Point.X, b.Point.X);
            result.Point.Y = Vector.ConditionalSelect(useA, a.Point.Y, b.Point.Y);
            result.DepthNumerator = Vector.ConditionalSelect(useA, a.DepthNumerator, b.DepthNumerator);
            result.DepthDenominator = Vector.ConditionalSelect(useA, a.DepthDenominator, b.DepthDenominator);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void TryAdd(in SimplexEntry newEntry, ref Simplex simplex, in Vector<int> terminatedLanes, out Vector<int> newReplacesA, out Vector<int> newReplacesB)
        {
            LessThan(newEntry, simplex.A, out newReplacesA);
            LessThan(newEntry, simplex.B, out newReplacesB);
            LessThan(newEntry, simplex.C, out var newReplacesC);
            //Terminated lanes should not be modified; if per-lane execution is dependent on the bundle, then determinism would depend on bundle order. That's no good!
            //The only value we end up examining at the end of execution is the best slot, so we only need to worry about A.
            newReplacesA = Vector.AndNot(newReplacesA, terminatedLanes);
            ConditionalSelect(newReplacesC, newEntry, simplex.C, out simplex.C);
            //If B or A is replaced by the new value, we don't throw them away like we threw away C- they just get bumped down a rung.
            //This incrementally maintains the order.
            ConditionalSelect(newReplacesB, simplex.B, simplex.C, out simplex.C);
            ConditionalSelect(newReplacesB, newEntry, simplex.B, out simplex.B);
            ConditionalSelect(newReplacesA, simplex.A, simplex.B, out simplex.B);
            ConditionalSelect(newReplacesA, newEntry, simplex.A, out simplex.A);

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Finalize(in Simplex simplex, in Vector3Wide x, in Vector3Wide y, in Vector3Wide initialNormal, out Vector3Wide normal, out Vector<float> depth)
        {
            Vector3Wide.Scale(x, simplex.A.Point.X, out var xContribution);
            Vector3Wide.Scale(y, simplex.A.Point.Y, out var yContribution);
            Vector3Wide.Add(initialNormal, xContribution, out var rawNormal);
            Vector3Wide.Add(rawNormal, yContribution, out rawNormal);

            var normalizationScale = Vector<float>.One / Vector.SquareRoot(simplex.A.DepthDenominator);
            Vector3Wide.Scale(rawNormal, normalizationScale, out normal);
            var sqrtAbsNumerator = Vector.SquareRoot(Vector.Abs(simplex.A.DepthNumerator));
            depth = Vector.ConditionalSelect(Vector.LessThan(simplex.A.DepthNumerator, Vector<float>.Zero), -sqrtAbsNumerator, sqrtAbsNumerator) * normalizationScale;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void GetDepth(in Vector<float> numerator, in Vector<float> denominator, out Vector<float> depth)
        {
            var absDepth = Vector.SquareRoot(Vector.Abs(numerator) / denominator);
            depth = Vector.ConditionalSelect(Vector.LessThan(numerator, Vector<float>.Zero), -absDepth, absDepth);
        }

        public enum SimplexGeneratingStep
        {
            Initialized,
            Reflection,
            Expansion,
            Contraction
        }

        public struct DebugSimplex
        {
            public Vector2 A;
            public float DepthA;
            public Vector2 B;
            public float DepthB;
            public Vector2 C;
            public float DepthC;
            public SimplexGeneratingStep Step;

            internal DebugSimplex(in Simplex simplex)
            {
                Vector2Wide.ReadFirst(simplex.B.Point, out B);
                Vector2Wide.ReadFirst(simplex.C.Point, out C);
                Vector2Wide.ReadFirst(simplex.A.Point, out A);
                GetDepth(simplex.A.DepthNumerator, simplex.A.DepthDenominator, out var depthA);
                GetDepth(simplex.B.DepthNumerator, simplex.B.DepthDenominator, out var depthB);
                GetDepth(simplex.C.DepthNumerator, simplex.C.DepthDenominator, out var depthC);
                DepthA = depthA[0];
                DepthB = depthB[0];
                DepthC = depthC[0];
                Step = SimplexGeneratingStep.Initialized;
            }
            internal DebugSimplex(in Simplex simplex, in Vector<int> usedExpansion, in Vector<int> usedContraction) : this(simplex)
            {
                if (usedExpansion[0] < 0)
                    Step = SimplexGeneratingStep.Expansion;
                else if (usedContraction[0] < 0)
                    Step = SimplexGeneratingStep.Contraction;
                else
                    Step = SimplexGeneratingStep.Reflection;
            }
        }

        public class ExecutionDebugData
        {
            public List<DebugSimplex> Simplices = new List<DebugSimplex>();
        }

        public static void SampleDebugDepths(in TShapeWideA a, in TShapeWideA b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB, in Vector3Wide initialNormalGuess, Vector2 min, Vector2 max, Int2 sampleCounts, out float[,] depths)
        {
            depths = new float[sampleCounts.X, sampleCounts.Y];
            Helpers.BuildOrthnormalBasis(initialNormalGuess, out var x, out var y);
            var span = max - min;
            var sampleSize = span / new Vector2(sampleCounts.X, sampleCounts.Y);
            var depthTester = default(TDepthTester);
            for (int i = 0; i < sampleCounts.X; ++i)
            {
                for (int j = 0; j < sampleCounts.X; ++j)
                {
                    var sample = min + sampleSize * new Vector2(0.5f + i, 0.5f + j);
                    Vector2Wide.Broadcast(sample, out var sampleWide);
                    //We could actually do this widely, but this is just for debug purposes so it really doesn't matter.
                    Sample(sampleWide, a, b, localOffsetB, localOrientationB, x, y, initialNormalGuess, ref depthTester, out var depthNumerator, out var depthDenominator);
                    GetDepth(depthNumerator, depthDenominator, out var depthWide);
                    depths[i, j] = depthWide[0];
                }
            }
        }


        /// <summary>
        /// Attempts to refine a given normal toward a local minimum depth.
        /// </summary>
        /// <param name="a">First shape in the convex pair.</param>
        /// <param name="b">Second shape in the convex pair.</param>
        /// <param name="localOffsetB">Offset from the center of A to the center of B in the local space of A.</param>
        /// <param name="localOrientationB">Orientation of B in the local space of A.</param>
        /// <param name="initialNormalGuess">Initial guess at the minimum depth normal. Must be unit length.</param>
        /// <param name="initialDepth">Depth associated with the guess.</param>
        /// <param name="inactiveLanes">Mask of lanes that should not be considered. Any lane with a value of -1 will be ignored.</param>
        /// <param name="simplexTerminationEpsilon">If the test simplex's size (as measured by point distances, not area) drops below this threshold, the refinement loop will terminate.</param>
        /// <param name="minimumDepthThreshold">If a lane finds a depth less than this threshold, the lane will terminate.</param>
        /// <param name="normal">Refined normal.</param>
        /// <param name="maximumIterations">Maximum number of iterations to execute before terminating.</param>
        public static void Refine(in TShapeWideA a, in TShapeWideA b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB,
            in Vector3Wide initialNormalGuess, in Vector<float> initialDepth, in Vector<int> inactiveLanes, in Vector<float> simplexTerminationEpsilon, in Vector<float> minimumDepthThreshold,
            out Vector3Wide normal, out Vector<float> depth, out ExecutionDebugData debugData, int maximumIterations = 250)
        {
            debugData = new ExecutionDebugData();
            var terminatedLanes = inactiveLanes;

            Debug.Assert(Vector.LessThanAll(
                Vector.BitwiseOr(inactiveLanes,
                Vector.LessThan(Vector.Abs(Vector<float>.One - initialNormalGuess.X * initialNormalGuess.X - initialNormalGuess.Y * initialNormalGuess.Y - initialNormalGuess.Z * initialNormalGuess.Z), new Vector<float>(1e-5f))),
                Vector<int>.Zero), "All active lanes must have started with a normalized initial guess.");

            //We work in a 2d space. This means the search will never find any minima pointing in the opposite direction, but given remotely reasonable initial guesses, that won't be a problem.
            //2d space cuts down on the simplex size and all the associated work and can help convergence by limiting the degrees of freedom.
            Helpers.BuildOrthnormalBasis(initialNormalGuess, out var x, out var y);
            var initialOffsetLength = new Vector<float>(0.5f);
            Simplex simplex;
            simplex.A.Point.X = Vector<float>.Zero;
            simplex.A.Point.Y = Vector<float>.Zero;
            simplex.A.DepthNumerator = initialDepth * initialDepth;
            simplex.A.DepthNumerator = Vector.ConditionalSelect(Vector.LessThan(initialDepth, Vector<float>.Zero), -simplex.A.DepthNumerator, simplex.A.DepthNumerator);
            simplex.A.DepthDenominator = Vector<float>.One;

            terminatedLanes = Vector.BitwiseOr(terminatedLanes, Vector.LessThan(initialDepth, minimumDepthThreshold));
            normal = initialNormalGuess;
            depth = initialDepth;
            if (Vector.LessThanAll(terminatedLanes, Vector<int>.Zero))
            {
                return;
            }

            //We'll use the TryAdd to maintain proper order without extra work, so ensure that any new depths get accepted.
            //(Could optimize this marginally, but not a big deal.)
            simplex.B.DepthNumerator = new Vector<float>(float.MaxValue);
            simplex.C.DepthNumerator = simplex.B.DepthNumerator;
            simplex.B.DepthDenominator = Vector<float>.One;
            simplex.C.DepthDenominator = Vector<float>.One;

            SimplexEntry candidateX;
            //Sample another couple of normals based on the initial guess.
            //The choice is not incredibly important- we'll try a couple of spots 30 degrees away along the tangent axes.
            //(We may want to allow input of the other two slots- the value of using known-suboptimal locations is not very clear, though. Would just save a little init time.)
            candidateX.Point.X = new Vector<float>(.5f);
            candidateX.Point.Y = Vector<float>.Zero;
            var depthTester = default(TDepthTester);
            Sample(candidateX.Point, a, b, localOffsetB, localOrientationB, x, y, initialNormalGuess, ref depthTester, out candidateX.DepthNumerator, out candidateX.DepthDenominator);
            TryAdd(candidateX, ref simplex, terminatedLanes, out _, out _);
            SimplexEntry candidateY;
            candidateY.Point.X = Vector<float>.Zero;
            candidateY.Point.Y = candidateX.Point.X;
            Sample(candidateY.Point, a, b, localOffsetB, localOrientationB, x, y, initialNormalGuess, ref depthTester, out candidateY.DepthNumerator, out candidateY.DepthDenominator);
            TryAdd(candidateY, ref simplex, terminatedLanes, out _, out _);
            if (terminatedLanes[0] == 0)
                debugData.Simplices.Add(new DebugSimplex(simplex));

            var minimumDepthThresholdNumerator = minimumDepthThreshold * minimumDepthThreshold;
            minimumDepthThresholdNumerator = Vector.ConditionalSelect(Vector.LessThan(minimumDepthThreshold, Vector<float>.Zero), -minimumDepthThresholdNumerator, minimumDepthThresholdNumerator);
            terminatedLanes = Vector.BitwiseOr(terminatedLanes, Vector.LessThan(simplex.A.DepthNumerator, minimumDepthThresholdNumerator * simplex.A.DepthDenominator));
            if (Vector.LessThanAll(terminatedLanes, Vector<int>.Zero))
            {
                Finalize(simplex, x, y, initialNormalGuess, out normal, out depth);
                return;
            }

            Vector<int> useNonReflectionSamplePoint = default;
            Vector2Wide nonReflectionSamplePoint;
            Vector<int> debugUsedExpansion = default, debugUsedContraction = default;

            var terminationEpsilonSquared = simplexTerminationEpsilon * simplexTerminationEpsilon;

            var half = new Vector<float>(0.5f);
            for (int i = 0; i < maximumIterations; ++i)
            {
                //There are three ways to choose the next sample point: reflection, expansion, and contraction.
                //Since we want to perform a single sample per iteration, we maintain a bit of extra data between iterations to act as a state machine.
                //The transitions look like this:
                //Reflection -> Expansion if the reflection found a new best location.
                //Reflection -> Contraction if the reflection is no better than the second worst point in the simplex.
                //Reflection -> Reflection is the reflection found a new second-best location.
                //Expansion -> Reflection always.
                //Contraction -> Reflection always.

                //In reflection, just take the current worst point and reflect it over the centroid of the line between the best two points in the simplex.
                //Choose the next location by reflecting the worst point across the centroid of the better two points.
                Vector2Wide.Add(simplex.A.Point, simplex.B.Point, out var reflectionCentroid);
                Vector2Wide.Scale(reflectionCentroid, half, out reflectionCentroid);
                Vector2Wide.Subtract(reflectionCentroid, simplex.C.Point, out var reflectionOffset);
                Vector2Wide.Add(reflectionCentroid, reflectionOffset, out var reflectionSamplePoint);

                SimplexEntry candidate;
                Vector2Wide.ConditionalSelect(useNonReflectionSamplePoint, nonReflectionSamplePoint, reflectionSamplePoint, out candidate.Point);
                Sample(candidate.Point, a, b, localOffsetB, localOrientationB, x, y, initialNormalGuess, ref depthTester, out candidate.DepthNumerator, out candidate.DepthDenominator);
                TryAdd(candidate, ref simplex, terminatedLanes, out var newBest, out var newSecondBest);

                //Only use expansion next if the source of this sample was just reflection.
                var useExpansion = Vector<int>.Zero;// Vector.AndNot(newBest, useNonReflectionSamplePoint);

                //Sample(simplex.A.Point, a, b, localOffsetB, localOrientationB, x, y, initialNormalGuess, ref depthTester, out var testNumeratorA, out var testDenominatorA);
                //Sample(simplex.B.Point, a, b, localOffsetB, localOrientationB, x, y, initialNormalGuess, ref depthTester, out var testNumeratorB, out var testDenominatorB);
                //Sample(simplex.C.Point, a, b, localOffsetB, localOrientationB, x, y, initialNormalGuess, ref depthTester, out var testNumeratorC, out var testDenominatorC);

                Vector2Wide.Subtract(simplex.B.Point, simplex.A.Point, out var ab);
                Vector2Wide.Subtract(simplex.C.Point, simplex.A.Point, out var ac);
                Vector2Wide.LengthSquared(ab, out var abLengthSquared);
                Vector2Wide.LengthSquared(ac, out var acLengthSquared);
                var shouldExitDueToSmallSimplex = Vector.BitwiseAnd(Vector.LessThan(abLengthSquared, terminationEpsilonSquared), Vector.LessThan(acLengthSquared, terminationEpsilonSquared));
                var shouldExitDueToDepthThreshold = Vector.LessThan(simplex.A.DepthNumerator, minimumDepthThresholdNumerator * simplex.A.DepthDenominator);
                terminatedLanes = Vector.BitwiseOr(terminatedLanes, Vector.BitwiseOr(shouldExitDueToDepthThreshold, shouldExitDueToSmallSimplex));
                //If all lanes are done, we can quit.
                if (terminatedLanes[0] == 0)
                    debugData.Simplices.Add(new DebugSimplex(simplex, debugUsedExpansion, debugUsedContraction));
                if (Vector.LessThanAll(terminatedLanes, Vector<int>.Zero))
                    break;

                //Only use contraction if the source of the failed sample was just reflection.
                var useContraction = Vector.AndNot(Vector.AndNot(Vector.OnesComplement(useExpansion), newSecondBest), useNonReflectionSamplePoint);
                //In expansion, use the reflection offset to take another step forward.
                //Note that we don't conditional select this- no need, the contraction conditional select covers the contraction lanes, and the other lanes won't look at this value.
                Vector2Wide.Add(reflectionOffset, reflectionSamplePoint, out nonReflectionSamplePoint);
                //In contraction, sample a point halfway between the worst point and the centroid.
                Vector2Wide.Scale(reflectionOffset, half, out var halfOffset);
                Vector2Wide.Subtract(reflectionCentroid, halfOffset, out var contractionSamplePoint);
                Vector2Wide.ConditionalSelect(useContraction, contractionSamplePoint, nonReflectionSamplePoint, out nonReflectionSamplePoint);

                useNonReflectionSamplePoint = Vector.BitwiseOr(useContraction, useExpansion);

                debugUsedContraction = useContraction;
                debugUsedExpansion = useExpansion;

                //(The original nedler-mead method includes another transformation- 'shrink'- but I excluded it due to relative rarity and 
                //the fact that it requires two evaluations unlike the other paths. We want to have pretty simple and unified code flow for SIMD purposes, so we ignore it.
                //Note that it is still possible to do it with one sample per iteration if you're willing to add more complexity to the state machine.)
            }

            Finalize(simplex, x, y, initialNormalGuess, out normal, out depth);
        }

    }
}
