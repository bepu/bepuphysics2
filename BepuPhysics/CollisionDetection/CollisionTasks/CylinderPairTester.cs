using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.SweepTasks;
using BepuUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct CylinderPairTester : IPairTester<CylinderWide, CylinderWide, Convex4ContactManifoldWide>
    {
        public int BatchSize => 32;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetDepth(in Vector3Wide localAxisYA, in Vector3Wide localOffsetB, in Vector3Wide localNormal, in CylinderWide a, in CylinderWide b, out Vector<float> depth)
        {
            Vector3Wide.Dot(localAxisYA, localNormal, out var dotNAY);
            Vector3Wide.Dot(localOffsetB, localNormal, out var dotOffsetN);
            depth =
                a.HalfLength * Vector.Abs(dotNAY) + a.Radius * Vector.SquareRoot(Vector.Max(Vector<float>.Zero, Vector<float>.One - dotNAY * dotNAY)) +
                b.HalfLength * Vector.Abs(localNormal.Y) + b.Radius * Vector.SquareRoot(Vector.Max(Vector<float>.Zero, Vector<float>.One - localNormal.Y * localNormal.Y)) -
                Vector.Abs(dotOffsetN);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GradientDescent(in Vector3Wide localAxisYA, in Vector3Wide localOffsetB, in CylinderWide a, in CylinderWide b, ref Vector3Wide localNormal)
        {
            //Cylinder-cylinder's depth calculation is analytically differentiable.
            //depth(N) = cylinderContribution(A, N) + cylinderContribution(B, N) - offsetSeparation(N)
            //cylinderContribution(a, B) = a.HalfLength * abs(dot(N, a.AxisY) + a.Radius * sqrt(1 - dot(N, a.AxisY)^2)
            //d/dNx(cylinderContribution(a, B)) = a.HalfLength * sign(dot(N, a.AxisY)) * d/dNx(dot(N, a.AxisY) - a.Radius * d/dNx(dot(N, a.AxisY)) * dot(N, a.AxisY) / sqrt(1 - dot(N, a.AxisY)^2)
            //d/dNx(dot(N, a.AxisY)) = a.AxisY.X
            //d/dNx(cylinderContribution(a, B)) = a.AxisY.X * (a.HalfLength * sign(dot(N, a.AxisY)) - a.Radius * dot(N, a.AxisY) / sqrt(1 - dot(N, a.AxisY)^2))

            //Extrapolating that to the other axes and the other cylinder is simple; for the offset:
            //offsetSeparation(N) = abs(dot(offsetB, N))
            //d/dNx(offsetSeparation(N)) = sign(dot(offsetB, N)) * d/dNx(dot(offsetB, N)) = sign(dot(offsetB, N)) * offsetB.X
            var gradientScale = new Vector<float>(0.25f);
            for (int i = 0; i < 10; ++i)
            {
                Vector3Wide.Dot(localAxisYA, localNormal, out var dotNAY);
                var scaleA = Vector.ConditionalSelect(Vector.LessThan(dotNAY, Vector<float>.Zero), -a.HalfLength, a.HalfLength) - a.Radius * dotNAY / Vector.SquareRoot(Vector.Max(Vector<float>.Zero, Vector<float>.One - dotNAY * dotNAY));
                //Working in B's local space, so no need for an axis dot.
                var scaleB = Vector.ConditionalSelect(Vector.LessThan(localNormal.Y, Vector<float>.Zero), -b.HalfLength, b.HalfLength) - b.Radius * localNormal.Y / Vector.SquareRoot(Vector.Max(Vector<float>.Zero, Vector<float>.One - localNormal.Y * localNormal.Y));

                Vector3Wide.Dot(localOffsetB, localNormal, out var dotOffsetN);
                var useNegatedOffset = Vector.LessThan(dotOffsetN, Vector<float>.Zero);
                Vector3Wide gradient;
                //Note that b's contribution along X and Z is zero since we're in B's local space.
                gradient.X = localAxisYA.X * scaleA - Vector.ConditionalSelect(useNegatedOffset, -localOffsetB.X, localOffsetB.X);
                gradient.Y = localAxisYA.Y * scaleA + scaleB - Vector.ConditionalSelect(useNegatedOffset, -localOffsetB.Y, localOffsetB.Y);
                gradient.Z = localAxisYA.Z * scaleA - Vector.ConditionalSelect(useNegatedOffset, -localOffsetB.Z, localOffsetB.Z);

                Vector3Wide newNormal;
                newNormal.X = localNormal.X - gradient.X * gradientScale;
                newNormal.Y = localNormal.Y - gradient.Y * gradientScale;
                newNormal.Z = localNormal.Z - gradient.Z * gradientScale;
                //gradientScale *= 0.66f;
                Vector3Wide.Length(newNormal, out var newNormalLength);
                Vector3Wide.Scale(newNormal, Vector<float>.One / newNormalLength, out localNormal);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GradientDescent2(in Vector3Wide localAxisYA, in Vector3Wide localOffsetB, in CylinderWide a, in CylinderWide b, ref Vector3Wide localNormal, out Vector<float> depth)
        {
            var gradientScale = new Vector<float>(1f);
            const float epsilon = 1e-3f;
            var gradientDifferenceStepSize = new Vector<float>(epsilon);
            var inverseGradientDifferenceStepSize = new Vector<float>(1f / epsilon);
            GetDepth(localAxisYA, localOffsetB, localNormal, a, b, out depth);
            for (int i = 0; i < 10; ++i)
            {
                Vector3Wide x, y, z;
                x.X = localNormal.X + gradientDifferenceStepSize;
                x.Y = localNormal.Y;
                x.Z = localNormal.Z;
                y.X = localNormal.X;
                y.Y = localNormal.Y + gradientDifferenceStepSize;
                y.Z = localNormal.Z;
                z.X = localNormal.X;
                z.Y = localNormal.Y;
                z.Z = localNormal.Z + gradientDifferenceStepSize;
                Vector3Wide.Normalize(x, out x);
                Vector3Wide.Normalize(y, out y);
                Vector3Wide.Normalize(z, out z);
                GetDepth(localAxisYA, localOffsetB, x, a, b, out var xDepth);
                GetDepth(localAxisYA, localOffsetB, y, a, b, out var yDepth);
                GetDepth(localAxisYA, localOffsetB, z, a, b, out var zDepth);
                Vector3Wide gradient;
                gradient.X = (xDepth - depth) * inverseGradientDifferenceStepSize;
                gradient.Y = (yDepth - depth) * inverseGradientDifferenceStepSize;
                gradient.Z = (zDepth - depth) * inverseGradientDifferenceStepSize;

                Vector3Wide newNormal;
                newNormal.X = localNormal.X - gradient.X * gradientScale;
                newNormal.Y = localNormal.Y - gradient.Y * gradientScale;
                newNormal.Z = localNormal.Z - gradient.Z * gradientScale;
                gradientScale *= 0.7f;
                Vector3Wide.Length(newNormal, out var newNormalLength);
                Vector3Wide.Scale(newNormal, Vector<float>.One / newNormalLength, out localNormal);
                GetDepth(localAxisYA, localOffsetB, localNormal, a, b, out depth);
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GradientDescent3(in Vector3Wide localAxisYA, in Vector3Wide localOffsetB, in CylinderWide a, in CylinderWide b, ref Vector3Wide localNormal, out Vector<float> depth)
        {
            const float epsilon = 1e-4f;
            var gradientDifferenceStepSize = new Vector<float>(epsilon);
            //Depths are proportional to shape size. Normals are unit length. Directly using depths as a gradient scale would make it scale sensitive, so try to normalize.
            var scaleNormalizingFactor = Vector.Min(Vector.Max(a.HalfLength, a.Radius), Vector.Max(b.HalfLength, b.Radius));
            var depthChangeToGradient = new Vector<float>(0.04f / epsilon) / scaleNormalizingFactor;
            GetDepth(localAxisYA, localOffsetB, localNormal, a, b, out depth);
            for (int i = 0; i < 10; ++i)
            {
                Helpers.BuildOrthnormalBasis(localNormal, out var x, out var y);
                Vector3Wide offsetNormalX, offsetNormalY;
                offsetNormalX.X = x.X * gradientDifferenceStepSize + localNormal.X;
                offsetNormalX.Y = x.Y * gradientDifferenceStepSize + localNormal.Y;
                offsetNormalX.Z = x.Z * gradientDifferenceStepSize + localNormal.Z;
                offsetNormalY.X = y.X * gradientDifferenceStepSize + localNormal.X;
                offsetNormalY.Y = y.Y * gradientDifferenceStepSize + localNormal.Y;
                offsetNormalY.Z = y.Z * gradientDifferenceStepSize + localNormal.Z;
                Vector3Wide.Normalize(offsetNormalX, out offsetNormalX);
                Vector3Wide.Normalize(offsetNormalY, out offsetNormalY);
                GetDepth(localAxisYA, localOffsetB, offsetNormalX, a, b, out var xDepth);
                GetDepth(localAxisYA, localOffsetB, offsetNormalY, a, b, out var yDepth);
                Vector2Wide gradient;
                gradient.X = (xDepth - depth) * depthChangeToGradient;
                gradient.Y = (yDepth - depth) * depthChangeToGradient;

                Vector3Wide newNormal;
                newNormal.X = localNormal.X - gradient.X * x.X - gradient.Y * y.X;
                newNormal.Y = localNormal.Y - gradient.X * x.Y - gradient.Y * y.Y;
                newNormal.Z = localNormal.Z - gradient.X * x.Z - gradient.Y * y.Z;
                depthChangeToGradient *= 0.6f;
                Vector3Wide.Length(newNormal, out var newNormalLength);
                Vector3Wide.Scale(newNormal, Vector<float>.One / newNormalLength, out localNormal);
                GetDepth(localAxisYA, localOffsetB, localNormal, a, b, out depth);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GradientDescent4(in Vector3Wide localAxisYA, in Vector3Wide localOffsetB, in CylinderWide a, in CylinderWide b, ref Vector3Wide localNormal)
        {
            //Cylinder-cylinder's depth calculation is analytically differentiable.
            //depth(N) = cylinderContribution(A, N) + cylinderContribution(B, N) - offsetSeparation(N)
            //cylinderContribution(a, B) = a.HalfLength * abs(dot(N, a.AxisY) + a.Radius * sqrt(1 - dot(N, a.AxisY)^2)
            //dN/dTX(cylinderContribution(a, B)) = a.HalfLength * sign(dot(N, a.AxisY)) * dN/dTX(dot(N, a.AxisY)) - a.Radius * dN/dTX(dot(N, a.AxisY)) * dot(N, a.AxisY) / sqrt(1 - dot(N, a.AxisY)^2)
            //dN/dTX(dot(N, a.AxisY)) = dot(TX, a.AxisY)
            //dN/dTX(cylinderContribution(a, B)) = dot(TX, a.AxisY) * (a.HalfLength * sign(dot(N, a.AxisY)) - a.Radius * dot(N, a.AxisY) / sqrt(1 - dot(N, a.AxisY)^2))
            //Note that the dot(TX, aAxis.Y) scale is always going to be no larger than sqrt(1 - dot(N, a.AxisY)^2), so this doesn't actually grow to infinity. The discontinuity at dot(N, a.AxisY)^2 == 1 must be handled, though.

            //Extrapolating that to the other tangent TY and the other cylinder is simple; for the offset:
            //offsetSeparation(N) = abs(dot(offsetB, N))
            //dN/dTX(offsetSeparation(N)) = sign(dot(offsetB, N)) * dN/dTX(dot(offsetB, N)) = sign(dot(offsetB, N)) * dot(offsetB, TX)
            var gradientScale = new Vector<float>(0.3f);
            var divisorEpsilon = new Vector<float>(1e-10f);
            Vector3Wide previousGradient = default;
            for (int i = 0; i < 10; ++i)
            {
                Vector3Wide.Dot(localAxisYA, localNormal, out var dotNAY);
                var scaleA = Vector.ConditionalSelect(Vector.LessThan(dotNAY, Vector<float>.Zero), -a.HalfLength, a.HalfLength) - a.Radius * dotNAY / Vector.SquareRoot(Vector.Max(divisorEpsilon, Vector<float>.One - dotNAY * dotNAY));
                //Working in B's local space, so no need for an axis dot.
                var scaleB = Vector.ConditionalSelect(Vector.LessThan(localNormal.Y, Vector<float>.Zero), -b.HalfLength, b.HalfLength) - b.Radius * localNormal.Y / Vector.SquareRoot(Vector.Max(divisorEpsilon, Vector<float>.One - localNormal.Y * localNormal.Y));

                Vector3Wide.Dot(localOffsetB, localNormal, out var dotOffsetN);
                var useNegatedOffset = Vector.LessThan(dotOffsetN, Vector<float>.Zero);
                Helpers.BuildOrthnormalBasis(localNormal, out var x, out var y);
                Vector3Wide.Dot(x, localAxisYA, out var aYDotTX);
                Vector3Wide.Dot(x, localOffsetB, out var offsetBDotTX);
                Vector3Wide.Dot(y, localAxisYA, out var aYDotTY);
                Vector3Wide.Dot(y, localOffsetB, out var offsetBDotTY);
                Vector2Wide gradient;
                gradient.X = aYDotTX * scaleA + x.Y * scaleB - Vector.ConditionalSelect(useNegatedOffset, -offsetBDotTX, offsetBDotTX);
                gradient.Y = aYDotTY * scaleA + y.Y * scaleB - Vector.ConditionalSelect(useNegatedOffset, -offsetBDotTY, offsetBDotTY);
                Vector3Wide transformedGradient;
                transformedGradient.X = gradient.X * x.X + gradient.Y * y.X;
                transformedGradient.Y = gradient.X * x.Y + gradient.Y * y.Y;
                transformedGradient.Z = gradient.X * x.Z + gradient.Y * y.Z;
                if (i > 0)
                {
                    Vector3Wide.Dot(transformedGradient, previousGradient, out var gradientDot);
                    gradientScale = gradientScale * Vector.ConditionalSelect(Vector.LessThan(gradientDot, Vector<float>.Zero), new Vector<float>(0.3f), new Vector<float>(0.7f));
                }
                previousGradient = transformedGradient;

                Vector3Wide newNormal;
                newNormal.X = localNormal.X - transformedGradient.X * gradientScale;
                newNormal.Y = localNormal.Y - transformedGradient.Y * gradientScale;
                newNormal.Z = localNormal.Z - transformedGradient.Z * gradientScale;
                Vector3Wide.Length(newNormal, out var newNormalLength);
                Vector3Wide.Scale(newNormal, Vector<float>.One / newNormalLength, out localNormal);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GradientDescent5(in Vector3Wide localAxisYA, in Vector3Wide localOffsetB, in CylinderWide a, in CylinderWide b, ref Vector3Wide localNormal)
        {
            //Cylinder-cylinder's depth calculation is analytically differentiable.
            //depth(N) = cylinderContribution(A, N) + cylinderContribution(B, N) - offsetSeparation(N)
            //cylinderContribution(a, B) = a.HalfLength * abs(dot(N, a.AxisY) + a.Radius * sqrt(1 - dot(N, a.AxisY)^2)
            //dN/dTX(cylinderContribution(a, B)) = a.HalfLength * sign(dot(N, a.AxisY)) * dN/dTX(dot(N, a.AxisY)) - a.Radius * dN/dTX(dot(N, a.AxisY)) * dot(N, a.AxisY) / sqrt(1 - dot(N, a.AxisY)^2)
            //dN/dTX(dot(N, a.AxisY)) = dot(TX, a.AxisY)
            //dN/dTX(cylinderContribution(a, B)) = dot(TX, a.AxisY) * (a.HalfLength * sign(dot(N, a.AxisY)) - a.Radius * dot(N, a.AxisY) / sqrt(1 - dot(N, a.AxisY)^2))
            //Note that the dot(TX, aAxis.Y) scale is always going to be no larger than sqrt(1 - dot(N, a.AxisY)^2), so this doesn't actually grow to infinity. The discontinuity at dot(N, a.AxisY)^2 == 1 must be handled, though.

            //Extrapolating that to the other tangent TY and the other cylinder is simple; for the offset:
            //offsetSeparation(N) = abs(dot(offsetB, N))
            //dN/dTX(offsetSeparation(N)) = sign(dot(offsetB, N)) * dN/dTX(dot(offsetB, N)) = sign(dot(offsetB, N)) * dot(offsetB, TX)
            var gradientScale = new Vector<float>(0.6f);
            var divisorEpsilon = new Vector<float>(1e-10f);
            Vector3Wide previousGradient = default;
            for (int i = 0; i < 10; ++i)
            {
                Vector3Wide.Dot(localAxisYA, localNormal, out var dotNAY);
                var scaleA = Vector.ConditionalSelect(Vector.LessThan(dotNAY, Vector<float>.Zero), -a.HalfLength, a.HalfLength) - a.Radius * dotNAY / Vector.SquareRoot(Vector.Max(divisorEpsilon, Vector<float>.One - dotNAY * dotNAY));
                //Working in B's local space, so no need for an axis dot.
                var scaleB = Vector.ConditionalSelect(Vector.LessThan(localNormal.Y, Vector<float>.Zero), -b.HalfLength, b.HalfLength) - b.Radius * localNormal.Y / Vector.SquareRoot(Vector.Max(divisorEpsilon, Vector<float>.One - localNormal.Y * localNormal.Y));

                Vector3Wide.Dot(localOffsetB, localNormal, out var dotOffsetN);
                var useNegatedOffset = Vector.LessThan(dotOffsetN, Vector<float>.Zero);
                Helpers.BuildOrthnormalBasis(localNormal, out var x, out var y);
                Vector3Wide.Dot(x, localAxisYA, out var aYDotTX);
                Vector3Wide.Dot(x, localOffsetB, out var offsetBDotTX);
                Vector3Wide.Dot(y, localAxisYA, out var aYDotTY);
                Vector3Wide.Dot(y, localOffsetB, out var offsetBDotTY);
                Vector2Wide rawGradient;
                rawGradient.X = aYDotTX * scaleA + x.Y * scaleB - Vector.ConditionalSelect(useNegatedOffset, -offsetBDotTX, offsetBDotTX);
                rawGradient.Y = aYDotTY * scaleA + y.Y * scaleB - Vector.ConditionalSelect(useNegatedOffset, -offsetBDotTY, offsetBDotTY);
                Vector2Wide.Length(rawGradient, out var gradientLength);
                Vector2Wide.Scale(rawGradient, Vector.Min(Vector<float>.One, Vector<float>.One / gradientLength), out var gradient);
                Vector3Wide transformedGradient;
                transformedGradient.X = gradient.X * x.X + gradient.Y * y.X;
                transformedGradient.Y = gradient.X * x.Y + gradient.Y * y.Y;
                transformedGradient.Z = gradient.X * x.Z + gradient.Y * y.Z;
                if (i > 0)
                {
                    Vector3Wide.Dot(transformedGradient, previousGradient, out var gradientDot);
                    const float baseline = 0.25f;
                    var downScale = new Vector<float>(baseline) + (1f - baseline) * Vector.Max(Vector<float>.Zero, gradientDot);
                    gradientScale *= downScale;
                }
                previousGradient = transformedGradient;

                Vector3Wide newNormal;
                newNormal.X = localNormal.X - transformedGradient.X * gradientScale;
                newNormal.Y = localNormal.Y - transformedGradient.Y * gradientScale;
                newNormal.Z = localNormal.Z - transformedGradient.Z * gradientScale;
                Vector3Wide.Length(newNormal, out var newNormalLength);
                Vector3Wide.Scale(newNormal, Vector<float>.One / newNormalLength, out localNormal);
            }
        }


        public static void GradientDescent6<TAScalar, TA, TSupportFinderA, TBScalar, TB, TSupportFinderB>(in TA a, in TB b, in Matrix3x3Wide localOrientationB, in Vector3Wide localOffsetB, ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB,
            in Vector3Wide initialGuess, out Vector3Wide localNormal)
            where TAScalar : IConvexShape
            where TA : IShapeWide<TAScalar>
            where TSupportFinderA : ISupportFinder<TAScalar, TA>
            where TBScalar : IConvexShape
            where TB : IShapeWide<TBScalar>
            where TSupportFinderB : ISupportFinder<TBScalar, TB>
        {
            //1) any convex shape can be represented by a point cloud of arbitrary density
            //2) the 'extreme' for a normal N is the point in a shape which has the highest value for dot(point, N), or in other words extreme(N, shapeA) = max(dot(N, shapeA[0], dot(N, shapeA[1]), ...dot(N, shapeA[i]))
            //3) take a function that computes 'depth'(i.e.interval overlap) along a normal N pointing from shape B to shape A, depth(N) = extreme(N, shapeB) - extreme(-N, shapeA))

            //4) let's apply gradient descent to minimizing the depth function by stepping the normal- parameterize the normal step in terms of tangents TX and TY (two arbitrary tangent vectors perpendicular to the normal and each other)
            //5) the gradient is (dN/dTX(depth(N)), dN/dTY(depth(N))
            //6) zooming in on just the dN/dTX component: dN/dTX(depth(N)) = dN/dTX(extreme(N, shapeB)) - dN/dTX(extreme(-N, shapeA))
            //7) derivative of max(a, b, c...) is just 'select the max element and take the derivative of it, so dN/dTX(extreme(N, shapeB) = dN/dTX(dot(N, shapeB[extremeIndex])) = dot(TX, shapeB[extremeIndex])
            //8) dN/dTX(depth(N)) = dot(TX, shapeB[extremeIndex]) - dot(TX, shapeA[extremeIndex])
            //9) gradient(depth(N)) = (dot(TX, shapeB[extremeIndex]) - dot(TX, shapeA[extremeIndex]), dot(TY, shapeB[extremeIndex]) - dot(TY, shapeA[extremeIndex]))

            //so, the step for minimizing depth for any pair of convex shapes is:
            //(dot(TX, shapeB[extremeIndex]) + dot(TX, shapeA[extremeIndex]), dot(TY, shapeB[extremeIndex]) + dot(TY, shapeA[extremeIndex])) * stepScale
            //Obviously taking some liberties here with the fact that the normal is being renormalized, but does it converge *well enough*?

            localNormal = initialGuess;
            var stepScale = new Vector<float>(-0.2f);
            //Vector3Wide previousChange;
            for (int i = 0; i < 10; ++i)
            {
                //Normals are always calibrated to point from B to A.
                supportFinderB.ComputeSupport(b, localOrientationB, localNormal, out var extremeB);
                Vector3Wide.Add(extremeB, localOffsetB, out extremeB);
                Vector3Wide.Negate(localNormal, out var negatedNormal);
                supportFinderA.ComputeLocalSupport(a, negatedNormal, out var extremeA);

                Vector3Wide.Subtract(extremeB, extremeA, out var debugExtremeOffset);
                Vector3Wide.Dot(debugExtremeOffset, localNormal, out var debugIterationDepth);

                Helpers.BuildOrthnormalBasis(localNormal, out var x, out var y);
                Vector3Wide.Dot(x, extremeA, out var xDotA);
                Vector3Wide.Dot(y, extremeA, out var yDotA);
                Vector3Wide.Dot(x, extremeB, out var xDotB);
                Vector3Wide.Dot(y, extremeB, out var yDotB);

                Vector2Wide changeAlongTangents;
                changeAlongTangents.X = (xDotB - xDotA) * stepScale;
                changeAlongTangents.Y = (yDotB - yDotA) * stepScale;

                Vector3Wide newNormal;
                newNormal.X = localNormal.X + changeAlongTangents.X * x.X + changeAlongTangents.Y * y.X;
                newNormal.Y = localNormal.Y + changeAlongTangents.X * x.Y + changeAlongTangents.Y * y.Y;
                newNormal.Z = localNormal.Z + changeAlongTangents.X * x.Z + changeAlongTangents.Y * y.Z;

                //changeAlongTangents.X = (xDotB - xDotA);
                //changeAlongTangents.Y = (yDotB - yDotA);

                //Vector2Wide.Length(changeAlongTangents, out var length);
                //Vector2Wide.Scale(changeAlongTangents, Vector<float>.One / Vector.Max(new Vector<float>(1e-10f), length), out changeAlongTangents);

                //Vector3Wide normalChange;
                //normalChange.X = changeAlongTangents.X * x.X + changeAlongTangents.Y * y.X;
                //normalChange.Y = changeAlongTangents.X * x.Y + changeAlongTangents.Y * y.Y;
                //normalChange.Z = changeAlongTangents.X * x.Z + changeAlongTangents.Y * y.Z;
                //Vector3Wide.Dot(previousChange, normalChange, out var changeDot);
                //previousChange = normalChange;
                //stepScale *= Vector.ConditionalSelect(Vector.LessThan(changeDot, Vector<float>.Zero), new Vector<float>(0.35f), new Vector<float>(0.9f));
                //Vector3Wide.Scale(normalChange, stepScale, out normalChange);

                //Vector3Wide newNormal;
                //newNormal.X = localNormal.X + normalChange.X;
                //newNormal.Y = localNormal.Y + normalChange.Y;
                //newNormal.Z = localNormal.Z + normalChange.Z;

                Vector3Wide.Length(newNormal, out var normalLength);
                Vector3Wide.Scale(newNormal, Vector<float>.One / normalLength, out localNormal);
            }
        }


        public static void GradientDescent7<TAScalar, TA, TSupportFinderA, TBScalar, TB, TSupportFinderB>(in TA a, in TB b, in Matrix3x3Wide localOrientationB, in Vector3Wide localOffsetB, ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB,
           in Vector3Wide initialGuess, out Vector3Wide localNormal)
           where TAScalar : IConvexShape
           where TA : IShapeWide<TAScalar>
           where TSupportFinderA : ISupportFinder<TAScalar, TA>
           where TBScalar : IConvexShape
           where TB : IShapeWide<TBScalar>
           where TSupportFinderB : ISupportFinder<TBScalar, TB>
        {
            //1) any convex shape can be represented by a point cloud of arbitrary density
            //2) the 'extreme' for a normal N is the point in a shape which has the highest value for dot(point, N), or in other words extreme(N, shapeA) = max(dot(N, shapeA[0], dot(N, shapeA[1]), ...dot(N, shapeA[i]))
            //3) take a function that computes 'depth'(i.e.interval overlap) along a normal N pointing from shape B to shape A, depth(N) = extreme(N, shapeB) - extreme(-N, shapeA))

            //4) let's apply gradient descent to minimizing the depth function by stepping the normal- parameterize the normal step in terms of tangents TX and TY (two arbitrary tangent vectors perpendicular to the normal and each other)
            //5) the gradient is (dN/dTX(depth(N)), dN/dTY(depth(N))
            //6) zooming in on just the dN/dTX component: dN/dTX(depth(N)) = dN/dTX(extreme(N, shapeB)) - dN/dTX(extreme(-N, shapeA))
            //7) derivative of max(a, b, c...) is just 'select the max element and take the derivative of it, so dN/dTX(extreme(N, shapeB) = dN/dTX(dot(N, shapeB[extremeIndex])) = dot(TX, shapeB[extremeIndex])
            //8) dN/dTX(depth(N)) = dot(TX, shapeB[extremeIndex]) - dot(TX, shapeA[extremeIndex])
            //9) gradient(depth(N)) = (dot(TX, shapeB[extremeIndex]) - dot(TX, shapeA[extremeIndex]), dot(TY, shapeB[extremeIndex]) - dot(TY, shapeA[extremeIndex]))

            //so, the step for minimizing depth for any pair of convex shapes is:
            //(dot(TX, shapeB[extremeIndex]) + dot(TX, shapeA[extremeIndex]), dot(TY, shapeB[extremeIndex]) + dot(TY, shapeA[extremeIndex])) * stepScale
            //Obviously taking some liberties here with the fact that the normal is being renormalized, but does it converge *well enough*?

            localNormal = initialGuess;
            var stepScale = new Vector<float>(-0.2f);
            var finiteDifferenceEpsilon = new Vector<float>(1e-3f);
            //Vector3Wide previousChange;
            for (int i = 0; i < 10; ++i)
            {
                Helpers.BuildOrthnormalBasis(localNormal, out var x, out var y);
                //Normals are always calibrated to point from B to A.
                supportFinderB.ComputeSupport(b, localOrientationB, localNormal, out var extremeB);
                Vector3Wide.Add(extremeB, localOffsetB, out extremeB);
                Vector3Wide.Negate(localNormal, out var negatedNormal);
                supportFinderA.ComputeLocalSupport(a, negatedNormal, out var extremeA);
                Vector3Wide.Subtract(extremeB, extremeA, out var extremeOffset);
                Vector3Wide.Dot(extremeOffset, localNormal, out var depth);


                Vector3Wide localNormalX;
                localNormalX.X = localNormal.X + x.X * finiteDifferenceEpsilon;
                localNormalX.Y = localNormal.Y + x.Y * finiteDifferenceEpsilon;
                localNormalX.Z = localNormal.Z + x.Z * finiteDifferenceEpsilon;
                Vector3Wide.Normalize(localNormalX, out localNormalX);
                supportFinderB.ComputeSupport(b, localOrientationB, localNormalX, out var extremeBX);
                Vector3Wide.Add(extremeBX, localOffsetB, out extremeBX);
                Vector3Wide.Negate(localNormalX, out var negatedNormalX);
                supportFinderA.ComputeLocalSupport(a, negatedNormalX, out var extremeAX);
                Vector3Wide.Subtract(extremeBX, extremeAX, out var extremeOffsetX);
                Vector3Wide.Dot(extremeOffsetX, localNormalX, out var depthX);


                Vector3Wide localNormalY;
                localNormalY.X = localNormal.X + y.X * finiteDifferenceEpsilon;
                localNormalY.Y = localNormal.Y + y.Y * finiteDifferenceEpsilon;
                localNormalY.Z = localNormal.Z + y.Z * finiteDifferenceEpsilon;
                Vector3Wide.Normalize(localNormalY, out localNormalY);
                supportFinderB.ComputeSupport(b, localOrientationB, localNormalY, out var extremeBY);
                Vector3Wide.Add(extremeBY, localOffsetB, out extremeBY);
                Vector3Wide.Negate(localNormalY, out var negatedNormalY);
                supportFinderA.ComputeLocalSupport(a, negatedNormalY, out var extremeAY);
                Vector3Wide.Subtract(extremeBY, extremeAY, out var extremeOffsetY);
                Vector3Wide.Dot(extremeOffsetY, localNormalY, out var depthY);

                Vector2Wide numericalChangeAlongTangents;
                numericalChangeAlongTangents.X = (depthX - depth) * stepScale / finiteDifferenceEpsilon;
                numericalChangeAlongTangents.Y = (depthY - depth) * stepScale / finiteDifferenceEpsilon;


                Vector3Wide.Dot(x, extremeA, out var xDotA);
                Vector3Wide.Dot(y, extremeA, out var yDotA);
                Vector3Wide.Dot(x, extremeB, out var xDotB);
                Vector3Wide.Dot(y, extremeB, out var yDotB);

                Vector2Wide changeAlongTangents;
                changeAlongTangents.X = (xDotB - xDotA) * stepScale;
                changeAlongTangents.Y = (yDotB - yDotA) * stepScale;

                Vector3Wide newNormal;
                newNormal.X = localNormal.X + changeAlongTangents.X * x.X + changeAlongTangents.Y * y.X;
                newNormal.Y = localNormal.Y + changeAlongTangents.X * x.Y + changeAlongTangents.Y * y.Y;
                newNormal.Z = localNormal.Z + changeAlongTangents.X * x.Z + changeAlongTangents.Y * y.Z;

                //changeAlongTangents.X = (xDotB - xDotA);
                //changeAlongTangents.Y = (yDotB - yDotA);

                //Vector2Wide.Length(changeAlongTangents, out var length);
                //Vector2Wide.Scale(changeAlongTangents, Vector<float>.One / Vector.Max(new Vector<float>(1e-10f), length), out changeAlongTangents);

                //Vector3Wide normalChange;
                //normalChange.X = changeAlongTangents.X * x.X + changeAlongTangents.Y * y.X;
                //normalChange.Y = changeAlongTangents.X * x.Y + changeAlongTangents.Y * y.Y;
                //normalChange.Z = changeAlongTangents.X * x.Z + changeAlongTangents.Y * y.Z;
                //Vector3Wide.Dot(previousChange, normalChange, out var changeDot);
                //previousChange = normalChange;
                //stepScale *= Vector.ConditionalSelect(Vector.LessThan(changeDot, Vector<float>.Zero), new Vector<float>(0.35f), new Vector<float>(0.9f));
                //Vector3Wide.Scale(normalChange, stepScale, out normalChange);

                //Vector3Wide newNormal;
                //newNormal.X = localNormal.X + normalChange.X;
                //newNormal.Y = localNormal.Y + normalChange.Y;
                //newNormal.Z = localNormal.Z + normalChange.Z;

                Vector3Wide.Length(newNormal, out var normalLength);
                Vector3Wide.Scale(newNormal, Vector<float>.One / normalLength, out localNormal);
            }
        }

        public static void Newton<TAScalar, TA, TSupportFinderA, TBScalar, TB, TSupportFinderB>(in TA a, in TB b, in Matrix3x3Wide localOrientationB, in Vector3Wide localOffsetB, ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB,
            in Vector3Wide initialGuess, out Vector3Wide localNormal)
            where TAScalar : IConvexShape
            where TA : IShapeWide<TAScalar>
            where TSupportFinderA : ISupportFinder<TAScalar, TA>
            where TBScalar : IConvexShape
            where TB : IShapeWide<TBScalar>
            where TSupportFinderB : ISupportFinder<TBScalar, TB>
        {
            //a1) any convex shape can be represented by a point cloud of arbitrary density
            //a2) the 'extreme' for a normal N is the point in a shape which has the highest value for dot(point, N), or in other words extreme(N, shapeA) = max(dot(N, shapeA[0], dot(N, shapeA[1]), ...dot(N, shapeA[i]))
            //a3) take a function that computes 'depth'(i.e.interval overlap) along a normal N pointing from shape B to shape A, depth(N) = extreme(N, shapeB) - extreme(-N, shapeA))

            //b0) we can use root finders / optimizers for computing locally minimal depth if the depth function can be differentiated
            //b1) newton's method for finding roots is x1 = x0 - f(x0) / f'(x0) iterated
            //b2) newton's method can be used for optimization by finding a 0 in the first derivative, so you do the same thing one step down, x1 = x0 - f'(x0) / f''(x0) iterated
            //b3) this generalizes to multivariate functions, so for a vector x: x1 = x0 - H ^ -1(x0) * gradient(x0)
            //b4) H^-1(x0) is the inverse hessian matrix of the function at x0, gradient is the gradient of the function at x0
            //b5) a hessian matrix is a multivariable generalization of the second derivative; for a 2d function f(x), the hessian matrix is a 2x2 matrix with each component representing a partial derivative along one of the two axes, 
            //followed by another partial derivative along one of the two axes- so H[0][0] would be df/dx(df/dx(f(x))), H[0][1] would be df/dx(df/dy(f(x)), and so on

            //c1) let's apply newton's method to minimizing the depth function by stepping the normal- parameterize the normal step in terms of tangents TX and TY(two arbitrary tangent vectors perpendicular to the normal and each other)
            //c2) going back to newton's method, we need the gradient and inverse hessian matrix: the gradient is (dN/dTX(depth(N)), dN/dTY(depth(N))
            //c3) zooming in on just the dN/dTX component: dN/dTX(depth(N)) = dN/dTX(extreme(N, shapeB)) - dN/dTX(extreme(-N, shapeA))
            //c4) derivative of max(a, b, c...) is just 'select the max element and take the derivative of it, so dN/dTX(extreme(N, shapeB) = dN/dTX(dot(N, shapeB[extremeIndex])) = dot(TX, shapeB[extremeIndex])
            //c5) dN/dTX(depth(N)) = dot(TX, shapeB[extremeIndex]) - dot(TX, shapeA[extremeIndex])
            //c6) gradient(depth(N)) = (dot(TX, shapeB[extremeIndex]) - dot(TX, shapeA[extremeIndex]), dot(TY, shapeB[extremeIndex]) - dot(TY, shapeA[extremeIndex]))
            //c7) For the hessian, derive each gradient component by both tangents again. Note that the tangent TX is bound to N, so dN/dTX(TX) != 0. Pushing N along TX rotates TX around TY, against the direction of N...
            //c8) dN/dTX(gradient.X) = dN/dTX(dot(TX, shapeB[extremeIndex])) - dN/dTX(dot(TX, shapeA[extremeIndex]) = dot(-N, shapeB[extremeIndex]) - dot(-N, shapeA[extremeIndex])
            //c9) dN/dTY(gradient.X) = dN/dTY(dot(TX, shapeB[extremeIndex])) - dN/dTY(dot(TX, shapeA[extremeIndex]) = 0
            //c10) dN/dTX(gradient.Y) = dN/dTX(dot(TY, shapeB[extremeIndex])) - dN/dTX(dot(TY, shapeA[extremeIndex]) = 0
            //c11) dN/dTY(gradient.Y) = dN/dTY(dot(TY, shapeB[extremeIndex])) - dN/dTY(dot(TY, shapeA[extremeIndex]) = dot(-N, shapeB[extremeIndex]) - dot(-N, shapeA[extremeIndex])
            //c12) the hessian is a uniform scaling matrix equal to negative depth; the inverse hessian is equivalent to just 1/(dot(-N, shapeB[extremeIndex]) - dot(-N, shapeA[extremeIndex]))

            //result: the newton step for minimizing depth for any pair of convex shapes is:
            //N1 = N0 - (dot(TX, shapeB[extremeIndex]) + dot(TX, shapeA[extremeIndex]), dot(TY, shapeB[extremeIndex]) + dot(TY, shapeA[extremeIndex])) / (dot(N, shapeA[extremeIndex]) - dot(N, shapeB[extremeIndex]))
            //Obviously taking some liberties here with the fact that the normal is being renormalized, but does it converge *well enough*?

            localNormal = initialGuess;
            for (int i = 0; i < 100; ++i)
            {
                //Normals are always calibrated to point from B to A.
                supportFinderB.ComputeSupport(b, localOrientationB, localNormal, out var extremeB);
                Vector3Wide.Add(extremeB, localOffsetB, out extremeB);
                Vector3Wide.Negate(localNormal, out var negatedNormal);
                supportFinderA.ComputeLocalSupport(a, negatedNormal, out var extremeA);

                Vector3Wide.Subtract(extremeB, extremeA, out var extremeOffset);
                Vector3Wide.Dot(extremeOffset, localNormal, out var depth);

                Helpers.BuildOrthnormalBasis(localNormal, out var x, out var y);
                Vector3Wide.Dot(x, extremeA, out var xDotA);
                Vector3Wide.Dot(y, extremeA, out var yDotA);
                Vector3Wide.Dot(x, extremeB, out var xDotB);
                Vector3Wide.Dot(y, extremeB, out var yDotB);

                //Protect against potential division by zero by clamping to a large finite value. Preserve sign (but note that the hessian is negative depth).
                var inverseHessian = Vector.ConditionalSelect(Vector.LessThan(depth, Vector<float>.Zero), Vector<float>.One, new Vector<float>(-1f)) /
                    Vector.Max(new Vector<float>(1e-10f), Vector.Abs(depth));
                Vector2Wide changeAlongTangents;
                changeAlongTangents.X = (xDotB - xDotA) * inverseHessian;
                changeAlongTangents.Y = (yDotB - yDotA) * inverseHessian;

                Vector3Wide newNormal;
                newNormal.X = localNormal.X + changeAlongTangents.X * x.X + changeAlongTangents.Y * y.X;
                newNormal.Y = localNormal.Y + changeAlongTangents.X * x.Y + changeAlongTangents.Y * y.Y;
                newNormal.Z = localNormal.Z + changeAlongTangents.X * x.Z + changeAlongTangents.Y * y.Z;

                Vector3Wide.Length(newNormal, out var normalLength);
                Vector3Wide.Scale(newNormal, Vector<float>.One / normalLength, out localNormal);
            }
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Test(
            ref CylinderWide a, ref CylinderWide b, ref Vector<float> speculativeMargin,
            ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount,
            out Convex4ContactManifoldWide manifold)
        {
            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var worldRA);
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var worldRB);
            //Work in b's local space.
            Matrix3x3Wide.MultiplyByTransposeWithoutOverlap(worldRA, worldRB, out var rA);
            ref var capsuleAxis = ref rA.Y;
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offsetB, worldRB, out var localOffsetB);
            Vector3Wide.Negate(localOffsetB, out var localOffsetA);

            //Cylinder-sphere was easy, cylinder-capsule was a bit complicated, and things go downhill from there.
            //The potential normal generators are:
            //capNormalA: trivial
            //capNormalB: trivial
            //sideA vs sideB: capNormalA x capNormalB
            //capEdgeA vs sideB: minimum depth of circle and line, slightly more complex than the distance based implementation in capsule-cylinder because cylinders don't have a spherical margin
            //sideA vs capEdgeB: minimum depth of circle and line again
            //capEdgeA vs capEdgeB: uh oh

            //For reference, the distance between a line and circle (capEdge vs side) involves finding the minimal root of a fourth degree polynomial. 
            //The distance between two circles involves finding the minimal root of an eighth degree polynomial.
            //(We aren't computing *distance* here, but rather minimal depth, but computing minimal depth is, at best, no easier than distance.)

            //It's possible to do something with a similar cost to compute minimal depth for the capEdge-side case, but that's about 150ns per bundle on a 3770K. 
            //If we used a similar implementation for capEdge-capEdge, it would likely cost at least 200ns, so just for normal generators we're looking at 150 + 150 + 200 = 500ns per bundle.
            //That's a long time, and it requires quite a bit of bespoke mathing.

            //Given that we are going to encounter similar difficulties on cylinder-box, cylinder-triangle, and cylinder-hull, can we use some kind of trick to save time and effort?
            //The common approach would be minkowski space methods like GJK and MPR. v1 used them heavily, but I moved away from them for v2 due to numerical concerns.
            //But many of those numerical concerns had to do with contact generation, not finding normals; we can still use custom analytic contact generators with GJK and friends.

            //A couple of notes on convexity:
            //Separated convex objects have a global minimum (negative) depth; the distance function between convex objects is itself convex. GJK takes advantage of this.
            //Intersecting convex objects, in contrast, may have multiple local minima for depth. MPR doesn't find a global minimum depth- in fact, it might not even be a local minimum.
            //You can still sometimes get away with an suboptimal choice of depth, though. The important thing is that colliding objects are always found to be colliding, and separated objects are always found to be separated.

            //GJK and MPR are not the only possible algorithms in this space. For example, consider gradient descent. Starting from an initial guess normal, you can incrementally walk toward a local minimum 
            //through numerical (or in some cases even analytic) computation of the gradient. Gradient descent isn't always the quickest at converging, but if we have reasonably good starting points it can work.

            //Further, at least at a conceptual level, pure gradient descent is just as powerful as GJK or MPR for the purpose of finding a good normal. Enumerating the possible cases:
            //1) Intersecting; GD finds intersection: this will find, at worst, a local minimum informed by any initial queries.
            //2) Separated; GD finds separation: the separating case is convex, so GD can find the global optimum (ignoring convergence speed)
            //3) Intersecting; GD finds separation: impossible by the separating axis theorem
            //4) Separated; GD finds intersection: impossible with sufficient time- again, due to separation convexity, GD will (eventually) converge to the global separated optimum

            //So we have some options.


            Vector3Wide.Broadcast(new Vector3(0, 1, 0), out var localNormal);

            manifold = default;

        }

        public void Test(ref CylinderWide a, ref CylinderWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public void Test(ref CylinderWide a, ref CylinderWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }
}
