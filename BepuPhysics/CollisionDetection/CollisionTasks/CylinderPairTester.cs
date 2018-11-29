using BepuPhysics.Collidables;
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
