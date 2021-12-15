using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.SweepTasks;
using BepuUtilities;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    using DepthRefiner = BepuPhysics.CollisionDetection.DepthRefiner<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>;

    public struct CylinderPairTester : IPairTester<CylinderWide, CylinderWide, Convex4ContactManifoldWide>
    {
        public int BatchSize => 16;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void ProjectOntoCapA(in Vector<float> capCenterBY, in Vector3Wide capCenterA, in Matrix3x3Wide rA, in Vector<float> inverseNDotAY, in Vector3Wide localNormal, in Vector2Wide point, out Vector2Wide projected)
        {
            Vector3Wide point3D;
            point3D.X = point.X;
            point3D.Y = capCenterBY;
            point3D.Z = point.Y;
            ProjectOntoCapA(capCenterA, rA, inverseNDotAY, localNormal, point3D, out projected);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void ProjectOntoCapA(in Vector3Wide capCenterA, in Matrix3x3Wide rA, in Vector<float> inverseNDotAY, in Vector3Wide localNormal, in Vector3Wide point, out Vector2Wide projected)
        {
            Vector3Wide.Subtract(capCenterA, point, out var pointToCapCenterA);
            Vector3Wide.Dot(pointToCapCenterA, rA.Y, out var tDistance);
            var tBOnA = tDistance * inverseNDotAY;
            Vector3Wide.Scale(localNormal, tBOnA, out var projectionOffsetB);
            Vector3Wide.Add(point, projectionOffsetB, out var projectedPoint);
            Vector3Wide.Subtract(projectedPoint, capCenterA, out var capCenterAToProjectedPoint);
            Vector3Wide.Dot(capCenterAToProjectedPoint, rA.X, out projected.X);
            Vector3Wide.Dot(capCenterAToProjectedPoint, rA.Z, out projected.Y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void ProjectOntoCapB(in Vector<float> capCenterBY, in Vector<float> inverseLocalNormalY, in Vector3Wide localNormal, in Vector3Wide point, out Vector2Wide projected)
        {
            var tAOnB = (point.Y - capCenterBY) * inverseLocalNormalY;
            projected.X = point.X - localNormal.X * tAOnB;
            projected.Y = point.Z - localNormal.Z * tAOnB;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void IntersectLineCircle(in Vector2Wide linePosition, in Vector2Wide lineDirection, in Vector<float> radius, out Vector<float> tMin, out Vector<float> tMax)
        {
            //||linePosition + lineDirection * t|| = radius
            //dot(linePosition + lineDirection * t, linePosition + lineDirection * t) = radius * radius
            //dot(linePosition, linePosition) - radius * radius + t * 2 * dot(linePosition, lineDirection) + t^2 * dot(lineDirection, lineDirection) = 0
            Vector2Wide.Dot(lineDirection, lineDirection, out var a);
            var inverseA = Vector<float>.One / a;
            Vector2Wide.Dot(linePosition, lineDirection, out var b);
            Vector2Wide.Dot(linePosition, linePosition, out var c);
            var radiusSquared = radius * radius;
            c -= radiusSquared;
            var tOffset = Vector.SquareRoot(Vector.Max(Vector<float>.Zero, b * b - a * c)) * inverseA;
            var tBase = -b * inverseA;
            tMin = tBase - tOffset;
            tMax = tBase + tOffset;
            //If the projected line direction is zero, just compress the interval to tBase.
            var useFallback = Vector.LessThan(Vector.Abs(a), new Vector<float>(1e-12f));
            tMin = Vector.ConditionalSelect(useFallback, tBase, tMin);
            tMax = Vector.ConditionalSelect(useFallback, tBase, tMax);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void FromCapBTo3D(in Vector2Wide contact, in Vector<float> capCenterBY, out Vector3Wide localContact3D)
        {
            localContact3D.X = contact.X;
            localContact3D.Y = capCenterBY;
            localContact3D.Z = contact.Y;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void TransformContact(
            in Vector3Wide contact, in Vector3Wide localFeaturePositionA, in Vector3Wide localFeatureNormalA, Vector<float> inverseFeatureNormalADotLocalNormal,
            in Vector3Wide localOffsetB, in Matrix3x3Wide orientationB, Vector<float> negativeSpeculativeMargin,
            out Vector3Wide aToContact, out Vector<float> depth, ref Vector<int> contactExists)
        {
            //While we have computed a global depth, each contact has its own depth.
            //Project the contact on B along the contact normal to the 'face' of A.
            //The full computation is: 
            //t = dot(contact - featurePositionA, featureNormalA) / dot(featureNormalA, localNormal)
            //depth = dot(t * localNormal, localNormal) = t
            Vector3Wide.Subtract(contact, localFeaturePositionA, out var featureOffset);
            Vector3Wide.Dot(featureOffset, localFeatureNormalA, out var tDistance);
            depth = tDistance * inverseFeatureNormalADotLocalNormal;
            Vector3Wide.Add(contact, localOffsetB, out var localAToContact);
            Matrix3x3Wide.TransformWithoutOverlap(localAToContact, orientationB, out aToContact);
            contactExists = Vector.BitwiseAnd(contactExists, Vector.GreaterThanOrEqual(depth, negativeSpeculativeMargin));
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

            //We can't actually use the same implementation as capsule-cylinder because it assumed nonnegative distance. In the intersecting case, it uses an approximation-
            //that's fine for a capsule which has a large margin, but it can't work for two cylinders.
            //Cylinder-cylinder has to handle the intersecting case accurately, which means you're stuck (conceptually) enumerating 4 or 8 roots per feature pair.
            //There are some pretty fast solvers around, but it's hard to avoid quite a few square roots given the shapes involved.
            //Plus, it would require a lot of special case mathing.

            //Given that we are going to encounter similar difficulties on cylinder-box, cylinder-triangle, and cylinder-hull, can we use some kind of trick to save time and effort?
            //The common approach would be minkowski space methods like GJK and MPR. v1 used them heavily, but I moved away from them for v2 due to numerical concerns.
            //But many of those numerical concerns had to do with contact generation, not finding normals; we can still use custom analytic contact generators with GJK and friends.

            //A couple of notes on convexity:
            //Separated convex objects have a global minimum (negative) depth; the distance function between convex objects is itself convex. GJK takes advantage of this.
            //Intersecting convex objects, in contrast, may have multiple local minima for depth. MPR doesn't find a global minimum depth- in fact, it might not even be a local minimum.

            //GJK and MPR are not the only possible algorithms in this space: 
            //the DepthRefiner uses an algorithm which I'll call Tootbird Search, in the hopes that someone will refer to "Tootbird Search" in a paper or presentation at some point.

            //Here, we'll make a few guesses at the best normal, then hand off the result to a minkowski based depth refiner.

            Vector3Wide.Length(localOffsetA, out var length);
            Vector3Wide.Scale(localOffsetA, Vector<float>.One / length, out var localNormal);
            var useInitialSampleFallback = Vector.LessThan(length, new Vector<float>(1e-10f));
            localNormal.X = Vector.ConditionalSelect(useInitialSampleFallback, Vector<float>.Zero, localNormal.X);
            localNormal.Y = Vector.ConditionalSelect(useInitialSampleFallback, Vector<float>.One, localNormal.Y);
            localNormal.Z = Vector.ConditionalSelect(useInitialSampleFallback, Vector<float>.Zero, localNormal.Z);
            CylinderSupportFinder supportFinder;

            //There are some other axes we could try, like axisA x axisB, but they tend to be far less relevant for stacking.
            //Also, keep in mind that feature pairs which form faces, edges, or vertices in minkowski space tend to converge extremely quickly.
            //Blindly trying a bunch of low probability feature pairs- especially those which would fall into the above- isn't very wise.

            //We now have a decent estimate for the local normal and an initial simplex to work from. Refine it to a local minimum.
            var inactiveLanes = BundleIndexing.CreateTrailingMaskForCountInBundle(pairCount);

            var depthThreshold = -speculativeMargin;
            var epsilonScale = Vector.Min(Vector.Max(a.HalfLength, a.Radius), Vector.Max(b.HalfLength, b.Radius));
            DepthRefiner.FindMinimumDepth(
                b, a, localOffsetA, rA, ref supportFinder, ref supportFinder, localNormal, inactiveLanes, epsilonScale * new Vector<float>(1e-6f), depthThreshold,
                out var depth, out localNormal, out var closestOnB, maximumIterations: 25);

            inactiveLanes = Vector.BitwiseOr(inactiveLanes, Vector.LessThan(depth, depthThreshold));
            if (Vector.LessThanAll(inactiveLanes, Vector<int>.Zero))
            {
                //All lanes are either inactive or were found to have a depth lower than the speculative margin, so we can just quit early.
                manifold = default;
                return;
            }

            //We generate contacts according to the dominant features along the collision normal.
            //The possible pairs are:
            //Cap A-Cap B
            //Cap A-Side B
            //Side A-Cap B
            //Side A-Side B
            Vector3Wide.Dot(rA.Y, localNormal, out var nDotAY);
            var inverseNDotAY = Vector<float>.One / nDotAY;
            var inverseLocalNormalY = Vector<float>.One / localNormal.Y;
            Vector3Wide.Scale(rA.Y, Vector.ConditionalSelect(Vector.GreaterThan(nDotAY, Vector<float>.Zero), -a.HalfLength, a.HalfLength), out var capCenterA);
            Vector3Wide.Add(capCenterA, localOffsetA, out capCenterA);
            var capCenterBY = Vector.ConditionalSelect(Vector.LessThan(localNormal.Y, Vector<float>.Zero), -b.HalfLength, b.HalfLength);

            var capThreshold = new Vector<float>(0.70710678118f);
            var useCapA = Vector.GreaterThan(Vector.Abs(nDotAY), capThreshold);
            var useCapB = Vector.GreaterThan(Vector.Abs(localNormal.Y), capThreshold);
            Unsafe.SkipInit(out Vector3Wide contact0);
            Unsafe.SkipInit(out Vector3Wide contact1);
            Unsafe.SkipInit(out Vector3Wide contact2);
            Unsafe.SkipInit(out Vector3Wide contact3);
            manifold.Contact0Exists = default;
            manifold.Contact1Exists = default;
            manifold.Contact2Exists = default;
            manifold.Contact3Exists = default;
            var useCapCap = Vector.AndNot(Vector.BitwiseAnd(useCapA, useCapB), inactiveLanes);

            //The extreme points along the contact normal are shared between multiple contact generator paths, so we just do them up front.
            Vector3Wide.Scale(localNormal, -depth, out var bToAOffset);
            Vector3Wide.Add(closestOnB, bToAOffset, out var extremeA);
            Vector3Wide.Subtract(extremeA, localOffsetA, out var extremeAHorizontalOffset);
            Vector3Wide.Dot(extremeAHorizontalOffset, rA.Y, out var verticalDot);
            Vector3Wide.Scale(rA.Y, verticalDot, out var toRemove);
            Vector3Wide.Subtract(extremeAHorizontalOffset, toRemove, out extremeAHorizontalOffset);

            Vector2Wide extremeB;
            extremeB.X = closestOnB.X;
            extremeB.Y = closestOnB.Z;

            var useNegative = Vector.GreaterThan(nDotAY, Vector<float>.Zero);
            Vector3Wide capFeatureNormalA;
            capFeatureNormalA.X = Vector.ConditionalSelect(useNegative, -rA.Y.X, rA.Y.X);
            capFeatureNormalA.Y = Vector.ConditionalSelect(useNegative, -rA.Y.Y, rA.Y.Y);
            capFeatureNormalA.Z = Vector.ConditionalSelect(useNegative, -rA.Y.Z, rA.Y.Z);
            //We'll just assume every lane is cap-cap to start with; initialize the feature normal accordingly.
            var featureNormalA = capFeatureNormalA;
            var featurePositionA = capCenterA;

            if (Vector.LessThanAny(useCapCap, Vector<int>.Zero))
            {
                //At least one active lane needs cap-cap contacts.
                //While the obvious approach is to intersect the cylinder caps, doing so on the contact plane involves intersecting two ellipses.
                //That involves solving a fourth degree polynomial. Doable, but not super duper cheap. Further, when the caps are not parallel, 
                //intersecting the caps doesn't necessarily produce 'better' contacts. It's just a heuristic.

                //Instead, we'll start with extreme points, use them to draw a line, test that line against the caps, 
                //and then use a perpendicular line at the midpoint of the first line's intersection interval.
                const float parallelThresholdScalar = 0.9999f;
                const float parallelInterpolationMaxScalar = 0.99995f;
                const float inverseParallelInterpolationSpanScalar = 1f / (parallelInterpolationMaxScalar - parallelThresholdScalar);
                var parallelThreshold = new Vector<float>(parallelThresholdScalar);
                var inverseParallelInterpolationSpan = new Vector<float>(inverseParallelInterpolationSpanScalar);
                var absADot = Vector.Abs(nDotAY);
                var absBDot = Vector.Abs(localNormal.Y);
                var aCapNotParallel = Vector.LessThan(absADot, parallelThreshold);
                var bCapNotParallel = Vector.LessThan(absBDot, parallelThreshold);

                //If both caps are not parallel, we'll use the deepest point on B.
                Vector2Wide capContact0 = extremeB;

                //Only use one contact if neither cap face is involved.
                var bothNotParallel = Vector.BitwiseAnd(aCapNotParallel, bCapNotParallel);
                if (Vector.LessThanAny(Vector.AndNot(Vector.OnesComplement(bothNotParallel), inactiveLanes), Vector<int>.Zero))
                {
                    //The local normal is aligned with at least one of the two cap normals.     
                    ProjectOntoCapB(capCenterBY, inverseLocalNormalY, localNormal, capCenterA, out var capCenterAOnB);
                    Vector2Wide.Length(capCenterAOnB, out var horizontalOffsetLength);
                    var inverseHorizontalOffsetLength = Vector<float>.One / horizontalOffsetLength;
                    Vector2Wide.Scale(capCenterAOnB, inverseHorizontalOffsetLength, out var horizontalOffsetDirection);
                    var useBothParallelFallback = Vector.LessThan(horizontalOffsetLength, new Vector<float>(1e-14f));
                    horizontalOffsetDirection.X = Vector.ConditionalSelect(useBothParallelFallback, Vector<float>.One, horizontalOffsetDirection.X);
                    horizontalOffsetDirection.Y = Vector.ConditionalSelect(useBothParallelFallback, Vector<float>.Zero, horizontalOffsetDirection.Y);
                    Vector2Wide.Scale(horizontalOffsetDirection, b.Radius, out var initialLineStart);
                                       
                    Vector2Wide.Negate(initialLineStart, out var contact1LineEndpoint);
                    //The above created a line stating at contact0 and pointing to the other side of the cap that contact0 was generated from.
                    //That is, if the extreme point from A was used, then the line direction is localNormal projected on capA.
                    ProjectOntoCapA(capCenterBY, capCenterA, rA, inverseNDotAY, localNormal, initialLineStart, out var lineStartOnA);
                    ProjectOntoCapA(capCenterBY, capCenterA, rA, inverseNDotAY, localNormal, contact1LineEndpoint, out var lineEndOnA);
                    Vector2Wide.Subtract(lineEndOnA, lineStartOnA, out var lineDirectionOnA);
                    Vector2Wide.Subtract(contact1LineEndpoint, initialLineStart, out var contact1LineDirectionOnB);
                    IntersectLineCircle(lineStartOnA, lineDirectionOnA, a.Radius, out var contact1TMinA, out var contact1TMaxA);
                    var firstLineTMin = Vector.Max(contact1TMinA, Vector<float>.Zero);
                    var firstLineTMax = Vector.Min(contact1TMaxA, Vector<float>.One);
                    Vector2Wide.Scale(contact1LineDirectionOnB, firstLineTMin, out capContact0);
                    Vector2Wide.Add(initialLineStart, capContact0, out capContact0);
                    Vector2Wide.Scale(contact1LineDirectionOnB, firstLineTMax, out var capContact1);
                    Vector2Wide.Add(initialLineStart, capContact1, out capContact1);

                    //Just use a perpendicular direction to the centerB->centerA offset and position it to get a decent cross section of the contact manifold- 
                    //at point where it would find the A-B circle intersections, or at the midpoint of A if the circle intersections are further out.
                    //Note that for not-entirely-parallel faces, it won't quite match up with the true intersections since the projected cap of A is an ellipse.
                    var circleIntersectionT = 0.5f * (horizontalOffsetLength + (b.Radius * b.Radius - a.Radius * a.Radius) * inverseHorizontalOffsetLength);
                    var secondLineStartT = Vector.Min(horizontalOffsetLength, Vector.Max(Vector<float>.Zero, circleIntersectionT));
                    Vector2Wide.Scale(horizontalOffsetDirection, secondLineStartT, out var secondLineStartOnB);
                    Vector2Wide secondLineDirectionOnB;
                    secondLineDirectionOnB.X = horizontalOffsetDirection.Y;
                    secondLineDirectionOnB.Y = -horizontalOffsetDirection.X;

                    Vector2Wide.Add(secondLineStartOnB, secondLineDirectionOnB, out var secondLineEndOnB);
                    ProjectOntoCapA(capCenterBY, capCenterA, rA, inverseNDotAY, localNormal, secondLineStartOnB, out var secondLineStartOnA);
                    ProjectOntoCapA(capCenterBY, capCenterA, rA, inverseNDotAY, localNormal, secondLineEndOnB, out var secondLineEndOnA);
                    Vector2Wide.Subtract(secondLineEndOnA, secondLineStartOnA, out var secondLineDirectionOnA);

                    IntersectLineCircle(secondLineStartOnA, secondLineDirectionOnA, a.Radius, out var secondLineTMinA, out var secondLineTMaxA);
                    IntersectLineCircle(secondLineStartOnB, secondLineDirectionOnB, b.Radius, out var secondLineTMinB, out var secondLineTMaxB);
                    var secondLineTMin = Vector.Max(secondLineTMinA, secondLineTMinB);
                    var secondLineTMax = Vector.Min(secondLineTMaxA, secondLineTMaxB);

                    Vector2Wide.Scale(secondLineDirectionOnB, secondLineTMin, out var capContact2);
                    Vector2Wide.Scale(secondLineDirectionOnB, secondLineTMax, out var capContact3);
                    Vector2Wide.Add(secondLineStartOnB, capContact2, out capContact2);
                    Vector2Wide.Add(secondLineStartOnB, capContact3, out capContact3);

                    //We've built everything assuming perfectly parallel caps. The manifold we have so far may not include the deepest points.
                    //Replace one of the four points with the deepest point (extremeB) if necessary.
                    var weightAParallel = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, (absADot - parallelThreshold) * inverseParallelInterpolationSpan));
                    var weightBParallel = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, (absBDot - parallelThreshold) * inverseParallelInterpolationSpan));
                    var parallelWeight = weightAParallel * weightBParallel;
                    var extremeWeight = Vector<float>.One - parallelWeight;
                    Vector2Wide.Subtract(extremeB, secondLineStartOnB, out var manifoldCenterToExtremeB);
                    var replaceDot0 = horizontalOffsetDirection.X * manifoldCenterToExtremeB.X + horizontalOffsetDirection.Y * manifoldCenterToExtremeB.Y;
                    var replaceDot2 = secondLineDirectionOnB.X * manifoldCenterToExtremeB.X + secondLineDirectionOnB.Y * manifoldCenterToExtremeB.Y;
                    var replace0Or1 = Vector.GreaterThan(Vector.Abs(replaceDot0), Vector.Abs(replaceDot2));
                    //var replace0 = Vector.BitwiseAnd(Vector.GreaterThan(replaceDot0, Vector<float>.Zero), replace0Or1);
                    var replace0 = Vector.BitwiseOr(bothNotParallel, Vector.BitwiseAnd(Vector.GreaterThan(replaceDot0, Vector<float>.Zero), replace0Or1));
                    var replace1 = Vector.BitwiseAnd(Vector.LessThanOrEqual(replaceDot0, Vector<float>.Zero), replace0Or1);
                    var replace2 = Vector.AndNot(Vector.LessThan(replaceDot2, Vector<float>.Zero), replace0Or1);
                    var replace3 = Vector.AndNot(Vector.GreaterThanOrEqual(replaceDot2, Vector<float>.Zero), replace0Or1);
                    capContact0.X = Vector.ConditionalSelect(replace0, extremeB.X * extremeWeight + capContact0.X * parallelWeight, capContact0.X);
                    capContact0.Y = Vector.ConditionalSelect(replace0, extremeB.Y * extremeWeight + capContact0.Y * parallelWeight, capContact0.Y);
                    capContact1.X = Vector.ConditionalSelect(replace1, extremeB.X * extremeWeight + capContact1.X * parallelWeight, capContact1.X);
                    capContact1.Y = Vector.ConditionalSelect(replace1, extremeB.Y * extremeWeight + capContact1.Y * parallelWeight, capContact1.Y);
                    capContact2.X = Vector.ConditionalSelect(replace2, extremeB.X * extremeWeight + capContact2.X * parallelWeight, capContact2.X);
                    capContact2.Y = Vector.ConditionalSelect(replace2, extremeB.Y * extremeWeight + capContact2.Y * parallelWeight, capContact2.Y);
                    capContact3.X = Vector.ConditionalSelect(replace3, extremeB.X * extremeWeight + capContact3.X * parallelWeight, capContact3.X);
                    capContact3.Y = Vector.ConditionalSelect(replace3, extremeB.Y * extremeWeight + capContact3.Y * parallelWeight, capContact3.Y);


                    FromCapBTo3D(capContact1, capCenterBY, out contact1);
                    FromCapBTo3D(capContact2, capCenterBY, out contact2);
                    FromCapBTo3D(capContact3, capCenterBY, out contact3);
                    manifold.Contact1Exists = Vector.AndNot(Vector.BitwiseAnd(useCapCap, Vector.GreaterThan(firstLineTMax, firstLineTMin)), bothNotParallel);
                    //If 0 and 1 are in the same spot, there aren't going to be any useful additional contacts.
                    manifold.Contact2Exists = manifold.Contact1Exists;
                    manifold.Contact3Exists = Vector.BitwiseAnd(manifold.Contact1Exists, Vector.GreaterThan(secondLineTMax, secondLineTMin));
                }
                FromCapBTo3D(capContact0, capCenterBY, out contact0);
                manifold.Contact0Exists = useCapCap;
            }
            var useCapSide = Vector.AndNot(Vector.BitwiseOr(Vector.AndNot(useCapA, useCapB), Vector.AndNot(useCapB, useCapA)), inactiveLanes);
            //The side normal is used in both of the following contact generator cases.
            Vector3Wide.Dot(rA.X, localNormal, out var ax);
            Vector3Wide.Dot(rA.Z, localNormal, out var az);
            var horizontalNormalLengthA = Vector.SquareRoot(ax * ax + az * az);
            var inverseHorizontalNormalLengthA = Vector<float>.One / horizontalNormalLengthA;
            var xScale = ax * inverseHorizontalNormalLengthA;
            var zScale = az * inverseHorizontalNormalLengthA;
            Vector3Wide.Scale(rA.X, xScale, out var sideFeatureNormalAX);
            Vector3Wide.Scale(rA.Z, zScale, out var sideFeatureNormalAZ);
            Vector3Wide.Add(sideFeatureNormalAX, sideFeatureNormalAZ, out var sideFeatureNormalA);
            Vector3Wide sideCenterA, sideCenterB;
            sideCenterA.X = extremeAHorizontalOffset.X + localOffsetA.X;
            sideCenterA.Y = extremeAHorizontalOffset.Y + localOffsetA.Y;
            sideCenterA.Z = extremeAHorizontalOffset.Z + localOffsetA.Z;
            sideCenterB.X = extremeB.X;
            sideCenterB.Y = Vector<float>.Zero;
            sideCenterB.Z = extremeB.Y;
            if (Vector.LessThanAny(useCapSide, Vector<int>.Zero))
            {
                //At least one lane needs cap-side contact generation.
                //This is relatively simple:
                //Pick a line on the side of the cylinder in the direction of the extreme point.
                //Pick the cap on the other cylinder in the direction of the extreme point.
                //Intersect the line with the cap in the cap's local space (by projecting the line along the contact normal into the cap's local space).

                //Compute both lines and choose which one to use.
                Vector3Wide.Add(sideCenterA, rA.Y, out var sideLineEndA);
                Vector3Wide sideLineEndB;
                sideLineEndB.X = sideCenterB.X;
                sideLineEndB.Y = Vector<float>.One;
                sideLineEndB.Z = sideCenterB.Z;
                ProjectOntoCapA(capCenterA, rA, inverseNDotAY, localNormal, sideCenterB, out var projectedLineStartBOnA);
                ProjectOntoCapB(capCenterBY, inverseLocalNormalY, localNormal, sideCenterA, out var projectedLineStartAOnB);
                ProjectOntoCapA(capCenterA, rA, inverseNDotAY, localNormal, sideLineEndB, out var projectedLineEndBOnA);
                ProjectOntoCapB(capCenterBY, inverseLocalNormalY, localNormal, sideLineEndA, out var projectedLineEndAOnB);

                Vector2Wide.ConditionalSelect(useCapA, projectedLineStartBOnA, projectedLineStartAOnB, out var projectedLineStart);
                Vector2Wide.ConditionalSelect(useCapA, projectedLineEndBOnA, projectedLineEndAOnB, out var projectedLineEnd);
                var radius = Vector.ConditionalSelect(useCapA, a.Radius, b.Radius);
                var sideHalfLength = Vector.ConditionalSelect(useCapA, b.HalfLength, a.HalfLength);
                Vector2Wide.Subtract(projectedLineEnd, projectedLineStart, out var projectedLineDirection);
                IntersectLineCircle(projectedLineStart, projectedLineDirection, radius, out var tMin, out var tMax);
                tMin = Vector.Min(sideHalfLength, Vector.Max(-sideHalfLength, tMin));
                tMax = Vector.Min(sideHalfLength, tMax);

                //To be consistent with the other contact generation cases, we want contacts to be on cylinder B. So:
                //If the cap was on A, that means the side was B and we should scale the sideLineB to get our contacts.
                //If the cap was on B, that means the side was on A and we should use the *projected* sideLineA to get our contacts.
                Vector3Wide contact0ForCapA;
                contact0ForCapA.X = sideCenterB.X;
                contact0ForCapA.Y = tMin;
                contact0ForCapA.Z = sideCenterB.Z;
                Vector3Wide contact1ForCapA;
                contact1ForCapA.X = sideCenterB.X;
                contact1ForCapA.Y = tMax;
                contact1ForCapA.Z = sideCenterB.Z;
                //Note that we're using the post-conditionalselect projectedLineStart and direction here.
                //That's fine; we only use these results if that choice is actually correct anyway.
                Vector3Wide contact0ForCapB;
                contact0ForCapB.X = projectedLineStart.X + tMin * projectedLineDirection.X;
                contact0ForCapB.Y = capCenterBY;
                contact0ForCapB.Z = projectedLineStart.Y + tMin * projectedLineDirection.Y;
                Vector3Wide contact1ForCapB;
                contact1ForCapB.X = projectedLineStart.X + tMax * projectedLineDirection.X;
                contact1ForCapB.Y = capCenterBY;
                contact1ForCapB.Z = projectedLineStart.Y + tMax * projectedLineDirection.Y;
                Vector3Wide.ConditionalSelect(useCapA, contact0ForCapA, contact0ForCapB, out var capSideContact0);
                Vector3Wide.ConditionalSelect(useCapA, contact1ForCapA, contact1ForCapB, out var capSideContact1);
                Vector3Wide.ConditionalSelect(useCapSide, capSideContact0, contact0, out contact0);
                Vector3Wide.ConditionalSelect(useCapSide, capSideContact1, contact1, out contact1);
                manifold.Contact0Exists = Vector.ConditionalSelect(useCapSide, new Vector<int>(-1), manifold.Contact0Exists);
                manifold.Contact1Exists = Vector.ConditionalSelect(useCapSide, Vector.GreaterThan(tMax, tMin), manifold.Contact1Exists);

                Vector3Wide.ConditionalSelect(useCapA, capFeatureNormalA, sideFeatureNormalA, out var capSideFeatureNormalA);
                Vector3Wide.ConditionalSelect(useCapSide, capSideFeatureNormalA, featureNormalA, out featureNormalA);

                Vector3Wide.ConditionalSelect(useCapA, capCenterA, sideCenterA, out var capSideFeaturePositionA);
                Vector3Wide.ConditionalSelect(useCapSide, capSideFeaturePositionA, featurePositionA, out featurePositionA);
            }
            var useSideSide = Vector.AndNot(Vector.AndNot(Vector.OnesComplement(useCapA), useCapB), inactiveLanes);
            if (Vector.LessThanAny(useSideSide, Vector<int>.Zero))
            {
                //At least one lane needs side-side contacts.
                //This is similar to capsule-capsule; we have two line segments and we want a contact interval on B.
                //Note that we test the line on the side of A against the line *in the center of* B. We then use its interval on the side of B.
                //(Using the side of B to detect the interval can result in issues in medium-depth penetration where the deepest point is ignored in favor of the closest point between the side lines.)
                Vector3Wide.Negate(sideCenterA, out var sideCenterAToLineB);
                var horizontalNormalLengthSquaredB = localNormal.X * localNormal.X + localNormal.Z * localNormal.Z;
                var inverseHorizontalNormalLengthSquaredB = Vector<float>.One / horizontalNormalLengthSquaredB;
                CapsuleCylinderTester.GetContactIntervalBetweenSegments(a.HalfLength, b.HalfLength, rA.Y, localNormal, inverseHorizontalNormalLengthSquaredB, sideCenterAToLineB, out var contactTMin, out var contactTMax);

                contact0.X = Vector.ConditionalSelect(useSideSide, extremeB.X, contact0.X);
                contact0.Y = Vector.ConditionalSelect(useSideSide, contactTMin, contact0.Y);
                contact0.Z = Vector.ConditionalSelect(useSideSide, extremeB.Y, contact0.Z);
                contact1.X = Vector.ConditionalSelect(useSideSide, extremeB.X, contact1.X);
                contact1.Y = Vector.ConditionalSelect(useSideSide, contactTMax, contact1.Y);
                contact1.Z = Vector.ConditionalSelect(useSideSide, extremeB.Y, contact1.Z);
                manifold.Contact0Exists = Vector.ConditionalSelect(useSideSide, new Vector<int>(-1), manifold.Contact0Exists);
                manifold.Contact1Exists = Vector.ConditionalSelect(useSideSide, Vector.GreaterThan(contactTMax, contactTMin), manifold.Contact1Exists);
                Vector3Wide.ConditionalSelect(useSideSide, sideFeatureNormalA, featureNormalA, out featureNormalA);
                Vector3Wide.ConditionalSelect(useSideSide, sideCenterA, featurePositionA, out featurePositionA);
            }
            Vector3Wide.Dot(featureNormalA, localNormal, out var featureNormalADotLocalNormal);
            //No division guard; the feature normal we picked is never more than 45 degrees away from the local normal.
            var inverseFeatureNormalADotLocalNormal = Vector<float>.One / featureNormalADotLocalNormal;
            var negativeSpeculativeMargin = -speculativeMargin;
            TransformContact(contact0, featurePositionA, featureNormalA, inverseFeatureNormalADotLocalNormal, localOffsetB, worldRB, negativeSpeculativeMargin, out manifold.OffsetA0, out manifold.Depth0, ref manifold.Contact0Exists);
            TransformContact(contact1, featurePositionA, featureNormalA, inverseFeatureNormalADotLocalNormal, localOffsetB, worldRB, negativeSpeculativeMargin, out manifold.OffsetA1, out manifold.Depth1, ref manifold.Contact1Exists);
            TransformContact(contact2, featurePositionA, featureNormalA, inverseFeatureNormalADotLocalNormal, localOffsetB, worldRB, negativeSpeculativeMargin, out manifold.OffsetA2, out manifold.Depth2, ref manifold.Contact2Exists);
            TransformContact(contact3, featurePositionA, featureNormalA, inverseFeatureNormalADotLocalNormal, localOffsetB, worldRB, negativeSpeculativeMargin, out manifold.OffsetA3, out manifold.Depth3, ref manifold.Contact3Exists);
            Matrix3x3Wide.TransformWithoutOverlap(localNormal, worldRB, out manifold.Normal);
            //Contact generators all obey a reasonably solid order, so we can use a trivial feature description.
            //Note that this will conflate different contact generator cases, but that's okay- we tend to keep the most important contact in the first slot, for example, and that is the contact that is most likely to persist across feature pair changes.
            manifold.FeatureId0 = Vector<int>.Zero;
            manifold.FeatureId1 = Vector<int>.One;
            manifold.FeatureId2 = new Vector<int>(2);
            manifold.FeatureId3 = new Vector<int>(3);
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