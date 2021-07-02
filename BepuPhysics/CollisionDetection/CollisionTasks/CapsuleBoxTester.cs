using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct CapsuleBoxTester : IPairTester<CapsuleWide, BoxWide, Convex2ContactManifoldWide>
    {
        public int BatchSize => 32;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void Prepare(
            ref CapsuleWide a, ref BoxWide b, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
            out Vector3Wide localOffsetA, out Vector3Wide capsuleAxis, out Vector3Wide edgeCenters)
        {
            QuaternionWide.Conjugate(orientationB, out var toLocalB);
            QuaternionWide.TransformWithoutOverlap(offsetB, toLocalB, out localOffsetA);
            Vector3Wide.Negate(ref localOffsetA);
            QuaternionWide.ConcatenateWithoutOverlap(orientationA, toLocalB, out var boxLocalOrientationA);
            capsuleAxis = QuaternionWide.TransformUnitY(boxLocalOrientationA);

            //Get the closest point on the capsule segment to the box center to choose which edge to use.
            //(Pointless to test the other 9; they're guaranteed to be further away.)
            //closestPointOnCapsuleToBoxPosition = clamp((boxPosition - capsulePosition) * capsuleAxis, halfLength) * capsuleAxis + capsulePosition
            //offsetFromBoxToCapsule = closestPointOnCapsuleToBoxPosition - boxPosition
            //offsetFromBoxToCapsule = clamp(-localOffsetA * capsuleAxis, halfLength) * capsuleAxis + localOffsetA
            //offsetFromBoxToCapsule = localOffsetA - clamp(localOffsetA * capsuleAxis, halfLength) * capsuleAxis

            Vector3Wide.Dot(localOffsetA, capsuleAxis, out var dot);
            var clampedDot = Vector.Min(a.HalfLength, Vector.Max(-a.HalfLength, dot));
            Vector3Wide.Scale(capsuleAxis, clampedDot, out var offsetToCapsuleFromBox);
            Vector3Wide.Subtract(localOffsetA, offsetToCapsuleFromBox, out offsetToCapsuleFromBox);
            edgeCenters.X = Vector.ConditionalSelect(Vector.LessThan(offsetToCapsuleFromBox.X, Vector<float>.Zero), -b.HalfWidth, b.HalfWidth);
            edgeCenters.Y = Vector.ConditionalSelect(Vector.LessThan(offsetToCapsuleFromBox.Y, Vector<float>.Zero), -b.HalfHeight, b.HalfHeight);
            edgeCenters.Z = Vector.ConditionalSelect(Vector.LessThan(offsetToCapsuleFromBox.Z, Vector<float>.Zero), -b.HalfLength, b.HalfLength);
        }

        //Hideous parameter list because this function is used with swizzled vectors.
        //It's built to handle the Z edge hardcoded, so we just reorient things at the point of use to handle X and Y.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static void TestBoxEdge(
            ref Vector<float> offsetAX, ref Vector<float> offsetAY, ref Vector<float> offsetAZ,
            ref Vector<float> capsuleAxisX, ref Vector<float> capsuleAxisY, ref Vector<float> capsuleAxisZ,
            ref Vector<float> capsuleHalfLength, ref Vector<float> boxEdgeCenterX, ref Vector<float> boxEdgeCenterY,
            ref Vector<float> boxHalfWidth, ref Vector<float> boxHalfHeight, ref Vector<float> boxHalfLength,
            out Vector<float> taMin, out Vector<float> taMax, out Vector3Wide closestPointOnA,
            out Vector<float> nX, out Vector<float> nY, out Vector<float> nZ,
            out Vector<float> ta, out Vector<float> epsilon)
        {
            //From CapsulePairCollisionTask, point of closest approach along the capsule axis, unbounded:
            //ta = (da * (b - a) + (db * (a - b)) * (da * db)) / (1 - ((da * db) * (da * db))  
            //where da = capsuleAxis, db = (0,0,1), a = offsetA, b = (boxEdgeCenterX, boxEdgeCenterY, 0)
            //so da * db is simply capsuleAxis.Z.
            var abX = boxEdgeCenterX - offsetAX;
            var abY = boxEdgeCenterY - offsetAY;
            var daOffsetB = capsuleAxisX * abX + capsuleAxisY * abY - capsuleAxisZ * offsetAZ;
            //Note potential division by zero. Even though ta is not mathematically relevant when parallel (or even coplanar),
            //the max is a very easy way to stop NaNs from infecting later calculations.
            ta = (daOffsetB + offsetAZ * capsuleAxisZ) / Vector.Max(new Vector<float>(1e-15f), Vector<float>.One - capsuleAxisZ * capsuleAxisZ);
            //tb = ta * (da * db) - db * (b - a)
            var tb = ta * capsuleAxisZ + offsetAZ;

            //Clamp solution to valid regions on edge line segment.
            //B onto A: +-BHalfLength * (da * db) + da * offsetB
            //A onto B: +-AHalfLength * (da * db) + db * offsetA
            var absdadb = Vector.Abs(capsuleAxisZ);
            var bOntoAOffset = boxHalfLength * absdadb;
            var aOntoBOffset = capsuleHalfLength * absdadb;
            taMin = Vector.Max(-capsuleHalfLength, Vector.Min(capsuleHalfLength, daOffsetB - bOntoAOffset));
            taMax = Vector.Min(capsuleHalfLength, Vector.Max(-capsuleHalfLength, daOffsetB + bOntoAOffset));
            var bMin = Vector.Max(-boxHalfLength, Vector.Min(boxHalfLength, offsetAZ - aOntoBOffset));
            var bMax = Vector.Min(boxHalfLength, Vector.Max(-boxHalfLength, offsetAZ + aOntoBOffset));
            ta = Vector.Min(Vector.Max(ta, taMin), taMax);
            tb = Vector.Min(Vector.Max(tb, bMin), bMax);

            //Note that we leave the normal as non-unit length. If this turns out to be zero length, we will have to resort to the interior test for the normal.
            //We can, however, still make use the interval information for position.
            closestPointOnA.X = ta * capsuleAxisX + offsetAX;
            closestPointOnA.Y = ta * capsuleAxisY + offsetAY;
            closestPointOnA.Z = ta * capsuleAxisZ + offsetAZ;
            nX = closestPointOnA.X - boxEdgeCenterX;
            nY = closestPointOnA.Y - boxEdgeCenterY;
            nZ = closestPointOnA.Z - tb;
            var squaredLength = nX * nX + nY * nY + nZ * nZ;
            //If the box edge and capsule segment intersect, the normal will be zero. That's inconvenient, so add in a fallback based on cross(capsuleAxis, boxEdge).
            //(That simplifies to (-capsuleAxisY, capsuleAxisX, 0) thanks to the fact that boxEdge is (0,0,1).)
            var fallbackSquaredLength = capsuleAxisY * capsuleAxisY + capsuleAxisX * capsuleAxisX;
            //But that can ALSO be zero length if the axes are parallel. If both those conditions are met, then we just pick (1,0,0).
            epsilon = new Vector<float>(1e-10f);
            var useFallback = Vector.LessThan(squaredLength, epsilon);
            var useSecondFallback = Vector.BitwiseAnd(useFallback, Vector.LessThan(fallbackSquaredLength, epsilon));
            squaredLength = Vector.ConditionalSelect(useSecondFallback, Vector<float>.One, Vector.ConditionalSelect(useFallback, fallbackSquaredLength, squaredLength));
            nX = Vector.ConditionalSelect(useSecondFallback, Vector<float>.One, Vector.ConditionalSelect(useFallback, -capsuleAxisY, nX));
            nY = Vector.ConditionalSelect(useSecondFallback, Vector<float>.Zero, Vector.ConditionalSelect(useFallback, capsuleAxisX, nY));
            nZ = Vector.ConditionalSelect(useSecondFallback, Vector<float>.Zero, Vector.ConditionalSelect(useFallback, Vector<float>.Zero, nZ));

            //Calibrate the normal to point from B to A.
            var calibrationDot = nX * offsetAX + nY * offsetAY + nZ * offsetAZ;
            var shouldNegate = Vector.LessThan(calibrationDot, Vector<float>.Zero);
            nX = Vector.ConditionalSelect(shouldNegate, -nX, nX);
            nY = Vector.ConditionalSelect(shouldNegate, -nY, nY);
            nZ = Vector.ConditionalSelect(shouldNegate, -nZ, nZ);

            var inverseLength = Vector<float>.One / Vector.SquareRoot(squaredLength);
            nX *= inverseLength;
            nY *= inverseLength;
            nZ *= inverseLength;

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void TestAndRefineBoxEdge(
            ref Vector<float> offsetAX, ref Vector<float> offsetAY, ref Vector<float> offsetAZ,
            ref Vector<float> capsuleAxisX, ref Vector<float> capsuleAxisY, ref Vector<float> capsuleAxisZ,
            ref Vector<float> capsuleHalfLength,
            ref Vector<float> boxEdgeCenterX, ref Vector<float> boxEdgeCenterY,
            ref Vector<float> boxHalfWidth, ref Vector<float> boxHalfHeight, ref Vector<float> boxHalfLength,
            out Vector<float> ta, out Vector<float> depth, out Vector<float> nX, out Vector<float> nY, out Vector<float> nZ)

        {
            TestBoxEdge(
                ref offsetAX, ref offsetAY, ref offsetAZ,
                ref capsuleAxisX, ref capsuleAxisY, ref capsuleAxisZ,
                ref capsuleHalfLength, ref boxEdgeCenterX, ref boxEdgeCenterY,
                ref boxHalfWidth, ref boxHalfHeight, ref boxHalfLength,
                out _, out _, out var closestPointOnA,
                out nX, out nY, out nZ,
                out ta, out var epsilon);

            //Compute the depth along that normal.
            var boxExtreme = Vector.Abs(nX) * boxHalfWidth + Vector.Abs(nY) * boxHalfHeight + Vector.Abs(nZ) * boxHalfLength;
            var capsuleExtreme = nX * closestPointOnA.X + nY * closestPointOnA.Y + nZ * closestPointOnA.Z;
            depth = boxExtreme - capsuleExtreme;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void TestBoxFace(ref Vector<float> offsetAZ,
            ref Vector<float> capsuleAxisZ, ref Vector<float> capsuleHalfLength,
            ref Vector<float> boxHalfLength,
            out Vector<float> depth, out Vector<float> normalSign)
        {
            normalSign = Vector.ConditionalSelect(Vector.GreaterThan(offsetAZ, Vector<float>.Zero), Vector<float>.One, new Vector<float>(-1f));
            depth = boxHalfLength + Vector.Abs(capsuleAxisZ) * capsuleHalfLength - normalSign * offsetAZ;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Select(
            ref Vector<float> depth, ref Vector<float> ta,
            ref Vector<float> localNormalX, ref Vector<float> localNormalY, ref Vector<float> localNormalZ,
            ref Vector<float> depthCandidate, ref Vector<float> taCandidate, 
            ref Vector<float> localNormalCandidateX, ref Vector<float> localNormalCandidateY, ref Vector<float> localNormalCandidateZ)
        {
            var useCandidate = Vector.LessThan(depthCandidate, depth);
            ta = Vector.ConditionalSelect(useCandidate, taCandidate, ta);
            depth = Vector.ConditionalSelect(useCandidate, depthCandidate, depth);
            localNormalX = Vector.ConditionalSelect(useCandidate, localNormalCandidateX, localNormalX);
            localNormalY = Vector.ConditionalSelect(useCandidate, localNormalCandidateY, localNormalY);
            localNormalZ = Vector.ConditionalSelect(useCandidate, localNormalCandidateZ, localNormalZ);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Select(
         ref Vector<float> depth,
         ref Vector<float> localNormalX, ref Vector<float> localNormalY, ref Vector<float> localNormalZ,
         ref Vector<float> depthCandidate,
         ref Vector<float> localNormalCandidateX, ref Vector<float> localNormalCandidateY, ref Vector<float> localNormalCandidateZ)
        {
            var useCandidate = Vector.LessThan(depthCandidate, depth);
            depth = Vector.ConditionalSelect(useCandidate, depthCandidate, depth);
            localNormalX = Vector.ConditionalSelect(useCandidate, localNormalCandidateX, localNormalX);
            localNormalY = Vector.ConditionalSelect(useCandidate, localNormalCandidateY, localNormalY);
            localNormalZ = Vector.ConditionalSelect(useCandidate, localNormalCandidateZ, localNormalZ);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Test(
            ref CapsuleWide a, ref BoxWide b, ref Vector<float> speculativeMargin,
            ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount,
            out Convex2ContactManifoldWide manifold)
        {
            Prepare(ref a, ref b, ref offsetB, ref orientationA, ref orientationB, out var localOffsetA, out var capsuleAxis, out var edgeCenters);

            //Swizzle XYZ -> YZX
            Vector3Wide localNormal;
            TestAndRefineBoxEdge(ref localOffsetA.Y, ref localOffsetA.Z, ref localOffsetA.X,
                ref capsuleAxis.Y, ref capsuleAxis.Z, ref capsuleAxis.X,
                ref a.HalfLength,
                ref edgeCenters.Y, ref edgeCenters.Z,
                ref b.HalfHeight, ref b.HalfLength, ref b.HalfWidth,
                out var ta, out var depth, out localNormal.Y, out localNormal.Z, out localNormal.X);
            //Swizzle XYZ -> ZXY
            TestAndRefineBoxEdge(ref localOffsetA.Z, ref localOffsetA.X, ref localOffsetA.Y,
                ref capsuleAxis.Z, ref capsuleAxis.X, ref capsuleAxis.Y,
                ref a.HalfLength,
                ref edgeCenters.Z, ref edgeCenters.X,
                ref b.HalfLength, ref b.HalfWidth, ref b.HalfHeight,
                out var eyta, out var eyDepth, out var eynZ, out var eynX, out var eynY);
            Select(ref depth, ref ta, ref localNormal.X, ref localNormal.Y, ref localNormal.Z,
                ref eyDepth, ref eyta, ref eynX, ref eynY, ref eynZ);
            //Swizzle XYZ -> XYZ
            TestAndRefineBoxEdge(ref localOffsetA.X, ref localOffsetA.Y, ref localOffsetA.Z,
                ref capsuleAxis.X, ref capsuleAxis.Y, ref capsuleAxis.Z,
                ref a.HalfLength,
                ref edgeCenters.X, ref edgeCenters.Y,
                ref b.HalfWidth, ref b.HalfHeight, ref b.HalfLength,
                out var ezta, out var ezDepth, out var eznX, out var eznY, out var eznZ);
            Select(ref depth, ref ta, ref localNormal.X, ref localNormal.Y, ref localNormal.Z,
                ref ezDepth, ref ezta, ref eznX, ref eznY, ref eznZ);

            var zero = Vector<float>.Zero;
            //Face X
            TestBoxFace(ref localOffsetA.X,
                ref capsuleAxis.X, ref a.HalfLength,
                ref b.HalfWidth,
                out var fxDepth, out var fxn);
            Select(ref depth, ref localNormal.X, ref localNormal.Y, ref localNormal.Z,
                ref fxDepth, ref fxn, ref zero, ref zero);
            //Face Y
            TestBoxFace(ref localOffsetA.Y,
                ref capsuleAxis.Y, ref a.HalfLength,
                ref b.HalfHeight,
                out var fyDepth, out var fyn);
            Select(ref depth, ref localNormal.X, ref localNormal.Y, ref localNormal.Z,
                ref fyDepth, ref zero, ref fyn, ref zero);
            //Face Z
            TestBoxFace(ref localOffsetA.Z,
                ref capsuleAxis.Z, ref a.HalfLength,
                ref b.HalfLength,
                out var fzDepth, out var fzn);
            Select(ref depth, ref localNormal.X, ref localNormal.Y, ref localNormal.Z,
                ref fzDepth, ref zero, ref zero, ref fzn);

            //While the above chooses a minimal depth, edge-edge contact will frequently produce interval lengths of 0 and end up overwriting near-equivalent face intervals.
            //One option would be to bias the comparison to accept face contacts more readily, but we can do something a little less fragile, and which also gives us a 
            //way to safely compute depth on a per contact basis:
            //choose a representative box face based on the collision normal detected above, and compute the interval of intersection along the capsule axis of the box face projected onto the capsule axis.
            //We'll compute this by unprojecting the capsule axis onto the box face plane along the local normal.
            //Note that this interval always includes the closest point.
            var xDot = localNormal.X * fxn;
            var yDot = localNormal.Y * fyn;
            var zDot = localNormal.Z * fzn;
            var useX = Vector.GreaterThan(xDot, Vector.Max(yDot, zDot));
            var useY = Vector.AndNot(Vector.GreaterThan(yDot, zDot), useX);
            var useZ = Vector.AndNot(Vector.OnesComplement(useX), useY);

            //Unproject the capsule center and capsule axis onto the representative face plane.
            //unprojectedAxis = capsuleAxis - localNormal * dot(capsuleAxis, faceNormal) / dot(localNormal, faceNormal)
            //unprojectedCenter = capsuleCenter - localNormal * dot(capsuleCenter - pointOnFace, faceNormal) / dot(localNormal, faceNormal)
            var faceNormalDotLocalNormal = Vector.ConditionalSelect(useX, xDot, Vector.ConditionalSelect(useY, yDot, zDot));
            var inverseFaceNormalDotLocalNormal = Vector<float>.One / Vector.Max(new Vector<float>(1e-15f), faceNormalDotLocalNormal);
            var capsuleAxisDotFaceNormal = Vector.ConditionalSelect(useX, capsuleAxis.X * fxn, Vector.ConditionalSelect(useY, capsuleAxis.Y * fyn, capsuleAxis.Z * fzn));
            var capsuleCenterDotFaceNormal = Vector.ConditionalSelect(useX, localOffsetA.X * fxn, Vector.ConditionalSelect(useY, localOffsetA.Y * fyn, localOffsetA.Z * fzn));
            var facePlaneOffset = Vector.ConditionalSelect(useX, b.HalfWidth, Vector.ConditionalSelect(useY, b.HalfHeight, b.HalfLength));
            var tAxis = capsuleAxisDotFaceNormal * inverseFaceNormalDotLocalNormal;
            var tCenter = (capsuleCenterDotFaceNormal - facePlaneOffset) * inverseFaceNormalDotLocalNormal;

            //Work in tangent space. 
            //Face X uses tangents Y and Z.
            //Face Y uses tangents X and Z.
            //Face Z uses tangents X and Y.
            Vector3Wide.Scale(localNormal, tAxis, out var axisOffset);
            Vector3Wide.Scale(localNormal, tCenter, out var centerOffset);
            Vector3Wide.Subtract(capsuleAxis, axisOffset, out var unprojectedAxis);
            Vector3Wide.Subtract(localOffsetA, centerOffset, out var unprojectedCenter);
            Vector2Wide tangentSpaceCenter, tangentSpaceAxis;
            tangentSpaceAxis.X = Vector.ConditionalSelect(useX, unprojectedAxis.Y, unprojectedAxis.X);
            tangentSpaceAxis.Y = Vector.ConditionalSelect(useZ, unprojectedAxis.Y, unprojectedAxis.Z);
            tangentSpaceCenter.X = Vector.ConditionalSelect(useX, unprojectedCenter.Y, unprojectedCenter.X);
            tangentSpaceCenter.Y = Vector.ConditionalSelect(useZ, unprojectedCenter.Y, unprojectedCenter.Z);
            //Slightly boost the size of the face to avoid minor numerical issues that could block coplanar contacts.
            var epsilonScale = Vector.Min(Vector.Max(b.HalfWidth, Vector.Max(b.HalfHeight, b.HalfLength)), Vector.Max(a.HalfLength, a.Radius));
            var epsilon = epsilonScale * 1e-3f;
            var halfExtentX = epsilon + Vector.ConditionalSelect(useX, b.HalfHeight, b.HalfWidth);
            var halfExtentY = epsilon + Vector.ConditionalSelect(useZ, b.HalfHeight, b.HalfLength);

            //Compute interval bounded by edge normals pointing along tangentX.
            //tX = -dot(tangentSpaceCenter +- halfExtentX, edgeNormal) / dot(unprojectedCapsuleAxis, edgeNormal)

            var negativeOne = new Vector<float>(-1);
            var inverseAxisX = negativeOne / tangentSpaceAxis.X;
            var inverseAxisY = negativeOne / tangentSpaceAxis.Y;
            var tX0 = (tangentSpaceCenter.X - halfExtentX) * inverseAxisX;
            var tX1 = (tangentSpaceCenter.X + halfExtentX) * inverseAxisX;
            var tY0 = (tangentSpaceCenter.Y - halfExtentY) * inverseAxisY;
            var tY1 = (tangentSpaceCenter.Y + halfExtentY) * inverseAxisY;
            var minX = Vector.Min(tX0, tX1);
            var maxX = Vector.Max(tX0, tX1);
            var minY = Vector.Min(tY0, tY1);
            var maxY = Vector.Max(tY0, tY1);
            //Protect against division by zero. If the unprojected capsule is within the slab, use an infinite interval. If it's outside and parallel, use an invalid interval.
            var useFallbackX = Vector.LessThan(Vector.Abs(tangentSpaceAxis.X), new Vector<float>(1e-15f));
            var useFallbackY = Vector.LessThan(Vector.Abs(tangentSpaceAxis.Y), new Vector<float>(1e-15f));
            var centerContainedX = Vector.LessThanOrEqual(Vector.Abs(tangentSpaceCenter.X), halfExtentX);
            var centerContainedY = Vector.LessThanOrEqual(Vector.Abs(tangentSpaceCenter.Y), halfExtentY);
            var largeNegative = new Vector<float>(-float.MaxValue);
            var largePositive = new Vector<float>(float.MaxValue);
            minX = Vector.ConditionalSelect(useFallbackX, Vector.ConditionalSelect(centerContainedX, largeNegative, largePositive), minX);
            maxX = Vector.ConditionalSelect(useFallbackX, Vector.ConditionalSelect(centerContainedX, largePositive, largeNegative), maxX);
            minY = Vector.ConditionalSelect(useFallbackY, Vector.ConditionalSelect(centerContainedY, largeNegative, largePositive), minY);
            maxY = Vector.ConditionalSelect(useFallbackY, Vector.ConditionalSelect(centerContainedY, largePositive, largeNegative), maxY);

            var faceMin = Vector.Max(minX, minY);
            var faceMax = Vector.Min(maxX, maxY);
            //Clamp the resulting interval to the capsule axis.
            var tMin = Vector.Max(Vector.Min(faceMin, a.HalfLength), -a.HalfLength);
            var tMax = Vector.Max(Vector.Min(faceMax, a.HalfLength), -a.HalfLength);
            var faceIntervalExists = Vector.GreaterThanOrEqual(faceMax, faceMin);
            tMin = Vector.ConditionalSelect(faceIntervalExists, Vector.Min(tMin, ta), ta);
            tMax = Vector.ConditionalSelect(faceIntervalExists, Vector.Max(tMax, ta), ta);

            //Each contact may have its own depth.
            //Imagine a face collision- if the capsule axis isn't fully parallel with the plane's surface, it would be strange to use the same depth for both contacts.
            //We have two points on the capsule and box. We can reuse the unprojeection from earlier to compute the offset between them.
            var separationMin = tCenter + tAxis * tMin;
            var separationMax = tCenter + tAxis * tMax;
            manifold.Depth0 = a.Radius - separationMin;
            manifold.Depth1 = a.Radius - separationMax;

            Vector3Wide.Scale(capsuleAxis, tMin, out var localA0);
            Vector3Wide.Scale(capsuleAxis, tMax, out var localA1);

            manifold.FeatureId0 = Vector<int>.Zero;
            manifold.FeatureId1 = Vector<int>.One;

            //Transform A0, A1, and the normal into world space.
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var orientationMatrixB);
            Matrix3x3Wide.TransformWithoutOverlap(localNormal, orientationMatrixB, out manifold.Normal);
            Matrix3x3Wide.TransformWithoutOverlap(localA0, orientationMatrixB, out manifold.OffsetA0);
            Matrix3x3Wide.TransformWithoutOverlap(localA1, orientationMatrixB, out manifold.OffsetA1);

            //Apply the normal offset to the contact positions.           
            var negativeOffsetFromA0 = manifold.Depth0 * 0.5f - a.Radius;
            var negativeOffsetFromA1 = manifold.Depth1 * 0.5f - a.Radius;
            Vector3Wide.Scale(manifold.Normal, negativeOffsetFromA0, out var normalPush0);
            Vector3Wide.Scale(manifold.Normal, negativeOffsetFromA1, out var normalPush1);
            Vector3Wide.Add(manifold.OffsetA0, normalPush0, out manifold.OffsetA0);
            Vector3Wide.Add(manifold.OffsetA1, normalPush1, out manifold.OffsetA1);

            var minimumAcceptedDepth = -speculativeMargin;
            manifold.Contact0Exists = Vector.GreaterThanOrEqual(manifold.Depth0, minimumAcceptedDepth);
            manifold.Contact1Exists = Vector.BitwiseAnd(
                Vector.GreaterThanOrEqual(manifold.Depth1, minimumAcceptedDepth),
                Vector.GreaterThan(tMax - tMin, new Vector<float>(1e-7f) * a.HalfLength));
        }

        public void Test(ref CapsuleWide a, ref BoxWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out Convex2ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public void Test(ref CapsuleWide a, ref BoxWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out Convex2ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }
}
