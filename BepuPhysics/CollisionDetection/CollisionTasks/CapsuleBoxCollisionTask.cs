using BepuPhysics.Collidables;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct CapsuleBoxTester : IPairTester<CapsuleWide, BoxWide, Convex2ContactManifoldWide>
    {
        //Hideous parameter list because this function is used with swizzled vectors.
        //It's built to handle the Z edge hardcoded, so we just reorient things at the point of use to handle X and Y.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void TestBoxEdge(
            ref Vector<float> offsetAX, ref Vector<float> offsetAY, ref Vector<float> offsetAZ,
            ref Vector<float> capsuleAxisX, ref Vector<float> capsuleAxisY, ref Vector<float> capsuleAxisZ,
            ref Vector<float> capsuleHalfLength,
            ref Vector<float> boxEdgeCenterX, ref Vector<float> boxEdgeCenterY,
            ref Vector<float> boxHalfWidth, ref Vector<float> boxHalfHeight, ref Vector<float> boxHalfLength,
            out Vector<float> taMin, out Vector<float> taMax, out Vector<float> depth, out Vector<float> nX, out Vector<float> nY, out Vector<float> nZ)

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
            var ta = (daOffsetB + offsetAZ * capsuleAxisZ) / Vector.Max(new Vector<float>(1e-15f), Vector<float>.One - capsuleAxisZ * capsuleAxisZ);
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
            var closestPointOnAX = ta * capsuleAxisX + offsetAX;
            var closestPointOnAY = ta * capsuleAxisY + offsetAY;
            var closestPointOnAZ = ta * capsuleAxisZ + offsetAZ;
            nX = closestPointOnAX - boxEdgeCenterX;
            nY = closestPointOnAY - boxEdgeCenterY;
            nZ = closestPointOnAZ - tb;
            var squaredLength = nX * nX + nY * nY + nZ * nZ;
            //If the box edge and capsule segment intersect, the normal will be zero. That's inconvenient, so add in a fallback based on cross(capsuleAxis, boxEdge).
            //(That simplifies to (-capsuleAxisY, capsuleAxisX, 0) thanks to the fact that boxEdge is (0,0,1).)
            var fallbackSquaredLength = capsuleAxisY * capsuleAxisY + capsuleAxisX * capsuleAxisX;
            //But that can ALSO be zero length if the axes are parallel. If both those conditions are met, then we just pick (1,0,0).
            var epsilon = new Vector<float>(1e-10f);
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

            //Compute the depth along that normal.
            var boxExtreme = Vector.Abs(nX) * boxHalfWidth + Vector.Abs(nY) * boxHalfHeight + Vector.Abs(nZ) * boxHalfLength;
            var capsuleExtreme = nX * closestPointOnAX + nY * closestPointOnAY + nZ * closestPointOnAZ;
            depth = boxExtreme - capsuleExtreme;

            //In the event that the edge and capsule axis are coplanar, we accept the whole interval as a source of contact.
            //As the capsule axis drifts away from coplanarity, the accepted interval rapidly narrows to zero length, centered on ta.
            //We rate the degree of coplanarity based on the angle between the capsule axis and the plane defined by the box edge and contact normal:
            //sin(angle) = dot(capsuleAxis, planeNormal)
            //Rather than explicitly calculating a planeNormal with a cross product, we can do something similar by removing any of the capsuleAxis which points along the 
            //contact normal or the box edge, noting that box edge is (0,0,1):
            //axisRemainderX = capsuleAxis.X - contactNormal.X * (dot(capsuleAxis, contactNormal) / ||contactNormal||^2)
            //axisRemainderY = capsuleAxis.Y - contactNormal.Y * (dot(capsuleAxis, contactNormal) / ||contactNormal||^2)
            //axisRemainderZ = 0
            //So, to restate in the original form:
            //sin(angle) = dot(capsuleAxis, planeNormal) = ||axisRemainder||
            //Finally, note that we are dealing with extremely small angles, and for small angles sin(angle) ~= angle:
            //angle^2 ~= dot(axisRemainder, axisRemainder)
            //Since fade behavior is completely arbitrary, we can directly use squared angle without any concern.
            var removalScale = (capsuleAxisX * nX + capsuleAxisY * nY + capsuleAxisZ * nZ) / squaredLength;
            var axisRemainderX = capsuleAxisX - nX * removalScale;
            var axisRemainderY = capsuleAxisY - nY * removalScale;
            var squaredAngle = axisRemainderX * axisRemainderX + axisRemainderY * axisRemainderY;
            //Convert the squared angle to a lerp parameter. For squared angle from 0 to lowerThreshold, we should use the full interval (1). From lowerThreshold to upperThreshold, lerp to 0.
            const float lowerThresholdAngle = 0.001f;
            const float upperThresholdAngle = 0.005f;
            const float lowerThreshold = lowerThresholdAngle * lowerThresholdAngle;
            const float upperThreshold = upperThresholdAngle * upperThresholdAngle;
            var intervalWeight = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, (new Vector<float>(upperThreshold) - squaredAngle) * new Vector<float>(1f / (upperThreshold - lowerThreshold))));
            var weightedTa = ta - ta * intervalWeight;
            taMin = intervalWeight * taMin + weightedTa;
            taMax = intervalWeight * taMax + weightedTa;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void TestBoxFace(ref Vector<float> offsetAZ,
            ref Vector<float> capsuleAxisZ, ref Vector<float> capsuleHalfLength,
            ref Vector<float> tCandidateMinX, ref Vector<float> tCandidateMaxX, ref Vector<float> tCandidateMinY, ref Vector<float> tCandidateMaxY,
            ref Vector<float> boxHalfLength,
            out Vector<float> depth, out Vector<float> taMin, out Vector<float> taMax, out Vector<float> normalSign)
        {
            taMin = Vector.Max(tCandidateMinX, tCandidateMinY);
            taMax = Vector.Min(tCandidateMaxX, tCandidateMaxY);
            normalSign = Vector.ConditionalSelect(Vector.GreaterThan(offsetAZ, Vector<float>.Zero), Vector<float>.One, new Vector<float>(-1f));
            depth = boxHalfLength + Vector.Abs(capsuleAxisZ) * capsuleHalfLength - normalSign * offsetAZ;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Select(
            ref Vector<float> depth, ref Vector<float> taMin, ref Vector<float> taMax,
            ref Vector<float> localNormalX, ref Vector<float> localNormalY, ref Vector<float> localNormalZ,
            ref Vector<float> depthCandidate, ref Vector<float> taMinCandidate, ref Vector<float> taMaxCandidate,
            ref Vector<float> localNormalCandidateX, ref Vector<float> localNormalCandidateY, ref Vector<float> localNormalCandidateZ)
        {
            var useCandidate = Vector.LessThan(depthCandidate, depth);
            taMin = Vector.ConditionalSelect(useCandidate, taMinCandidate, taMin);
            taMax = Vector.ConditionalSelect(useCandidate, taMaxCandidate, taMax);
            depth = Vector.ConditionalSelect(useCandidate, depthCandidate, depth);
            localNormalX = Vector.ConditionalSelect(useCandidate, localNormalCandidateX, localNormalX);
            localNormalY = Vector.ConditionalSelect(useCandidate, localNormalCandidateY, localNormalY);
            localNormalZ = Vector.ConditionalSelect(useCandidate, localNormalCandidateZ, localNormalZ);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Test(
            ref CapsuleWide a, ref BoxWide b,
            ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
            out Convex2ContactManifoldWide manifold)
        {
            QuaternionWide.Conjugate(ref orientationB, out var toLocalB);
            QuaternionWide.TransformWithoutOverlap(ref offsetB, ref toLocalB, out var localOffsetA);
            Vector3Wide.Negate(ref localOffsetA);
            QuaternionWide.ConcatenateWithoutOverlap(ref orientationA, ref toLocalB, out var boxLocalOrientationA);
            QuaternionWide.TransformUnitY(ref boxLocalOrientationA, out var capsuleAxis);

            //Get the capsule-axis-perpendicular offset from the box to the capsule and use it to choose which edges to test.
            //(Pointless to test the other 9; they're guaranteed to be further away.)
            Vector3Wide.Dot(ref localOffsetA, ref capsuleAxis, out var axisOffsetADot);
            Vector3Wide.Scale(ref capsuleAxis, ref axisOffsetADot, out var toRemove);
            Vector3Wide.Subtract(ref localOffsetA, ref toRemove, out var perpendicularOffset);
            Vector3Wide edgeCenters;
            edgeCenters.X = Vector.ConditionalSelect(Vector.LessThan(perpendicularOffset.X, Vector<float>.Zero), -b.HalfWidth, b.HalfWidth);
            edgeCenters.Y = Vector.ConditionalSelect(Vector.LessThan(perpendicularOffset.Y, Vector<float>.Zero), -b.HalfHeight, b.HalfHeight);
            edgeCenters.Z = Vector.ConditionalSelect(Vector.LessThan(perpendicularOffset.Z, Vector<float>.Zero), -b.HalfLength, b.HalfLength);

            //Swizzle XYZ -> YZX
            Vector3Wide localNormal;
            TestBoxEdge(ref localOffsetA.Y, ref localOffsetA.Z, ref localOffsetA.X,
                ref capsuleAxis.Y, ref capsuleAxis.Z, ref capsuleAxis.X,
                ref a.HalfLength,
                ref edgeCenters.Y, ref edgeCenters.Z,
                ref b.HalfHeight, ref b.HalfLength, ref b.HalfWidth,
                out var taMin, out var taMax, out var depth, out localNormal.Y, out localNormal.Z, out localNormal.X);
            //Swizzle XYZ -> ZXY
            TestBoxEdge(ref localOffsetA.Z, ref localOffsetA.X, ref localOffsetA.Y,
                ref capsuleAxis.Z, ref capsuleAxis.X, ref capsuleAxis.Y,
                ref a.HalfLength,
                ref edgeCenters.Z, ref edgeCenters.X,
                ref b.HalfLength, ref b.HalfWidth, ref b.HalfHeight,
                out var eytaMin, out var eytaMax, out var eyDepth, out var eynZ, out var eynX, out var eynY);
            Select(ref depth, ref taMin, ref taMax, ref localNormal.X, ref localNormal.Y, ref localNormal.Z,
                ref eyDepth, ref eytaMin, ref eytaMax, ref eynX, ref eynY, ref eynZ);
            //Swizzle XYZ -> XYZ
            TestBoxEdge(ref localOffsetA.X, ref localOffsetA.Y, ref localOffsetA.Z,
                ref capsuleAxis.X, ref capsuleAxis.Y, ref capsuleAxis.Z,
                ref a.HalfLength,
                ref edgeCenters.X, ref edgeCenters.Y,
                ref b.HalfWidth, ref b.HalfHeight, ref b.HalfLength,
                out var eztaMin, out var eztaMax, out var ezDepth, out var eznX, out var eznY, out var eznZ);
            Select(ref depth, ref taMin, ref taMax, ref localNormal.X, ref localNormal.Y, ref localNormal.Z,
                ref ezDepth, ref eztaMin, ref eztaMax, ref eznX, ref eznY, ref eznZ);

            //For each face, clip the capsule axis against the face bounds. The closest offset between the clipped capsule axis endpoints and the box is the normal candidate.
            var positive = Vector<float>.One;
            var negative = -positive;
            var divisionGuard = new Vector<float>(1e-15f);
            //Division by zero is hacked away by clamping the magnitude to some nonzero value and recovering the sign in the numerator.
            //The resulting interval will be properly signed and enormous, so it serves the necessary purpose.
            //Note the negation: t = dot(N, +-halfExtent - offsetA) / dot(N, capsuleAxis) = dot(N, offsetA +- halfExtent) / -dot(N, capsuleAxis)
            var scaleX = Vector.ConditionalSelect(Vector.LessThan(capsuleAxis.X, Vector<float>.Zero), positive, negative) / Vector.Max(Vector.Abs(capsuleAxis.X), divisionGuard);
            var scaleY = Vector.ConditionalSelect(Vector.LessThan(capsuleAxis.Y, Vector<float>.Zero), positive, negative) / Vector.Max(Vector.Abs(capsuleAxis.Y), divisionGuard);
            var scaleZ = Vector.ConditionalSelect(Vector.LessThan(capsuleAxis.Z, Vector<float>.Zero), positive, negative) / Vector.Max(Vector.Abs(capsuleAxis.Z), divisionGuard);
            Vector3Wide scaledExtents, scaledOffset;
            //Clip slightly beyond the actual face limit to avoid pointlessly cutting the interval for near-edge-parallel capsules.
            var epsilonScale = new Vector<float>(1f + 1e-6f);
            scaledExtents.X = epsilonScale * b.HalfWidth * Vector.Abs(scaleX);
            scaledExtents.Y = epsilonScale * b.HalfHeight * Vector.Abs(scaleY);
            scaledExtents.Z = epsilonScale * b.HalfLength * Vector.Abs(scaleZ);
            scaledOffset.X = localOffsetA.X * scaleX;
            scaledOffset.Y = localOffsetA.Y * scaleY;
            scaledOffset.Z = localOffsetA.Z * scaleZ;
            Vector3Wide.Subtract(ref scaledOffset, ref scaledExtents, out var tCandidateMin);
            var negativeHalfLength = -a.HalfLength;
            Vector3Wide.Min(ref a.HalfLength, ref tCandidateMin, out tCandidateMin);
            Vector3Wide.Max(ref negativeHalfLength, ref tCandidateMin, out tCandidateMin);
            Vector3Wide.Add(ref scaledOffset, ref scaledExtents, out var tCandidateMax);
            Vector3Wide.Min(ref a.HalfLength, ref tCandidateMax, out tCandidateMax);
            Vector3Wide.Max(ref negativeHalfLength, ref tCandidateMax, out tCandidateMax);
            var zero = Vector<float>.Zero;
            //Face X
            TestBoxFace(ref localOffsetA.X,
                ref capsuleAxis.X, ref a.HalfLength,
                ref tCandidateMin.Y, ref tCandidateMax.Y, ref tCandidateMin.Z, ref tCandidateMax.Z,
                ref b.HalfWidth,
                out var fxDepth, out var fxtaMin, out var fxtaMax, out var fxn);
            Select(ref depth, ref taMin, ref taMax, ref localNormal.X, ref localNormal.Y, ref localNormal.Z,
                ref fxDepth, ref fxtaMin, ref fxtaMax, ref fxn, ref zero, ref zero);
            //Face Y
            TestBoxFace(ref localOffsetA.Y,
                ref capsuleAxis.Y, ref a.HalfLength,
                ref tCandidateMin.X, ref tCandidateMax.X, ref tCandidateMin.Z, ref tCandidateMax.Z,
                ref b.HalfHeight,
                out var fyDepth, out var fytaMin, out var fytaMax, out var fyn);
            Select(ref depth, ref taMin, ref taMax, ref localNormal.X, ref localNormal.Y, ref localNormal.Z,
                ref fyDepth, ref fytaMin, ref fytaMax, ref zero, ref fyn, ref zero);
            //Face Z
            TestBoxFace(ref localOffsetA.Z,
                ref capsuleAxis.Z, ref a.HalfLength,
                ref tCandidateMin.X, ref tCandidateMax.X, ref tCandidateMin.Y, ref tCandidateMax.Y,
                ref b.HalfLength,
                out var fzDepth, out var fztaMin, out var fztaMax, out var fzn);
            Select(ref depth, ref taMin, ref taMax, ref localNormal.X, ref localNormal.Y, ref localNormal.Z,
                ref fzDepth, ref fztaMin, ref fztaMax, ref zero, ref zero, ref fzn);

            //Each contact may have its own depth.
            //Imagine a face collision- if the capsule axis isn't fully parallel with the plane's surface, it would be strange to use the same depth for both contacts.
            //Compute the interval of the box on the normal. Note that the normal is already calibrated to point from B to A (box to capsule).
            //(This is partially redundant with the per-case calculations, but simply redoing some cheap ALU work is easier than trying to keep track of per-contact depths across all cases.)
            Vector3Wide.Scale(ref capsuleAxis, ref taMin, out var localA0);
            Vector3Wide.Scale(ref capsuleAxis, ref taMax, out var localA1);
            Vector3Wide.Add(ref localOffsetA, ref localA0, out var bToA0);
            Vector3Wide.Add(ref localOffsetA, ref localA1, out var bToA1);
            var boxExtreme = Vector.Abs(localNormal.X * b.HalfWidth) + Vector.Abs(localNormal.Y * b.HalfHeight) + Vector.Abs(localNormal.Z * b.HalfLength);
            Vector3Wide.Dot(ref localNormal, ref bToA0, out var dot0);
            Vector3Wide.Dot(ref localNormal, ref bToA1, out var dot1);
            manifold.Depth0 = a.Radius + boxExtreme - dot0;
            manifold.Depth1 = a.Radius + boxExtreme - dot1;
            manifold.FeatureId0 = Vector<int>.Zero;
            manifold.FeatureId1 = Vector<int>.One;

            //Transform A0, A1, and the normal into world space.
            Matrix3x3Wide.CreateFromQuaternion(ref orientationB, out var orientationMatrixB);
            Matrix3x3Wide.TransformWithoutOverlap(ref localNormal, ref orientationMatrixB, out manifold.Normal);
            Matrix3x3Wide.TransformWithoutOverlap(ref localA0, ref orientationMatrixB, out manifold.OffsetA0);
            Matrix3x3Wide.TransformWithoutOverlap(ref localA1, ref orientationMatrixB, out manifold.OffsetA1);

            //Apply the normal offset to the contact positions.           
            var negativeOffsetFromA0 = manifold.Depth0 * 0.5f - a.Radius;
            var negativeOffsetFromA1 = manifold.Depth1 * 0.5f - a.Radius;
            Vector3Wide.Scale(ref manifold.Normal, ref negativeOffsetFromA0, out var normalPush0);
            Vector3Wide.Scale(ref manifold.Normal, ref negativeOffsetFromA1, out var normalPush1);
            Vector3Wide.Add(ref manifold.OffsetA0, ref normalPush0, out manifold.OffsetA0);
            Vector3Wide.Add(ref manifold.OffsetA1, ref normalPush1, out manifold.OffsetA1);

            manifold.Count = Vector.ConditionalSelect(Vector.LessThan(taMax - taMin, new Vector<float>(1e-7f) * a.HalfLength), Vector<int>.One, new Vector<int>(2));
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Test2(
            ref CapsuleWide a, ref BoxWide b,
            ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
            out Convex2ContactManifoldWide manifold)
        {
            QuaternionWide.Conjugate(ref orientationB, out var toLocalB);
            QuaternionWide.TransformWithoutOverlap(ref offsetB, ref toLocalB, out var localOffsetA);
            Vector3Wide.Negate(ref localOffsetA);
            QuaternionWide.ConcatenateWithoutOverlap(ref orientationA, ref toLocalB, out var boxLocalOrientationA);
            QuaternionWide.TransformUnitY(ref boxLocalOrientationA, out var capsuleAxis);

            //For each face of the box, clip the capsule axis against its bounding planes and then clamp the result to the box's extent along that axis.
            //The face which has the largest clipped interval of the capsule line segment will be picked as the representative.
            var positive = Vector<float>.One;
            var negative = -positive;
            var divisionGuard = new Vector<float>(1e-15f);
            //Division by zero is hacked away by clamping the magnitude to some nonzero value and recovering the sign in the numerator.
            //The resulting interval will be properly signed and enormous, so it serves the necessary purpose.
            //Note the negation: t = dot(N, +-halfExtent - offsetA) / dot(N, capsuleAxis) = dot(N, offsetA +- halfExtent) / -dot(N, capsuleAxis)
            var scaleX = Vector.ConditionalSelect(Vector.LessThan(capsuleAxis.X, Vector<float>.Zero), positive, negative) / Vector.Max(Vector.Abs(capsuleAxis.X), divisionGuard);
            var scaleY = Vector.ConditionalSelect(Vector.LessThan(capsuleAxis.Y, Vector<float>.Zero), positive, negative) / Vector.Max(Vector.Abs(capsuleAxis.Y), divisionGuard);
            var scaleZ = Vector.ConditionalSelect(Vector.LessThan(capsuleAxis.Z, Vector<float>.Zero), positive, negative) / Vector.Max(Vector.Abs(capsuleAxis.Z), divisionGuard);
            Vector3Wide scaledExtents, scaledOffset;
            scaledExtents.X = b.HalfWidth * Vector.Abs(scaleX);
            scaledExtents.Y = b.HalfHeight * Vector.Abs(scaleY);
            scaledExtents.Z = b.HalfLength * Vector.Abs(scaleZ);
            scaledOffset.X = localOffsetA.X * scaleX;
            scaledOffset.Y = localOffsetA.Y * scaleY;
            scaledOffset.Z = localOffsetA.Z * scaleZ;
            Vector3Wide.Subtract(ref scaledOffset, ref scaledExtents, out var tCandidateMin);
            var negativeHalfLength = -a.HalfLength;
            Vector3Wide.Max(ref negativeHalfLength, ref tCandidateMin, out tCandidateMin);
            Vector3Wide.Add(ref scaledOffset, ref scaledExtents, out var tCandidateMax);
            Vector3Wide.Min(ref a.HalfLength, ref tCandidateMax, out tCandidateMax);

            //We now have clipped intervals for all faces. The faces along axis X, for example, use interval max(tMin.Y, tMin.Z) to min(tMax.Y, tMax.Z).
            //Find the longest interval. Note that it's possible for the interval length to be negative when a line segment does not enter the face's voronoi region.
            //It may be negative for all axes, even; that can happen when the line segment is outside all face voronoi regions. That's fine, though.
            //It's simply a signal that the line segment is in one of the face-adjacent voronoi regions, and clamping the endpoints will provide the closest point on the box to the segment.
            var intervalMaxX = Vector.Min(tCandidateMax.Y, tCandidateMax.Z);
            var intervalMaxY = Vector.Min(tCandidateMax.X, tCandidateMax.Z);
            var intervalMaxZ = Vector.Min(tCandidateMax.X, tCandidateMax.Y);
            var intervalMinX = Vector.Max(tCandidateMin.Y, tCandidateMin.Z);
            var intervalMinY = Vector.Max(tCandidateMin.X, tCandidateMin.Z);
            var intervalMinZ = Vector.Max(tCandidateMin.X, tCandidateMin.Y);
            var intervalLengthX = Vector.Max(Vector<float>.Zero, intervalMaxX - intervalMinX);
            var intervalLengthY = Vector.Max(Vector<float>.Zero, intervalMaxY - intervalMinY);
            var intervalLengthZ = Vector.Max(Vector<float>.Zero, intervalMaxZ - intervalMinZ);
            var x2 = capsuleAxis.X * capsuleAxis.X;
            var y2 = capsuleAxis.Y * capsuleAxis.Y;
            var z2 = capsuleAxis.Z * capsuleAxis.Z;
            var projectedLengthSquaredX = (y2 + z2) * intervalLengthX * intervalLengthX;
            var projectedLengthSquaredY = (x2 + z2) * intervalLengthY * intervalLengthY;
            var projectedLengthSquaredZ = (x2 + y2) * intervalLengthZ * intervalLengthZ;
            var maxLengthSquared = Vector.Max(projectedLengthSquaredX, Vector.Max(projectedLengthSquaredY, projectedLengthSquaredZ));
            var useX = Vector.Equals(projectedLengthSquaredX, maxLengthSquared);
            var useY = Vector.Equals(projectedLengthSquaredY, maxLengthSquared);
            var tMin = Vector.ConditionalSelect(useX, intervalMinX, Vector.ConditionalSelect(useY, intervalMinY, intervalMinZ));
            var tMax = Vector.ConditionalSelect(useX, intervalMaxX, Vector.ConditionalSelect(useY, intervalMaxY, intervalMaxZ));

            Vector3Wide.Scale(ref capsuleAxis, ref tMin, out var capsuleStart);
            Vector3Wide.Scale(ref capsuleAxis, ref tMax, out var capsuleEnd);
            Vector3Wide.Add(ref localOffsetA, ref capsuleStart, out capsuleStart);
            Vector3Wide.Add(ref localOffsetA, ref capsuleEnd, out capsuleEnd);

            //Clamp the start and end to the box. Note that we haven't kept track of which axis we're working on, so we just do clamping on all three axes. Two axes will be wasted,
            //but it's easier to do this than to try to reconstruct which axis is the one we're supposed to work on.
            Vector3Wide boxStart, boxEnd;
            boxStart.X = Vector.Max(Vector.Min(capsuleStart.X, b.HalfWidth), -b.HalfWidth);
            boxStart.Y = Vector.Max(Vector.Min(capsuleStart.Y, b.HalfHeight), -b.HalfHeight);
            boxStart.Z = Vector.Max(Vector.Min(capsuleStart.Z, b.HalfLength), -b.HalfLength);
            boxEnd.X = Vector.Max(Vector.Min(capsuleEnd.X, b.HalfWidth), -b.HalfWidth);
            boxEnd.Y = Vector.Max(Vector.Min(capsuleEnd.Y, b.HalfHeight), -b.HalfHeight);
            boxEnd.Z = Vector.Max(Vector.Min(capsuleEnd.Z, b.HalfLength), -b.HalfLength);

            //Compute the closest point on the line segment to the clamped start and end.
            Vector3Wide.Subtract(ref boxStart, ref localOffsetA, out var aToBoxStart);
            Vector3Wide.Subtract(ref boxEnd, ref localOffsetA, out var aToBoxEnd);
            Vector3Wide.Dot(ref capsuleAxis, ref aToBoxStart, out var dotStart);
            Vector3Wide.Dot(ref capsuleAxis, ref aToBoxEnd, out var dotEnd);
            dotStart = Vector.Max(Vector.Min(dotStart, a.HalfLength), negativeHalfLength);
            dotEnd = Vector.Max(Vector.Min(dotEnd, a.HalfLength), negativeHalfLength);

            //Now compute the distances. The normal will be defined by the shorter of the two offsets.
            Vector3Wide.Scale(ref capsuleAxis, ref dotStart, out var closestFromBoxStart);
            Vector3Wide.Scale(ref capsuleAxis, ref dotEnd, out var closestFromBoxEnd);
            Vector3Wide.Add(ref closestFromBoxStart, ref localOffsetA, out closestFromBoxStart);
            Vector3Wide.Add(ref closestFromBoxEnd, ref localOffsetA, out closestFromBoxEnd);
            //Note that these offsets are pointing from the box to the capsule (B to A), matching contact normal convention.
            Vector3Wide.Subtract(ref closestFromBoxStart, ref boxStart, out var startOffset);
            Vector3Wide.Subtract(ref closestFromBoxEnd, ref boxEnd, out var endOffset);

            Vector3Wide.LengthSquared(ref startOffset, out var startLengthSquared);
            Vector3Wide.LengthSquared(ref endOffset, out var endLengthSquared);

            var minLengthSquared = Vector.Min(startLengthSquared, endLengthSquared);
            var useStart = Vector.Equals(minLengthSquared, startLengthSquared);
            Vector3Wide.ConditionalSelect(ref useStart, ref startOffset, ref endOffset, out var localNormal);
            var inverseLength = Vector<float>.One / Vector.SquareRoot(minLengthSquared);
            Vector3Wide.Scale(ref localNormal, ref inverseLength, out localNormal);

            //Note that we defer contact position offseting by the normal * depth until later when we know the normal isn't going to change.
            Vector3Wide.Subtract(ref closestFromBoxStart, ref localOffsetA, out var localA0);
            Vector3Wide.Subtract(ref closestFromBoxEnd, ref localOffsetA, out var localA1);
            Vector3Wide.Dot(ref localNormal, ref startOffset, out var dot0);
            Vector3Wide.Dot(ref localNormal, ref endOffset, out var dot1);
            manifold.Depth0 = a.Radius - dot0;
            manifold.Depth1 = a.Radius - dot1;
            manifold.FeatureId0 = Vector<int>.Zero;
            manifold.FeatureId1 = Vector<int>.One;

            //Capsule-box has an extremely important property:
            //Capsule internal line segments almost never intersect the box due to the capsule's radius.
            //We can effectively split this implementation into two tests- exterior and interior.
            //The interior test only needs to be run if one of the subpairs happens to be deeply intersecting, which should be ~never in practice.
            //That's fortunate, because computing an accurate interior normal is painful!
            var useInterior = Vector.LessThan(minLengthSquared, new Vector<float>(1e-10f));
            if (Vector.EqualsAny(useInterior, new Vector<int>(-1)))
            {
                //There are six possible separating axes when the capsule's internal line segment intersects the box:
                //box.X
                //box.Y
                //box.Z
                //box.X x capsule.Y
                //box.Y x capsule.Y
                //box.Z x capsule.Y
                //Choose the axis with the minimum penetration depth.
                //By working in the box's local space, box.X/Y/Z become simply (1,0,0), (0,1,0), and (0,0,1). This cancels out a whole bunch of arithmetic.

                //Note that these depths will not take into account the radius until the very end. We pretend that the capsule is simply a line until then, since the radius changes nothing.
                var faceDepthX = b.HalfWidth - localOffsetA.X + Vector.Abs(capsuleAxis.X * a.HalfLength);
                var faceDepthY = b.HalfHeight - localOffsetA.Y + Vector.Abs(capsuleAxis.Y * a.HalfLength);
                var faceDepthZ = b.HalfLength - localOffsetA.Z + Vector.Abs(capsuleAxis.Z * a.HalfLength);
                var faceDepth = Vector.Min(Vector.Min(faceDepthX, faceDepthY), faceDepthZ);
                var useFaceX = Vector.Equals(faceDepth, faceDepthX);
                var useFaceY = Vector.AndNot(Vector.Equals(faceDepth, faceDepthY), useFaceX);
                var useFaceZ = Vector.OnesComplement(Vector.BitwiseOr(useFaceX, useFaceY));
                //Note that signs are calibrated as a last step so that edge normals do not need their own calibration.
                Vector3Wide faceNormal;
                faceNormal.X = Vector.ConditionalSelect(useFaceX, Vector<float>.One, Vector<float>.Zero);
                faceNormal.Y = Vector.ConditionalSelect(useFaceY, Vector<float>.One, Vector<float>.Zero);
                faceNormal.Z = Vector.ConditionalSelect(useFaceZ, Vector<float>.One, Vector<float>.Zero);

                //For each edge, given the zero components, the cross product boils down to creating a perpendicular 2d vector on the plane of the box axis from the cylinder axis.
                var infiniteDepth = new Vector<float>(float.MaxValue);
                var bestDepth = infiniteDepth;
                var edgeXLength = Vector.SquareRoot(y2 + z2);
                var edgeYLength = Vector.SquareRoot(x2 + z2);
                var edgeZLength = Vector.SquareRoot(x2 + y2);
                //depth = dot(N, halfExtents - offsetA), where N = (edgeAxis x capsuleAxis) / ||(edgeAxis x capsuleAxis)||, and both N and halfExtents have their signs calibrated.
                //Using abs allows us to handle the sign calibration inline at the cost of splitting the dot product.
                var inverseLengthX = Vector<float>.One / edgeXLength;
                var inverseLengthY = Vector<float>.One / edgeYLength;
                var inverseLengthZ = Vector<float>.One / edgeZLength;
                var edgeXDepth = (Vector.Abs(b.HalfHeight * capsuleAxis.Z - b.HalfLength * capsuleAxis.Y) - Vector.Abs(offsetB.Y * capsuleAxis.Z - offsetB.Z * capsuleAxis.Y)) * inverseLengthX;
                var edgeYDepth = (Vector.Abs(b.HalfWidth * capsuleAxis.Z - b.HalfLength * capsuleAxis.X) - Vector.Abs(offsetB.X * capsuleAxis.Z - offsetB.Z * capsuleAxis.X)) * inverseLengthY;
                var edgeZDepth = (Vector.Abs(b.HalfWidth * capsuleAxis.Y - b.HalfHeight * capsuleAxis.X) - Vector.Abs(offsetB.X * capsuleAxis.Y - offsetB.Y * capsuleAxis.X)) * inverseLengthZ;
                var epsilon = new Vector<float>(1e-9f);
                //If the capsule internal segment and the edge are parallel, we ignore the edge by replacing its depth with ~infinity.
                //Such parallel axes are guaranteed to do no better than one of the face directions since this path is only relevant during segment-box deep collisions.
                edgeXDepth = Vector.ConditionalSelect(Vector.LessThan(edgeXLength, epsilon), infiniteDepth, edgeXDepth);
                edgeYDepth = Vector.ConditionalSelect(Vector.LessThan(edgeYLength, epsilon), infiniteDepth, edgeYDepth);
                edgeZDepth = Vector.ConditionalSelect(Vector.LessThan(edgeZLength, epsilon), infiniteDepth, edgeZDepth);

                var edgeDepth = Vector.Min(Vector.Min(edgeXDepth, edgeYDepth), edgeZDepth);
                var useEdgeX = Vector.Equals(edgeDepth, edgeXDepth);
                var useEdgeY = Vector.Equals(edgeDepth, edgeYDepth);
                Vector3Wide edgeNormal;
                edgeNormal.X = Vector.ConditionalSelect(useEdgeX, Vector<float>.Zero, Vector.ConditionalSelect(useEdgeY, capsuleAxis.Z, capsuleAxis.Y));
                edgeNormal.Y = Vector.ConditionalSelect(useEdgeX, capsuleAxis.Z, Vector.ConditionalSelect(useEdgeY, Vector<float>.Zero, -capsuleAxis.X));
                edgeNormal.Z = Vector.ConditionalSelect(useEdgeX, -capsuleAxis.Y, Vector.ConditionalSelect(useEdgeY, -capsuleAxis.X, Vector<float>.Zero));

                var useEdge = Vector.LessThan(edgeDepth, faceDepth);
                Vector3Wide.ConditionalSelect(ref useEdge, ref edgeNormal, ref faceNormal, out var interiorNormal);

                //Calibrate the normal such that it points from B to A.
                Vector3Wide.Dot(ref interiorNormal, ref offsetB, out var dot);
                var shouldNegateNormal = Vector.GreaterThan(dot, Vector<float>.Zero);
                interiorNormal.X = Vector.ConditionalSelect(shouldNegateNormal, -interiorNormal.X, interiorNormal.X);
                interiorNormal.Y = Vector.ConditionalSelect(shouldNegateNormal, -interiorNormal.Y, interiorNormal.Y);
                interiorNormal.Z = Vector.ConditionalSelect(shouldNegateNormal, -interiorNormal.Z, interiorNormal.Z);

                //Each contact may have its own depth.
                //Imagine a face collision- if the capsule axis isn't fully parallel with the plane's surface, it would be strange to use the same depth for both contacts.
                //Compute the interval of the box on the normal. Note that the normal is already calibrated to point from B to A (box to capsule).
                var boxExtreme = Vector.Abs(localNormal.X * b.HalfWidth) + Vector.Abs(localNormal.Y * b.HalfHeight) + Vector.Abs(localNormal.Z * b.HalfLength);
                Vector3Wide.Dot(ref localNormal, ref closestFromBoxStart, out dot0);
                Vector3Wide.Dot(ref localNormal, ref closestFromBoxEnd, out dot1);
                manifold.Depth0 = Vector.ConditionalSelect(useInterior, a.Radius + boxExtreme - dot0, manifold.Depth0);
                manifold.Depth1 = Vector.ConditionalSelect(useInterior, a.Radius + boxExtreme - dot1, manifold.Depth1);

                //The interior normal doesn't check all possible separating axes in the non-segment-intersecting case.
                //It would be wrong to overwrite the initial test if the interior test was unnecessary.
                Vector3Wide.ConditionalSelect(ref useInterior, ref interiorNormal, ref localNormal, out localNormal);
            }

            //Transform A0, A1, and the normal into world space.
            Matrix3x3Wide.CreateFromQuaternion(ref orientationB, out var orientationMatrixB);
            Matrix3x3Wide.TransformWithoutOverlap(ref localNormal, ref orientationMatrixB, out manifold.Normal);
            Matrix3x3Wide.TransformWithoutOverlap(ref localA0, ref orientationMatrixB, out manifold.OffsetA0);
            Matrix3x3Wide.TransformWithoutOverlap(ref localA1, ref orientationMatrixB, out manifold.OffsetA1);

            //Apply the normal offset to the contact positions.           
            var negativeOffsetFromA0 = manifold.Depth0 * 0.5f - a.Radius;
            var negativeOffsetFromA1 = manifold.Depth1 * 0.5f - a.Radius;
            Vector3Wide.Scale(ref manifold.Normal, ref negativeOffsetFromA0, out var normalPush0);
            Vector3Wide.Scale(ref manifold.Normal, ref negativeOffsetFromA1, out var normalPush1);
            Vector3Wide.Add(ref manifold.OffsetA0, ref normalPush0, out manifold.OffsetA0);
            Vector3Wide.Add(ref manifold.OffsetA1, ref normalPush1, out manifold.OffsetA1);

            //Note that it is possible that the two contacts is redundant. Only keep them both if they're separated.
            manifold.Count = Vector.ConditionalSelect(Vector.LessThan(Vector.Abs(dotStart - dotEnd), a.HalfLength * new Vector<float>(1e-7f)), Vector<int>.One, new Vector<int>(2));
        }

        public void Test(ref CapsuleWide a, ref BoxWide b, ref Vector3Wide offsetB, ref QuaternionWide orientationB, out Convex2ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public void Test(ref CapsuleWide a, ref BoxWide b, ref Vector3Wide offsetB, out Convex2ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }

    public class CapsuleBoxCollisionTask : CollisionTask
    {
        public CapsuleBoxCollisionTask()
        {
            BatchSize = 32;
            ShapeTypeIndexA = default(Capsule).TypeId;
            ShapeTypeIndexB = default(Box).TypeId;
        }


        //Every single collision task type will mirror this general layout.
        public unsafe override void ExecuteBatch<TContinuations, TFilters>(ref UntypedList batch, ref StreamingBatcher batcher, ref TContinuations continuations, ref TFilters filters)
        {
            CollisionTaskCommon.ExecuteBatch
                <TContinuations, TFilters,
                Capsule, CapsuleWide, Box, BoxWide, TestPairWide<Capsule, CapsuleWide, Box, BoxWide>,
                Convex2ContactManifoldWide, CapsuleBoxTester>(ref batch, ref batcher, ref continuations, ref filters);
        }
    }
}
