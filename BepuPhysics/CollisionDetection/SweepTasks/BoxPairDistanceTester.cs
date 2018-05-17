//using BepuPhysics.Collidables;
//using BepuUtilities;
//using System;
//using System.Numerics;
//using System.Runtime.CompilerServices;

//namespace BepuPhysics.CollisionDetection.SweepTasks
//{
//    //TODO: As written, this is slower than GJK. It also has some conceptual bugs- it can pick edges as representatives for the closest point that aren't truly the closest point.
//    //It should be possible to outperform GJK for box-box and thereby avoid GJK's potential numerical issues, but for now we're going to just move on.
//    public struct BoxPairDistanceTester : IPairDistanceTester<BoxWide, BoxWide>
//    {
//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        static void TestEdgeEdge(
//               ref BoxWide a, ref Matrix3x3Wide rA, ref Vector<float> halfExtentA, ref Vector3Wide edgeCenterA, ref Vector3Wide edgeDirectionA,
//               ref BoxWide b, ref Matrix3x3Wide rB, ref Vector<float> halfExtentB, ref Vector3Wide edgeCenterB, ref Vector3Wide edgeDirectionB,
//               ref Vector3Wide offsetB,
//               out Vector<float> depth, out Vector3Wide normal, out Vector3Wide closestA)
//        {
//            //While we based the edgeCenterB on the rotation, we deferred the translation since it's common to all cases.
//            Vector3Wide.Add(ref edgeCenterB, ref offsetB, out edgeCenterB);
//            Vector3Wide.Subtract(ref edgeCenterB, ref edgeCenterA, out var edgeCentersOffsetAB);
//            Vector3Wide.CrossWithoutOverlap(ref edgeDirectionA, ref edgeDirectionB, out normal);
//            Vector3Wide.Length(ref normal, out var length);
//            var inverseLength = Vector<float>.One / length;
//            Vector3Wide.Scale(ref normal, ref inverseLength, out normal);
//            Vector3Wide.Dot(ref normal, ref offsetB, out var calibrationDot);
//            var shouldNegate = Vector.GreaterThan(calibrationDot, Vector<float>.Zero);
//            normal.X = Vector.ConditionalSelect(shouldNegate, -normal.X, normal.X);
//            normal.Y = Vector.ConditionalSelect(shouldNegate, -normal.Y, normal.Y);
//            normal.Z = Vector.ConditionalSelect(shouldNegate, -normal.Z, normal.Z);
//            Vector3Wide.Dot(ref normal, ref edgeCentersOffsetAB, out var depthOld);

//            Matrix3x3Wide.TransformByTransposedWithoutOverlap(ref normal, ref rA, out var localNormalA);
//            Matrix3x3Wide.TransformByTransposedWithoutOverlap(ref normal, ref rB, out var localNormalB);

//            depth =
//                (a.HalfWidth * Vector.Abs(localNormalA.X) + a.HalfHeight * Vector.Abs(localNormalA.Y) + a.HalfLength * Vector.Abs(localNormalA.Z)) +
//                (b.HalfWidth * Vector.Abs(localNormalB.X) + b.HalfHeight * Vector.Abs(localNormalB.Y) + b.HalfLength * Vector.Abs(localNormalB.Z)) - Vector.Abs(calibrationDot);


//            //The edge distance is only meaningful if the normal is well defined and the edges projected intervals overlap.
//            //(Parallel edges and edge endpoint cases are handled by the vertex-box cases.)
//            Vector3Wide.Dot(ref edgeDirectionA, ref edgeDirectionB, out var dadb);
//            Vector3Wide.Dot(ref edgeDirectionA, ref edgeCentersOffsetAB, out var daOffsetAB);
//            Vector3Wide.Dot(ref edgeDirectionB, ref edgeCentersOffsetAB, out var dbOffsetAB);
//            //Note potential division by zero when the axes are parallel. Later validation catches it.
//            var ta = (daOffsetAB - dbOffsetAB * dadb) / (Vector<float>.One - dadb * dadb);
//            //tb = ta * (da * db) - db * (b - a)
//            var tb = ta * dadb - dbOffsetAB;
//            Vector3Wide.Scale(ref edgeDirectionA, ref ta, out closestA);
//            Vector3Wide.Add(ref closestA, ref edgeCenterA, out closestA);

//            var outsideA = Vector.BitwiseOr(Vector.GreaterThan(ta, halfExtentA), Vector.LessThan(ta, -halfExtentA));
//            var outsideB = Vector.BitwiseOr(Vector.GreaterThan(tb, halfExtentB), Vector.LessThan(tb, -halfExtentB));
//            var closestPointOutsideOfEdges = Vector.BitwiseOr(outsideA, outsideB);
//            var badNormal = Vector.LessThan(length, new Vector<float>(1e-15f));
//            var edgeDistanceInvalid = Vector.BitwiseOr(badNormal, closestPointOutsideOfEdges);
//            depth = Vector.ConditionalSelect(edgeDistanceInvalid, new Vector<float>(float.MaxValue), depth);
//        }

//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        static void Select(
//            ref Vector<float> depth, ref Vector3Wide normal, ref Vector3Wide closestA,
//            ref Vector<float> depthCandidate, ref Vector3Wide normalCandidate, ref Vector3Wide closestACandidate)
//        {
//            var useCandidate = Vector.LessThan(depthCandidate, depth);
//            depth = Vector.Min(depth, depthCandidate);
//            Vector3Wide.ConditionalSelect(useCandidate, normalCandidate, normal, out normal);
//            Vector3Wide.ConditionalSelect(useCandidate, closestACandidate, closestA, out closestA);
//        }

//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        private static void VertexBox(ref BoxWide a, ref BoxWide b, ref Vector3Wide x, ref Vector3Wide y, ref Vector3Wide z, ref Matrix3x3Wide localRB, ref Vector3Wide localOffsetB,
//            out Vector<float> depth, out Vector3Wide localNormalA, out Vector3Wide closestA)
//        {
//            Vector3Wide.Add(ref x, ref y, out var vertexB);
//            Vector3Wide.Add(ref z, ref localOffsetB, out var add);
//            Vector3Wide.Add(ref add, ref vertexB, out vertexB);
//            closestA.X = Vector.Min(a.HalfWidth, Vector.Max(-a.HalfWidth, vertexB.X));
//            closestA.Y = Vector.Min(a.HalfHeight, Vector.Max(-a.HalfHeight, vertexB.Y));
//            closestA.Z = Vector.Min(a.HalfLength, Vector.Max(-a.HalfLength, vertexB.Z));
//            Vector3Wide.Subtract(ref closestA, ref vertexB, out var vertexOffset);
//            Vector3Wide.Length(ref vertexOffset, out var length);
//            var inverseLength = Vector<float>.One / length;
//            Vector3Wide.Scale(ref vertexOffset, ref inverseLength, out localNormalA);
//            Vector3Wide.Dot(ref localOffsetB, ref localNormalA, out var calibrationDot);
//            var shouldNegate = Vector.GreaterThan(calibrationDot, Vector<float>.Zero);
//            localNormalA.X = Vector.ConditionalSelect(shouldNegate, -localNormalA.X, localNormalA.X);
//            localNormalA.Y = Vector.ConditionalSelect(shouldNegate, -localNormalA.Y, localNormalA.Y);
//            localNormalA.Z = Vector.ConditionalSelect(shouldNegate, -localNormalA.Z, localNormalA.Z);
//            Matrix3x3Wide.TransformByTransposedWithoutOverlap(ref localNormalA, ref localRB, out var localNormalB);
//            depth =
//                (Vector.Abs(localNormalA.X) * a.HalfWidth + Vector.Abs(localNormalA.Y) * a.HalfHeight + Vector.Abs(localNormalA.Z) * a.HalfLength) +
//                (Vector.Abs(localNormalB.X) * b.HalfWidth + Vector.Abs(localNormalB.Y) * b.HalfHeight + Vector.Abs(localNormalB.Z) * b.HalfLength) - Vector.Abs(calibrationDot);
//            //Avoid leaking NaN-infected values.
//            depth = Vector.ConditionalSelect(Vector.LessThan(length, new Vector<float>(1e-15f)), new Vector<float>(float.MaxValue), depth);
//        }

//        [MethodImpl(MethodImplOptions.AggressiveInlining)]
//        private static void VerticesBox(ref BoxWide a, ref BoxWide b, ref Matrix3x3Wide rA, ref Matrix3x3Wide rB, ref Vector3Wide offsetB,
//            out Vector<float> depth, out Vector3Wide offset, out Vector3Wide closestB)
//        {
//            Matrix3x3Wide.MultiplyByTransposeWithoutOverlap(ref rB, ref rA, out var rBInA);
//            Matrix3x3Wide.TransformByTransposedWithoutOverlap(ref offsetB, ref rA, out var localOffsetB);
//            Vector3Wide.Scale(ref rBInA.X, ref b.HalfWidth, out var x);
//            Vector3Wide.Scale(ref rBInA.Y, ref b.HalfHeight, out var y);
//            Vector3Wide.Scale(ref rBInA.Z, ref b.HalfLength, out var z);
//            Vector3Wide.Negate(ref x, out var negativeX);
//            Vector3Wide.Negate(ref y, out var negativeY);
//            Vector3Wide.Negate(ref z, out var negativeZ);
//            VertexBox(ref a, ref b, ref x, ref y, ref z, ref rBInA, ref localOffsetB, out depth, out var localNormalA, out var localClosestA);
//            VertexBox(ref a, ref b, ref x, ref y, ref negativeZ, ref rBInA, ref localOffsetB, out var depthCandidate, out var normalCandidate, out var closestACandidate);
//            Select(ref depth, ref localNormalA, ref localClosestA, ref depthCandidate, ref normalCandidate, ref closestACandidate);
//            VertexBox(ref a, ref b, ref x, ref negativeY, ref z, ref rBInA, ref localOffsetB, out depthCandidate, out normalCandidate, out closestACandidate);
//            Select(ref depth, ref localNormalA, ref localClosestA, ref depthCandidate, ref normalCandidate, ref closestACandidate);
//            VertexBox(ref a, ref b, ref x, ref negativeY, ref negativeZ, ref rBInA, ref localOffsetB, out depthCandidate, out normalCandidate, out closestACandidate);
//            Select(ref depth, ref localNormalA, ref localClosestA, ref depthCandidate, ref normalCandidate, ref closestACandidate);
//            VertexBox(ref a, ref b, ref negativeX, ref y, ref z, ref rBInA, ref localOffsetB, out depthCandidate, out normalCandidate, out closestACandidate);
//            Select(ref depth, ref localNormalA, ref localClosestA, ref depthCandidate, ref normalCandidate, ref closestACandidate);
//            VertexBox(ref a, ref b, ref negativeX, ref y, ref negativeZ, ref rBInA, ref localOffsetB, out depthCandidate, out normalCandidate, out closestACandidate);
//            Select(ref depth, ref localNormalA, ref localClosestA, ref depthCandidate, ref normalCandidate, ref closestACandidate);
//            VertexBox(ref a, ref b, ref negativeX, ref negativeY, ref z, ref rBInA, ref localOffsetB, out depthCandidate, out normalCandidate, out closestACandidate);
//            Select(ref depth, ref localNormalA, ref localClosestA, ref depthCandidate, ref normalCandidate, ref closestACandidate);
//            VertexBox(ref a, ref b, ref negativeX, ref negativeY, ref negativeZ, ref rBInA, ref localOffsetB, out depthCandidate, out normalCandidate, out closestACandidate);
//            Select(ref depth, ref localNormalA, ref localClosestA, ref depthCandidate, ref normalCandidate, ref closestACandidate);

//            Matrix3x3Wide.TransformByTransposedWithoutOverlap(ref localNormalA, ref rBInA, out var localNormalB);
//            Vector3Wide localExtremeOnB;
//            localExtremeOnB.X = Vector.ConditionalSelect(Vector.LessThan(localNormalB.X, Vector<float>.Zero), -b.HalfWidth, b.HalfWidth);
//            localExtremeOnB.Y = Vector.ConditionalSelect(Vector.LessThan(localNormalB.Y, Vector<float>.Zero), -b.HalfHeight, b.HalfHeight);
//            localExtremeOnB.Z = Vector.ConditionalSelect(Vector.LessThan(localNormalB.Z, Vector<float>.Zero), -b.HalfLength, b.HalfLength);
//            Matrix3x3Wide.TransformWithoutOverlap(ref localExtremeOnB, ref rB, out var extremeOnB);
//            Vector3Wide.Add(ref extremeOnB, ref offsetB, out closestB);

//            //We were working in rA's local space; push it back into world.
//            Matrix3x3Wide.TransformWithoutOverlap(ref localNormalA, ref rA, out offset);

//        }

//        public void Test(ref BoxWide a, ref BoxWide b, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
//            out Vector<int> intersected, out Vector<float> distance, out Vector3Wide closestA, out Vector3Wide normal)
//        {
//            Matrix3x3Wide.CreateFromQuaternion(ref orientationA, out var rA);
//            Matrix3x3Wide.CreateFromQuaternion(ref orientationB, out var rB);
//            Matrix3x3Wide.TransformByTransposedWithoutOverlap(ref offsetB, ref rA, out var offsetBLocalA);
//            Vector3Wide signedHalfExtentsA;
//            signedHalfExtentsA.X = Vector.ConditionalSelect(Vector.LessThan(offsetBLocalA.X, Vector<float>.Zero), -a.HalfWidth, a.HalfWidth);
//            signedHalfExtentsA.Y = Vector.ConditionalSelect(Vector.LessThan(offsetBLocalA.Y, Vector<float>.Zero), -a.HalfHeight, a.HalfHeight);
//            signedHalfExtentsA.Z = Vector.ConditionalSelect(Vector.LessThan(offsetBLocalA.Z, Vector<float>.Zero), -a.HalfLength, a.HalfLength);
//            Matrix3x3Wide.TransformByTransposedWithoutOverlap(ref offsetB, ref rB, out var offsetBLocalB);
//            Vector3Wide signedHalfExtentsB;
//            signedHalfExtentsB.X = Vector.ConditionalSelect(Vector.LessThan(offsetBLocalB.X, Vector<float>.Zero), b.HalfWidth, -b.HalfWidth);
//            signedHalfExtentsB.Y = Vector.ConditionalSelect(Vector.LessThan(offsetBLocalB.Y, Vector<float>.Zero), b.HalfHeight, -b.HalfHeight);
//            signedHalfExtentsB.Z = Vector.ConditionalSelect(Vector.LessThan(offsetBLocalB.Z, Vector<float>.Zero), b.HalfLength, -b.HalfLength);

//            Vector3Wide.Scale(ref rA.X, ref signedHalfExtentsA.X, out Vector3Wide xa);
//            Vector3Wide.Scale(ref rA.Y, ref signedHalfExtentsA.Y, out Vector3Wide ya);
//            Vector3Wide.Scale(ref rA.Z, ref signedHalfExtentsA.Z, out Vector3Wide za);
//            Vector3Wide.Scale(ref rB.X, ref signedHalfExtentsB.X, out Vector3Wide xb);
//            Vector3Wide.Scale(ref rB.Y, ref signedHalfExtentsB.Y, out Vector3Wide yb);
//            Vector3Wide.Scale(ref rB.Z, ref signedHalfExtentsB.Z, out Vector3Wide zb);
//            //aX x bX
//            Vector3Wide.Add(ref ya, ref za, out var edgeCenterA);
//            Vector3Wide.Add(ref yb, ref zb, out var edgeCenterB);
//            TestEdgeEdge(ref a, ref rA, ref a.HalfWidth, ref edgeCenterA, ref rA.X, ref b, ref rB, ref b.HalfWidth, ref edgeCenterB, ref rB.X, ref offsetB,
//                out var depth, out normal, out closestA);
//            //aX x bY
//            Vector3Wide.Add(ref xb, ref zb, out edgeCenterB);
//            TestEdgeEdge(ref a, ref rA, ref a.HalfWidth, ref edgeCenterA, ref rA.X, ref b, ref rB, ref b.HalfHeight, ref edgeCenterB, ref rB.Y, ref offsetB,
//                out var edgeDepthCandidate, out var normalCandidate, out var closestACandidate);
//            Select(ref depth, ref normal, ref closestA, ref edgeDepthCandidate, ref normalCandidate, ref closestACandidate);
//            //aX x bZ
//            Vector3Wide.Add(ref xb, ref yb, out edgeCenterB);
//            TestEdgeEdge(ref a, ref rA, ref a.HalfWidth, ref edgeCenterA, ref rA.X, ref b, ref rB, ref b.HalfLength, ref edgeCenterB, ref rB.Z, ref offsetB,
//                out edgeDepthCandidate, out normalCandidate, out closestACandidate);
//            Select(ref depth, ref normal, ref closestA, ref edgeDepthCandidate, ref normalCandidate, ref closestACandidate);
//            //aY x bX
//            Vector3Wide.Add(ref xa, ref za, out edgeCenterA);
//            Vector3Wide.Add(ref yb, ref zb, out edgeCenterB);
//            TestEdgeEdge(ref a, ref rA, ref a.HalfHeight, ref edgeCenterA, ref rA.Y, ref b, ref rB, ref b.HalfWidth, ref edgeCenterB, ref rB.X, ref offsetB,
//                out edgeDepthCandidate, out normalCandidate, out closestACandidate);
//            Select(ref depth, ref normal, ref closestA, ref edgeDepthCandidate, ref normalCandidate, ref closestACandidate);
//            //aY x bY
//            Vector3Wide.Add(ref xb, ref zb, out edgeCenterB);
//            TestEdgeEdge(ref a, ref rA, ref a.HalfHeight, ref edgeCenterA, ref rA.Y, ref b, ref rB, ref b.HalfHeight, ref edgeCenterB, ref rB.Y, ref offsetB,
//                out edgeDepthCandidate, out normalCandidate, out closestACandidate);
//            Select(ref depth, ref normal, ref closestA, ref edgeDepthCandidate, ref normalCandidate, ref closestACandidate);
//            //aY x bZ
//            Vector3Wide.Add(ref xb, ref yb, out edgeCenterB);
//            TestEdgeEdge(ref a, ref rA, ref a.HalfHeight, ref edgeCenterA, ref rA.Y, ref b, ref rB, ref b.HalfLength, ref edgeCenterB, ref rB.Z, ref offsetB,
//                out edgeDepthCandidate, out normalCandidate, out closestACandidate);
//            Select(ref depth, ref normal, ref closestA, ref edgeDepthCandidate, ref normalCandidate, ref closestACandidate);
//            //aZ x bX
//            Vector3Wide.Add(ref xa, ref ya, out edgeCenterA);
//            Vector3Wide.Add(ref yb, ref zb, out edgeCenterB);
//            TestEdgeEdge(ref a, ref rA, ref a.HalfLength, ref edgeCenterA, ref rA.Z, ref b, ref rB, ref b.HalfWidth, ref edgeCenterB, ref rB.X, ref offsetB,
//                out edgeDepthCandidate, out normalCandidate, out closestACandidate);
//            Select(ref depth, ref normal, ref closestA, ref edgeDepthCandidate, ref normalCandidate, ref closestACandidate);
//            //aZ x bY
//            Vector3Wide.Add(ref xb, ref zb, out edgeCenterB);
//            TestEdgeEdge(ref a, ref rA, ref a.HalfLength, ref edgeCenterA, ref rA.Z, ref b, ref rB, ref b.HalfHeight, ref edgeCenterB, ref rB.Y, ref offsetB,
//                out edgeDepthCandidate, out normalCandidate, out closestACandidate);
//            Select(ref depth, ref normal, ref closestA, ref edgeDepthCandidate, ref normalCandidate, ref closestACandidate);
//            //aZ x bZ
//            Vector3Wide.Add(ref xb, ref yb, out edgeCenterB);
//            TestEdgeEdge(ref a, ref rA, ref a.HalfLength, ref edgeCenterA, ref rA.Z, ref b, ref rB, ref b.HalfLength, ref edgeCenterB, ref rB.Z, ref offsetB,
//                out edgeDepthCandidate, out normalCandidate, out closestACandidate);
//            Select(ref depth, ref normal, ref closestA, ref edgeDepthCandidate, ref normalCandidate, ref closestACandidate);

//            VerticesBox(ref a, ref b, ref rA, ref rB, ref offsetB, out var verticesDepth, out var verticesNormal, out var verticesClosestOnB);
//            Vector3Wide.Scale(ref verticesNormal, ref verticesDepth, out var offset);
//            Vector3Wide.Add(ref verticesClosestOnB, ref offset, out var verticesClosestA);

//            Vector3Wide.Negate(ref offsetB, out var offsetA);
//            VerticesBox(ref b, ref a, ref rB, ref rA, ref offsetA, out var verticesDepthCandidate, out var verticesNormalCandidate, out var verticesClosestACandidate);
//            //Note that the test of the vertices of B output the closest point as belonging to B, so we apply the offset to get the other point.
//            Vector3Wide.Add(ref verticesClosestACandidate, ref offsetB, out verticesClosestACandidate);
//            Vector3Wide.Negate(ref verticesNormalCandidate, out verticesNormalCandidate);
//            //Vector3Wide.Scale(ref verticesNormalB, ref verticesDepthB, out var offset);
//            //Vector3Wide.Subtract(ref verticesClosestB, ref offset, out verticesClosestB);
//            Select(ref verticesDepth, ref verticesNormal, ref verticesClosestA, ref verticesDepthCandidate, ref verticesNormalCandidate, ref verticesClosestACandidate);

//            Select(ref depth, ref normal, ref closestA, ref verticesDepth, ref verticesNormal, ref verticesClosestA);
//            distance = -depth;
//            intersected = Vector.GreaterThanOrEqual(depth, Vector<float>.Zero);

//        }


//    }
//}
