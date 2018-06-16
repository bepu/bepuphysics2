using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using static BepuUtilities.GatherScatter;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct TrianglePairTester : IPairTester<TriangleWide, TriangleWide, Convex4ContactManifoldWide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void GetIntervalForNormal(in Vector3Wide a, in Vector3Wide b, in Vector3Wide c, in Vector3Wide normal, out Vector<float> min, out Vector<float> max)
        {
            Vector3Wide.Dot(normal, a, out var dA);
            Vector3Wide.Dot(normal, b, out var dB);
            Vector3Wide.Dot(normal, c, out var dC);
            min = Vector.Min(dA, Vector.Min(dB, dC));
            max = Vector.Max(dA, Vector.Max(dB, dC));
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void GetDepthForNormal(in Vector3Wide aA, in Vector3Wide bA, in Vector3Wide cA, in Vector3Wide aB, in Vector3Wide bB, in Vector3Wide cB,
            in Vector3Wide normal, out Vector<float> depth)
        {
            GetIntervalForNormal(aA, bA, cA, normal, out var minA, out var maxA);
            GetIntervalForNormal(aB, bB, cB, normal, out var minB, out var maxB);
            depth = Vector.Min(maxA - minB, maxB + minA);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void TestEdgeEdge(
            in Vector3Wide edgeDirectionA, in Vector3Wide edgeDirectionB,
            in Vector3Wide triangleCenterAToTriangleCenterB,
            in Vector3Wide aA, in Vector3Wide bA, in Vector3Wide cA, in Vector3Wide aB, in Vector3Wide bB, in Vector3Wide cB,
            out Vector<float> depth, out Vector3Wide normal)
        {
            //Calibrate the normal to point from the triangle to the box while normalizing.
            Vector3Wide.CrossWithoutOverlap(edgeDirectionA, edgeDirectionB, out normal);
            Vector3Wide.Length(normal, out var normalLength);
            //The normal should point from B to A by convention.
            Vector3Wide.Dot(triangleCenterAToTriangleCenterB, normal, out var calibrationDot);
            Vector3Wide.Scale(normal, Vector.ConditionalSelect(Vector.GreaterThan(calibrationDot, Vector<float>.Zero), new Vector<float>(-1f), Vector<float>.One) / normalLength, out normal);
            GetDepthForNormal(aA, bA, cA, aB, bB, cB, normal, out depth);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Select(
            ref Vector<float> depth, ref Vector3Wide normal,
            in Vector<float> depthCandidate, in Vector3Wide normalCandidate)
        {
            var useCandidate = Vector.LessThan(depthCandidate, depth);
            Vector3Wide.ConditionalSelect(useCandidate, normalCandidate, normal, out normal);
            depth = Vector.Min(depth, depthCandidate);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Select(
            ref Vector<float> depth, ref Vector3Wide normal,
            in Vector<float> depthCandidate, in Vector<float> nxCandidate, in Vector<float> nyCandidate, in Vector<float> nzCandidate)
        {
            var useCandidate = Vector.LessThan(depthCandidate, depth);
            normal.X = Vector.ConditionalSelect(useCandidate, nxCandidate, normal.X);
            normal.Y = Vector.ConditionalSelect(useCandidate, nyCandidate, normal.Y);
            normal.Z = Vector.ConditionalSelect(useCandidate, nzCandidate, normal.Z);
            depth = Vector.Min(depth, depthCandidate);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void Add(in Vector3Wide pointOnTriangle, in Vector3Wide triangleCenter, in Vector3Wide triangleTangentX, in Vector3Wide triangleTangentY, in Vector<int> featureId,
            in Vector<int> exists, ref ManifoldCandidate candidates, ref Vector<int> candidateCount)
        {
            Vector3Wide.Subtract(pointOnTriangle, triangleCenter, out var offset);
            ManifoldCandidate candidate;
            Vector3Wide.Dot(offset, triangleTangentX, out candidate.X);
            Vector3Wide.Dot(offset, triangleTangentY, out candidate.Y);
            candidate.FeatureId = featureId;
            ManifoldCandidateHelper.AddCandidate(ref candidates, ref candidateCount, candidate, exists);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void AddEdge(in Vector<float> tEntry, in Vector<float> tExit, in Vector<int> flip, in Vector<float> edgeHalfLength,
            in Vector<float> triangleCenterToEdgeDotX, in Vector<float> triangleCenterToEdgeDotY,
            in Vector<float> edgeDirectionDotX, in Vector<float> edgeDirectionDotY,
            in Vector<int> edgeId, in Vector<int> exitIdOffset,
            in Vector<float> epsilon, in Vector<int> six, ref ManifoldCandidate candidates, ref Vector<int> candidateCount)
        {
            ManifoldCandidate candidate;
            //This doesn't actually need to be a dynamic choice, but this was just the simplest way to do it. Could revisit it later, but... saving 16 instructions is hard to justify.
            var entry = Vector.ConditionalSelect(flip, -tExit, tEntry);
            var exit = Vector.ConditionalSelect(flip, -tEntry, tExit);
            var directionDotX = Vector.ConditionalSelect(flip, -edgeDirectionDotX, edgeDirectionDotX);
            var directionDotY = Vector.ConditionalSelect(flip, -edgeDirectionDotY, edgeDirectionDotY);
            //Entry
            var exists = Vector.BitwiseAnd(
                Vector.LessThan(candidateCount, six),
                Vector.BitwiseAnd(
                    Vector.GreaterThanOrEqual(exit - entry, epsilon),
                    Vector.LessThan(Vector.Abs(entry), edgeHalfLength)));
            candidate.X = triangleCenterToEdgeDotX + directionDotX * entry;
            candidate.Y = triangleCenterToEdgeDotY + directionDotY * entry;
            candidate.FeatureId = edgeId;
            ManifoldCandidateHelper.AddCandidate(ref candidates, ref candidateCount, candidate, exists);
            //Edge 0 exit
            exists = Vector.BitwiseAnd(
                Vector.LessThan(candidateCount, six),
                Vector.BitwiseAnd(
                    Vector.GreaterThanOrEqual(exit, entry),
                    Vector.LessThanOrEqual(Vector.Abs(exit), edgeHalfLength)));
            candidate.X = triangleCenterToEdgeDotX + directionDotX * exit;
            candidate.Y = triangleCenterToEdgeDotY + directionDotY * exit;
            candidate.FeatureId = edgeId + exitIdOffset;
            ManifoldCandidateHelper.AddCandidate(ref candidates, ref candidateCount, candidate, exists);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void ClipBoxEdgesAgainstTriangle(in Vector3Wide vA, in Vector3Wide vB, in Vector3Wide vC,
            in Vector3Wide abxN, in Vector3Wide bcxN, in Vector3Wide caxN,
            in Vector3Wide triangleCenter, in Vector3Wide triangleTangentX, in Vector3Wide triangleTangentY,
            in Vector3Wide boxEdgeDirection, in Vector3Wide boxEdgeCenterOffsetDirection, in Vector<float> edgeHalfLength, in Vector<float> edgeCenterOffset,
            in Vector3Wide boxFaceCenter, in Vector<int> baseId, in Vector<int> edgeDirectionId, in Vector<int> edgeCenterOffsetId,
            in Vector<int> flip0, in Vector<int> flip1, in Vector<float> epsilonScale, ref ManifoldCandidate candidates, ref Vector<int> candidateCount)
        {
            //The base id is the id of the vertex in the corner along the negative boxEdgeDirection and boxEdgeCenterOffsetDirection.
            //The edgeDirectionId is the amount to add when you move along the boxEdgeDirection to the other vertex.
            //The edgeCenterOffsetId is the amount to add when you move along the boxEdgeCenterOffsetDirection to the other vertex.

            //We have three edge planes, created by abxN and vA, bcxN and vB, and caxN and vC.
            //We want to test the box edge against all three of the edges.
            //intersection = dot((ab x N) / ||ab x N||, vA - boxEdgeCenter) / dot((ab x N) / ||ab x N||, boxEdgeDirection) = dot(abxN, vA - boxEdgeCenter) / dot(abxN, boxEdgeDirection)
            Vector3Wide.Scale(boxEdgeCenterOffsetDirection, edgeCenterOffset, out var boxEdgeCenterOffset);
            Vector3Wide.Subtract(boxFaceCenter, boxEdgeCenterOffset, out var boxEdgeCenter0);
            Vector3Wide.Add(boxFaceCenter, boxEdgeCenterOffset, out var boxEdgeCenter1);
            Vector3Wide.Dot(boxEdgeDirection, abxN, out var abVelocity);
            Vector3Wide.Dot(boxEdgeDirection, bcxN, out var bcVelocity);
            Vector3Wide.Dot(boxEdgeDirection, caxN, out var caVelocity);
            var inverseAB = Vector<float>.One / abVelocity;
            var inverseBC = Vector<float>.One / bcVelocity;
            var inverseCA = Vector<float>.One / caVelocity;
            Vector3Wide.Subtract(vA, boxEdgeCenter0, out var edgeCenter0ToA);
            Vector3Wide.Subtract(vB, boxEdgeCenter0, out var edgeCenter0ToB);
            Vector3Wide.Subtract(vA, boxEdgeCenter1, out var edgeCenter1ToA);
            Vector3Wide.Subtract(vB, boxEdgeCenter1, out var edgeCenter1ToB);
            Vector3Wide.Dot(abxN, edgeCenter0ToA, out var distanceAB0);
            Vector3Wide.Dot(bcxN, edgeCenter0ToB, out var distanceBC0);
            Vector3Wide.Dot(caxN, edgeCenter0ToA, out var distanceCA0);
            Vector3Wide.Dot(abxN, edgeCenter1ToA, out var distanceAB1);
            Vector3Wide.Dot(bcxN, edgeCenter1ToB, out var distanceBC1);
            Vector3Wide.Dot(caxN, edgeCenter1ToA, out var distanceCA1);
            var tAB0 = distanceAB0 * inverseAB;
            var tBC0 = distanceBC0 * inverseBC;
            var tCA0 = distanceCA0 * inverseCA;
            var tAB1 = distanceAB1 * inverseAB;
            var tBC1 = distanceBC1 * inverseBC;
            var tCA1 = distanceCA1 * inverseCA;
            //The interval for each box edge is created from the latest entry and earliest exit.
            //An 'entry' is when the box center starts with a nonnegative distance. An 'exit' is when the box center starts with a negative distance.
            var abIsEntry = Vector.GreaterThanOrEqual(abVelocity, Vector<float>.Zero);
            var bcIsEntry = Vector.GreaterThanOrEqual(bcVelocity, Vector<float>.Zero);
            var caIsEntry = Vector.GreaterThanOrEqual(caVelocity, Vector<float>.Zero);
            var max = new Vector<float>(float.MaxValue);
            var min = new Vector<float>(-float.MaxValue);
            var t0Entry = Vector.ConditionalSelect(abIsEntry, tAB0, min);
            var t0Exit = Vector.ConditionalSelect(abIsEntry, max, tAB0);
            t0Entry = Vector.Max(t0Entry, Vector.ConditionalSelect(bcIsEntry, tBC0, min));
            t0Exit = Vector.Min(t0Exit, Vector.ConditionalSelect(bcIsEntry, max, tBC0));
            t0Entry = Vector.Max(t0Entry, Vector.ConditionalSelect(caIsEntry, tCA0, min));
            t0Exit = Vector.Min(t0Exit, Vector.ConditionalSelect(caIsEntry, max, tCA0));

            var t1Entry = Vector.ConditionalSelect(abIsEntry, tAB1, min);
            var t1Exit = Vector.ConditionalSelect(abIsEntry, max, tAB1);
            t1Entry = Vector.Max(t1Entry, Vector.ConditionalSelect(bcIsEntry, tBC1, min));
            t1Exit = Vector.Min(t1Exit, Vector.ConditionalSelect(bcIsEntry, max, tBC1));
            t1Entry = Vector.Max(t1Entry, Vector.ConditionalSelect(caIsEntry, tCA1, min));
            t1Exit = Vector.Min(t1Exit, Vector.ConditionalSelect(caIsEntry, max, tCA1));

            t0Entry = Vector.Max(-edgeHalfLength, t0Entry);
            t0Exit = Vector.Min(edgeHalfLength, t0Exit);
            t1Entry = Vector.Max(-edgeHalfLength, t1Entry);
            t1Exit = Vector.Min(edgeHalfLength, t1Exit);

            //We now have intervals for both box edges.
            //If -halfSpan<min<halfSpan && (max-min)>epsilon for an edge, use the min intersection as a contact.
            //If -halfSpan<=max<=halfSpan && max>=min, use the max intersection as a contact.
            //Note the comparisons: if the max lies on a face vertex, it is used, but if the min lies on a face vertex, it is not. This avoids redundant entries.

            //Note that contact locations are orthographically projected onto the triangle's surface. 
            //We clipped the box edges against triangle edge planes perpendicular to the triangle's normal.
            //So, to compute the triangle tangent space locations, we can just do simple dot products.

            //dot(boxEdgeCenter0 + boxEdgeDirection * t0Entry - triangleCenter, triangleTangentX)
            //dot(boxEdgeCenter0 + boxEdgeDirection * t0Entry - triangleCenter, triangleTangentY)
            //dot(boxEdgeCenter0 + boxEdgeDirection * t0Exit - triangleCenter, triangleTangentX)
            //dot(boxEdgeCenter0 + boxEdgeDirection * t0Exit - triangleCenter, triangleTangentY)
            //dot(boxEdgeCenter1 + boxEdgeDirection * t1Entry - triangleCenter, triangleTangentX)
            //dot(boxEdgeCenter1 + boxEdgeDirection * t1Entry - triangleCenter, triangleTangentY)
            //dot(boxEdgeCenter1 + boxEdgeDirection * t1Exit - triangleCenter, triangleTangentX)
            //dot(boxEdgeCenter1 + boxEdgeDirection * t1Exit - triangleCenter, triangleTangentY)
            Vector3Wide.Subtract(boxEdgeCenter0, triangleCenter, out var triangleCenterToEdge0);
            Vector3Wide.Subtract(boxEdgeCenter1, triangleCenter, out var triangleCenterToEdge1);
            Vector3Wide.Dot(triangleCenterToEdge0, triangleTangentX, out var triangleCenterToEdge0DotX);
            Vector3Wide.Dot(triangleCenterToEdge0, triangleTangentY, out var triangleCenterToEdge0DotY);
            Vector3Wide.Dot(triangleCenterToEdge1, triangleTangentX, out var triangleCenterToEdge1DotX);
            Vector3Wide.Dot(triangleCenterToEdge1, triangleTangentY, out var triangleCenterToEdge1DotY);
            Vector3Wide.Dot(boxEdgeDirection, triangleTangentX, out var edgeDirectionDotX);
            Vector3Wide.Dot(boxEdgeDirection, triangleTangentY, out var edgeDirectionDotY);

            //Edge feature ids:
            //(1<<6) + (0 or 1) * boxFaceNormalId + boxEdgeDirection * (1<<2) + (0 or 1) * boxEdgeCenterOffset * (1<<4)
            //The base 1<<6 distinguishes it from triangle vertex contacts.
            //For contacts created from the max endpoint of an edge interval, we'll add an extra (1<<7) to distinguish it from the min endpoint.
            var featureBase = baseId + new Vector<int>(1 << 6);
            var exitIdOffset = new Vector<int>(1 << 7);
            var directionContribution = edgeDirectionId * (1 << 2);
            var edge0Id = featureBase + directionContribution;
            var edge1Id = featureBase + directionContribution + edgeCenterOffsetId * (1 << 4);
            var epsilon = new Vector<float>(1e-5f) * epsilonScale;
            var six = new Vector<int>(6);

            AddEdge(t0Entry, t0Exit, flip0,
                edgeHalfLength, triangleCenterToEdge0DotX, triangleCenterToEdge0DotY,
                edgeDirectionDotX, edgeDirectionDotY, edge0Id, exitIdOffset, epsilon, six, ref candidates, ref candidateCount);
            AddEdge(t1Entry, t1Exit, flip1,
                edgeHalfLength, triangleCenterToEdge1DotX, triangleCenterToEdge1DotY,
                edgeDirectionDotX, edgeDirectionDotY, edge1Id, exitIdOffset, epsilon, six, ref candidates, ref candidateCount);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void TryAddTriangleVertex(in Vector3Wide triangleVertex, in Vector<int> vertexId,
            in Vector3Wide triangleCenter, in Vector3Wide triangleX, in Vector3Wide triangleY, in Vector3Wide triangleNormal,
            in Vector3Wide boxTangentX, in Vector3Wide boxTangentY, in Vector<float> halfExtentX, in Vector<float> halfExtentY, in Vector<float> halfExtentZ,
            in Vector3Wide boxFaceNormal, in Vector<float> inverseTriangleNormalDotBoxFaceNormal,
            ref ManifoldCandidate candidates, ref Vector<int> candidateCount)
        {
            //Note that we cannot simply project orthographically onto the box face. All edge contacts are created by orthographic projection onto the *triangle*, so to test
            //containment of a triangle vertex on the box face, it must first be unprojected. Ray cast from triangleVertex along triangleNormal to the box plane.
            //t = -dot(boxFaceCenter - triangleVertex, boxFaceNormal) / dot(triangleNormal, boxFaceNormal)
            //t = -(dot(boxFaceCenter, boxFaceNormal) - dot(triangleVertex, boxFaceNormal)) / dot(triangleNormal, boxFaceNormal)
            //dot(boxFaceCenter, boxFaceNormal) == halfExtentZ, since the box is symmetric.
            //t = (dot(triangleVertex, boxFaceNormal) - halfExtentZ) / dot(triangleNormal, boxFaceNormal)

            //x = dot(triangleVertex + t * triangleNormal, boxX)
            //y = dot(triangleVertex + t * triangleNormal, boxY)

            Vector3Wide.Dot(triangleVertex, boxFaceNormal, out var vertexAlongBoxNormal);
            var distance = vertexAlongBoxNormal - halfExtentZ;
            var t = distance * inverseTriangleNormalDotBoxFaceNormal;

            //Can avoid 2 add instructions here by precomputing dot products but... complexity penalty.
            Vector3Wide.Scale(triangleNormal, t, out var offset);
            Vector3Wide.Add(offset, triangleVertex, out var unprojectedVertex);
            Vector3Wide.Dot(unprojectedVertex, boxTangentX, out var x);
            Vector3Wide.Dot(unprojectedVertex, boxTangentY, out var y);
            var contained = Vector.BitwiseAnd(
                Vector.LessThanOrEqual(Vector.Abs(x), halfExtentX),
                Vector.LessThanOrEqual(Vector.Abs(y), halfExtentY));
            Add(triangleVertex, triangleCenter, triangleX, triangleY, vertexId, contained, ref candidates, ref candidateCount);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Test(
            ref TriangleWide a, ref TriangleWide b, ref Vector<float> speculativeMargin,
            ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
            out Convex4ContactManifoldWide manifold)
        {
            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var worldRA);
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var worldRB);
            Matrix3x3Wide.MultiplyByTransposeWithoutOverlap(worldRB, worldRA, out var rB);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offsetB, worldRA, out var localOffsetB);
            Matrix3x3Wide.Transform(b.A, rB, out var bA);
            Vector3Wide.Add(bA, localOffsetB, out bA);
            Matrix3x3Wide.Transform(b.B, rB, out var bB);
            Vector3Wide.Add(bB, localOffsetB, out bB);
            Matrix3x3Wide.Transform(b.C, rB, out var bC);
            Vector3Wide.Add(bC, localOffsetB, out bC);

            Vector3Wide.Add(bA, bB, out var localTriangleCenterB);
            Vector3Wide.Add(localTriangleCenterB, bC, out localTriangleCenterB);
            Vector3Wide.Scale(localTriangleCenterB, new Vector<float>(1f / 3f), out localTriangleCenterB);

            Vector3Wide.Subtract(bB, bA, out var abB);
            Vector3Wide.Subtract(bC, bB, out var bcB);
            Vector3Wide.Subtract(bA, bC, out var caB);

            Vector3Wide.Add(a.A, a.B, out var localTriangleCenterA);
            Vector3Wide.Add(localTriangleCenterA, a.C, out localTriangleCenterA);
            Vector3Wide.Scale(localTriangleCenterA, new Vector<float>(1f / 3f), out localTriangleCenterA);

            Vector3Wide.Subtract(a.B, a.A, out var abA);
            Vector3Wide.Subtract(a.C, a.B, out var bcA);
            Vector3Wide.Subtract(a.A, a.C, out var caA);

            Vector3Wide.Subtract(localTriangleCenterB, localTriangleCenterA, out var centerAToCenterB);



            TestEdgeEdge(abA, abB, centerAToCenterB, a.A, a.B, a.C, bA, bB, bC, out var depth, out var localNormal);
            TestEdgeEdge(abA, bcB, centerAToCenterB, a.A, a.B, a.C, bA, bB, bC, out var depthCandidate, out var localNormalCandidate);
            Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            TestEdgeEdge(abA, caB, centerAToCenterB, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
            Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);

            TestEdgeEdge(bcA, abB, centerAToCenterB, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
            Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            TestEdgeEdge(bcA, bcB, centerAToCenterB, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
            Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            TestEdgeEdge(bcA, caB, centerAToCenterB, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
            Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);

            TestEdgeEdge(caA, abB, centerAToCenterB, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
            Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            TestEdgeEdge(caA, bcB, centerAToCenterB, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
            Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            TestEdgeEdge(caA, caB, centerAToCenterB, a.A, a.B, a.C, bA, bB, bC, out depthCandidate, out localNormalCandidate);
            Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);

            Vector3Wide.CrossWithoutOverlap()
            //Test face normals of A. Working in local space of A means potential axes are just (1,0,0) etc.
            var xNormalSign = Vector.ConditionalSelect(Vector.LessThan(localTriangleCenter.X, Vector<float>.Zero), Vector<float>.One, new Vector<float>(-1f));
            var yNormalSign = Vector.ConditionalSelect(Vector.LessThan(localTriangleCenter.Y, Vector<float>.Zero), Vector<float>.One, new Vector<float>(-1f));
            var zNormalSign = Vector.ConditionalSelect(Vector.LessThan(localTriangleCenter.Z, Vector<float>.Zero), Vector<float>.One, new Vector<float>(-1f));

            //Note that we ignore sign on interval tests here- the depth is the same regardless of what the sign is, so we don't have to take it into account.
            GetDepthForInterval(a.HalfWidth, bA.X, bB.X, bC.X, out var faceAXDepth);
            GetDepthForInterval(a.HalfHeight, bA.Y, bB.Y, bC.Y, out var faceAYDepth);
            GetDepthForInterval(a.HalfLength, bA.Z, bB.Z, bC.Z, out var faceAZDepth);
            Select(ref depth, ref localNormal, faceAXDepth, xNormalSign, Vector<float>.Zero, Vector<float>.Zero);
            Select(ref depth, ref localNormal, faceAYDepth, Vector<float>.Zero, yNormalSign, Vector<float>.Zero);
            Select(ref depth, ref localNormal, faceAZDepth, Vector<float>.Zero, Vector<float>.Zero, zNormalSign);

            //Test face normal of B.
            Vector3Wide.CrossWithoutOverlap(ab, ca, out var triangleNormal);
            Vector3Wide.Length(triangleNormal, out var triangleNormalLength);
            Vector3Wide.Scale(triangleNormal, Vector<float>.One / triangleNormalLength, out triangleNormal);
            //Calibrate the normal to point from B to A. This is needed because the choice to eliminate backface contacts depends on dot(localNormal, triangleNormal).
            Vector3Wide.Dot(triangleNormal, localTriangleCenter, out var trianglePlaneOffset);
            Vector3Wide.Negate(triangleNormal, out var negatedTriangleNormal);
            Vector3Wide.ConditionalSelect(Vector.GreaterThan(trianglePlaneOffset, Vector<float>.Zero), negatedTriangleNormal, triangleNormal, out var calibratedTriangleNormal);
            var triangleFaceDepth =
                Vector.Abs(triangleNormal.X) * a.HalfWidth + Vector.Abs(triangleNormal.Y) * a.HalfHeight + Vector.Abs(triangleNormal.Z) * a.HalfLength - Vector.Abs(trianglePlaneOffset);
            Select(ref depth, ref localNormal, triangleFaceDepth, calibratedTriangleNormal);

            //At this point, we have computed the minimum depth and associated local normal.
            //We now need to compute some contact locations, their per-contact depths, and the feature ids.

            //Contact generation always assumes face-face clipping. Other forms of contact generation are just special cases of face-face, and since we pay
            //for all code paths, there's no point in handling them separately.


            //For feature ids, we will use triangle vertex index (0, 1, 2) for box face contacts.
            //Edge contacts are more complex- we'll use:
            //(1<<6) + (0 or 1) * boxFaceNormalId + boxEdgeDirection * (1<<2) + (0 or 1) * boxEdgeCenterOffset * (1<<4)
            //The base 1<<6 distinguishes it from triangle vertex contacts.
            //For contacts created from the max endpoint of an edge interval, we'll add an extra (1<<7) to distinguish it from the min endpoint.
            var localXId = Vector<int>.Zero;
            var localYId = Vector<int>.One;
            var localZId = new Vector<int>(2);
            var axisIdTangentX = Vector.ConditionalSelect(useAX, localZId, localXId);
            var axisIdTangentY = Vector.ConditionalSelect(useAY, localZId, localYId);
            var axisIdNormal =
                Vector.ConditionalSelect(useAX, Vector.ConditionalSelect(
                    normalIsNegativeX, localXId, Vector<int>.Zero),
                Vector.ConditionalSelect(useAY, Vector.ConditionalSelect(
                    normalIsNegativeY, localYId, Vector<int>.Zero),
                Vector.ConditionalSelect(
                    normalIsNegativeZ, localZId, Vector<int>.Zero)));


            //Note that using a raw absolute epsilon would have a varying effect based on the scale of the involved shapes.
            //The minimum across the maxes is intended to avoid cases like a huge box being used as a plane, causing a massive size disparity.
            //Using its sizes as a threshold would tend to kill off perfectly valid contacts.
            Vector3Wide.LengthSquared(ab, out var abLengthSquared);
            Vector3Wide.LengthSquared(bc, out var bcLengthSquared);
            Vector3Wide.LengthSquared(ca, out var caLengthSquared);
            var epsilonScale = Vector.Min(
                Vector.Max(a.HalfWidth, Vector.Max(a.HalfHeight, a.HalfLength)),
                Vector.SquareRoot(Vector.Max(abLengthSquared, Vector.Max(bcLengthSquared, caLengthSquared))));

            //We will be working on the surface of the triangle, but we'd still like a 2d parameterization of the surface for contact reduction.
            //So, we'll create tangent axes from the edge and edge x normal.
            Vector3Wide.Scale(ab, Vector<float>.One / Vector.SquareRoot(abLengthSquared), out var triangleTangentX);
            Vector3Wide.CrossWithoutOverlap(triangleTangentX, triangleNormal, out var triangleTangentY);

            //Note that we only allocate up to 6 candidates. Each triangle edge can contribute at most two contacts (any more would require a nonconvex clip region).
            //Numerical issues can cause more to occur, but they're guarded against (both directly, and in the sense of checking count before adding any candidates beyond the sixth).
            int byteCount = Unsafe.SizeOf<ManifoldCandidate>() * 6;
            var buffer = stackalloc byte[byteCount];
            var candidateCount = Vector<int>.Zero;
            ref var candidates = ref Unsafe.As<byte, ManifoldCandidate>(ref *buffer);

            //While the edge clipping will find any triangleEdge-boxEdge or boxVertex--triangleFace contacts, it will not find triangleVertex-boxFace contacts.
            //Add them independently.
            //(Adding these firsts allows us to simply skip capacity tests, since there can only be a total of three triangle-boxface contacts.)
            Vector3Wide.Dot(boxFaceNormal, triangleNormal, out var boxFaceNormalDotTriangleNormal);
            var inverseBoxFaceNormalDotTriangleNormal = Vector.ConditionalSelect(
                Vector.LessThan(boxFaceNormalDotTriangleNormal, Vector<float>.Zero), negativeOne, Vector<float>.One) /
                Vector.Max(Vector.Abs(boxFaceNormalDotTriangleNormal), new Vector<float>(1e-15f));
            TryAddTriangleVertex(bA, Vector<int>.Zero, localTriangleCenter, triangleTangentX, triangleTangentY, triangleNormal, boxTangentX, boxTangentY, halfExtentX, halfExtentY, halfExtentZ, boxFaceNormal, inverseBoxFaceNormalDotTriangleNormal, ref candidates, ref candidateCount);
            TryAddTriangleVertex(bB, Vector<int>.One, localTriangleCenter, triangleTangentX, triangleTangentY, triangleNormal, boxTangentX, boxTangentY, halfExtentX, halfExtentY, halfExtentZ, boxFaceNormal, inverseBoxFaceNormalDotTriangleNormal, ref candidates, ref candidateCount);
            TryAddTriangleVertex(bC, new Vector<int>(2), localTriangleCenter, triangleTangentX, triangleTangentY, triangleNormal, boxTangentX, boxTangentY, halfExtentX, halfExtentY, halfExtentZ, boxFaceNormal, inverseBoxFaceNormalDotTriangleNormal, ref candidates, ref candidateCount);

            //Note that box edges will also add box vertices that are within the triangle bounds, so no box vertex case is required.
            //Note that each of these calls can generate 4 contacts, so we have to start checking capacities.
            //abxN = triangleTangentY * ||ab x N||. The magnitude of the value doesn't matter, so we just use the triangleTangentY directly for that edge plane normal.
            //Note that we flip the second X edge, and the first Y edge. That ensures winding so that the contacts generated at the max end of edge box edge don't end up in redundant spots.
            Vector3Wide.CrossWithoutOverlap(bc, triangleNormal, out var bcxN);
            Vector3Wide.CrossWithoutOverlap(ca, triangleNormal, out var caxN);
            ClipBoxEdgesAgainstTriangle(bA, bB, bC, triangleTangentY, bcxN, caxN,
                localTriangleCenter, triangleTangentX, triangleTangentY,
                boxTangentX, boxTangentY, halfExtentX, halfExtentY, boxFaceCenter,
                axisIdNormal, axisIdTangentX, axisIdTangentY,
                Vector<int>.Zero, new Vector<int>(-1), epsilonScale, ref candidates, ref candidateCount);
            ClipBoxEdgesAgainstTriangle(bA, bB, bC, triangleTangentY, bcxN, caxN,
                localTriangleCenter, triangleTangentX, triangleTangentY,
                boxTangentY, boxTangentX, halfExtentY, halfExtentX, boxFaceCenter,
                axisIdNormal, axisIdTangentY, axisIdTangentX,
                new Vector<int>(-1), Vector<int>.Zero, epsilonScale, ref candidates, ref candidateCount);

            Vector3Wide.Subtract(boxFaceCenter, localTriangleCenter, out var faceCenterBToFaceCenterA);
            ManifoldCandidateHelper.Reduce(ref candidates, candidateCount, 6, boxFaceNormal, localNormal, faceCenterBToFaceCenterA, triangleTangentX, triangleTangentY, epsilonScale,
                out var contact0, out var contact1, out var contact2, out var contact3,
                out manifold.Contact0Exists, out manifold.Contact1Exists, out manifold.Contact2Exists, out manifold.Contact3Exists);

            //Transform the contacts into the manifold.
            var minimumAcceptedDepth = -speculativeMargin;
            //Move the basis into world rotation so that we don't have to transform the individual contacts.
            Matrix3x3Wide.TransformWithoutOverlap(triangleTangentX, worldRA, out var worldTangentBX);
            Matrix3x3Wide.TransformWithoutOverlap(triangleTangentY, worldRA, out var worldTangentBY);
            Matrix3x3Wide.TransformWithoutOverlap(localTriangleCenter, worldRA, out var worldTriangleCenter);
            Matrix3x3Wide.TransformWithoutOverlap(localNormal, worldRA, out manifold.Normal);
            //If the local normal points against the triangle normal, then it's on the backside and should not collide.
            Vector3Wide.Dot(localNormal, triangleNormal, out var normalDot);
            var allowContacts = Vector.GreaterThanOrEqual(normalDot, Vector<float>.Zero);
            manifold.Contact0Exists = Vector.BitwiseAnd(manifold.Contact0Exists, allowContacts);
            manifold.Contact1Exists = Vector.BitwiseAnd(manifold.Contact1Exists, allowContacts);
            manifold.Contact2Exists = Vector.BitwiseAnd(manifold.Contact2Exists, allowContacts);
            manifold.Contact3Exists = Vector.BitwiseAnd(manifold.Contact3Exists, allowContacts);
            TransformContactToManifold(contact0, worldTriangleCenter, worldTangentBX, worldTangentBY, minimumAcceptedDepth, ref manifold.Contact0Exists, out manifold.OffsetA0, out manifold.Depth0, out manifold.FeatureId0);
            TransformContactToManifold(contact1, worldTriangleCenter, worldTangentBX, worldTangentBY, minimumAcceptedDepth, ref manifold.Contact1Exists, out manifold.OffsetA1, out manifold.Depth1, out manifold.FeatureId1);
            TransformContactToManifold(contact2, worldTriangleCenter, worldTangentBX, worldTangentBY, minimumAcceptedDepth, ref manifold.Contact2Exists, out manifold.OffsetA2, out manifold.Depth2, out manifold.FeatureId2);
            TransformContactToManifold(contact3, worldTriangleCenter, worldTangentBX, worldTangentBY, minimumAcceptedDepth, ref manifold.Contact3Exists, out manifold.OffsetA3, out manifold.Depth3, out manifold.FeatureId3);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void TransformContactToManifold(
            in ManifoldCandidate rawContact, in Vector3Wide faceCenterB, in Vector3Wide tangentBX, in Vector3Wide tangentBY, in Vector<float> minimumAcceptedDepth,
            ref Vector<int> contactExists, out Vector3Wide manifoldOffsetA, out Vector<float> manifoldDepth, out Vector<int> manifoldFeatureId)
        {
            Vector3Wide.Scale(tangentBX, rawContact.X, out manifoldOffsetA);
            Vector3Wide.Scale(tangentBY, rawContact.Y, out var y);
            Vector3Wide.Add(manifoldOffsetA, y, out manifoldOffsetA);
            Vector3Wide.Add(manifoldOffsetA, faceCenterB, out manifoldOffsetA);
            //Note that we delayed the speculative margin depth test until the end. This ensures area maximization has meaningful contacts to work with.
            //If we were more aggressive about the depth testing, the final manifold would tend to have more contacts, but less meaningful contacts.
            contactExists = Vector.BitwiseAnd(contactExists, Vector.GreaterThanOrEqual(rawContact.Depth, minimumAcceptedDepth));
            manifoldDepth = rawContact.Depth;
            manifoldFeatureId = rawContact.FeatureId;
        }

        public void Test(ref TriangleWide a, ref TriangleWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public void Test(ref TriangleWide a, ref TriangleWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }

    public class TrianglePairCollisionTask : CollisionTask
    {
        public TrianglePairCollisionTask()
        {
            BatchSize = 32;
            ShapeTypeIndexA = default(Box).TypeId;
            ShapeTypeIndexB = default(Triangle).TypeId;
        }


        //Every single collision task type will mirror this general layout.
        public unsafe override void ExecuteBatch<TCallbacks>(ref UntypedList batch, ref CollisionBatcher<TCallbacks> batcher)
        {
            ConvexCollisionTaskCommon.ExecuteBatch
                <TCallbacks,
                Triangle, TriangleWide, Triangle, TriangleWide, UnflippableTestPairWide<Triangle, TriangleWide, Triangle, TriangleWide>,
                Convex4ContactManifoldWide, TrianglePairTester>(ref batch, ref batcher);
        }
    }
}
