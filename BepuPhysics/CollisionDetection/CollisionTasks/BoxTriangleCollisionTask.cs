using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using static BepuUtilities.GatherScatter;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct BoxTriangleTester : IPairTester<BoxWide, TriangleWide, Convex4ContactManifoldWide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void GetDepthForInterval(in Vector<float> boxExtreme, in Vector<float> a, in Vector<float> b, in Vector<float> c, out Vector<float> depth)
        {
            var minB = Vector.Min(a, Vector.Min(b, c));
            var maxB = Vector.Max(a, Vector.Max(b, c));
            depth = Vector.Min(boxExtreme - minB, maxB + boxExtreme);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void TestBoxEdgeAgainstTriangleEdge(
            in Vector<float> triangleEdgeOffsetY, in Vector<float> triangleEdgeOffsetZ,
            in Vector<float> triangleCenterY, in Vector<float> triangleCenterZ,
            in Vector<float> edgeOffsetYSquared, in Vector<float> edgeOffsetZSquared,
            in Vector<float> halfHeight, in Vector<float> halfLength,
            in Vector3Wide vA, in Vector3Wide vB, in Vector3Wide vC,
            out Vector<float> depth, out Vector3Wide localNormal)
        {
            //We assume we're working with the box's X edge direction. Caller will swizzle.
            localNormal.X = Vector<float>.Zero;
            localNormal.Y = triangleEdgeOffsetZ;
            localNormal.Z = -triangleEdgeOffsetY;

            //Calibrate the normal to point from the triangle to the box while normalizing.
            var calibrationDot = triangleCenterY * localNormal.Y + triangleCenterZ * localNormal.Z;
            var length = Vector.SquareRoot(edgeOffsetYSquared + edgeOffsetZSquared);
            var inverseLength = Vector.ConditionalSelect(Vector.LessThan(calibrationDot, Vector<float>.Zero), Vector<float>.One, new Vector<float>(-1f)) / length;
            localNormal.Y *= inverseLength;
            localNormal.Z *= inverseLength;

            var extremeA = Vector.Abs(localNormal.Y) * halfHeight + Vector.Abs(localNormal.Z) * halfLength;
            var nVA = vA.Y * localNormal.Y + vA.Z * localNormal.Z;
            var nVB = vB.Y * localNormal.Y + vB.Z * localNormal.Z;
            var nVC = vC.Y * localNormal.Y + vC.Z * localNormal.Z;
            GetDepthForInterval(extremeA, nVA, nVB, nVC, out depth);
            depth = Vector.ConditionalSelect(Vector.LessThan(length, new Vector<float>(1e-7f)), new Vector<float>(float.MaxValue), depth);

        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void TestBoxEdgesAgainstTriangleEdge(
            in BoxWide a,
            in Vector3Wide triangleEdgeOffset, in Vector3Wide triangleCenter,
            in Vector3Wide vA, in Vector3Wide vB, in Vector3Wide vC,
            out Vector<float> depth, out Vector3Wide localNormal)
        {

            var x2 = triangleEdgeOffset.X * triangleEdgeOffset.X;
            var y2 = triangleEdgeOffset.Y * triangleEdgeOffset.Y;
            var z2 = triangleEdgeOffset.Z * triangleEdgeOffset.Z;
            //A.X x edgeB
            TestBoxEdgeAgainstTriangleEdge(triangleEdgeOffset.Y, triangleEdgeOffset.Z,
                triangleCenter.Y, triangleCenter.Z,
                y2, z2, a.HalfHeight, a.HalfLength,
                vA, vB, vC, out depth, out localNormal);

            //A.Y x edgeB
            TestBoxEdgeAgainstTriangleEdge(triangleEdgeOffset.X, triangleEdgeOffset.Z,
                triangleCenter.X, triangleCenter.Z,
                x2, z2, a.HalfWidth, a.HalfLength,
                vA, vB, vC, out var depthCandidate, out var localNormalCandidate);
            Vector3Wide.ConditionalSelect(Vector.LessThan(depthCandidate, depth), localNormalCandidate, localNormal, out localNormal);
            depth = Vector.Min(depth, depthCandidate);

            //A.Z x edgeB
            TestBoxEdgeAgainstTriangleEdge(triangleEdgeOffset.X, triangleEdgeOffset.Y,
                triangleCenter.X, triangleCenter.Y,
                x2, y2, a.HalfWidth, a.HalfHeight,
                vA, vB, vC, out depthCandidate, out localNormalCandidate);
            Vector3Wide.ConditionalSelect(Vector.LessThan(depthCandidate, depth), localNormalCandidate, localNormal, out localNormal);
            depth = Vector.Min(depth, depthCandidate);

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
        private void TryAddTriangleVertex(in Vector3Wide triangleVertex, in Vector<int> vertexId, in Vector3Wide triangleCenter, in Vector3Wide triangleX, in Vector3Wide triangleY,
            in Vector3Wide boxTangentX, in Vector3Wide boxTangentY, in Vector<float> halfExtentX, in Vector<float> halfExtentY, in Vector3Wide boxFaceCenter,
            ref ManifoldCandidate candidates, ref Vector<int> candidateCount)
        {
            Vector3Wide.Subtract(triangleVertex, boxFaceCenter, out var offset);
            Vector3Wide.Dot(boxTangentX, offset, out var boxX);
            Vector3Wide.Dot(boxTangentY, offset, out var boxY);
            var contained = Vector.BitwiseAnd(
                Vector.LessThanOrEqual(Vector.Abs(boxX), halfExtentX),
                Vector.LessThanOrEqual(Vector.Abs(boxY), halfExtentY));
            Add(triangleVertex, triangleCenter, triangleX, triangleY, vertexId, contained, ref candidates, ref candidateCount);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Test(
            ref BoxWide a, ref TriangleWide b, ref Vector<float> speculativeMargin,
            ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
            out Convex4ContactManifoldWide manifold)
        {
            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var worldRA);
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var worldRB);
            Matrix3x3Wide.MultiplyByTransposeWithoutOverlap(worldRB, worldRA, out var rB);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offsetB, worldRA, out var localOffsetB);
            Matrix3x3Wide.Transform(b.A, rB, out var vA);
            Vector3Wide.Add(vA, localOffsetB, out vA);
            Matrix3x3Wide.Transform(b.B, rB, out var vB);
            Vector3Wide.Add(vB, localOffsetB, out vB);
            Matrix3x3Wide.Transform(b.C, rB, out var vC);
            Vector3Wide.Add(vC, localOffsetB, out vC);

            Vector3Wide.Add(vA, vB, out var localTriangleCenter);
            Vector3Wide.Add(localTriangleCenter, vC, out localTriangleCenter);
            Vector3Wide.Scale(localTriangleCenter, new Vector<float>(1f / 3f), out localTriangleCenter);

            Vector3Wide.Subtract(vB, vA, out var ab);
            Vector3Wide.Subtract(vC, vB, out var bc);
            Vector3Wide.Subtract(vA, vC, out var ca);

            TestBoxEdgesAgainstTriangleEdge(a, ab, localTriangleCenter, vA, vB, vC, out var depth, out var localNormal);
            TestBoxEdgesAgainstTriangleEdge(a, bc, localTriangleCenter, vA, vB, vC, out var depthCandidate, out var localNormalCandidate);
            Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);
            TestBoxEdgesAgainstTriangleEdge(a, ca, localTriangleCenter, vA, vB, vC, out depthCandidate, out localNormalCandidate);
            Select(ref depth, ref localNormal, depthCandidate, localNormalCandidate);

            //Test face normals of A. Working in local space of A means potential axes are just (1,0,0) etc.
            var xNormalSign = Vector.ConditionalSelect(Vector.LessThan(localTriangleCenter.X, Vector<float>.Zero), Vector<float>.One, new Vector<float>(-1f));
            var yNormalSign = Vector.ConditionalSelect(Vector.LessThan(localTriangleCenter.Y, Vector<float>.Zero), Vector<float>.One, new Vector<float>(-1f));
            var zNormalSign = Vector.ConditionalSelect(Vector.LessThan(localTriangleCenter.Z, Vector<float>.Zero), Vector<float>.One, new Vector<float>(-1f));

            GetDepthForInterval(a.HalfWidth, xNormalSign * vA.X, xNormalSign * vB.X, xNormalSign * vC.X, out var faceAXDepth);
            GetDepthForInterval(a.HalfHeight, yNormalSign * vA.Y, yNormalSign * vB.Y, yNormalSign * vC.Y, out var faceAYDepth);
            GetDepthForInterval(a.HalfLength, zNormalSign * vA.Z, zNormalSign * vB.Z, zNormalSign * vC.Z, out var faceAZDepth);            
            Select(ref depth, ref localNormal, faceAXDepth, xNormalSign, Vector<float>.Zero, Vector<float>.Zero);
            Select(ref depth, ref localNormal, faceAYDepth, Vector<float>.Zero, yNormalSign, Vector<float>.Zero);
            Select(ref depth, ref localNormal, faceAZDepth, Vector<float>.Zero, Vector<float>.Zero, zNormalSign);

            //Test face normal of B.
            Vector3Wide.CrossWithoutOverlap(ab, ca, out var triangleNormal);
            Vector3Wide.Length(triangleNormal, out var triangleNormalLength);
            Vector3Wide.Scale(triangleNormal, Vector<float>.One / triangleNormalLength, out triangleNormal);
            //Note that we do not calibrate the triangle normal. The backside of the triangle does not collide, so any normal that ends up pointing that direction will be ignored anyway.
            Vector3Wide.Dot(triangleNormal, localTriangleCenter, out var trianglePlaneOffset);
            var triangleFaceDepth =
                Vector.Abs(triangleNormal.X) * a.HalfWidth + Vector.Abs(triangleNormal.Y) * a.HalfHeight + Vector.Abs(triangleNormal.Z) * a.HalfLength + trianglePlaneOffset;
            Select(ref depth, ref localNormal, triangleFaceDepth, triangleNormal);

            //At this point, we have computed the minimum depth and associated local normal.
            //We now need to compute some contact locations, their per-contact depths, and the feature ids.

            //Contact generation always assumes face-face clipping. Other forms of contact generation are just special cases of face-face, and since we pay
            //for all code paths, there's no point in handling them separately.
            //We just have to guarantee that the face chosen on the box is guaranteed to include the deepest feature along the contact normal.
            //To do this, choose the face on box A associated with the minimum axis dot with the collision normal. (The normal is calibrated to point triangle->box, hence minimum.)

            //We represent the chosen box face as its two tangent axes and the extent along those axes.
            var absNX = Vector.Abs(localNormal.X);
            var absNY = Vector.Abs(localNormal.Y);
            var absNZ = Vector.Abs(localNormal.Z);
            var xBiggerThanY = Vector.GreaterThan(absNX, absNY);
            var xBiggerThanZ = Vector.GreaterThan(absNX, absNZ);
            var yBiggerThanZ = Vector.GreaterThan(absNY, absNZ);
            var useAX = Vector.BitwiseAnd(xBiggerThanY, xBiggerThanZ);
            var useAY = Vector.AndNot(yBiggerThanZ, useAX);
            var useAZ = Vector.OnesComplement(Vector.BitwiseOr(useAX, useAY));

            var normalIsNegativeX = Vector.LessThan(localNormal.X, Vector<float>.Zero);
            var normalIsNegativeY = Vector.LessThan(localNormal.Y, Vector<float>.Zero);
            var normalIsNegativeZ = Vector.LessThan(localNormal.Z, Vector<float>.Zero);

            //If we're using face with normal X, the tangent axes are Z and Y
            //If we're using face with normal Y, the tangent axes are X and Z
            //If we're using face with normal Z, the tangent axes are X and Y
            Vector3Wide boxTangentX, boxTangentY, boxFaceNormal;
            boxTangentX.X = Vector.ConditionalSelect(Vector.BitwiseOr(useAY, useAZ), Vector<float>.One, Vector<float>.Zero);
            boxTangentX.Y = Vector<float>.Zero;
            boxTangentX.Z = Vector.ConditionalSelect(useAX, Vector<float>.One, Vector<float>.Zero);
            boxTangentY.X = Vector<float>.Zero;
            boxTangentY.Y = Vector.ConditionalSelect(Vector.BitwiseOr(useAX, useAZ), Vector<float>.One, Vector<float>.Zero);
            boxTangentY.Z = Vector.ConditionalSelect(useAY, Vector<float>.One, Vector<float>.Zero);
            var negativeOne = new Vector<float>(-1f);
            boxFaceNormal.X = Vector.ConditionalSelect(useAX, Vector.ConditionalSelect(normalIsNegativeX, Vector<float>.One, negativeOne), Vector<float>.Zero);
            boxFaceNormal.Y = Vector.ConditionalSelect(useAY, Vector.ConditionalSelect(normalIsNegativeY, Vector<float>.One, negativeOne), Vector<float>.Zero);
            boxFaceNormal.Z = Vector.ConditionalSelect(useAZ, Vector.ConditionalSelect(normalIsNegativeZ, Vector<float>.One, negativeOne), Vector<float>.Zero);

            var halfExtentX = Vector.ConditionalSelect(useAX, a.HalfLength, a.HalfWidth);
            var halfExtentY = Vector.ConditionalSelect(useAY, a.HalfLength, a.HalfHeight);
            var halfExtentZ = Vector.ConditionalSelect(useAX, a.HalfWidth, Vector.ConditionalSelect(useAY, a.HalfHeight, a.HalfLength));
            Vector3Wide.Scale(boxFaceNormal, halfExtentZ, out var boxFaceCenter);

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
            TryAddTriangleVertex(vA, Vector<int>.Zero, localTriangleCenter, triangleTangentX, triangleTangentY, boxTangentX, boxTangentY, halfExtentX, halfExtentY, boxFaceCenter, ref candidates, ref candidateCount);
            TryAddTriangleVertex(vB, Vector<int>.One, localTriangleCenter, triangleTangentX, triangleTangentY, boxTangentX, boxTangentY, halfExtentX, halfExtentY, boxFaceCenter, ref candidates, ref candidateCount);
            TryAddTriangleVertex(vC, new Vector<int>(2), localTriangleCenter, triangleTangentX, triangleTangentY, boxTangentX, boxTangentY, halfExtentX, halfExtentY, boxFaceCenter, ref candidates, ref candidateCount);

            //Note that box edges will also add box vertices that are within the triangle bounds, so no box vertex case is required.
            //Note that each of these calls can generate 4 contacts, so we have to start checking capacities.
            //abxN = triangleTangentY * ||ab x N||. The magnitude of the value doesn't matter, so we just use the triangleTangentY directly for that edge plane normal.
            //Note that we flip the second X edge, and the first Y edge. That ensures winding so that the contacts generated at the max end of edge box edge don't end up in redundant spots.
            Vector3Wide.CrossWithoutOverlap(bc, triangleNormal, out var bcxN);
            Vector3Wide.CrossWithoutOverlap(ca, triangleNormal, out var caxN);
            ClipBoxEdgesAgainstTriangle(vA, vB, vC, triangleTangentY, bcxN, caxN, 
                localTriangleCenter, triangleTangentX, triangleTangentY, 
                boxTangentX, boxTangentY, halfExtentX, halfExtentY, boxFaceCenter,
                axisIdNormal, axisIdTangentX, axisIdTangentY,
                Vector<int>.Zero, new Vector<int>(-1), epsilonScale, ref candidates, ref candidateCount);
            ClipBoxEdgesAgainstTriangle(vA, vB, vC, triangleTangentY, bcxN, caxN,
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

        public void Test(ref BoxWide a, ref TriangleWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public void Test(ref BoxWide a, ref TriangleWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }

    public class BoxTriangleCollisionTask : CollisionTask
    {
        public BoxTriangleCollisionTask()
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
                Box, BoxWide, Triangle, TriangleWide, TestPairWide<Box, BoxWide, Triangle, TriangleWide>,
                Convex4ContactManifoldWide, BoxTriangleTester>(ref batch, ref batcher);
        }
    }
}
