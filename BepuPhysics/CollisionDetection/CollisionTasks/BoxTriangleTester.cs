using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public struct BoxTriangleTester : IPairTester<BoxWide, TriangleWide, Convex4ContactManifoldWide>
    {
        public int BatchSize => 32;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void GetDepthForInterval(in Vector<float> boxExtreme, in Vector<float> a, in Vector<float> b, in Vector<float> c, out Vector<float> depth)
        {
            var minB = Vector.Min(a, Vector.Min(b, c));
            var maxB = Vector.Max(a, Vector.Max(b, c));
            depth = Vector.Min(boxExtreme - minB, maxB + boxExtreme);
        }
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void TestBoxEdgeAgainstTriangleEdge(
            in Vector<float> triangleEdgeOffsetY, in Vector<float> triangleEdgeOffsetZ,
            in Vector<float> triangleCenterY, in Vector<float> triangleCenterZ,
            in Vector<float> edgeOffsetYSquared, in Vector<float> edgeOffsetZSquared,
            in Vector<float> halfHeight, in Vector<float> halfLength,
            in Vector<float> vAY, in Vector<float> vAZ, in Vector<float> vBY, in Vector<float> vBZ, in Vector<float> vCY, in Vector<float> vCZ,
            out Vector<float> depth, out Vector<float> localNormalX, out Vector<float> localNormalY, out Vector<float> localNormalZ)
        {
            //We assume we're working with the box's X edge direction. Caller will swizzle.
            localNormalX = Vector<float>.Zero;
            localNormalY = triangleEdgeOffsetZ;
            localNormalZ = -triangleEdgeOffsetY;

            //Calibrate the normal to point from the triangle to the box while normalizing.
            var calibrationDot = triangleCenterY * localNormalY + triangleCenterZ * localNormalZ;
            var length = Vector.SquareRoot(edgeOffsetYSquared + edgeOffsetZSquared);
            var inverseLength = Vector.ConditionalSelect(Vector.LessThan(calibrationDot, Vector<float>.Zero), Vector<float>.One, new Vector<float>(-1f)) / length;
            localNormalY *= inverseLength;
            localNormalZ *= inverseLength;

            var extremeA = Vector.Abs(localNormalY) * halfHeight + Vector.Abs(localNormalZ) * halfLength;
            var nVA = vAY * localNormalY + vAZ * localNormalZ;
            var nVB = vBY * localNormalY + vBZ * localNormalZ;
            var nVC = vCY * localNormalY + vCZ * localNormalZ;
            GetDepthForInterval(extremeA, nVA, nVB, nVC, out depth);
            depth = Vector.ConditionalSelect(Vector.LessThan(length, new Vector<float>(1e-7f)), new Vector<float>(float.MaxValue), depth);

        }
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
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
                vA.Y, vA.Z, vB.Y, vB.Z, vC.Y, vC.Z,
                out depth, out localNormal.X, out localNormal.Y, out localNormal.Z);

            //A.Y x edgeB
            Vector3Wide localNormalCandidate;
            TestBoxEdgeAgainstTriangleEdge(triangleEdgeOffset.X, triangleEdgeOffset.Z,
                triangleCenter.X, triangleCenter.Z,
                x2, z2, a.HalfWidth, a.HalfLength,
                vA.X, vA.Z, vB.X, vB.Z, vC.X, vC.Z,
                out var depthCandidate, out localNormalCandidate.Y, out localNormalCandidate.X, out localNormalCandidate.Z);
            Vector3Wide.ConditionalSelect(Vector.LessThan(depthCandidate, depth), localNormalCandidate, localNormal, out localNormal);
            depth = Vector.Min(depth, depthCandidate);

            //A.Z x edgeB
            TestBoxEdgeAgainstTriangleEdge(triangleEdgeOffset.X, triangleEdgeOffset.Y,
                triangleCenter.X, triangleCenter.Y,
                x2, y2, a.HalfWidth, a.HalfHeight,
                vA.X, vA.Y, vB.X, vB.Y, vC.X, vC.Y,
                out depthCandidate, out localNormalCandidate.Z, out localNormalCandidate.X, out localNormalCandidate.Y);
            Vector3Wide.ConditionalSelect(Vector.LessThan(depthCandidate, depth), localNormalCandidate, localNormal, out localNormal);
            depth = Vector.Min(depth, depthCandidate);

        }
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Select(
            ref Vector<float> depth, ref Vector3Wide normal,
            in Vector<float> depthCandidate, in Vector3Wide normalCandidate)
        {
            var useCandidate = Vector.LessThan(depthCandidate, depth);
            Vector3Wide.ConditionalSelect(useCandidate, normalCandidate, normal, out normal);
            depth = Vector.Min(depth, depthCandidate);
        }
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
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


        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Add(in Vector3Wide pointOnTriangle, in Vector3Wide triangleCenter, in Vector3Wide triangleTangentX, in Vector3Wide triangleTangentY, in Vector<int> featureId,
            in Vector<int> exists, ref ManifoldCandidate candidates, ref Vector<int> candidateCount, int pairCount)
        {
            Vector3Wide.Subtract(pointOnTriangle, triangleCenter, out var offset);
            ManifoldCandidate candidate;
            Vector3Wide.Dot(offset, triangleTangentX, out candidate.X);
            Vector3Wide.Dot(offset, triangleTangentY, out candidate.Y);
            candidate.FeatureId = featureId;
            Debug.Assert(Vector.EqualsAll(Vector.BitwiseOr(Vector.OnesComplement(exists), Vector.LessThan(candidateCount, new Vector<int>(6))), new Vector<int>(-1)), "Can't add more than 6 contacts to any candidate manifold.");
            ManifoldCandidateHelper.AddCandidate(ref candidates, ref candidateCount, candidate, exists, pairCount);
        }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ClipTriangleEdgeAgainstPlanes(in Vector3Wide edgeDirection, in Vector3Wide triangleEdgeStartToBoxEdgeAnchor0, in Vector3Wide triangleEdgeStartToBoxEdgeAnchor1,
            in Vector3Wide boxEdgePlaneNormal, out Vector<float> min, out Vector<float> max)
        {
            Vector3Wide.Dot(triangleEdgeStartToBoxEdgeAnchor0, boxEdgePlaneNormal, out var distance0);
            Vector3Wide.Dot(triangleEdgeStartToBoxEdgeAnchor1, boxEdgePlaneNormal, out var distance1);
            Vector3Wide.Dot(boxEdgePlaneNormal, edgeDirection, out var velocity);
            var inverseVelocity = Vector<float>.One / velocity;

            //If the distances to the planes have opposing signs, then the start must be between the two.
            var edgeStartIsInside = Vector.LessThanOrEqual(distance0 * distance1, Vector<float>.Zero);
            var dontUseFallback = Vector.GreaterThan(Vector.Abs(velocity), new Vector<float>(1e-15f));
            var t0 = distance0 * inverseVelocity;
            var t1 = distance1 * inverseVelocity;
            //If the edge direction and plane surface is parallel, then the interval is defined entirely by whether the edge starts inside or outside.
            //If it's inside, then it's -inf to inf. If it's outside, then there is no overlap and we'll use inf to -inf.
            var largeNegative = new Vector<float>(-float.MaxValue);
            var largePositive = new Vector<float>(float.MaxValue);
            min = Vector.ConditionalSelect(dontUseFallback, Vector.Min(t0, t1), Vector.ConditionalSelect(edgeStartIsInside, largeNegative, largePositive));
            max = Vector.ConditionalSelect(dontUseFallback, Vector.Max(t0, t1), Vector.ConditionalSelect(edgeStartIsInside, largePositive, largeNegative));
        }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ClipTriangleEdgeAgainstBoxFace(in Vector3Wide edgeStart, in Vector3Wide edgeDirection, in Vector<int> edgeId,
            in Vector3Wide boxVertex00, in Vector3Wide boxVertex11, in Vector3Wide edgePlaneNormalX, in Vector3Wide edgePlaneNormalY,
            in Vector3Wide triangleCenter, in Vector3Wide triangleTangentX, in Vector3Wide triangleTangentY,
            in Vector<int> allowContacts, ref ManifoldCandidate candidates, ref Vector<int> candidateCount, int pairCount)
        {
            Vector3Wide.Subtract(boxVertex00, edgeStart, out var triangleEdgeStartToV00);
            Vector3Wide.Subtract(boxVertex11, edgeStart, out var triangleEdgeStartToV11);
            ClipTriangleEdgeAgainstPlanes(edgeDirection, triangleEdgeStartToV00, triangleEdgeStartToV11, edgePlaneNormalX, out var minX, out var maxX);
            ClipTriangleEdgeAgainstPlanes(edgeDirection, triangleEdgeStartToV00, triangleEdgeStartToV11, edgePlaneNormalY, out var minY, out var maxY);
            var min = Vector.Max(minX, minY);
            var max = Vector.Min(Vector<float>.One, Vector.Min(maxX, maxY));
            Vector3Wide.Scale(edgeDirection, min, out var minLocation);
            Vector3Wide.Scale(edgeDirection, max, out var maxLocation);
            Vector3Wide.Add(edgeStart, minLocation, out minLocation);
            Vector3Wide.Add(edgeStart, maxLocation, out maxLocation);

            //We now have intervals for both box edges.
            //If 0<min<1 && (max-min)>epsilon for an edge, use the min intersection as a contact.
            //If 0<=max<=1 && max>=min, use the max intersection as a contact.
            //Note the comparisons: if the max lies on a face vertex, it is used, but if the min lies on a face vertex, it is not. This avoids redundant entries.
            //Note that the epsilon is a fixed value. That's because the max and min values are in terms of the edge direction length, so it already compensates for shape size.
            var six = new Vector<int>(6);
            var minExists = Vector.BitwiseAnd(
                Vector.BitwiseAnd(
                    Vector.BitwiseAnd(
                        allowContacts,
                        Vector.LessThan(candidateCount, six)),
                    Vector.GreaterThanOrEqual(max - min, new Vector<float>(1e-5f))),
                Vector.BitwiseAnd(
                    Vector.LessThan(min, Vector<float>.One),
                    Vector.GreaterThan(min, Vector<float>.Zero)));
            Add(minLocation, triangleCenter, triangleTangentX, triangleTangentY, edgeId, minExists, ref candidates, ref candidateCount, pairCount);

            var maxExists = Vector.BitwiseAnd(
                Vector.BitwiseAnd(
                    Vector.BitwiseAnd(
                        allowContacts,
                        Vector.LessThan(candidateCount, six)),
                    Vector.GreaterThanOrEqual(max, min)),
                Vector.BitwiseAnd(
                    Vector.LessThanOrEqual(max, Vector<float>.One),
                    Vector.GreaterThanOrEqual(max, Vector<float>.Zero)));
            Add(maxLocation, triangleCenter, triangleTangentX, triangleTangentY, edgeId + new Vector<int>(8), maxExists, ref candidates, ref candidateCount, pairCount);
        }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ClipTriangleEdgesAgainstBoxFace(
            in Vector3Wide a, in Vector3Wide b, in Vector3Wide c, in Vector3Wide triangleCenter, in Vector3Wide triangleTangentX, in Vector3Wide triangleTangentY,
            in Vector3Wide ab, in Vector3Wide bc, in Vector3Wide ca,
            in Vector3Wide boxVertex00, in Vector3Wide boxVertex11, in Vector3Wide boxTangentX, in Vector3Wide boxTangentY, in Vector3Wide contactNormal,
            in Vector<int> allowContacts, ref ManifoldCandidate candidates, ref Vector<int> candidateCount, int pairCount)
        {
            //The critical observation here is that we are working in a contact plane defined by the contact normal- not the triangle face normal or the box face normal.
            //So, when performing clipping, we actually want to clip on the contact normal plane.
            //This is consistent with the box vertex test where we cast a ray from the box vertex to triangle along the contact normal.

            //This is almost identical to testing against the unmodified box face, but we need to change the face edge plane normals.
            //Rather than being the simple tangent axes, we must compute the plane normals such that the plane embeds the box edge while being perpendicular to the contact normal.
            Vector3Wide.CrossWithoutOverlap(boxTangentY, contactNormal, out var edgePlaneNormalX);
            Vector3Wide.CrossWithoutOverlap(boxTangentX, contactNormal, out var edgePlaneNormalY);

            //Edge feature ids:
            //4 + [0, 1, 2], depending on which triangle edge is used.
            //The extra 4 distinguishes it from box vertex contacts.
            //For contacts created from the max endpoint of an edge interval, we'll add an extra 8 to distinguish it from the min endpoint.

            //Note that winding is important- need to follow the edge vectors consistently so that the max of one edge butts up against the min of the next edge.
            //We assume that when deciding what contacts to create for each edge (only max endpoints generate contacts when unbounded by a box face).
            var baseId = new Vector<int>(4);
            ClipTriangleEdgeAgainstBoxFace(a, ab, baseId, boxVertex00, boxVertex11, edgePlaneNormalX, edgePlaneNormalY, triangleCenter, triangleTangentX, triangleTangentY, allowContacts, ref candidates, ref candidateCount, pairCount);
            ClipTriangleEdgeAgainstBoxFace(b, bc, baseId + Vector<int>.One, boxVertex00, boxVertex11, edgePlaneNormalX, edgePlaneNormalY, triangleCenter, triangleTangentX, triangleTangentY, allowContacts, ref candidates, ref candidateCount, pairCount);
            ClipTriangleEdgeAgainstBoxFace(c, ca, baseId + new Vector<int>(2), boxVertex00, boxVertex11, edgePlaneNormalX, edgePlaneNormalY, triangleCenter, triangleTangentX, triangleTangentY, allowContacts, ref candidates, ref candidateCount, pairCount);
        }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void AddBoxVertex(in Vector3Wide a, in Vector3Wide b, in Vector3Wide v, in Vector3Wide triangleNormal, in Vector3Wide contactNormal, in Vector<float> inverseNormalDot,
            in Vector3Wide abEdgePlaneNormal, in Vector3Wide bcEdgePlaneNormal, in Vector3Wide caEdgePlaneNormal,
            in Vector3Wide triangleCenter, in Vector3Wide triangleX, in Vector3Wide triangleY, in Vector<int> featureId,
            in Vector<int> allowContacts, ref ManifoldCandidate candidates, ref Vector<int> candidateCount, int pairCount)
        {
            //TODO: There are some other ways to express this. For example, containment testing using barycentric coordinates, or using edge plane normals based on the contact normal.
            //This is just a very direct implementation- some value in simplicity.

            //Cast a ray from the box vertex down to the triangle along the contact normal.
            Vector3Wide.Subtract(v, a, out var pointOnTriangleToBoxVertex);
            Vector3Wide.Dot(triangleNormal, pointOnTriangleToBoxVertex, out var planeDistance);
            Vector3Wide.Scale(contactNormal, planeDistance * inverseNormalDot, out var offset);
            //Contact normal points from triangle to box by convention, so we have to subtract.
            Vector3Wide.Subtract(v, offset, out var vOnPlane);

            //Test the unprojected location against the edge normals.
            Vector3Wide.Subtract(vOnPlane, a, out var aToV);
            Vector3Wide.Subtract(vOnPlane, b, out var bToV);
            Vector3Wide.Dot(aToV, abEdgePlaneNormal, out var abDot);
            Vector3Wide.Dot(bToV, bcEdgePlaneNormal, out var bcDot);
            Vector3Wide.Dot(aToV, caEdgePlaneNormal, out var caDot);

            //Note that plane normals are assumed to point inward.
            var contained = Vector.BitwiseAnd(
                Vector.BitwiseAnd(
                    allowContacts,
                    Vector.GreaterThanOrEqual(abDot, Vector<float>.Zero)),
                Vector.BitwiseAnd(
                    Vector.GreaterThanOrEqual(bcDot, Vector<float>.Zero),
                    Vector.GreaterThanOrEqual(caDot, Vector<float>.Zero)));

            Add(vOnPlane, triangleCenter, triangleX, triangleY, featureId, contained, ref candidates, ref candidateCount, pairCount);
        }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void AddBoxVertices(in Vector3Wide a, in Vector3Wide b, in Vector3Wide ab, in Vector3Wide bc, in Vector3Wide ca, in Vector3Wide triangleNormal, in Vector3Wide contactNormal,
            in Vector3Wide v00, in Vector3Wide v01, in Vector3Wide v10, in Vector3Wide v11,
            in Vector3Wide triangleCenter, in Vector3Wide triangleX, in Vector3Wide triangleY, in Vector<int> baseFeatureId, in Vector<int> featureIdX, in Vector<int> featureIdY,
            in Vector<int> allowContacts, ref ManifoldCandidate candidates, ref Vector<int> candidateCount, int pairCount)
        {
            Vector3Wide.Dot(triangleNormal, contactNormal, out var normalDot);
            //Note that we don't worry about cases where the triangle normal faces away from the contact normal- those cases don't generate contacts anyway.
            var inverseNormalDot = Vector.ConditionalSelect(Vector.GreaterThan(Vector.Abs(normalDot), new Vector<float>(1e-10f)), Vector<float>.One / normalDot, new Vector<float>(float.MaxValue));

            //Edge plane normals point inward for these tests.
            Vector3Wide.CrossWithoutOverlap(ab, triangleNormal, out var abEdgePlaneNormal);
            Vector3Wide.CrossWithoutOverlap(bc, triangleNormal, out var bcEdgePlaneNormal);
            Vector3Wide.CrossWithoutOverlap(ca, triangleNormal, out var caEdgePlaneNormal);

            AddBoxVertex(a, b, v00, triangleNormal, contactNormal, inverseNormalDot, abEdgePlaneNormal, bcEdgePlaneNormal, caEdgePlaneNormal,
                triangleCenter, triangleX, triangleY, baseFeatureId, allowContacts, ref candidates, ref candidateCount, pairCount);
            AddBoxVertex(a, b, v01, triangleNormal, contactNormal, inverseNormalDot, abEdgePlaneNormal, bcEdgePlaneNormal, caEdgePlaneNormal,
                triangleCenter, triangleX, triangleY, baseFeatureId + featureIdY, allowContacts, ref candidates, ref candidateCount, pairCount);
            AddBoxVertex(a, b, v10, triangleNormal, contactNormal, inverseNormalDot, abEdgePlaneNormal, bcEdgePlaneNormal, caEdgePlaneNormal,
                triangleCenter, triangleX, triangleY, baseFeatureId + featureIdX, allowContacts, ref candidates, ref candidateCount, pairCount);
            AddBoxVertex(a, b, v11, triangleNormal, contactNormal, inverseNormalDot, abEdgePlaneNormal, bcEdgePlaneNormal, caEdgePlaneNormal,
                triangleCenter, triangleX, triangleY, baseFeatureId + featureIdX + featureIdY, allowContacts, ref candidates, ref candidateCount, pairCount);
        }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Test(
            ref BoxWide a, ref TriangleWide b, ref Vector<float> speculativeMargin,
            ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, int pairCount,
            out Convex4ContactManifoldWide manifold)
        {
            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var worldRA);
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var worldRB);
            Matrix3x3Wide.MultiplyByTransposeWithoutOverlap(worldRB, worldRA, out var rB);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offsetB, worldRA, out var localOffsetB);
            Matrix3x3Wide.TransformWithoutOverlap(b.A, rB, out var vA);
            Vector3Wide.Add(vA, localOffsetB, out vA);
            Matrix3x3Wide.TransformWithoutOverlap(b.B, rB, out var vB);
            Vector3Wide.Add(vB, localOffsetB, out vB);
            Matrix3x3Wide.TransformWithoutOverlap(b.C, rB, out var vC);
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

            //Note that we ignore sign on interval tests here- the depth is the same regardless of what the sign is, so we don't have to take it into account.
            GetDepthForInterval(a.HalfWidth, vA.X, vB.X, vC.X, out var faceAXDepth);
            GetDepthForInterval(a.HalfHeight, vA.Y, vB.Y, vC.Y, out var faceAYDepth);
            GetDepthForInterval(a.HalfLength, vA.Z, vB.Z, vC.Z, out var faceAZDepth);
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

            //For feature ids, we will use box vertex index for triangle face contacts: (x > 0 ? 1 : 0) + (y > 0 ? 2 : 0) + (z > 0 ? 4 : 0)
            //Edge contacts are more complex- we'll use:
            //4 + triangleEdgeId + (0 if min contact, 1 if max contact) * 8
            //The base 4 distinguishes it from box vertex contacts.
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

            //While the edge clipping will find any triangleEdge-boxEdge or triangleVertex-boxFace contacts, it will not find boxVertex-triangleFace contacts.
            //Add them independently.
            //(Adding these first allows us to simply skip capacity tests, since there can only be a total of three triangle-boxface contacts.)            
            Vector3Wide.Scale(boxTangentX, halfExtentX, out var boxEdgeOffsetX);
            Vector3Wide.Scale(boxTangentY, halfExtentY, out var boxEdgeOffsetY);
            Vector3Wide.Add(boxFaceCenter, boxEdgeOffsetX, out var positiveX);
            Vector3Wide.Subtract(boxFaceCenter, boxEdgeOffsetX, out var negativeX);
            Vector3Wide.Subtract(negativeX, boxEdgeOffsetY, out var boxVertex00);
            Vector3Wide.Add(negativeX, boxEdgeOffsetY, out var boxVertex01);
            Vector3Wide.Subtract(positiveX, boxEdgeOffsetY, out var boxVertex10);
            Vector3Wide.Add(positiveX, boxEdgeOffsetY, out var boxVertex11);
            //If the local normal points against the triangle normal, then it's on the backside and should not collide.
            Vector3Wide.Dot(localNormal, triangleNormal, out var normalDot);
            var allowContacts = Vector.GreaterThan(normalDot, Vector<float>.Zero);
            AddBoxVertices(vA, vB, ab, bc, ca, triangleNormal, localNormal, boxVertex00, boxVertex01, boxVertex10, boxVertex11,
                localTriangleCenter, triangleTangentX, triangleTangentY, axisIdNormal, axisIdTangentX, axisIdTangentY, allowContacts, ref candidates, ref candidateCount, pairCount);

            //Note that triangle edges will also add triangle vertices that are within the box's bounds, so no triangle vertex case is required.
            //Note that these functions will have to check capacity since we might have added up to four contacts in the box vertex pass.
            ClipTriangleEdgesAgainstBoxFace(vA, vB, vC, localTriangleCenter, triangleTangentX, triangleTangentY, ab, bc, ca,
                boxVertex00, boxVertex11, boxTangentX, boxTangentY, localNormal, allowContacts, ref candidates, ref candidateCount, pairCount);

            Vector3Wide.Subtract(boxFaceCenter, localTriangleCenter, out var faceCenterBToFaceCenterA);
            ManifoldCandidateHelper.Reduce(ref candidates, candidateCount, 6, boxFaceNormal, localNormal, faceCenterBToFaceCenterA, triangleTangentX, triangleTangentY, epsilonScale, -speculativeMargin, pairCount,
                out var contact0, out var contact1, out var contact2, out var contact3,
                out manifold.Contact0Exists, out manifold.Contact1Exists, out manifold.Contact2Exists, out manifold.Contact3Exists);

            //Transform the contacts into the manifold.
            //Move the basis into world rotation so that we don't have to transform the individual contacts.
            Matrix3x3Wide.TransformWithoutOverlap(triangleTangentX, worldRA, out var worldTangentBX);
            Matrix3x3Wide.TransformWithoutOverlap(triangleTangentY, worldRA, out var worldTangentBY);
            Matrix3x3Wide.TransformWithoutOverlap(localTriangleCenter, worldRA, out var worldTriangleCenter);
            Matrix3x3Wide.TransformWithoutOverlap(localNormal, worldRA, out manifold.Normal);
            TransformContactToManifold(contact0, worldTriangleCenter, worldTangentBX, worldTangentBY, out manifold.OffsetA0, out manifold.Depth0, out manifold.FeatureId0);
            TransformContactToManifold(contact1, worldTriangleCenter, worldTangentBX, worldTangentBY, out manifold.OffsetA1, out manifold.Depth1, out manifold.FeatureId1);
            TransformContactToManifold(contact2, worldTriangleCenter, worldTangentBX, worldTangentBY, out manifold.OffsetA2, out manifold.Depth2, out manifold.FeatureId2);
            TransformContactToManifold(contact3, worldTriangleCenter, worldTangentBX, worldTangentBY, out manifold.OffsetA3, out manifold.Depth3, out manifold.FeatureId3);
            //We mark the manifold as being generated by the triangle face if the normal is close enough. Note that this is better than using a thresholded depth-
            //if all depths are similar, the triangle depth could be similar to an edge depth with a normal that is 90 degrees away from the triangle normal.
            //Given that this is used as input to mesh boundary smoothing, that's counterproductive.
            var faceFlag = Vector.ConditionalSelect(
                Vector.GreaterThanOrEqual(normalDot, new Vector<float>(MeshReduction.MinimumDotForFaceCollision)), new Vector<int>(MeshReduction.FaceCollisionFlag), Vector<int>.Zero);
            manifold.FeatureId0 += faceFlag;
        }


        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void TransformContactToManifold(
            in ManifoldCandidate rawContact, in Vector3Wide faceCenterB, in Vector3Wide tangentBX, in Vector3Wide tangentBY,
            out Vector3Wide manifoldOffsetA, out Vector<float> manifoldDepth, out Vector<int> manifoldFeatureId)
        {
            Vector3Wide.Scale(tangentBX, rawContact.X, out manifoldOffsetA);
            Vector3Wide.Scale(tangentBY, rawContact.Y, out var y);
            Vector3Wide.Add(manifoldOffsetA, y, out manifoldOffsetA);
            Vector3Wide.Add(manifoldOffsetA, faceCenterB, out manifoldOffsetA);
            manifoldDepth = rawContact.Depth;
            manifoldFeatureId = rawContact.FeatureId;
        }

        public void Test(ref BoxWide a, ref TriangleWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        public void Test(ref BoxWide a, ref TriangleWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out Convex4ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }
}
