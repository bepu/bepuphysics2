using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    //Individual pair testers are designed to be used outside of the narrow phase. They need to be usable for queries and such, so all necessary data must be gathered externally.
    public struct SphereTriangleTester : IPairTester<SphereWide, TriangleWide, Convex1ContactManifoldWide>
    {
        public int BatchSize => 32;

        public void Test(ref SphereWide a, ref TriangleWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
            int pairCount, out Convex1ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Select(ref Vector<float> distanceSquared, ref Vector3Wide localNormal, ref Vector<float> distanceSquaredCandidate, ref Vector3Wide localNormalCandidate)
        {
            var useCandidate = Vector.LessThan(distanceSquaredCandidate, distanceSquared);
            distanceSquared = Vector.Min(distanceSquaredCandidate, distanceSquared);
            Vector3Wide.ConditionalSelect(useCandidate, localNormalCandidate, localNormal, out localNormal);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Test(ref SphereWide a, ref TriangleWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, ref QuaternionWide orientationB, int pairCount,
            out Convex1ContactManifoldWide manifold)
        {
            Unsafe.SkipInit(out manifold);
            //Work in the local space of the triangle, since it's quicker to transform the sphere position than the vertices of the triangle.
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var rB);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offsetB, rB, out var localOffsetB);


            Vector3Wide.Subtract(b.B, b.A, out var ab);
            Vector3Wide.Subtract(b.C, b.A, out var ac);
            //localOffsetA = -localOffsetB, so pa = triangle.A + localOffsetB.
            Vector3Wide.Add(b.A, localOffsetB, out var pa);
            Vector3Wide.CrossWithoutOverlap(ab, ac, out var localTriangleNormal);
            Vector3Wide.Length(localTriangleNormal, out var triangleNormalLength);
            var inverseTriangleNormalLength = Vector<float>.One / triangleNormalLength;
            Vector3Wide.Scale(localTriangleNormal, inverseTriangleNormalLength, out localTriangleNormal);

            //EdgeAB plane test: (pa x ab) * (ab x ac) >= 0
            //EdgeAC plane test: (ac x pa) * (ab x ac) >= 0
            //Note that these are scaled versions of the barycentric coordinates. 
            //To normalize them such that the weights of a point within the triangle equal 1, we just need to divide by dot(ab x ac, ab x ac).
            //In other words, to test the third edge plane, we can ensure that the unnormalized weights are both positive and sum to a value less than dot(ab x ac, ab x ac).
            //If a point is outside of an edge plane, we know that it's not in the face region or any other edge region. It could, however, be in an adjacent vertex region.
            //Vertex cases can be handled by clamping an edge case. 
            //Further, note that for any query location, it is sufficient to only test one edge even if the point is outside two edge planes. If it's outside two edge planes,
            //that just means it's going to be on the shared vertex, so a clamped edge test captures the correct closest point.
            //So, at each edge, if the point is outside the plane, cache the edge. The last edge registering an outside result will be tested.
            Vector3Wide.CrossWithoutOverlap(pa, ab, out var paxab);
            Vector3Wide.CrossWithoutOverlap(ac, pa, out var acxpa);
            Vector3Wide.Dot(paxab, localTriangleNormal, out var edgePlaneTestAB);
            Vector3Wide.Dot(acxpa, localTriangleNormal, out var edgePlaneTestAC);
            var edgePlaneTestBC = Vector<float>.One - (edgePlaneTestAB + edgePlaneTestAC) * inverseTriangleNormalLength;

            var outsideAB = Vector.LessThan(edgePlaneTestAB, Vector<float>.Zero);
            var outsideAC = Vector.LessThan(edgePlaneTestAC, Vector<float>.Zero);
            var outsideBC = Vector.LessThan(edgePlaneTestBC, Vector<float>.Zero);

            var outsideAnyEdge = Vector.BitwiseOr(outsideAB, Vector.BitwiseOr(outsideAC, outsideBC));
            Unsafe.SkipInit(out Vector3Wide localClosestOnTriangle);
            var negativeOne = new Vector<int>(-1);
            var activeLanes = BundleIndexing.CreateMaskForCountInBundle(pairCount);
            if (Vector.EqualsAny(Vector.BitwiseAnd(activeLanes, outsideAnyEdge), negativeOne))
            {
                //At least one lane detected a point outside of the triangle. Choose one edge which is outside as the representative.
                Vector3Wide.ConditionalSelect(outsideAC, ac, ab, out var edgeDirection);
                Vector3Wide.Subtract(b.C, b.B, out var bc);
                Vector3Wide.ConditionalSelect(outsideBC, bc, edgeDirection, out edgeDirection);
                Vector3Wide.ConditionalSelect(outsideBC, b.B, b.A, out var edgeStart);

                Vector3Wide.Add(localOffsetB, edgeStart, out var negativeEdgeStartToP);
                //This does some partially redundant work if the edge is AB or AC, but given that we didn't have bcbc or bcpb, it's fine.
                Vector3Wide.Dot(negativeEdgeStartToP, edgeDirection, out var negativeOffsetDotEdge);
                Vector3Wide.Dot(edgeDirection, edgeDirection, out var edgeDotEdge);
                var edgeScale = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, -negativeOffsetDotEdge / edgeDotEdge));
                Vector3Wide.Scale(edgeDirection, edgeScale, out var pointOnEdge);
                Vector3Wide.Add(edgeStart, pointOnEdge, out pointOnEdge);

                Vector3Wide.ConditionalSelect(outsideAnyEdge, pointOnEdge, localClosestOnTriangle, out localClosestOnTriangle);

            }
            if (Vector.EqualsAny(Vector.AndNot(activeLanes, outsideAnyEdge), negativeOne))
            {
                //p + N * (pa * N) / ||N||^2 = N * (pa * N) / ||N||^2 - (-p)
                Vector3Wide.Dot(localTriangleNormal, pa, out var paN);
                Vector3Wide.Scale(localTriangleNormal, paN, out var offsetToPlane);
                Vector3Wide.Subtract(offsetToPlane, localOffsetB, out var pointOnFace);

                Vector3Wide.ConditionalSelect(outsideAnyEdge, localClosestOnTriangle, pointOnFace, out localClosestOnTriangle);
            }

            manifold.FeatureId = Vector.ConditionalSelect(outsideAnyEdge, Vector<int>.Zero, new Vector<int>(MeshReduction.FaceCollisionFlag));

            //We'll be using the contact position to perform boundary smoothing; in order to find other triangles, the contact position has to be on the mesh surface.
            Matrix3x3Wide.TransformWithoutOverlap(localClosestOnTriangle, rB, out manifold.OffsetA);
            Vector3Wide.Add(manifold.OffsetA, offsetB, out manifold.OffsetA);
            Vector3Wide.Length(manifold.OffsetA, out var distance);
            //Note the normal is calibrated to point from B to A.
            var normalScale = new Vector<float>(-1) / distance;
            Vector3Wide.Scale(manifold.OffsetA, normalScale, out manifold.Normal);
            manifold.Depth = a.Radius - distance;
            //In the event that the sphere's center point is touching the triangle, the normal is undefined. In that case, the 'correct' normal would be the triangle's normal.
            //However, given that this is a pretty rare degenerate case and that we already treat triangle backfaces as noncolliding, we'll treat zero distance as a backface non-collision.
            Vector3Wide.Dot(localTriangleNormal, manifold.Normal, out var faceNormalDotLocalNormal);
            TriangleWide.ComputeNondegenerateTriangleMask(ab, ac, triangleNormalLength, out _, out var nondegenerateMask);
            manifold.ContactExists = Vector.BitwiseAnd(
                Vector.BitwiseAnd(
                    Vector.GreaterThan(distance, Vector<float>.Zero),
                    nondegenerateMask),
                Vector.BitwiseAnd(
                    Vector.LessThanOrEqual(faceNormalDotLocalNormal, new Vector<float>(-TriangleWide.BackfaceNormalDotRejectionThreshold)),
                    Vector.GreaterThanOrEqual(manifold.Depth, -speculativeMargin)));
        }

        public void Test(ref SphereWide a, ref TriangleWide b, ref Vector<float> speculativeMargin, ref Vector3Wide offsetB, int pairCount, out Convex1ContactManifoldWide manifold)
        {
            throw new NotImplementedException();
        }
    }
}
