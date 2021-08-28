using BepuPhysics.Collidables;
using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.SweepTasks
{
    public struct SphereTriangleDistanceTester : IPairDistanceTester<SphereWide, TriangleWide>
    {
        public void Test(in SphereWide a, in TriangleWide b, in Vector3Wide offsetB, in QuaternionWide orientationA, in QuaternionWide orientationB, in Vector<int> inactiveLanes,
            out Vector<int> intersected, out Vector<float> distance, out Vector3Wide closestA, out Vector3Wide normal)
        {
            //Note that we're borrowing a lot here from the SphereTriangleCollisionTask. Could share more if you find yourself needing to change things dramatically.
            //Main difficulty in fully sharing is that sweep tests do not honor one sidedness, so some of the conditions change.

            //Work in the local space of the triangle, since it's quicker to transform the sphere position than the vertices of the triangle.
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var rB);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(offsetB, rB, out var localOffsetB);
            
            Vector3Wide.Subtract(b.B, b.A, out var ab);
            Vector3Wide.Subtract(b.C, b.A, out var ac);
            //localOffsetA = -localOffsetB, so pa = triangle.A + localOffsetB.
            Vector3Wide.Add(b.A, localOffsetB, out var pa);
            Vector3Wide.CrossWithoutOverlap(ab, ac, out var localTriangleNormal);
            Vector3Wide.Dot(localTriangleNormal, pa, out var paN);

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
            Vector3Wide.Dot(localTriangleNormal, localTriangleNormal, out var triangleNormalLengthSquared);
            var edgePlaneTestBC = triangleNormalLengthSquared - edgePlaneTestAB - edgePlaneTestAC;

            var outsideAB = Vector.LessThan(edgePlaneTestAB, Vector<float>.Zero);
            var outsideAC = Vector.LessThan(edgePlaneTestAC, Vector<float>.Zero);
            var outsideBC = Vector.LessThan(edgePlaneTestBC, Vector<float>.Zero);

            var outsideAnyEdge = Vector.BitwiseOr(outsideAB, Vector.BitwiseOr(outsideAC, outsideBC));
            Unsafe.SkipInit(out Vector3Wide localClosestOnTriangle);
            var negativeOne = new Vector<int>(-1);
            if (Vector.EqualsAny(outsideAnyEdge, negativeOne))
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
            if (Vector.EqualsAny(outsideAnyEdge, Vector<int>.Zero))
            {
                //p + N * (pa * N) / ||N||^2 = N * (pa * N) / ||N||^2 - (-p)
                var nScale = paN / triangleNormalLengthSquared;
                Vector3Wide.Scale(localTriangleNormal, nScale, out var offsetToPlane);
                Vector3Wide.Subtract(offsetToPlane, localOffsetB, out var pointOnFace);

                Vector3Wide.ConditionalSelect(outsideAnyEdge, localClosestOnTriangle, pointOnFace, out localClosestOnTriangle);
            }

            //normal = normalize(localOffsetA - localClosestOnTriangle) = (localOffsetB + localClosestOnTriangle) / (-||localOffsetB + localClosestOnTriangle||)
            Vector3Wide.Add(localOffsetB, localClosestOnTriangle, out var localNormal);
            Vector3Wide.Length(localNormal, out var localNormalLength);
            Vector3Wide.Scale(localNormal, new Vector<float>(-1f) / localNormalLength, out localNormal);
            Matrix3x3Wide.TransformWithoutOverlap(localNormal, rB, out normal);
            Vector3Wide.Scale(normal, -a.Radius, out closestA);
            distance = localNormalLength - a.Radius;
            intersected = Vector.LessThanOrEqual(distance, Vector<float>.Zero);

            
        }
    }


}
