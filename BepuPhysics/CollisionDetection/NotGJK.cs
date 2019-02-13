using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.SweepTasks;
using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.CollisionDetection
{

    public enum NotGJKSimplexNormalSource
    {
        Edge,
        TriangleNormal
    }

    public struct NotGJKSimplexVertex
    {
        public Vector3 Support;
        public Vector3 Normal;
        public float Depth;
    }

    public struct NotGJKSimplexStep
    {
        public NotGJKSimplexVertex A;
        public NotGJKSimplexVertex B;
        public NotGJKSimplexVertex C;
        public NotGJKSimplexNormalSource NormalSource;
        public Vector3 ClosestPointOnSimplex;
        public Vector3 NextNormal;
    }


    public static class NotGJK<TShapeA, TShapeWideA, TSupportFinderA, TShapeB, TShapeWideB, TSupportFinderB>
        where TShapeA : IConvexShape
        where TShapeWideA : IShapeWide<TShapeA>
        where TSupportFinderA : ISupportFinder<TShapeA, TShapeWideA>
        where TShapeB : IConvexShape
        where TShapeWideB : IShapeWide<TShapeB>
        where TSupportFinderB : ISupportFinder<TShapeB, TShapeWideB>
    {
        public struct Vertex
        {
            public Vector3Wide Support;
            public Vector3Wide Normal;
            public Vector<int> Exists;
        }

        public struct Simplex
        {
            public Vertex A;
            public Vertex B;
            public Vertex C;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void FillSlot(in Vector<int> shouldReplace, ref Vertex vertex, in Vector3Wide support, in Vector3Wide normal, out Vector<int> slotFilled)
        {
            slotFilled = Vector.AndNot(shouldReplace, vertex.Exists);
            //Note that this always fills empty slots. That's important- we avoid figuring out what subsimplex is active
            //and instead just treat it as a degenerate simplex with some duplicates. (Shares code with the actual degenerate path.)
            Vector3Wide.ConditionalSelect(vertex.Exists, vertex.Support, support, out vertex.Support);
            Vector3Wide.ConditionalSelect(vertex.Exists, vertex.Normal, normal, out vertex.Normal);
            vertex.Exists = Vector.BitwiseOr(vertex.Exists, shouldReplace);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void ForceFillSlot(in Vector<int> shouldFill, ref Vertex vertex, in Vector3Wide support, in Vector3Wide normal)
        {
            vertex.Exists = Vector.BitwiseOr(vertex.Exists, shouldFill);
            Vector3Wide.ConditionalSelect(shouldFill, support, vertex.Support, out vertex.Support);
            Vector3Wide.ConditionalSelect(shouldFill, normal, vertex.Normal, out vertex.Normal);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Create(in Vector3Wide normal, in Vector3Wide support, out Simplex simplex)
        {
            simplex.A.Support = support;
            simplex.A.Normal = normal;
            simplex.A.Exists = new Vector<int>(-1);
            simplex.B.Exists = Vector<int>.Zero;
            simplex.C.Exists = Vector<int>.Zero;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void FindSupport(in TShapeWideA a, in TShapeWideB b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB, ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB, in Vector3Wide direction, out Vector3Wide support)
        {
            //support(N, A) - support(-N, B)
            supportFinderA.ComputeLocalSupport(a, direction, out var extremeA);
            Vector3Wide.Negate(direction, out var negatedDirection);
            supportFinderB.ComputeSupport(b, localOrientationB, negatedDirection, out var extremeB);
            Vector3Wide.Add(extremeB, localOffsetB, out extremeB);

            Vector3Wide.Subtract(extremeA, extremeB, out support);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void FindMinimumDepth(in TShapeWideA a, in TShapeWideB b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB, ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB,
            in Vector3Wide initialNormal, in Vector<int> inactiveLanes, in Vector<float> convergenceThreshold, in Vector<float> minimumDepthThreshold, out Vector<float> depth, out Vector3Wide refinedNormal, List<SimplexWalkerStep> steps, int maximumIterations = 50)
        {
#if DEBUG
            Vector3Wide.LengthSquared(initialNormal, out var initialNormalLengthSquared);
            Debug.Assert(Vector.LessThanAll(Vector.BitwiseOr(inactiveLanes, Vector.LessThan(Vector.Abs(initialNormalLengthSquared - Vector<float>.One), new Vector<float>(1e-6f))), Vector<int>.Zero));
#endif
            FindSupport(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, initialNormal, out var initialSupport);
            Vector3Wide.Dot(initialSupport, initialNormal, out var initialDepth);
            Create(initialNormal, initialSupport, out var simplex);
            FindMinimumDepth(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, ref simplex, initialDepth, inactiveLanes, convergenceThreshold, minimumDepthThreshold, out depth, out refinedNormal, steps, maximumIterations);
        }

        static void UpdateSimplex(ref Simplex simplex, in Vector3Wide localOffsetB, in Vector3Wide normal, in Vector3Wide support, in Vector<int> terminatedLanes, List<NotGJKSimplexStep> steps, out Vector3Wide nextNormal, out Vector<int> normalSource)
        {
            var simplexFull = Vector.BitwiseAnd(simplex.A.Exists, Vector.BitwiseAnd(simplex.B.Exists, simplex.C.Exists));
            if (Vector.LessThanAny(Vector.AndNot(simplexFull, terminatedLanes), Vector<int>.Zero))
            {
                //At least one lane has a full simplex. With the new support, we have three subtriangles.
                //Choose the subtriangle with minimal plane depth.
                Vector3Wide.Subtract(support, simplex.A.Support, out var ad);
                Vector3Wide.Subtract(support, simplex.B.Support, out var bd);
                Vector3Wide.Subtract(support, simplex.C.Support, out var cd);
                Vector3Wide.Cross(ad, bd, out var adbN);
                Vector3Wide.Cross(bd, cd, out var bdcN);
                Vector3Wide.Cross(cd, ad, out var cdaN);
                //The normals here are not normalized.
                //planeDepth = dot(support, N / ||N||)
                //sign(dot(support, N)) * planeDepth^2 = sign(dot(support, N)) * dot(support, N)^2 / ||N||^2
                //planeDepthADB < planeDepthBDC becomes:
                //sign(dot(support, adbN)) * dot(support, adbN)^2 * ||bdcN||^2 < sign(dot(support, bdcN)) * dot(support, bdcN)^2 * ||adbN||^2
                //No sqrt or division required.
                Vector3Wide.Dot(adbN, support, out var adbDot);
                Vector3Wide.Dot(bdcN, support, out var bdcDot);
                Vector3Wide.Dot(cdaN, support, out var cdaDot);
                Vector3Wide.LengthSquared(adbN, out var adbNLengthSquared);
                Vector3Wide.LengthSquared(bdcN, out var bdcNLengthSquared);
                Vector3Wide.LengthSquared(cdaN, out var cdaNLengthSquared);

                var adbDotSquared = adbDot * adbDot;
                var bdcDotSquared = bdcDot * bdcDot;
                var cdaDotSquared = cdaDot * cdaDot;
                //Note that we computed subtriangle normals with consistent winding, but we haven't maintained the simplex's winding so we calibrate.
                Vector3Wide.Dot(adbN, localOffsetB, out var adbNCalibrationDot);
                var shouldCalibrateNormal = Vector.LessThan(adbNCalibrationDot, Vector<float>.Zero);
                adbDot = Vector.ConditionalSelect(shouldCalibrateNormal, -adbDot, adbDot);
                bdcDot = Vector.ConditionalSelect(shouldCalibrateNormal, -bdcDot, bdcDot);
                cdaDot = Vector.ConditionalSelect(shouldCalibrateNormal, -cdaDot, cdaDot);
                var adbDotSquaredSigned = Vector.ConditionalSelect(Vector.LessThan(adbDot, Vector<float>.Zero), -adbDotSquared, adbDotSquared);
                var bdcDotSquaredSigned = Vector.ConditionalSelect(Vector.LessThan(bdcDot, Vector<float>.Zero), -bdcDotSquared, bdcDotSquared);
                var cdaDotSquaredSigned = Vector.ConditionalSelect(Vector.LessThan(cdaDot, Vector<float>.Zero), -cdaDotSquared, cdaDotSquared);
                var adbLessThanBDC = Vector.LessThan(adbDotSquaredSigned * bdcNLengthSquared, bdcDotSquaredSigned * adbNLengthSquared);
                var adbLessThanCDA = Vector.LessThan(adbDotSquaredSigned * cdaNLengthSquared, cdaDotSquaredSigned * adbNLengthSquared);
                var bdcLessThanCDA = Vector.LessThan(bdcDotSquaredSigned * cdaNLengthSquared, cdaDotSquaredSigned * bdcNLengthSquared);
                var useADB = Vector.BitwiseAnd(adbLessThanBDC, adbLessThanCDA);
                var useBDC = Vector.AndNot(bdcLessThanCDA, useADB);
                var useCDA = Vector.AndNot(Vector.OnesComplement(useADB), useBDC);
                
                ForceFillSlot(useADB, ref simplex.C, support, normal);
                ForceFillSlot(useBDC, ref simplex.A, support, normal);
                ForceFillSlot(useCDA, ref simplex.B, support, normal);
            }
            //If there is an empty slot, fill it with the new support. Note that this cannot conflict with the 
            //simplexFull case above- that only occurs for lanes which started with a full simplex, and it completes with a full simplex.
            var tryFillSlot = Vector.OnesComplement(simplexFull);
            FillSlot(tryFillSlot, ref simplex.A, support, normal, out var slotFilled);
            tryFillSlot = Vector.AndNot(tryFillSlot, slotFilled);
            FillSlot(tryFillSlot, ref simplex.B, support, normal, out slotFilled);
            tryFillSlot = Vector.AndNot(tryFillSlot, slotFilled);
            FillSlot(tryFillSlot, ref simplex.C, support, normal, out slotFilled);

            //Compute the barycentric coordinates of the origin on the triangle.
            //Note that the above FillSlot calls filled all empty slots with the new support, so
            //we combine the empty simplex case with the degenerate case.
            Vector3Wide.Subtract(simplex.B.Support, simplex.A.Support, out var ab);
            Vector3Wide.Subtract(simplex.A.Support, simplex.C.Support, out var ca);
            Vector3Wide.CrossWithoutOverlap(ab, ca, out var n);
            Vector3Wide.LengthSquared(n, out var nLengthSquared);

            var simplexIsDegenerate = Vector.LessThan(nLengthSquared, new Vector<float>(1e-14f));
            nextNormal = default;
            normalSource = default;            

            //if (Vector.LessThanAny(Vector.AndNot(simplexIsDegenerate, terminatedLanes), Vector<int>.Zero))
            //{
            //    //At least one active lane cannot compute valid barycentric coordinates; the simplex is not a triangle.
            //    //It's either edge-like or vertex-like. Capture both cases by choosing the longest edge and using the closest point on it to the origin.
            //    Vector3Wide.Subtract(simplex.C.Support, simplex.B.Support, out var bc);
            //    Vector3Wide.LengthSquared(ab, out var abLengthSquared);
            //    Vector3Wide.LengthSquared(bc, out var bcLengthSquared);
            //    Vector3Wide.LengthSquared(ca, out var caLengthSquared);
            //    var useAB = Vector.GreaterThanOrEqual(abLengthSquared, Vector.Max(bcLengthSquared, caLengthSquared));
            //    var useBC = Vector.AndNot(Vector.GreaterThanOrEqual(bcLengthSquared, Vector.Max(abLengthSquared, caLengthSquared)), useAB);

            //    Vector3Wide edgeOffset, edgeStart;
            //    Vector3Wide normalStart, normalEnd;
            //    edgeOffset.X = Vector.ConditionalSelect(useAB, ab.X, Vector.ConditionalSelect(useBC, bc.X, ca.X));
            //    edgeOffset.Y = Vector.ConditionalSelect(useAB, ab.Y, Vector.ConditionalSelect(useBC, bc.Y, ca.Y));
            //    edgeOffset.Z = Vector.ConditionalSelect(useAB, ab.Z, Vector.ConditionalSelect(useBC, bc.Z, ca.Z));
            //    edgeStart.X = Vector.ConditionalSelect(useAB, simplex.A.Support.X, Vector.ConditionalSelect(useBC, simplex.B.Support.X, simplex.C.Support.X));
            //    edgeStart.Y = Vector.ConditionalSelect(useAB, simplex.A.Support.Y, Vector.ConditionalSelect(useBC, simplex.B.Support.Y, simplex.C.Support.Y));
            //    edgeStart.Z = Vector.ConditionalSelect(useAB, simplex.A.Support.Z, Vector.ConditionalSelect(useBC, simplex.B.Support.Z, simplex.C.Support.Z));
            //    normalStart.X = Vector.ConditionalSelect(useAB, simplex.A.Normal.X, Vector.ConditionalSelect(useBC, simplex.B.Normal.X, simplex.C.Normal.X));
            //    normalStart.Y = Vector.ConditionalSelect(useAB, simplex.A.Normal.Y, Vector.ConditionalSelect(useBC, simplex.B.Normal.Y, simplex.C.Normal.Y));
            //    normalStart.Z = Vector.ConditionalSelect(useAB, simplex.A.Normal.Z, Vector.ConditionalSelect(useBC, simplex.B.Normal.Z, simplex.C.Normal.Z));
            //    normalEnd.X = Vector.ConditionalSelect(useAB, simplex.B.Normal.X, Vector.ConditionalSelect(useBC, simplex.C.Normal.X, simplex.A.Normal.X));
            //    normalEnd.Y = Vector.ConditionalSelect(useAB, simplex.B.Normal.Y, Vector.ConditionalSelect(useBC, simplex.C.Normal.Y, simplex.A.Normal.Y));
            //    normalEnd.Z = Vector.ConditionalSelect(useAB, simplex.B.Normal.Z, Vector.ConditionalSelect(useBC, simplex.C.Normal.Z, simplex.A.Normal.Z));
            //    var lengthSquared = Vector.ConditionalSelect(useAB, abLengthSquared, Vector.ConditionalSelect(useBC, bcLengthSquared, caLengthSquared));

            //    //dot(origin - edgeStart, edgeOffset / ||edgeOffset||) * edgeOffset / ||edgeOffset||
            //    Vector3Wide.Dot(edgeStart, edgeOffset, out var dot);
            //    //TODO: Approximate rcp would be sufficient here.
            //    var t = -dot / lengthSquared;
            //    t = Vector.Min(Vector<float>.One, Vector.Max(Vector<float>.Zero, t));
            //    t = Vector.ConditionalSelect(Vector.LessThan(lengthSquared, new Vector<float>(1e-14f)), Vector<float>.Zero, t);
            //    Vector3Wide.Scale(edgeOffset, t, out var interpolatedOffset);
            //    Vector3Wide.Add(interpolatedOffset, edgeStart, out var interpolatedSupport);
            //    Vector3Wide.Scale(normalStart, Vector<float>.One - t, out var normalStartContribution);
            //    Vector3Wide.Scale(normalEnd, t, out var normalEndContribution);
            //    Vector3Wide.Add(normalStartContribution, normalEndContribution, out var interpolatedNormal);

            //    //Create a target point on the origin line that we want a tilted bounding plane to intersect.
            //    //We're pivoting on the interpolated support, tilting the interpolated normal a little closer to the origin.
            //    //This is not exactly a rigorous process- the goal is just to expand the simplex so a better heuristic can be used.
            //    Vector3Wide.LengthSquared(interpolatedSupport, out var originToSupportLengthSquared);
            //    //Note the use of the deepest depth and normal to create the point on origin line.
            //    var aIsBest = Vector.LessThan(simplex.A.Depth, Vector.Min(simplex.B.Depth, simplex.C.Depth));
            //    var bIsBest = Vector.LessThan(simplex.B.Depth, Vector.Min(simplex.A.Depth, simplex.C.Depth));
            //    var bestDepth = Vector.ConditionalSelect(aIsBest, simplex.A.Depth, Vector.ConditionalSelect(bIsBest, simplex.B.Depth, simplex.C.Depth));
            //    Vector3Wide.ConditionalSelect(aIsBest, simplex.A.Normal, simplex.B.Normal, out var bestNormal);
            //    Vector3Wide.ConditionalSelect(bIsBest, bestNormal, simplex.C.Normal, out bestNormal);
            //    Vector3Wide.Scale(bestNormal, bestDepth - originToSupportLengthSquared * 0.25f, out var pointOnOriginLine);
            //    Vector3Wide.Subtract(pointOnOriginLine, interpolatedSupport, out var supportToPointOnOriginLine);
            //    Vector3Wide.CrossWithoutOverlap(interpolatedNormal, supportToPointOnOriginLine, out var intermediate);
            //    Vector3Wide.CrossWithoutOverlap(supportToPointOnOriginLine, intermediate, out nextNormal);

            //    Vector3Wide.LengthSquared(nextNormal, out var nextNormalLengthSquared);
            //    Vector3Wide.ConditionalSelect(Vector.GreaterThan(nextNormalLengthSquared, new Vector<float>(1e-14f)), nextNormal, interpolatedNormal, out nextNormal);

            //    //The normal source for edge cases is 0; 'or' in the vertices to replace.
            //    normalSource = Vector.ConditionalSelect(useAB, new Vector<int>(1 << 4), Vector.ConditionalSelect(useBC, new Vector<int>(1 << 2), new Vector<int>(1 << 3)));

            //    if (Vector.AndNot(simplexIsDegenerate, terminatedLanes)[0] < 0)
            //    {
            //        step.NormalSource = SimplexNormalSource.Edge;
            //        Vector3Wide.ReadSlot(ref pointOnOriginLine, 0, out step.PointOnOriginLine);
            //        Vector3Wide.ReadSlot(ref interpolatedNormal, 0, out step.InterpolatedNormal);
            //        Vector3Wide.ReadSlot(ref interpolatedSupport, 0, out step.InterpolatedSupport);
            //    }
            //}
            //if (Vector.LessThanAny(Vector.AndNot(Vector.OnesComplement(simplexIsDegenerate), terminatedLanes), Vector<int>.Zero))
            //{
            //    //At least one active lane has a non-degenerate simplex.
            //    //TODO: Approximate RCP could be useful here.
            //    var inverseNLengthSquared = Vector<float>.One / nLengthSquared;

            //    Vector3Wide.CrossWithoutOverlap(ab, simplex.A.Support, out var axab);
            //    Vector3Wide.CrossWithoutOverlap(ca, simplex.C.Support, out var cxca);
            //    Vector3Wide.Dot(axab, n, out var cNumerator);
            //    Vector3Wide.Dot(cxca, n, out var bNumerator);
            //    var cWeight = cNumerator * inverseNLengthSquared;
            //    var bWeight = bNumerator * inverseNLengthSquared;
            //    var aWeight = Vector<float>.One - bWeight - cWeight;

            //    var originInTriangle = Vector.BitwiseAnd(Vector.BitwiseAnd(
            //        Vector.GreaterThanOrEqual(cWeight, Vector<float>.Zero),
            //        Vector.GreaterThanOrEqual(bWeight, Vector<float>.Zero)),
            //        Vector.GreaterThanOrEqual(aWeight, Vector<float>.Zero));

            //    //If the origin is in the triangle, then the next normal is simply the triangle normal.
            //    //Otherwise, use the barycentric coordinates to extrapolate the normal from the simplex normals.
            //    Vector3Wide.Scale(simplex.A.Normal, aWeight, out var normalContributionA);
            //    Vector3Wide.Scale(simplex.B.Normal, bWeight, out var normalContributionB);
            //    Vector3Wide.Scale(simplex.C.Normal, cWeight, out var normalContributionC);
            //    Vector3Wide.Add(normalContributionA, normalContributionB, out var interpolatedNormal);
            //    Vector3Wide.Add(normalContributionC, interpolatedNormal, out interpolatedNormal);

            //    Vector3Wide.LengthSquared(interpolatedNormal, out var interpolatedNormalLengthSquared);
            //    var useTriangleNormal = Vector.BitwiseOr(originInTriangle, Vector.LessThan(interpolatedNormalLengthSquared, new Vector<float>(1e-14f)));
            //    Vector3Wide.Dot(n, localOffsetB, out var nDotOffsetB);
            //    var shouldCalibrateTriangleNormal = Vector.LessThan(nDotOffsetB, Vector<float>.Zero);
            //    Vector3Wide.ConditionallyNegate(shouldCalibrateTriangleNormal, ref n);
            //    Vector3Wide.ConditionalSelect(useTriangleNormal, n, interpolatedNormal, out var normalCandidate);
            //    Vector3Wide.ConditionalSelect(simplexIsDegenerate, nextNormal, normalCandidate, out nextNormal);
            //    //In the event that we're using a barycentric-found normal, include a flag for which vertex should be replaced.
            //    //For this process, we ignore depths- instead, we go by barycentric weight signs. A negative weight associated with a vertex
            //    //implies that the origin is outside of the opposing edge. That is, a negative weight for vertex B implies the origin
            //    //is outside the CA edge plane. (It may *also* be outside the BC edge plane, but if the origin is outside
            //    //two edge planes, we just arbitrarily pick one of the opposing vertices to remove.)
            //    var replaceA = Vector.LessThan(aWeight, Vector<float>.Zero);
            //    var replaceB = Vector.LessThan(bWeight, Vector<float>.Zero);
            //    var replaceFlag = Vector.ConditionalSelect(replaceA, new Vector<int>(1 << 2), Vector.ConditionalSelect(replaceB, new Vector<int>(1 << 3), new Vector<int>(1 << 4)));
            //    normalSource = Vector.ConditionalSelect(simplexIsDegenerate, normalSource, Vector.ConditionalSelect(useTriangleNormal, new Vector<int>(2), Vector.BitwiseOr(new Vector<int>(1), replaceFlag)));

            //    if (Vector.AndNot(Vector.OnesComplement(simplexIsDegenerate), terminatedLanes)[0] < 0)
            //    {
            //        if (useTriangleNormal[0] < 0)
            //        {
            //            step.NormalSource = SimplexNormalSource.TriangleNormal;
            //        }
            //        else
            //        {
            //            step.NormalSource = SimplexNormalSource.Barycentric;
            //            Vector3Wide.ReadSlot(ref simplex.A.Support, 0, out var a);
            //            Vector3Wide.ReadSlot(ref simplex.B.Support, 0, out var b);
            //            Vector3Wide.ReadSlot(ref simplex.C.Support, 0, out var c);
            //            step.InterpolatedSupport = (a * aWeight[0] + b * bWeight[0] + c * cWeight[0]);
            //        }
            //    }
            //}
            ////TOOD: rsqrt would be nice here.
            //Vector3Wide.Length(nextNormal, out var normalLength);
            //Vector3Wide.Scale(nextNormal, Vector<float>.One / normalLength, out nextNormal);

            //if (steps != null)
            //{
            //    Vector3Wide.ReadSlot(ref nextNormal, 0, out step.NextNormal);
            //    steps.Add(step);
            //}
        }

        //        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        //        static void CreateDebugStep(ref Simplex simplex, out SimplexWalkerStep step)
        //        {
        //            step = default;
        //            Vector3Wide.ReadSlot(ref simplex.A.Support, 0, out step.A.Support);
        //            Vector3Wide.ReadSlot(ref simplex.A.Normal, 0, out step.A.Normal);
        //            step.A.Depth = simplex.A.Depth[0];
        //            Vector3Wide.ReadSlot(ref simplex.B.Support, 0, out step.B.Support);
        //            Vector3Wide.ReadSlot(ref simplex.B.Normal, 0, out step.B.Normal);
        //            step.B.Depth = simplex.B.Depth[0];
        //            Vector3Wide.ReadSlot(ref simplex.C.Support, 0, out step.C.Support);
        //            Vector3Wide.ReadSlot(ref simplex.C.Normal, 0, out step.C.Normal);
        //            step.C.Depth = simplex.C.Depth[0];
        //        }


        public static void FindMinimumDepth(in TShapeWideA a, in TShapeWideB b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB, ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB,
            ref Simplex simplex, in Vector<float> initialDepth,
            in Vector<int> inactiveLanes, in Vector<float> surfaceThreshold, in Vector<float> minimumDepthThreshold, out Vector<float> refinedDepth, out Vector3Wide refinedNormal, List<SimplexWalkerStep> steps, int maximumIterations = 50)
        {
#if DEBUG
            Vector3Wide.LengthSquared(simplex.A.Normal, out var initialNormalLengthSquared);
            Debug.Assert(Vector.LessThanAll(Vector.BitwiseOr(inactiveLanes, Vector.LessThan(Vector.Abs(initialNormalLengthSquared - Vector<float>.One), new Vector<float>(1e-6f))), Vector<int>.Zero));
#endif            
            var depthBelowThreshold = Vector.LessThan(initialDepth, minimumDepthThreshold);
            var terminatedLanes = Vector.BitwiseOr(depthBelowThreshold, inactiveLanes);

            if (Vector.LessThanAll(terminatedLanes, Vector<int>.Zero))
            {
                refinedNormal = simplex.A.Normal;
                refinedDepth = initialDepth;
                return;
            }

            //UpdateSimplex(ref simplex, localOffsetB, terminatedLanes, steps, out var normal, out var normalSource);

            //for (int i = 0; i < maximumIterations; ++i)
            //{
            //    FindSupport(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, normal, out var support);
            //    Vector3Wide.Dot(support, normal, out var depth);

            //    //If we used the triangle normal (because the origin was contained within the face region) and couldn't find a more extreme point,
            //    //then we have found the surface and can terminate.
            //    Vector3Wide.Dot(simplex.A.Support, normal, out var aDotN);
            //    var progressBelowEpsilon = Vector.BitwiseAnd(Vector.Equals(normalSource, new Vector<int>(2)), Vector.LessThan(depth - aDotN, surfaceThreshold));

            //    Add(ref simplex, support, normal, depth, normalSource);

            //    //If all lanes are sufficiently separated, then we can early out.
            //    depthBelowThreshold = Vector.LessThan(simplex.A.Depth, minimumDepthThreshold);
            //    terminatedLanes = Vector.BitwiseOr(depthBelowThreshold, Vector.BitwiseOr(progressBelowEpsilon, terminatedLanes));
            //    if (Vector.LessThanAll(terminatedLanes, Vector<int>.Zero))
            //        break;

            //    GetNextNormal(ref simplex, localOffsetB, terminatedLanes, steps, out normal, out normalSource);
            //}
            //refinedNormal = simplex.A.Normal;
            //refinedDepth = simplex.A.Depth;
        }
    }
}
