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
    public struct SimplexWalkerVertex
    {
        public Vector3 Support;
        public Vector3 Normal;
        public float Depth;
    }

    public struct SimplexWalkerStep
    {
        public SimplexWalkerVertex A;
        public SimplexWalkerVertex B;
        public SimplexWalkerVertex C;
    }


    public static class SimplexWalker<TShapeA, TShapeWideA, TSupportFinderA, TShapeB, TShapeWideB, TSupportFinderB>
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
            public Vector<float> Depth;
        }

        public struct Simplex
        {
            public Vertex A;
            public Vertex B;
            public Vertex C;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Replace(in Vector<int> shouldReplace, ref Vertex vertex, in Vector3Wide support, in Vector3Wide normal, in Vector<float> depth)
        {
            Vector3Wide.ConditionalSelect(shouldReplace, support, vertex.Support, out vertex.Support);
            Vector3Wide.ConditionalSelect(shouldReplace, normal, vertex.Normal, out vertex.Normal);
            vertex.Depth = Vector.ConditionalSelect(shouldReplace, depth, vertex.Depth);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Create(in Vector3Wide normal, in Vector3Wide support, in Vector<float> depth, out Simplex simplex)
        {
            //Populate the entire simplex with the given vertex.
            simplex.A.Support = support;
            simplex.A.Normal = normal;
            simplex.A.Depth = depth;
            simplex.B = simplex.A;
            simplex.C = simplex.A;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Add(ref Simplex simplex, in Vector3Wide support, in Vector3Wide normal, in Vector<float> depth)
        {
            //Preserve the ordering of simplex vertices by depth.
            var replaceC = Vector.LessThan(depth, simplex.C.Depth);
            Replace(replaceC, ref simplex.C, support, normal, depth);
            //simplex.C.Support = support;
            //simplex.C.Normal = normal;
            //simplex.C.Depth = depth;
            var replaceB = Vector.LessThan(depth, simplex.B.Depth);
            Replace(replaceB, ref simplex.C, simplex.B.Support, simplex.B.Normal, simplex.B.Depth);
            Replace(replaceB, ref simplex.B, support, normal, depth);
            var replaceA = Vector.LessThan(depth, simplex.A.Depth);
            Replace(replaceA, ref simplex.B, simplex.A.Support, simplex.A.Normal, simplex.A.Depth);
            Replace(replaceA, ref simplex.A, support, normal, depth);
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
            Create(initialNormal, initialSupport, initialDepth, out var simplex);
            FindMinimumDepth(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, ref simplex, inactiveLanes, convergenceThreshold, minimumDepthThreshold, out depth, out refinedNormal, steps, maximumIterations);
        }

        static void GetNextNormal(ref Simplex simplex, in Vector3Wide localOffsetB, in Vector<int> terminatedLanes, out Vector3Wide nextNormal, out Vector<int> normalIsSimplexNormal)
        {
            //Compute the barycentric coordinates of the origin on the triangle.
            Vector3Wide.Subtract(simplex.B.Support, simplex.A.Support, out var ab);
            Vector3Wide.Subtract(simplex.A.Support, simplex.C.Support, out var ca);
            Vector3Wide.CrossWithoutOverlap(ab, ca, out var n);
            Vector3Wide.LengthSquared(n, out var nLengthSquared);

            var simplexIsDegenerate = Vector.LessThan(nLengthSquared, new Vector<float>(1e-14f));
            nextNormal = default;
            normalIsSimplexNormal = Vector<int>.Zero;
            if (Vector.LessThanAny(Vector.AndNot(simplexIsDegenerate, terminatedLanes), Vector<int>.Zero))
            {
                //At least one active lane cannot compute valid barycentric coordinates; the simplex is not a triangle.
                //It's either edge-like or vertex-like. Capture both cases by choosing the longest edge and using the closest point on it to the origin.
                Vector3Wide.Subtract(simplex.C.Support, simplex.B.Support, out var bc);
                Vector3Wide.LengthSquared(ab, out var abLengthSquared);
                Vector3Wide.LengthSquared(bc, out var bcLengthSquared);
                Vector3Wide.LengthSquared(ca, out var caLengthSquared);
                var useAB = Vector.GreaterThan(abLengthSquared, Vector.Max(bcLengthSquared, caLengthSquared));
                var useBC = Vector.GreaterThan(bcLengthSquared, Vector.Max(abLengthSquared, caLengthSquared));

                Vector3Wide edgeOffset, edgeStart;
                Vector3Wide normalStart, normalEnd;
                edgeOffset.X = Vector.ConditionalSelect(useAB, ab.X, Vector.ConditionalSelect(useBC, bc.X, ca.X));
                edgeOffset.Y = Vector.ConditionalSelect(useAB, ab.Y, Vector.ConditionalSelect(useBC, bc.Y, ca.Y));
                edgeOffset.Z = Vector.ConditionalSelect(useAB, ab.Z, Vector.ConditionalSelect(useBC, bc.Z, ca.Z));
                edgeStart.X = Vector.ConditionalSelect(useAB, simplex.A.Support.X, Vector.ConditionalSelect(useBC, simplex.B.Support.X, simplex.C.Support.X));
                edgeStart.Y = Vector.ConditionalSelect(useAB, simplex.A.Support.Y, Vector.ConditionalSelect(useBC, simplex.B.Support.Y, simplex.C.Support.Y));
                edgeStart.Z = Vector.ConditionalSelect(useAB, simplex.A.Support.Z, Vector.ConditionalSelect(useBC, simplex.B.Support.Z, simplex.C.Support.Z));
                normalStart.X = Vector.ConditionalSelect(useAB, simplex.A.Normal.X, Vector.ConditionalSelect(useBC, simplex.B.Normal.X, simplex.C.Normal.X));
                normalStart.Y = Vector.ConditionalSelect(useAB, simplex.A.Normal.Y, Vector.ConditionalSelect(useBC, simplex.B.Normal.Y, simplex.C.Normal.Y));
                normalStart.Z = Vector.ConditionalSelect(useAB, simplex.A.Normal.Z, Vector.ConditionalSelect(useBC, simplex.B.Normal.Z, simplex.C.Normal.Z));
                normalEnd.X = Vector.ConditionalSelect(useAB, simplex.B.Normal.X, Vector.ConditionalSelect(useBC, simplex.C.Normal.X, simplex.A.Normal.X));
                normalEnd.Y = Vector.ConditionalSelect(useAB, simplex.B.Normal.Y, Vector.ConditionalSelect(useBC, simplex.C.Normal.Y, simplex.A.Normal.Y));
                normalEnd.Z = Vector.ConditionalSelect(useAB, simplex.B.Normal.Z, Vector.ConditionalSelect(useBC, simplex.C.Normal.Z, simplex.A.Normal.Z));
                var lengthSquared = Vector.ConditionalSelect(useAB, abLengthSquared, Vector.ConditionalSelect(useBC, bcLengthSquared, caLengthSquared));

                //dot(origin - edgeStart, edgeOffset / ||edgeOffset||) * edgeOffset / ||edgeOffset||
                Vector3Wide.Dot(edgeStart, edgeOffset, out var dot);
                //TODO: Approximate rcp would be sufficient here.
                //Note that division by zero will get filtered out by the clamp, so this works for the vertex case.
                var t = -dot / lengthSquared;
                t = Vector.Min(Vector<float>.One, Vector.Max(Vector<float>.Zero, t));
                Vector3Wide.Scale(edgeOffset, t, out var interpolatedOffset);
                Vector3Wide.Add(interpolatedOffset, edgeStart, out var interpolatedSupport);
                Vector3Wide.Scale(normalStart, Vector<float>.One - t, out var normalStartContribution);
                Vector3Wide.Scale(normalEnd, t, out var normalEndContribution);
                Vector3Wide.Add(normalStartContribution, normalEndContribution, out var interpolatedNormal);

                //Create a target point on the origin line that we want a tilted bounding plane to intersect.
                //We're pivoting on the interpolated support, tilting the interpolated normal a little closer to the origin.
                //This is not exactly a rigorous process- the goal is just to expand the simplex so a better heuristic can be used.
                Vector3Wide.LengthSquared(interpolatedSupport, out var originToSupportLengthSquared);
                Vector3Wide.Scale(simplex.A.Normal, simplex.A.Depth - originToSupportLengthSquared * 0.25f, out var pointOnOriginLine);
                Vector3Wide.Subtract(pointOnOriginLine, interpolatedSupport, out var supportToPointOnOriginLine);
                Vector3Wide.CrossWithoutOverlap(interpolatedNormal, supportToPointOnOriginLine, out var intermediate);
                Vector3Wide.CrossWithoutOverlap(supportToPointOnOriginLine, intermediate, out nextNormal);

                Vector3Wide.LengthSquared(nextNormal, out var nextNormalLengthSquared);
                Vector3Wide.ConditionalSelect(Vector.GreaterThan(nextNormalLengthSquared, new Vector<float>(1e-14f)), nextNormal, interpolatedNormal, out nextNormal);
            }
            if (Vector.LessThanAny(Vector.AndNot(Vector.OnesComplement(simplexIsDegenerate), terminatedLanes), Vector<int>.Zero))
            {
                //At least one active lane has a non-degenerate simplex.
                //TODO: Approximate RCP could be useful here.
                var inverseNLengthSquared = Vector<float>.One / nLengthSquared;

                Vector3Wide.CrossWithoutOverlap(simplex.A.Support, ab, out var axab);
                Vector3Wide.CrossWithoutOverlap(simplex.A.Support, ca, out var axca);
                Vector3Wide.Dot(axab, n, out var cNumerator);
                Vector3Wide.Dot(axca, n, out var bNumerator);
                var cWeight = cNumerator * inverseNLengthSquared;
                var bWeight = bNumerator * inverseNLengthSquared;
                var aWeight = Vector<float>.One - bWeight - cWeight;

                var originInTriangle = Vector.BitwiseAnd(Vector.BitwiseAnd(
                    Vector.GreaterThanOrEqual(cWeight, Vector<float>.Zero),
                    Vector.GreaterThanOrEqual(bWeight, Vector<float>.Zero)),
                    Vector.GreaterThanOrEqual(aWeight, Vector<float>.Zero));

                //If the origin is in the triangle, then the next normal is simply the triangle normal.
                //Otherwise, use the barycentric coordinates to extrapolate the normal from the simplex normals.
                Vector3Wide.Scale(simplex.A.Normal, aWeight, out var normalContributionA);
                Vector3Wide.Scale(simplex.B.Normal, bWeight, out var normalContributionB);
                Vector3Wide.Scale(simplex.C.Normal, cWeight, out var normalContributionC);
                Vector3Wide.Add(normalContributionA, normalContributionB, out var interpolatedNormal);
                Vector3Wide.Add(normalContributionC, interpolatedNormal, out interpolatedNormal);

                Vector3Wide.LengthSquared(interpolatedNormal, out var interpolatedNormalLengthSquared);
                var useTriangleNormal = Vector.BitwiseOr(originInTriangle, Vector.LessThan(interpolatedNormalLengthSquared, new Vector<float>(1e-14f)));
                Vector3Wide.Dot(n, localOffsetB, out var nDotOffsetB);
                var shouldCalibrateTriangleNormal = Vector.LessThan(nDotOffsetB, Vector<float>.Zero);
                Vector3Wide.ConditionallyNegate(shouldCalibrateTriangleNormal, ref n);
                Vector3Wide.ConditionalSelect(useTriangleNormal, n, interpolatedNormal, out var normalCandidate);
                Vector3Wide.ConditionalSelect(simplexIsDegenerate, nextNormal, normalCandidate, out nextNormal);
                Vector.ConditionalSelect(Vector.AndNot(useTriangleNormal, simplexIsDegenerate), new Vector<int>(-1), Vector<int>.Zero);
            }
            //TOOD: rsqrt would be nice here.
            Vector3Wide.Length(nextNormal, out var normalLength);
            Vector3Wide.Scale(nextNormal, Vector<float>.One / normalLength, out nextNormal);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void AddDebugStep(ref Simplex simplex, List<SimplexWalkerStep> steps)
        {
            if (steps != null)
            {
                SimplexWalkerStep step;
                Vector3Wide.ReadSlot(ref simplex.A.Support, 0, out step.A.Support);
                Vector3Wide.ReadSlot(ref simplex.A.Normal, 0, out step.A.Normal);
                step.A.Depth = simplex.A.Depth[0];
                Vector3Wide.ReadSlot(ref simplex.B.Support, 0, out step.B.Support);
                Vector3Wide.ReadSlot(ref simplex.B.Normal, 0, out step.B.Normal);
                step.B.Depth = simplex.B.Depth[0];
                Vector3Wide.ReadSlot(ref simplex.C.Support, 0, out step.C.Support);
                Vector3Wide.ReadSlot(ref simplex.C.Normal, 0, out step.C.Normal);
                step.C.Depth = simplex.C.Depth[0];
                steps.Add(step);
            }
        }


        public static void FindMinimumDepth(in TShapeWideA a, in TShapeWideB b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB, ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB,
            ref Simplex simplex,
            in Vector<int> inactiveLanes, in Vector<float> surfaceThreshold, in Vector<float> minimumDepthThreshold, out Vector<float> refinedDepth, out Vector3Wide refinedNormal, List<SimplexWalkerStep> steps, int maximumIterations = 50)
        {
#if DEBUG
            Vector3Wide.LengthSquared(simplex.A.Normal, out var initialNormalLengthSquared);
            Debug.Assert(Vector.LessThanAll(Vector.BitwiseOr(inactiveLanes, Vector.LessThan(Vector.Abs(initialNormalLengthSquared - Vector<float>.One), new Vector<float>(1e-6f))), Vector<int>.Zero));
#endif            
            var depthBelowThreshold = Vector.LessThan(simplex.A.Depth, minimumDepthThreshold);
            var terminatedLanes = Vector.BitwiseOr(depthBelowThreshold, inactiveLanes);

            if (Vector.LessThanAll(terminatedLanes, Vector<int>.Zero))
            {
                refinedNormal = simplex.A.Normal;
                refinedDepth = simplex.A.Depth;
                return;
            }

            GetNextNormal(ref simplex, localOffsetB, terminatedLanes, out var normal, out var normalIsSimplexNormal);

            AddDebugStep(ref simplex, steps);

            for (int i = 0; i < maximumIterations; ++i)
            {
                FindSupport(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, normal, out var support);
                Vector3Wide.Dot(support, normal, out var depth);

                Vector3Wide.Dot(simplex.A.Support, normal, out var aDotN);
                var noProgressMade = Vector.BitwiseAnd(normalIsSimplexNormal, Vector.LessThan(aDotN, surfaceThreshold));

                Add(ref simplex, support, normal, depth);

                //If all lanes are sufficiently separated, then we can early out.
                depthBelowThreshold = Vector.LessThan(simplex.A.Depth, minimumDepthThreshold);
                terminatedLanes = Vector.BitwiseOr(depthBelowThreshold, terminatedLanes);
                if (Vector.LessThanAll(terminatedLanes, Vector<int>.Zero))
                    break;

                GetNextNormal(ref simplex, localOffsetB, terminatedLanes, out normal, out normalIsSimplexNormal);

                

                
                
                AddDebugStep(ref simplex, steps);

      
            }
            refinedNormal = simplex.A.Normal;
            refinedDepth = simplex.A.Depth;
        }
    }
}
