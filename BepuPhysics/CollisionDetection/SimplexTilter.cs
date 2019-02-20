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

    public struct SimplexTilterVertex
    {
        public Vector3 Support;
        public Vector3 Normal;
    }

    public struct SimplexTilterStep
    {
        public NotGJKVertex A;
        public NotGJKVertex B;
        public NotGJKVertex C;
        public NotGJKVertex D;
        public bool EdgeCase;
        public Vector3 ClosestPointOnTriangleToOrigin;
        public Vector3 EdgeOffset;
        public Vector3 NextNormal;
        public float BestDepth;
        public Vector3 BestNormal;
    }


    public static class SimplexTilter<TShapeA, TShapeWideA, TSupportFinderA, TShapeB, TShapeWideB, TSupportFinderB>
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
        static void FillSlot(ref Vertex vertex, in Vector3Wide support, in Vector3Wide normal)
        {
            //Note that this always fills empty slots. That's important- we avoid figuring out what subsimplex is active
            //and instead just treat it as a degenerate simplex with some duplicates. (Shares code with the actual degenerate path.)
            Vector3Wide.ConditionalSelect(vertex.Exists, vertex.Support, support, out vertex.Support);
            Vector3Wide.ConditionalSelect(vertex.Exists, vertex.Normal, normal, out vertex.Normal);
            vertex.Exists = new Vector<int>(-1);
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
            //While only one slot is actually full, GetNextNormal expects every slot to have some kind of data-
            //for those slots which are not yet filled, it should be duplicates of other data.
            //(The sub-triangle case is treated the same as the degenerate case.)
            simplex.A.Support = support;
            simplex.B.Support = support;
            simplex.C.Support = support;
            simplex.A.Normal = normal;
            simplex.B.Normal = normal;
            simplex.C.Normal = normal;
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
        static void GetNormalForEdge(in Vector3Wide edgeStart, in Vector3Wide edgeOffset, out Vector3Wide normal, out Vector<float> distanceSquared)
        {
        }

        struct HasNewSupport { }
        struct HasNoNewSupport { }
        static void GetNextNormal<T>(ref Simplex simplex, in Vector3Wide localOffsetB, in Vector3Wide support, in Vector3Wide normal, in Vector<int> terminatedLanes, out Vector3Wide nextNormal, out SimplexTilterStep step)
        {
            //Try to add the new support to the simplex. A couple of cases:
            //1) The simplex is not yet a triangle, or the triangle does not yet contain the projected origin.
            //Choose the next sample location by detecting which simplex feature is closest to the origin and tilting in that direction.
            //Simplex entries unrelated to the nearest feature are discarded.
            //2) The simplex is a triangle and contains the projected origin. Choose the triangle's normal as the next sample direction.

            //If the simplex is already a triangle and there's a new support, that means the previous iteration created a sample direction from 
            //a triangle normal (no vertices were discarded). There are now three subtriangles- ABD, BCD, and CAD.
            //Choose one and let it fall through to the normal case.


            if (typeof(T) == typeof(HasNewSupport))
            {
                var simplexFull = Vector.BitwiseAnd(simplex.A.Exists, Vector.BitwiseAnd(simplex.B.Exists, simplex.C.Exists));
                //Fill any empty slots with the new support. Combines partial simplex case with degenerate simplex case.
                FillSlot(ref simplex.A, support, normal);
                FillSlot(ref simplex.B, support, normal);
                FillSlot(ref simplex.C, support, normal);

                var activeFullSimplex = Vector.AndNot(simplexFull, terminatedLanes);
                if (Vector.LessThanAny(activeFullSimplex, Vector<int>.Zero))
                {
                    //At least one active lane has a full simplex and an incoming new sample.
                    //We need to choose one of the three existing vertices to replace with the new sample.
                    //Pick by projecting the new support D onto the simplex ABC's plane, creating 3 edge planes between the triangles:
                    //abcN x AD, abcN x BD, and abcN x CD
                    //Detecting which side of the AD plane the origin is on:
                    //dot(origin - D, abcN x AD) = dot(D, AD x abcN) = dot(AD, abcN x D)
                    //So we can perform all three plane tests with only two cross products and three dots.
                    Vector3Wide.Subtract(simplex.B.Support, simplex.A.Support, out var edgeAB);
                    Vector3Wide.Subtract(simplex.A.Support, simplex.C.Support, out var edgeCA);
                    Vector3Wide.CrossWithoutOverlap(edgeAB, edgeCA, out var abcN);
                    Vector3Wide.CrossWithoutOverlap(abcN, support, out var abcNxD);
                    Vector3Wide.Subtract(support, simplex.A.Support, out var ad);
                    Vector3Wide.Subtract(support, simplex.B.Support, out var bd);
                    Vector3Wide.Subtract(support, simplex.C.Support, out var cd);
                    Vector3Wide.Dot(abcNxD, ad, out var adPlaneTest);
                    Vector3Wide.Dot(abcNxD, bd, out var bdPlaneTest);
                    Vector3Wide.Dot(abcNxD, cd, out var cdPlaneTest);

                    //Note that we're not concerned about abcN being zero. In order for this codepath to be relevant, the previous 
                    //iteration must have determined the projected origin was contained by ABC. A degenerate simplex would not have contained the origin.
                    //ad/bd/cd projected onto ABC's plane may also be zero length. That will pull the resulting plane test toward 0 and doesn't actually
                    //cause any issues- in order to be chosen, a triangle must pass two edge planes. If ABC is not degenerate, then only one edge plane
                    //will be degenerate at any one time, and there will be one subtriangle with two nondegenerate edge planes).
                    var useABD = Vector.BitwiseAnd(Vector.GreaterThanOrEqual(adPlaneTest, Vector<float>.Zero), Vector.LessThan(bdPlaneTest, Vector<float>.Zero));
                    var useBCD = Vector.BitwiseAnd(Vector.GreaterThanOrEqual(bdPlaneTest, Vector<float>.Zero), Vector.LessThan(cdPlaneTest, Vector<float>.Zero));
                    var useCAD = Vector.BitwiseAnd(Vector.GreaterThanOrEqual(cdPlaneTest, Vector<float>.Zero), Vector.LessThan(adPlaneTest, Vector<float>.Zero));

                    ForceFillSlot(Vector.BitwiseAnd(useBCD, simplexFull), ref simplex.A, support, normal);
                    ForceFillSlot(Vector.BitwiseAnd(useCAD, simplexFull), ref simplex.B, support, normal);
                    ForceFillSlot(Vector.BitwiseAnd(useABD, simplexFull), ref simplex.C, support, normal);
                }
            }
            else
            {
                FillSlot(ref simplex.A, simplex.A.Support, simplex.A.Normal);
                FillSlot(ref simplex.B, simplex.A.Support, simplex.A.Normal);
                FillSlot(ref simplex.C, simplex.A.Support, simplex.A.Normal);
            }
            //By now, we have a (potentially degenerate) simplex of 3 vertices.
            //Three possible cases to consider:
            //1) The triangle is nondegenerate and contains the origin. Containment determined by edge plane tests.
            //2) The triangle is nondegenerate and does not contain the origin. Edge to test determined by edge plane tests.
            //3) The triangle is degenerate. Edge to test determined by longest edge length.
            Vector3Wide.Subtract(simplex.B.Support, simplex.A.Support, out var ab);
            Vector3Wide.Subtract(simplex.A.Support, simplex.C.Support, out var ca);
            Vector3Wide.CrossWithoutOverlap(ab, ca, out var triangleNormal);
            Vector3Wide.LengthSquared(triangleNormal, out var triangleNormalLengthSquared);
            //Compute the plane sign tests. Note that these are barycentric weights that have not been scaled by the inverse triangle normal length squared;
            //we do not have to compute the correct magnitude to know the sign, and the sign is all we care about.
            Vector3Wide.CrossWithoutOverlap(ab, simplex.A.Support, out var abxa);
            Vector3Wide.CrossWithoutOverlap(ca, simplex.B.Support, out var caxc);
            Vector3Wide.Dot(abxa, triangleNormal, out var abPlaneTest);
            Vector3Wide.Dot(caxc, triangleNormal, out var caPlaneTest);
            var bcPlaneTest = triangleNormalLengthSquared - caPlaneTest - abPlaneTest;
            //Until informed otherwise, assume the origin is on the simplex face.
            nextNormal = triangleNormal;

            Vector3Wide.Subtract(simplex.C.Support, simplex.B.Support, out var bc);
            Vector3Wide.LengthSquared(ab, out var abLengthSquared);
            Vector3Wide.LengthSquared(bc, out var bcLengthSquared);
            Vector3Wide.LengthSquared(ca, out var caLengthSquared);
            var abLongest = Vector.BitwiseAnd(Vector.GreaterThan(abLengthSquared, bcLengthSquared), Vector.GreaterThan(abLengthSquared, caLengthSquared));
            var bcLongest = Vector.AndNot(Vector.GreaterThan(bcLengthSquared, caLengthSquared), abLongest);
            var caLongest = Vector.AndNot(Vector.OnesComplement(abLongest), bcLongest);
            //Area is proportional width * length. Using the longest length squared gives us an estimate for the area.
            //The triangle normal length squared is also proportional to area, so it's a good way to threshold degeneracy.
            //TODO: May want to use external epsilon to provide the scale or some lower bound.
            var longestEdgeLengthSquared = Vector.ConditionalSelect(abLongest, abLengthSquared, Vector.ConditionalSelect(bcLongest, bcLengthSquared, caLengthSquared));
            var simplexDegenerate = Vector.LessThanOrEqual(triangleNormalLengthSquared, longestEdgeLengthSquared * 1e-5f);

            var outsideAB = Vector.LessThan(abPlaneTest, Vector<float>.Zero);
            var outsideBC = Vector.LessThan(bcPlaneTest, Vector<float>.Zero);
            var outsideCA = Vector.LessThan(caPlaneTest, Vector<float>.Zero);
            var projectedOriginOutsideTriangle = Vector.BitwiseOr(outsideAB, Vector.BitwiseOr(outsideBC, outsideCA));

            var activeEdgeLane = Vector.AndNot(Vector.BitwiseOr(simplexDegenerate, projectedOriginOutsideTriangle), terminatedLanes);
            if (Vector.LessThanAny(activeEdgeLane, Vector<int>.Zero))
            {
                //At least one lane needs to perform an edge test.
                var testAB = Vector.BitwiseOr(Vector.AndNot(outsideAB, simplexDegenerate), Vector.BitwiseAnd(abLongest, simplexDegenerate));
                var testBC = Vector.BitwiseOr(Vector.AndNot(outsideBC, simplexDegenerate), Vector.BitwiseAnd(bcLongest, simplexDegenerate));

                Vector3Wide edgeStart, edgeOffset, normalStart, normalEnd;
                edgeStart.X = Vector.ConditionalSelect(testAB, simplex.A.Support.X, Vector.ConditionalSelect(testBC, simplex.B.Support.X, simplex.C.Support.X));
                edgeStart.Y = Vector.ConditionalSelect(testAB, simplex.A.Support.Y, Vector.ConditionalSelect(testBC, simplex.B.Support.Y, simplex.C.Support.Y));
                edgeStart.Z = Vector.ConditionalSelect(testAB, simplex.A.Support.Z, Vector.ConditionalSelect(testBC, simplex.B.Support.Z, simplex.C.Support.Z));
                edgeOffset.X = Vector.ConditionalSelect(testAB, ab.X, Vector.ConditionalSelect(testBC, bc.X, ca.X));
                edgeOffset.Y = Vector.ConditionalSelect(testAB, ab.Y, Vector.ConditionalSelect(testBC, bc.Y, ca.Y));
                edgeOffset.Z = Vector.ConditionalSelect(testAB, ab.Z, Vector.ConditionalSelect(testBC, bc.Z, ca.Z));
                normalStart.X = Vector.ConditionalSelect(testAB, simplex.A.Normal.X, Vector.ConditionalSelect(testBC, simplex.B.Normal.X, simplex.C.Normal.X));
                normalStart.Y = Vector.ConditionalSelect(testAB, simplex.A.Normal.Y, Vector.ConditionalSelect(testBC, simplex.B.Normal.Y, simplex.C.Normal.Y));
                normalStart.Z = Vector.ConditionalSelect(testAB, simplex.A.Normal.Z, Vector.ConditionalSelect(testBC, simplex.B.Normal.Z, simplex.C.Normal.Z));
                normalEnd.X = Vector.ConditionalSelect(testAB, simplex.B.Normal.X, Vector.ConditionalSelect(testBC, simplex.C.Normal.X, simplex.A.Normal.X));
                normalEnd.Y = Vector.ConditionalSelect(testAB, simplex.B.Normal.Y, Vector.ConditionalSelect(testBC, simplex.C.Normal.Y, simplex.A.Normal.Y));
                normalEnd.Z = Vector.ConditionalSelect(testAB, simplex.B.Normal.Z, Vector.ConditionalSelect(testBC, simplex.C.Normal.Z, simplex.A.Normal.Z));

                //TODO: If all lanes are guaranteed to be in tilt-only mode, you could branch into a divisionless version.
                //For example, if in AB's voronoi region, nextNormal = abcN x AB.
                //Using a closest point on edge implementation just shares a codepath across edge/vertex voronoi regions and tilt/GJK-style normal picking.
                Vector3Wide.LengthSquared(edgeOffset, out var edgeOffsetLengthSquared);
                //dot(origin - edgeStart, edgeOffset / ||edgeOffset||) * edgeOffset / ||edgeOffset||
                Vector3Wide.Dot(edgeStart, edgeOffset, out var dot);
                //TODO: Approximate rcp would be sufficient here.
                var t = -dot / edgeOffsetLengthSquared;
                t = Vector.Min(Vector<float>.Zero, Vector.Max(new Vector<float>(-1f), t));
                //Protecting against division by zero. TODO: Don't think there's a way to guarantee min/max NaN behavior across hardware to avoid this? Doing RCP first for inf...
                t = Vector.ConditionalSelect(Vector.LessThan(edgeOffsetLengthSquared, new Vector<float>(1e-14f)), Vector<float>.Zero, t);
                Vector3Wide.Scale(edgeOffset, t, out var interpolatedOffset);
                Vector3Wide.Add(interpolatedOffset, edgeStart, out var edgeNormal);
                Vector3Wide.LengthSquared(normal, out var distanceSquared);
                var edgeNormalValid = Vector.GreaterThan(distanceSquared, new Vector<float>(1e-15f));
                //If the origin is right on top of the edge or if the triangle isn't degenerate, use the triangle normal as a starting point.
                //If the triangle is also degenerate, use the interpolated normal as a starting point.
                if (Vector.LessThanAny(Vector.AndNot(Vector.BitwiseAnd(simplexDegenerate, activeEdgeLane), edgeNormalValid), Vector<int>.Zero))
                {

                }

                //Tilt the simplex toward the origin.
                //If the origin was closest to the inside of the line segment:
                Vector3Wide.Scale(normalStart, Vector<float>.One - t, out var weightedStart);
                Vector3Wide.Scale(normalEnd, t, out var weightedEnd);
                Vector3Wide.Add(weightedStart, weightedEnd, out var interpolatedNormal);
            }
            step = default;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void FindMinimumDepth(in TShapeWideA a, in TShapeWideB b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB, ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB,
    in Vector3Wide initialNormal, in Vector<int> inactiveLanes, in Vector<float> convergenceThreshold, in Vector<float> minimumDepthThreshold, out Vector<float> depth, out Vector3Wide refinedNormal, List<SimplexTilterStep> steps, int maximumIterations = 50)
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

        public static void FindMinimumDepth(in TShapeWideA a, in TShapeWideB b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB, ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB,
            ref Simplex simplex, in Vector<float> initialDepth,
            in Vector<int> inactiveLanes, in Vector<float> surfaceThreshold, in Vector<float> minimumDepthThreshold, out Vector<float> refinedDepth, out Vector3Wide refinedNormal, List<SimplexTilterStep> steps, int maximumIterations = 50)
        {
#if DEBUG
            Vector3Wide.LengthSquared(simplex.A.Normal, out var initialNormalLengthSquared);
            Debug.Assert(Vector.LessThanAll(Vector.BitwiseOr(inactiveLanes, Vector.LessThan(Vector.Abs(initialNormalLengthSquared - Vector<float>.One), new Vector<float>(1e-6f))), Vector<int>.Zero));
#endif            
            var depthBelowThreshold = Vector.LessThan(initialDepth, minimumDepthThreshold);
            var terminatedLanes = Vector.BitwiseOr(depthBelowThreshold, inactiveLanes);

            refinedNormal = simplex.A.Normal;
            refinedDepth = initialDepth;
            if (Vector.LessThanAll(terminatedLanes, Vector<int>.Zero))
            {
                return;
            }

            GetNextNormal<HasNoNewSupport>(ref simplex, localOffsetB, default, default, terminatedLanes, out var normal, out var debugStep);
            debugStep.BestDepth = refinedDepth[0];
            Vector3Wide.ReadSlot(ref refinedNormal, 0, out debugStep.BestNormal);
            steps.Add(debugStep);

            for (int i = 0; i < maximumIterations; ++i)
            {
                FindSupport(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, normal, out var support);
                Vector3Wide.Dot(support, normal, out var depth);

                var useNewDepth = Vector.LessThan(depth, refinedDepth);
                refinedDepth = Vector.ConditionalSelect(useNewDepth, depth, refinedDepth);
                Vector3Wide.ConditionalSelect(useNewDepth, normal, refinedNormal, out refinedNormal);

                GetNextNormal<HasNewSupport>(ref simplex, localOffsetB, support, normal, terminatedLanes, out normal, out debugStep);
                debugStep.BestDepth = refinedDepth[0];
                Vector3Wide.ReadSlot(ref refinedNormal, 0, out debugStep.BestNormal);
                steps.Add(debugStep);
            }
        }
    }
}
