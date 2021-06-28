using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection.SweepTasks
{
    public struct GJKDistanceTester<TShapeA, TShapeWideA, TSupportFinderA, TShapeB, TShapeWideB, TSupportFinderB> : IPairDistanceTester<TShapeWideA, TShapeWideB>
        where TShapeA : unmanaged, IConvexShape where TShapeB : unmanaged, IConvexShape
        where TShapeWideA : unmanaged, IShapeWide<TShapeA> where TShapeWideB : unmanaged, IShapeWide<TShapeB>
        where TSupportFinderA : struct, ISupportFinder<TShapeA, TShapeWideA>
        where TSupportFinderB : struct, ISupportFinder<TShapeB, TShapeWideB>
    {
        public const float TerminationEpsilonDefault = 1e-7f;
        public const float ContainmentEpsilonDefault = 0.000316227766f;

        public float TerminationEpsilon;
        public float ContainmentEpsilon;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void SampleMinkowskiDifference(
            in TShapeWideA a, in Matrix3x3Wide rA, ref TSupportFinderA supportFinderA,
            in TShapeWideB b, in Matrix3x3Wide rB, ref TSupportFinderB supportFinderB, in Vector3Wide offsetB, in Vector3Wide direction,
            in Vector<int> terminatedLanes, out Vector3Wide supportA, out Vector3Wide support)
        {
            supportFinderA.ComputeSupport(a, rA, direction, terminatedLanes, out supportA);
            Vector3Wide.Negate(direction, out var negatedDirection);
            supportFinderB.ComputeSupport(b, rB, negatedDirection, terminatedLanes, out var supportB);
            Vector3Wide.Add(supportB, offsetB, out supportB);
            Vector3Wide.Subtract(supportA, supportB, out support);
        }

        struct Simplex
        {
            public Vector3Wide A;
            public Vector3Wide B;
            public Vector3Wide C;
            public Vector3Wide D;
            //Contributing points from one object are tracked to create the closest points.
            public Vector3Wide AOnA;
            public Vector3Wide BOnA;
            public Vector3Wide COnA;
            public Vector3Wide DOnA;
            public Vector<int> Count;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Append(ref Vector<int> mask, ref Simplex simplex, ref Vector3Wide vA, ref Vector3Wide v)
        {
            for (int i = 0; i < Vector<float>.Count; ++i)
            {
                if (mask[i] == 0)
                {
                    ref var count = ref Unsafe.Add(ref GatherScatter.GetFirst(ref simplex.Count), i);
                    ref var vTargetSlot = ref GatherScatter.GetOffsetInstance(ref Unsafe.Add(ref simplex.A, count), i);
                    GatherScatter.GetFirst(ref vTargetSlot.X) = v.X[i];
                    GatherScatter.GetFirst(ref vTargetSlot.Y) = v.Y[i];
                    GatherScatter.GetFirst(ref vTargetSlot.Z) = v.Z[i];
                    ref var vATargetSlot = ref GatherScatter.GetOffsetInstance(ref Unsafe.Add(ref simplex.AOnA, count), i);
                    GatherScatter.GetFirst(ref vATargetSlot.X) = vA.X[i];
                    GatherScatter.GetFirst(ref vATargetSlot.Y) = vA.Y[i];
                    GatherScatter.GetFirst(ref vATargetSlot.Z) = vA.Z[i];
                    ++count;
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Select(ref Vector<int> mask,
            ref Vector<float> distanceSquared, ref Vector3Wide closest, ref Vector3Wide closestA, ref Vector<int> featureId,
            in Vector<float> distanceSquaredCandidate, in Vector3Wide closestCandidate, in Vector3Wide closestACandidate, in Vector<int> featureIdCandidate)
        {
            var useCandidate = Vector.BitwiseAnd(mask, Vector.LessThan(distanceSquaredCandidate, distanceSquared));
            distanceSquared = Vector.ConditionalSelect(useCandidate, distanceSquaredCandidate, distanceSquared);
            Vector3Wide.ConditionalSelect(useCandidate, closestCandidate, closest, out closest);
            Vector3Wide.ConditionalSelect(useCandidate, closestACandidate, closestA, out closestA);
            featureId = Vector.ConditionalSelect(useCandidate, featureIdCandidate, featureId);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Edge(ref Vector3Wide a, ref Vector3Wide b, ref Vector3Wide aOnA, ref Vector3Wide bOnA, out Vector3Wide ab, out Vector<float> abab, out Vector<float> abA, in Vector<int> aFeatureId, in Vector<int> bFeatureId,
            ref Vector<int> mask, ref Vector<float> distanceSquared, ref Vector3Wide closest, ref Vector3Wide closestA, ref Vector<int> featureId)
        {
            Vector3Wide.Subtract(b, a, out ab);
            Vector3Wide.Dot(ab, ab, out abab);
            Vector3Wide.Dot(ab, a, out abA);
            //Note that vertex B (and technically A, too) is handled by clamping the edge. No need to handle the vertices by themselves (apart from the base case).
            var abT = -abA / abab;
            //Protect against division by zero. It's a degenerate edge, so just pick vertex a as a representative.
            abT = Vector.ConditionalSelect(Vector.GreaterThan(Vector.Abs(abab), new Vector<float>(1e-15f)), abT, Vector<float>.Zero);
            var aFeatureContribution = Vector.ConditionalSelect(Vector.LessThan(abT, Vector<float>.One), aFeatureId, Vector<int>.Zero);
            var bFeatureContribution = Vector.ConditionalSelect(Vector.GreaterThan(abT, Vector<float>.Zero), bFeatureId, Vector<int>.Zero);
            var featureIdCandidate = Vector.BitwiseOr(aFeatureContribution, bFeatureContribution);

            abT = Vector.Max(Vector<float>.Zero, Vector.Min(Vector<float>.One, abT));
            Vector3Wide.Scale(ab, abT, out var closestOnAB);
            Vector3Wide.Add(closestOnAB, a, out closestOnAB);
            Vector3Wide.Subtract(bOnA, aOnA, out var abOnA);
            Vector3Wide.Scale(abOnA, abT, out var closestOnABOnA);
            Vector3Wide.Add(closestOnABOnA, aOnA, out closestOnABOnA);
            Vector3Wide.LengthSquared(closestOnAB, out var distanceSquaredCandidate);
            Select(ref mask,
                ref distanceSquared, ref closest, ref closestA, ref featureId,
                distanceSquaredCandidate, closestOnAB, closestOnABOnA, featureIdCandidate);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void TryRemove(ref Simplex simplex, int index, in Vector<int> featureId, in Vector<int> activeMask)
        {
            var shouldRemove = Vector.BitwiseAnd(activeMask, Vector.Equals(Vector.BitwiseAnd(featureId, new Vector<int>(1 << index)), Vector<int>.Zero));
            var lastSlot = simplex.Count - Vector<int>.One;
            simplex.Count = Vector.ConditionalSelect(shouldRemove, lastSlot, simplex.Count);
            var shouldPullLastSlot = Vector.BitwiseAnd(shouldRemove, Vector.LessThan(new Vector<int>(index), lastSlot));
            if (Vector.EqualsAny(shouldPullLastSlot, new Vector<int>(-1)))
            {
                ref var target = ref Unsafe.Add(ref simplex.A, index);
                ref var targetX = ref GatherScatter.GetFirst(ref target.X);
                ref var targetY = ref GatherScatter.GetFirst(ref target.Y);
                ref var targetZ = ref GatherScatter.GetFirst(ref target.Z);
                ref var targetOnA = ref Unsafe.Add(ref simplex.AOnA, index);
                ref var targetOnAX = ref GatherScatter.GetFirst(ref targetOnA.X);
                ref var targetOnAY = ref GatherScatter.GetFirst(ref targetOnA.Y);
                ref var targetOnAZ = ref GatherScatter.GetFirst(ref targetOnA.Z);
                for (int i = 0; i < Vector<int>.Count; ++i)
                {
                    if (shouldPullLastSlot[i] < 0)
                    {
                        ref var source = ref Unsafe.Add(ref simplex.A, lastSlot[i]);
                        Unsafe.Add(ref targetX, i) = source.X[i];
                        Unsafe.Add(ref targetY, i) = source.Y[i];
                        Unsafe.Add(ref targetZ, i) = source.Z[i];
                        ref var sourceOnA = ref Unsafe.Add(ref simplex.AOnA, lastSlot[i]);
                        Unsafe.Add(ref targetOnAX, i) = sourceOnA.X[i];
                        Unsafe.Add(ref targetOnAY, i) = sourceOnA.Y[i];
                        Unsafe.Add(ref targetOnAZ, i) = sourceOnA.Z[i];
                    }
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Triangle(
            ref Vector3Wide a, ref Vector3Wide b, ref Vector3Wide c,
            ref Vector3Wide aOnA, ref Vector3Wide bOnA, ref Vector3Wide cOnA,
            ref Vector<float> abA, ref Vector<float> acA,
            ref Vector3Wide ab, ref Vector3Wide ac,
            ref Vector<float> abab, ref Vector<float> acac,
            in Vector<int> featureIdCandidate,
            ref Vector<int> mask, ref Vector<float> distanceSquared, ref Vector3Wide closest, ref Vector3Wide closestA, ref Vector<int> featureId)
        {
            //Triangle distances are only accepted if the projected point is actually within the triangle. Note that this allows distances to be computed
            //for test point which is on the 'wrong' side of the plane in the tetrahedral case. That's fine, because the point is either inside 
            //(distance == 0, will be caught later in the tetrahedral case) or it's closer to a feature on the other side of the tetrahedron.
            //We have to test the three edge planes. We will do so by computing the barycentric coordinate of the point on the triangle.
            //The plane test portion, before scaling, looks like this:
            //(A * AB) * (AB * AC) - (A * AC) * (AB * AB) >= 0  (from (A x AB) * (AB x AC) >= 0)
            //(AC * AB) * (A * AC) - (AC * AC) * (A * AB) >= 0  (from (AC x A) * (AB x AC) >= 0)
            //The third edge plane test doesn't need be computed.
            //Note the cross product order and signs.

            //Instead of constructing the simplex closest point from barycentric weights, we'll cast a ray to the triangle along the triangle normal.
            //This is mathematically equivalent but numerically superior for face contacts- it guarantees that a closest point on the triangle's face
            //can never produce an offset that isn't aligned with the triangle normal.
            //(The triangle normal might still have some numerical issues, but it's far less severe than explicitly using the barycentric weights.)
            //N = ab x ac
            //closestPointOnTriangle = O + t * N
            //closestPointOnTriangle = O + N * dot(vertexA - O, N) / dot(N, N)
            //closestPointOnTriangle = N * dot(vertexA, N) / dot(N, N)
            Vector3Wide.CrossWithoutOverlap(ab, ac, out var n);
            Vector3Wide.LengthSquared(n, out var abxacLengthSquared);
            Vector3Wide.Dot(a, n, out var aN);
            var inverseNLengthSquared = Vector<float>.One / abxacLengthSquared;
            var t = aN * inverseNLengthSquared;
            Vector3Wide.Scale(n, t, out var closestCandidate);
            //Note that the above is not the fastest possible implementation. There are some shared terms that we're not taking advantage of for numerical reasons.
            //(But I bet there's still a clever way to be faster with equal or better numerical behavior!)
            //(But maximum performance improvement would be on the scale of 10% on the high end for GJK in isolation, so not exactly a big deal.)

            Vector3Wide.Dot(ab, ac, out var abac);
            var cWeight = (abA * abac - acA * abab) * inverseNLengthSquared;
            var bWeight = (abac * acA - acac * abA) * inverseNLengthSquared;
            var aWeight = Vector<float>.One - bWeight - cWeight;
            var projectionInTriangle = Vector.BitwiseAnd(
                Vector.BitwiseAnd(
                    Vector.GreaterThanOrEqual(aWeight, Vector<float>.Zero),
                    Vector.GreaterThanOrEqual(bWeight, Vector<float>.Zero)),
                Vector.GreaterThanOrEqual(cWeight, Vector<float>.Zero));

            Vector3Wide.Scale(aOnA, aWeight, out var aOnAContribution);
            Vector3Wide.Scale(bOnA, bWeight, out var bOnAContribution);
            Vector3Wide.Scale(cOnA, cWeight, out var cOnAContribution);
            Vector3Wide.Add(aOnAContribution, bOnAContribution, out var closestACandidate);
            Vector3Wide.Add(cOnAContribution, closestACandidate, out closestACandidate);
            Vector3Wide.LengthSquared(closestCandidate, out var distanceSquaredCandidate);
            var combinedMask = Vector.BitwiseAnd(mask, projectionInTriangle);
            Select(ref combinedMask,
                ref distanceSquared, ref closest, ref closestA, ref featureId,
                distanceSquaredCandidate, closestCandidate, closestACandidate, featureIdCandidate);
        }

        static void FindClosestPoint(ref Vector<int> outerLoopTerminatedMask, ref Simplex simplex, out Vector<float> distanceSquared, out Vector3Wide closestA, out Vector3Wide closest)
        {
            //The outer loop mask considers 0 to be executing, -1 to be terminated.
            var activeMask = Vector.OnesComplement(outerLoopTerminatedMask);
            //This function's job is to identify the simplex feature closest to the origin, and to compute the closest point on it.
            //While there are quite a few possible approaches here (direct solvers, enumerating feature distances, voronoi region testing), 
            //it's useful to keep in mind two details:
            //1) At the end of the function, the simplex should no longer contain any vertex which does not contribute to the closest point linear combination.
            //2) This is widely vectorized, so branching and divergence hurts even more than usual.

            //One option that fits reasonably well to both of these is to walk the simplex features up from point to tetrahedron, tracking distance and contributing vertices
            //as you go. Since every smaller feature is required by the next step up, there are no wasted calculations, and you can early out
            //if all SIMD lanes are below a given simplex size.

            //Vertex A:
            //Note that simplex sizes are always at least 1.
            Vector3Wide.LengthSquared(simplex.A, out distanceSquared);
            //Contributing vertices are tracked using a bitfield. A:1, B:2, C:4, D:8.
            var featureId = new Vector<int>(1);
            //For simplicity, we'll compute the closestA at each point and cache it if it is optimal.
            closest = simplex.A;
            closestA = simplex.AOnA;


            activeMask = Vector.BitwiseAnd(activeMask, Vector.GreaterThanOrEqual(simplex.Count, new Vector<int>(2)));
            if (Vector.EqualsAll(activeMask, Vector<int>.Zero))
            {
                //No possible reduction; simplices have at least one vertex.
                return;
            }

            //Edge AB:
            Edge(ref simplex.A, ref simplex.B, ref simplex.AOnA, ref simplex.BOnA, out var ab, out var abab, out var abA, new Vector<int>(1), new Vector<int>(2),
                ref activeMask, ref distanceSquared, ref closest, ref closestA, ref featureId);


            var nextActiveMask = Vector.BitwiseAnd(activeMask, Vector.GreaterThanOrEqual(simplex.Count, new Vector<int>(3)));
            if (Vector.EqualsAll(nextActiveMask, Vector<int>.Zero))
            {
                TryRemove(ref simplex, 1, featureId, activeMask);
                TryRemove(ref simplex, 0, featureId, activeMask);
                return;
            }
            activeMask = nextActiveMask;

            //Edge AC, BC:
            Edge(ref simplex.A, ref simplex.C, ref simplex.AOnA, ref simplex.COnA, out var ac, out var acac, out var acA, new Vector<int>(1), new Vector<int>(4),
                ref activeMask, ref distanceSquared, ref closest, ref closestA, ref featureId);
            Edge(ref simplex.B, ref simplex.C, ref simplex.BOnA, ref simplex.COnA, out var bc, out var bcbc, out var bcB, new Vector<int>(2), new Vector<int>(4),
                ref activeMask, ref distanceSquared, ref closest, ref closestA, ref featureId);

            //Triangle ABC:
            Triangle(
                ref simplex.A, ref simplex.B, ref simplex.C,
                ref simplex.AOnA, ref simplex.BOnA, ref simplex.COnA,
                ref abA, ref acA, ref ab, ref ac, ref abab, ref acac, new Vector<int>(1 | 2 | 4),
                ref activeMask, ref distanceSquared, ref closest, ref closestA, ref featureId);

            nextActiveMask = Vector.BitwiseAnd(activeMask, Vector.GreaterThanOrEqual(simplex.Count, new Vector<int>(4)));
            if (Vector.EqualsAll(nextActiveMask, Vector<int>.Zero))
            {
                TryRemove(ref simplex, 2, featureId, activeMask);
                TryRemove(ref simplex, 1, featureId, activeMask);
                TryRemove(ref simplex, 0, featureId, activeMask);
                return;
            }
            activeMask = nextActiveMask;

            //Edges AD, BD, CD:
            Edge(ref simplex.A, ref simplex.D, ref simplex.AOnA, ref simplex.DOnA, out var ad, out var adad, out var adA, new Vector<int>(1), new Vector<int>(8),
                ref activeMask, ref distanceSquared, ref closest, ref closestA, ref featureId);
            Edge(ref simplex.B, ref simplex.D, ref simplex.BOnA, ref simplex.DOnA, out var bd, out var bdbd, out var bdB, new Vector<int>(2), new Vector<int>(8),
                ref activeMask, ref distanceSquared, ref closest, ref closestA, ref featureId);
            Edge(ref simplex.C, ref simplex.D, ref simplex.COnA, ref simplex.DOnA, out var cd, out var cdcd, out var cdC, new Vector<int>(4), new Vector<int>(8),
                ref activeMask, ref distanceSquared, ref closest, ref closestA, ref featureId);

            //Triangle ACD:    
            Triangle(
                ref simplex.A, ref simplex.C, ref simplex.D,
                ref simplex.AOnA, ref simplex.COnA, ref simplex.DOnA,
                ref acA, ref adA, ref ac, ref ad, ref acac, ref adad, new Vector<int>(1 | 4 | 8),
                ref activeMask, ref distanceSquared, ref closest, ref closestA, ref featureId);

            //Triangle ABD:    
            Triangle(
                ref simplex.A, ref simplex.B, ref simplex.D,
                ref simplex.AOnA, ref simplex.BOnA, ref simplex.DOnA,
                ref abA, ref adA, ref ab, ref ad, ref abab, ref adad, new Vector<int>(1 | 2 | 8),
                ref activeMask, ref distanceSquared, ref closest, ref closestA, ref featureId);

            //Triangle BCD:    
            Triangle(
                ref simplex.B, ref simplex.C, ref simplex.D,
                ref simplex.BOnA, ref simplex.COnA, ref simplex.DOnA,
                ref bcB, ref bdB, ref bc, ref bd, ref bcbc, ref bdbd, new Vector<int>(2 | 4 | 8),
                ref activeMask, ref distanceSquared, ref closest, ref closestA, ref featureId);

            //Tetrahedron:
            //Test the plane of each triangle against the origin.
            //Calibrate based on winding: the origin should be on the same side of ABC from D.
            //Calibration: AD * (BC x AB) >= 0
            //ABC: -A * (BC x AB) >= 0
            //ABD: -A * (AD x BD) >= 0
            //ACD: -A * (CD x AC) >= 0
            //BCD: -B * (BD x CD) >= 0
            Vector3Wide.CrossWithoutOverlap(bc, ab, out var nabc);
            Vector3Wide.CrossWithoutOverlap(ad, bd, out var nabd);
            Vector3Wide.CrossWithoutOverlap(cd, ac, out var nacd);
            Vector3Wide.CrossWithoutOverlap(bd, cd, out var nbdc);
            Vector3Wide.Dot(ad, nabc, out var calibrationDotABC);
            var flipRequired = Vector.GreaterThanOrEqual(calibrationDotABC, Vector<float>.Zero);
            Vector3Wide.Dot(nabc, simplex.A, out var abcDot);
            Vector3Wide.Dot(nabd, simplex.A, out var abdDot);
            Vector3Wide.Dot(nacd, simplex.A, out var acdDot);
            Vector3Wide.Dot(nbdc, simplex.B, out var bdcDot);
            var abcInside = Vector.Xor(Vector.GreaterThanOrEqual(abcDot, Vector<float>.Zero), flipRequired);
            var abdInside = Vector.Xor(Vector.GreaterThanOrEqual(abdDot, Vector<float>.Zero), flipRequired);
            var acdInside = Vector.Xor(Vector.GreaterThanOrEqual(acdDot, Vector<float>.Zero), flipRequired);
            var bdcInside = Vector.Xor(Vector.GreaterThanOrEqual(bdcDot, Vector<float>.Zero), flipRequired);
            var tetrahedronContains = Vector.BitwiseAnd(Vector.BitwiseAnd(abcInside, abdInside), Vector.BitwiseAnd(acdInside, bdcInside));
            var useTetrahedralResult = Vector.BitwiseAnd(tetrahedronContains, activeMask);
            //Note that we don't guarantee correct closest points in the intersecting case, so there's no need to blend with barycentric weights.
            Select(ref useTetrahedralResult, ref distanceSquared, ref closest, ref closestA, ref featureId, Vector<float>.Zero, new Vector3Wide(), new Vector3Wide(), new Vector<int>(1 | 2 | 4 | 8));

            TryRemove(ref simplex, 3, featureId, activeMask);
            TryRemove(ref simplex, 2, featureId, activeMask);
            TryRemove(ref simplex, 1, featureId, activeMask);
            TryRemove(ref simplex, 0, featureId, activeMask);

        }


        public void Test(in TShapeWideA a, in TShapeWideB b, in Vector3Wide offsetB, in QuaternionWide orientationA, in QuaternionWide orientationB, in Vector<int> inactiveLanes,
            out Vector<int> intersected, out Vector<float> distance, out Vector3Wide closestA, out Vector3Wide normal)
        {
            Unsafe.SkipInit(out closestA);
            Unsafe.SkipInit(out normal);
            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var rA);
            Matrix3x3Wide.CreateFromQuaternion(orientationB, out var rB);
            var supportFinderA = default(TSupportFinderA);
            var supportFinderB = default(TSupportFinderB);
            Unsafe.SkipInit(out Simplex simplex);
            //TODO: It would be pretty easy to initialize to a triangle or tetrahedron. Might be worth it.
            SampleMinkowskiDifference(a, rA, ref supportFinderA, b, rB, ref supportFinderB, offsetB, offsetB, inactiveLanes, out simplex.AOnA, out simplex.A);
            simplex.Count = new Vector<int>(1);

            //GJK is a pretty branchy algorithm that doesn't map perfectly to widely vectorized implementations. We'll be using an SPMD model- if any SIMD lane needs to continue 
            //working, all lanes continue to work, and the unnecessary lane results are discarded.
            //In the context of the engine, distance tests are often quite coherent (by virtue of being used with sweeps), so we shouldn't often see pathologically low occupancy.
            var terminatedMask = inactiveLanes;
            intersected = Vector<int>.Zero;
            int iterationCount = 0;
            //Note that we use hardcoded defaults if this is default constructed.
            var epsilon = new Vector<float>(TerminationEpsilon > 0 ? TerminationEpsilon : TerminationEpsilonDefault);
            var containmentEpsilon = new Vector<float>(ContainmentEpsilon > 0 ? ContainmentEpsilon : ContainmentEpsilonDefault);
            //Use the JIT's ability to optimize away branches with generic type knowledge to set appropriate containment epsilons for shapes that have actual margins.
            if (supportFinderA.HasMargin && supportFinderB.HasMargin)
            {
                supportFinderA.GetMargin(a, out var marginA);
                supportFinderB.GetMargin(b, out var marginB);
                containmentEpsilon = Vector.Max(containmentEpsilon, marginA + marginB);
            }
            else if (supportFinderA.HasMargin)
            {
                supportFinderA.GetMargin(a, out var marginA);
                containmentEpsilon = Vector.Max(containmentEpsilon, marginA);
            }
            else if (supportFinderB.HasMargin)
            {
                supportFinderB.GetMargin(b, out var marginB);
                containmentEpsilon = Vector.Max(containmentEpsilon, marginB);
            }
            var containmentEpsilonSquared = containmentEpsilon * containmentEpsilon;
            var distanceSquared = new Vector<float>(float.MaxValue);
            while (true)
            {
                ++iterationCount;
                FindClosestPoint(ref terminatedMask, ref simplex, out var newDistanceSquared, out var simplexClosestA, out var simplexClosest);

                //For any lane that has not yet terminated, and which has an intersecting simplex, terminate with intersection.
                //Note hardcoded fixed epsilon: this isn't ideal, but normals become unreliable at scales below this threshold.
                //TODO: May want to make it configurable for users who don't care about normal accuracy and need smaller scale support.
                //Sweep tests are a difficult spot for configuration.
                var simplexContainsOrigin = Vector.LessThanOrEqual(newDistanceSquared, containmentEpsilonSquared);
                intersected = Vector.BitwiseOr(intersected, Vector.AndNot(simplexContainsOrigin, terminatedMask));
                terminatedMask = Vector.BitwiseOr(terminatedMask, simplexContainsOrigin);
                if (Vector.EqualsAll(terminatedMask, -Vector<int>.One))
                    break;

                //This is similar to the v vs simplexClosest test later, except this termination condition is immune to cycles.
                //It's used in conjunction with the later test because it is less aggressive- typically, the epsilon test will save an iteration.
                //This allows a more aggressive fallback termination compared to a fixed maximum iteration count without affecting the termination result.
                var noProgressMade = Vector.GreaterThanOrEqual(newDistanceSquared, distanceSquared);
                distanceSquared = Vector.Min(distanceSquared, newDistanceSquared);
                var aboutToTerminate = Vector.AndNot(noProgressMade, terminatedMask);
                Vector3Wide.ConditionalSelect(aboutToTerminate, simplexClosestA, closestA, out closestA);
                Vector3Wide.ConditionalSelect(aboutToTerminate, simplexClosest, normal, out normal);
                terminatedMask = Vector.BitwiseOr(terminatedMask, aboutToTerminate);
                if (Vector.EqualsAll(terminatedMask, -Vector<int>.One))
                    break;

                Vector3Wide.Negate(simplexClosest, out var sampleDirection);
                SampleMinkowskiDifference(a, rA, ref supportFinderA, b, rB, ref supportFinderB, offsetB, sampleDirection, terminatedMask, out var vA, out var v);

                //If the newly sampled is no better than the simplex-identified closest point, the lane is done.
                Vector3Wide.Subtract(v, simplexClosest, out var progressOffset);
                Vector3Wide.Dot(progressOffset, sampleDirection, out var progress);
                //The epsilon against which the progress is measured is scaled by the largest simplex vertex squared distance from the origin.
                //So we have shouldTerminate = (v - simplexClosest) * (-simplexClosest) <= max(a*a, b*b, c*c, d*d) * epsilon
                //This helps avoid numerically unnecessary iterations by accepting results which are near the limit of representable values.
                Vector3Wide.LengthSquared(simplex.A, out var aSquared);
                Vector3Wide.LengthSquared(simplex.B, out var bSquared);
                Vector3Wide.LengthSquared(simplex.C, out var cSquared);
                Vector3Wide.LengthSquared(simplex.D, out var dSquared);
                var max = Vector.ConditionalSelect(Vector.BitwiseAnd(Vector.GreaterThan(simplex.Count, new Vector<int>(1)), Vector.GreaterThan(bSquared, aSquared)), bSquared, aSquared);
                max = Vector.ConditionalSelect(Vector.BitwiseAnd(Vector.GreaterThan(simplex.Count, new Vector<int>(2)), Vector.GreaterThan(cSquared, max)), cSquared, max);
                max = Vector.ConditionalSelect(Vector.BitwiseAnd(Vector.GreaterThan(simplex.Count, new Vector<int>(3)), Vector.GreaterThan(dSquared, max)), dSquared, max);
                noProgressMade = Vector.LessThanOrEqual(progress, max * epsilon);
                //Any still-executing lane that failed to make progress is now verified to be nonintersecting.
                //Note that the default initialized state is nonintersecting, so we don't have to set the flag explicitly here.
                //Note that lanes which previously terminated are excluded from closest/normal updates.      
                aboutToTerminate = Vector.AndNot(noProgressMade, terminatedMask);
                Vector3Wide.ConditionalSelect(aboutToTerminate, simplexClosestA, closestA, out closestA);
                Vector3Wide.ConditionalSelect(aboutToTerminate, simplexClosest, normal, out normal);
                terminatedMask = Vector.BitwiseOr(terminatedMask, aboutToTerminate);
                if (Vector.EqualsAll(terminatedMask, -Vector<int>.One))
                    break;

                Append(ref terminatedMask, ref simplex, ref vA, ref v);
            }
            //Console.WriteLine($"Iteration count: {iterationCount}");
            //The normal gathered during the loop was the unnormalized offset.
            //Note that this normalization will cover lanes which weren't separated- that's fine. Valid lanes are described by the intersected flag.
            distance = Vector.SquareRoot(distanceSquared);
            var inverseDistance = Vector<float>.One / distance;
            //The containment epsilon acts as a margin around shapes that guarantees that normals remain numerically valid.
            //To avoid a case where the distance goes from near epsilon to intersecting in sub-epsilon motion, we report the distance as be adjusted by the containment epsilon. 
            //In other words, the shapes are spherically expanded a little. Not ideal for shapes that don't actually have margins, but not horrible.
            distance -= containmentEpsilon;
            Vector3Wide.Scale(normal, inverseDistance, out normal);

            //For shapes with substantial margins, it's important to compensate for the margin.
            Vector3Wide.Scale(normal, -containmentEpsilon, out var closestAMarginOffset);
            Vector3Wide.Add(closestAMarginOffset, closestA, out closestA);
        }
    }
}
