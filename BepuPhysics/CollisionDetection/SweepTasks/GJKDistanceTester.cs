using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.CollisionDetection.SweepTasks
{
    public interface ISupportFinder<TShape, TShapeWide> where TShape : IConvexShape where TShapeWide : IShapeWide<TShape>
    {
        void ComputeSupport(ref TShapeWide shape, ref Matrix3x3Wide orientation, ref Vector3Wide direction, out Vector3Wide support);
    }

    public struct GJKDistanceTester<TShapeA, TShapeWideA, TSupportFinderA, TShapeB, TShapeWideB, TSupportFinderB> : IPairDistanceTester<TShapeWideA, TShapeWideB>
        where TShapeA : IConvexShape where TShapeB : IConvexShape
        where TShapeWideA : IShapeWide<TShapeA> where TShapeWideB : IShapeWide<TShapeB>
        where TSupportFinderA : struct, ISupportFinder<TShapeA, TShapeWideA>
        where TSupportFinderB : struct, ISupportFinder<TShapeB, TShapeWideB>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void SampleMinkowskiDifference(
            ref TShapeWideA a, ref Matrix3x3Wide rA, ref TSupportFinderA supportFinderA,
            ref TShapeWideB b, ref Matrix3x3Wide rB, ref TSupportFinderB supportFinderB, ref Vector3Wide offsetB, ref Vector3Wide direction,
            out Vector3Wide supportA, out Vector3Wide support)
        {
            supportFinderA.ComputeSupport(ref a, ref rA, ref direction, out supportA);
            Vector3Wide.Negate(ref direction, out var negatedDirection);
            supportFinderA.ComputeSupport(ref a, ref rA, ref negatedDirection, out var supportB);
            Vector3Wide.Add(ref supportB, ref offsetB, out supportB);
            Vector3Wide.Subtract(ref supportA, ref supportB, out support);

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
        void Append(ref Vector<int> mask, ref Simplex simplex, ref Vector3Wide vA, ref Vector3Wide v)
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

        void FindClosestPoint(ref Simplex simplex, out Vector3Wide closestA, out Vector3Wide closest)
        {
            //This function's job is to identify the simplex feature closest to the origin, and to compute the closest point on it.
            //While there are quite a few possible approaches here (direct solvers, enumerating feature distances, voronoi region testing), 
            //it's useful to keep in mind two details:
            //1) At the end of the function, the simplex should no longer contain any vertex which does not contribute to the closest point linear combination.
            //2) This is widely vectorized, so branching and divergence hurts even more than usual.

            //One option that fits reasonably well to both of these is to walk the simplex cases from tetrahedron down to point.
            //At each simplex case, every vertex is tested for its ability to contribute to the final closest point linear combination.
            //For example, in a tetrahedron ABCD tested against point P, D is only relevant if P is in a region connected to D (ABCD, ABD, BCD, ACD, AD, BD, CD, or D).
            //So, test P against every voronoi region.

            //TETRAHEDRON
            Vector3Wide.Subtract(ref simplex.B, ref simplex.A, out var ab);
            Vector3Wide.Subtract(ref simplex.C, ref simplex.A, out var ac);
            Vector3Wide.Subtract(ref simplex.D, ref simplex.A, out var ad);
            Vector3Wide.Subtract(ref simplex.C, ref simplex.B, out var bc);
            Vector3Wide.Subtract(ref simplex.D, ref simplex.B, out var bd);
            Vector3Wide.Subtract(ref simplex.D, ref simplex.C, out var cd);

            //TODO: This does way more dot products than fundamentally required. It's just doing the most direct naive implementation. Could revisit.
            //(stuff like ab * ab = ab * (b - a) = ab * b - ab * a, or reformulating bounds to be nonzero and rely on previously computed result, and so on.)

            //Note that the test point is the origin, so the offset from a vertex to the test point is simply -vertex.
            //A: 
            //AB * A >= 0
            //AC * A >= 0
            //AD * A >= 0
            Vector3Wide.Dot(ref ab, ref simplex.A, out var abA);
            Vector3Wide.Dot(ref ac, ref simplex.A, out var acA);
            Vector3Wide.Dot(ref ad, ref simplex.A, out var adA);
            var aRequired = Vector.BitwiseAnd(
                Vector.GreaterThanOrEqual(abA, Vector<float>.Zero),
                Vector.BitwiseAnd(
                    Vector.GreaterThanOrEqual(acA, Vector<float>.Zero),
                    Vector.GreaterThanOrEqual(adA, Vector<float>.Zero)));
            //B: 
            //BA * B >= 0
            //BC * B >= 0
            //BD * B >= 0
            Vector3Wide.Dot(ref ab, ref simplex.B, out var abB);
            Vector3Wide.Dot(ref bc, ref simplex.B, out var bcB);
            Vector3Wide.Dot(ref bd, ref simplex.B, out var bdB);
            var bRequired = Vector.BitwiseAnd(
                Vector.LessThanOrEqual(abB, Vector<float>.Zero), //Note negation.
                Vector.BitwiseAnd(
                    Vector.GreaterThanOrEqual(bcB, Vector<float>.Zero),
                    Vector.GreaterThanOrEqual(bdB, Vector<float>.Zero)));
            //C: 
            //CA * C >= 0
            //CB * C >= 0
            //CD * C >= 0
            Vector3Wide.Dot(ref ac, ref simplex.C, out var acC);
            Vector3Wide.Dot(ref bc, ref simplex.C, out var bcC);
            Vector3Wide.Dot(ref cd, ref simplex.C, out var cdC);
            var cRequired = Vector.BitwiseAnd(
                Vector.LessThanOrEqual(acC, Vector<float>.Zero), //Note negation.
                Vector.BitwiseAnd(
                    Vector.LessThanOrEqual(bcC, Vector<float>.Zero),//Note negation.
                    Vector.GreaterThanOrEqual(cdC, Vector<float>.Zero)));
            //D: 
            //DA * D >= 0
            //DB * D >= 0
            //DC * D >= 0
            Vector3Wide.Dot(ref ad, ref simplex.D, out var adD);
            Vector3Wide.Dot(ref bd, ref simplex.D, out var bdD);
            Vector3Wide.Dot(ref cd, ref simplex.D, out var cdD);
            var dRequired = Vector.BitwiseAnd(
                Vector.LessThanOrEqual(adD, Vector<float>.Zero), //Note negation.
                Vector.BitwiseAnd(
                    Vector.LessThanOrEqual(bdD, Vector<float>.Zero), //Note negation.
                    Vector.LessThanOrEqual(cdD, Vector<float>.Zero))); //Note negation.

            //AB:
            //AB * A <= 0
            //BA * B <= 0
            //(A * AB) * (AB * AC) - (A * AC) * (AB * AB) <= 0 (from (A x AB) * (AB x AC) <= 0)
            //(A * AB) * (AB * AD) - (A * AD) * (AB * AB) <= 0 (from (A x AB) * (AB x AD) <= 0)
            //Note that these expressions can be derived from a more naive 'calibrated' plane test:
            //(-A * (AB x Nabc)) * (AC * (AB x Nabc)) <= 0
            //((AB x A) * Nabc) * ((AC x AB) * Nabc) <= 0
            //((AB x A) * (AB x AC) * ((AC x AB) * (AB x AC)) <= 0  //Note that the 'calibration' component is always negative because of how we defined Nabc.
            //(AB x A) * (AB x AC) >= 0 //Note that (W x X) * (Y x Z) = (W * Y) * (X * Z) - (W * Z) * (X * Y)
            //(AB * AB) * (A * AC) - (AB * AC) * (A * AB) >= 0
            //Which is equivalent.
            Vector3Wide.Dot(ref ab, ref ab, out var abab);
            Vector3Wide.Dot(ref ab, ref ac, out var abac);
            Vector3Wide.Dot(ref ab, ref ad, out var abad);
            var abABCPlaneTest = abA * abac - acA * abab;
            var abABDPlaneTest = abA * abad - adA * abab;
            var abActive = Vector.BitwiseAnd(
                Vector.BitwiseAnd(Vector.LessThanOrEqual(abABCPlaneTest, Vector<float>.Zero), Vector.LessThanOrEqual(abABDPlaneTest, Vector<float>.Zero)),
                Vector.BitwiseAnd(Vector.LessThanOrEqual(abA, Vector<float>.Zero), Vector.GreaterThanOrEqual(abB, Vector<float>.Zero)));
            aRequired = Vector.BitwiseOr(aRequired, abActive);
            bRequired = Vector.BitwiseOr(bRequired, abActive);

            //AC:
            //AC * A <= 0
            //CA * C <= 0
            //(A * AC) * (AC * AB) - (A * AB) * (AC * AC) <= 0 (from (A x AC) * (AC x AB) <= 0)
            //(A * AC) * (AC * AD) - (A * AD) * (AC * AC) <= 0 (from (A x AC) * (AC x AD) <= 0)
            Vector3Wide.Dot(ref ac, ref ad, out var acad);
            Vector3Wide.Dot(ref ac, ref ac, out var acac);
            var acABCPlaneTest = acA * abac - abA * acac;
            var acACDPlaneTest = acA * acad - adA * acac;
            var acActive = Vector.BitwiseAnd(
                Vector.BitwiseAnd(Vector.LessThanOrEqual(acABCPlaneTest, Vector<float>.Zero), Vector.LessThanOrEqual(acACDPlaneTest, Vector<float>.Zero)),
                Vector.BitwiseAnd(Vector.LessThanOrEqual(acA, Vector<float>.Zero), Vector.GreaterThanOrEqual(acC, Vector<float>.Zero)));
            aRequired = Vector.BitwiseOr(aRequired, acActive);
            cRequired = Vector.BitwiseOr(cRequired, acActive);

            //AD:
            //AD * A <= 0
            //DA * D <= 0
            //(A * AD) * (AD * AB) - (A * AB) * (AD * AD) <= 0 (from (A x AD) * (AD x AB) <= 0)
            //(A * AD) * (AD * AC) - (A * AC) * (AD * AD) <= 0 (from (A x AD) * (AD x AC) <= 0)
            Vector3Wide.Dot(ref ad, ref ad, out var adad);
            var adABDPlaneTest = adA * abad - abA * adad;
            var adACDPlaneTest = adA * acad - acA * adad;
            var adActive = Vector.BitwiseAnd(
                Vector.BitwiseAnd(Vector.LessThanOrEqual(adABDPlaneTest, Vector<float>.Zero), Vector.LessThanOrEqual(adACDPlaneTest, Vector<float>.Zero)),
                Vector.BitwiseAnd(Vector.LessThanOrEqual(adA, Vector<float>.Zero), Vector.GreaterThanOrEqual(adD, Vector<float>.Zero)));
            aRequired = Vector.BitwiseOr(aRequired, adActive);
            dRequired = Vector.BitwiseOr(dRequired, adActive);
            
            //BC:
            //BC * B <= 0
            //CB * C <= 0
            //(B * BC) * (BC * BA) - (B * BA) * (BC * BC) <= 0 (from (B x BC) * (BC x BA) <= 0)
            //(B * BC) * (BC * BD) - (B * BD) * (BC * BC) <= 0 (from (B x BC) * (BC x BD) <= 0)
            Vector3Wide.Dot(ref ab, ref bc, out var abbc);
            Vector3Wide.Dot(ref bc, ref bc, out var bcbc);
            Vector3Wide.Dot(ref bc, ref bd, out var bcbd);
            var bcABCPlaneTest = abB * bcbc - bcB * abbc;
            var bcBCDPlaneTest = bcB * bcbd - bdB * bcbc;
            var bcActive = Vector.BitwiseAnd(
                Vector.BitwiseAnd(Vector.LessThanOrEqual(bcABCPlaneTest, Vector<float>.Zero), Vector.LessThanOrEqual(bcBCDPlaneTest, Vector<float>.Zero)),
                Vector.BitwiseAnd(Vector.LessThanOrEqual(bcB, Vector<float>.Zero), Vector.GreaterThanOrEqual(bcC, Vector<float>.Zero)));
            bRequired = Vector.BitwiseOr(bRequired, bcActive);
            cRequired = Vector.BitwiseOr(cRequired, bcActive);

            //BD:
            //BD * B <= 0
            //DB * D <= 0
            //(B * BD) * (BD * BA) - (B * BA) * (BD * BD) <= 0 (from (B x BD) * (BD x BA) <= 0)
            //(B * BD) * (BD * BC) - (B * BC) * (BD * BD) <= 0 (from (B x BD) * (BD x BC) <= 0)
            Vector3Wide.Dot(ref ab, ref bd, out var abbd);
            Vector3Wide.Dot(ref bd, ref bd, out var bdbd);
            var bdABDPlaneTest = abB * bdbd - bdB * abbd;
            var bdBCDPlaneTest = bdB * bcbd - bcB * bdbd;
            var bdActive = Vector.BitwiseAnd(
                Vector.BitwiseAnd(Vector.LessThanOrEqual(bdABDPlaneTest, Vector<float>.Zero), Vector.LessThanOrEqual(bdBCDPlaneTest, Vector<float>.Zero)),
                Vector.BitwiseAnd(Vector.LessThanOrEqual(bdB, Vector<float>.Zero), Vector.GreaterThanOrEqual(bdD, Vector<float>.Zero)));
            bRequired = Vector.BitwiseOr(bRequired, bdActive);
            dRequired = Vector.BitwiseOr(dRequired, bdActive);

            //CD:
            //CD * C <= 0
            //DC * D <= 0
            //(C * CD) * (CD * CA) - (C * CA) * (CD * CD) <= 0 (from (C x CD) * (CD x CA) <= 0)
            //(C * CD) * (CD * CB) - (C * CB) * (CD * CD) <= 0 (from (C x CD) * (CD x CB) <= 0)
            Vector3Wide.Dot(ref cd, ref cd, out var cdcd);
            Vector3Wide.Dot(ref ac, ref cd, out var accd);
            Vector3Wide.Dot(ref bc, ref cd, out var bccd);
            var cdACDPlaneTest = acC * cdcd - cdC * accd;
            var cdBCDPlaneTest = bcC * cdcd - cdC * bccd;
            var cdActive = Vector.BitwiseAnd(
                Vector.BitwiseAnd(Vector.LessThanOrEqual(cdACDPlaneTest, Vector<float>.Zero), Vector.LessThanOrEqual(cdBCDPlaneTest, Vector<float>.Zero)),
                Vector.BitwiseAnd(Vector.LessThanOrEqual(cdC, Vector<float>.Zero), Vector.GreaterThanOrEqual(cdD, Vector<float>.Zero)));
            cRequired = Vector.BitwiseOr(cRequired, cdActive);
            dRequired = Vector.BitwiseOr(dRequired, cdActive);

            //Note that face tests are forced to use calibration due to the lack of consistent winding in the simplex. This could be improved,
            //but beware of adding overhead to guarantee winding- moving things around in slots introduces a lot of scalar work.
            //ABC:
            //Inside abABC, acABC, and bcABC planes.
            //(A * (AB x AC) * (AD * (AB x AC)) >= 0
            Vector3Wide.CrossWithoutOverlap(ref ab, ref ac, out var nabc);
            Vector3Wide.Dot(ref simplex.A, ref nabc, out var anabc);
            Vector3Wide.Dot(ref ad, ref nabc, out var adnabc);
            var outsideABC = Vector.GreaterThanOrEqual(anabc * adnabc, Vector<float>.Zero);
            var abcActive = Vector.BitwiseAnd(
                Vector.BitwiseAnd(
                    outsideABC,
                    Vector.GreaterThanOrEqual(abABCPlaneTest, Vector<float>.Zero)),
                Vector.BitwiseAnd(
                    Vector.GreaterThanOrEqual(acABCPlaneTest, Vector<float>.Zero),
                    Vector.GreaterThanOrEqual(bcABCPlaneTest, Vector<float>.Zero)));
            aRequired = Vector.BitwiseOr(abcActive, aRequired);
            bRequired = Vector.BitwiseOr(abcActive, bRequired);
            cRequired = Vector.BitwiseOr(abcActive, cRequired);

            //BCD:
            //Inside bcBCD, bdBCD, and cdBCD planes.     
            //(B * (BC x BD) * (BA * (BC x BD)) >= 0
            Vector3Wide.CrossWithoutOverlap(ref bc, ref bd, out var nbcd);
            Vector3Wide.Dot(ref simplex.B, ref nbcd, out var bnbcd);
            Vector3Wide.Dot(ref ab, ref nbcd, out var abnbcd);
            var outsideBCD = Vector.LessThanOrEqual(bnbcd * abnbcd, Vector<float>.Zero);
            var bcdActive = Vector.BitwiseAnd(
                Vector.BitwiseAnd(
                    outsideBCD,
                    Vector.GreaterThanOrEqual(bcBCDPlaneTest, Vector<float>.Zero)),
                Vector.BitwiseAnd(
                    Vector.GreaterThanOrEqual(bdBCDPlaneTest, Vector<float>.Zero),
                    Vector.GreaterThanOrEqual(cdBCDPlaneTest, Vector<float>.Zero)));
            bRequired = Vector.BitwiseOr(bcdActive, bRequired);
            cRequired = Vector.BitwiseOr(bcdActive, cRequired);
            dRequired = Vector.BitwiseOr(bcdActive, dRequired);

            //ACD:
            //Inside acACD, adACD, and cdACD planes.
            //(A * (AC x AD) * (AB * (AC x AD)) >= 0
            Vector3Wide.CrossWithoutOverlap(ref ac, ref ad, out var nacd);
            Vector3Wide.Dot(ref simplex.A, ref nacd, out var anacd);
            Vector3Wide.Dot(ref ab, ref nacd, out var abnacd);
            var outsideACD = Vector.LessThanOrEqual(anacd * abnacd, Vector<float>.Zero);
            var acdActive = Vector.BitwiseAnd(
                Vector.BitwiseAnd(
                    outsideACD,
                    Vector.GreaterThanOrEqual(acACDPlaneTest, Vector<float>.Zero)),
                Vector.BitwiseAnd(
                    Vector.GreaterThanOrEqual(adACDPlaneTest, Vector<float>.Zero),
                    Vector.GreaterThanOrEqual(cdACDPlaneTest, Vector<float>.Zero)));
            aRequired = Vector.BitwiseOr(acdActive, aRequired);
            cRequired = Vector.BitwiseOr(acdActive, cRequired);
            dRequired = Vector.BitwiseOr(acdActive, dRequired);

            //ABD:
            //Inside abABD, adABD, and bdABD planes.
            //(A * (AB x AD) * (AC * (AB x AD)) >= 0
            Vector3Wide.CrossWithoutOverlap(ref ab, ref ad, out var nabd);
            Vector3Wide.Dot(ref simplex.A, ref nabd, out var anabd);
            Vector3Wide.Dot(ref ac, ref nabd, out var acnabd);
            var outsideABD = Vector.LessThanOrEqual(anabd * acnabd, Vector<float>.Zero);
            var abdActive = Vector.BitwiseAnd(
                Vector.BitwiseAnd(
                    outsideABD,
                    Vector.GreaterThan(abABDPlaneTest, Vector<float>.Zero)),
                Vector.BitwiseAnd(
                    Vector.GreaterThan(adABDPlaneTest, Vector<float>.Zero),
                    Vector.GreaterThan(bdABDPlaneTest, Vector<float>.Zero)));
            aRequired = Vector.BitwiseOr(abdActive, aRequired);
            bRequired = Vector.BitwiseOr(abdActive, bRequired);
            dRequired = Vector.BitwiseOr(abdActive, dRequired);

            //ABCD:
            var insideTetrahedron = Vector.BitwiseAnd(
                Vector.AndNot(Vector.OnesComplement(outsideABC), outsideBCD), 
                Vector.AndNot(Vector.OnesComplement(outsideACD), outsideABD));
            aRequired = Vector.BitwiseOr(aRequired, insideTetrahedron);
            bRequired = Vector.BitwiseOr(bRequired, insideTetrahedron);
            cRequired = Vector.BitwiseOr(cRequired, insideTetrahedron);
            dRequired = Vector.BitwiseOr(dRequired, insideTetrahedron);

            //Note that the tetrahedral containment case implies intersection. Distance queries do not guarantee meaningful normals or closest points during intersection,
            //so there is no need to compute barycentric coordinates.

            //Sub-tetrahedral cases imply nonintersection.
            //For triangle barycentric coordinates, we can make use of the edge-face plane tests we computed earlier.
            //The barycentric weight of vertex C on triangle ABC for the origin is:
            //(A x AB) * (AB x AC) / ((AB x AC) * (AB x AC))
            //Once again applying an identity, this transforms to:
            //(A x AB) * (AB x AC) / ((AB * AB) * (AC * AC) - (AB * AC) * (AC * AB))
            //Which, once again, shares a bunch of ALU work we already did.
            var abcDenom = Vector<float>.One / (abab * acac - abac * abac);
            var abcAWeight = bcABCPlaneTest * abcDenom;
            var abcBWeight = acABCPlaneTest * abcDenom;
            var abcCWeight = abABCPlaneTest * abcDenom;
            var acdDenom = Vector<float>.One / (acac * adad - acad * acad);
            var acdAWeight = cdACDPlaneTest * acdDenom;
            var acdCWeight = adACDPlaneTest * acdDenom;
            var acdDWeight = acACDPlaneTest * acdDenom;
            var abdDenom = Vector<float>.One / (abab * adad - abad * abad);
            var abdAWeight = bdABDPlaneTest * abdDenom;
            var abdBWeight = adABDPlaneTest * abdDenom;
            var abdDWeight = abABDPlaneTest * abdDenom;
            var bcdDenom = Vector<float>.One / (bcbc * bdbd - bcbd * bcbd);
            var bcdBWeight = cdBCDPlaneTest * bcdDenom;
            var bcdCWeight = bdBCDPlaneTest * bcdDenom;
            var bcdDWeight = bcBCDPlaneTest * bcdDenom;

            var abAWeight = abB / abab;
            var abBWeight = Vector<float>.One - abAWeight;
            var acAWeight = acC / acac;
            var acCWeight = Vector<float>.One - acAWeight;
            var adAWeight = adD / adad;
            var adDWeight = Vector<float>.One - adAWeight;
            var bcBWeight = bcC / bcbc;
            var bcCWeight = Vector<float>.One - bcBWeight;
            var bdBWeight = bdD / bdbd;
            var bdDWeight = Vector<float>.One - bdBWeight;
            var cdCWeight = cdD / cdcd;
            var cdDWeight = Vector<float>.One - cdCWeight;
        }

        public void Test(ref TShapeWideA a, ref TShapeWideB b, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
            out Vector<int> intersected, out Vector<float> distance, out Vector3Wide closestA, out Vector3Wide normal)
        {
            Matrix3x3Wide.CreateFromQuaternion(ref orientationA, out var rA);
            Matrix3x3Wide.CreateFromQuaternion(ref orientationB, out var rB);
            var supportFinderA = default(TSupportFinderA);
            var supportFinderB = default(TSupportFinderB);
            Simplex simplex;
            //TODO: It would be pretty easy to initialize to a triangle or tetrahedron. Might be worth it.
            SampleMinkowskiDifference(ref a, ref rA, ref supportFinderA, ref b, ref rB, ref supportFinderB, ref offsetB, ref offsetB, out simplex.AOnA, out simplex.A);
            simplex.Count = new Vector<int>(1);

            //GJK is a pretty branchy algorithm that doesn't map perfectly to widely vectorized implementations. We'll be using an SPMD model- if any SIMD lane needs to continue 
            //working, all lanes continue to work, and the unnecessary lane results are discarded.
            //In the context of the engine, distance tests are often quite coherent (by virtue of being used with sweeps), so we shouldn't often see pathologically low occupancy.
            var mask = Vector<int>.Zero;
            intersected = Vector<int>.Zero;
            while (true)
            {
                FindClosestPoint(ref simplex, out var simplexClosestA, out var simplexClosest);

                var simplexContainsOrigin = Vector.BitwiseAnd(
                    Vector.BitwiseAnd(
                        Vector.Equals(simplexClosest.X, Vector<float>.Zero),
                        Vector.Equals(simplexClosest.Y, Vector<float>.Zero)),
                    Vector.Equals(simplexClosest.Z, Vector<float>.Zero));
                //For any lane that has not yet terminated, and which has an intersecting simplex, terminate with intersection.
                intersected = Vector.BitwiseOr(intersected, Vector.AndNot(simplexContainsOrigin, mask));
                mask = Vector.BitwiseOr(mask, simplexContainsOrigin);
                if (Vector.EqualsAll(mask, -Vector<int>.One))
                    break;

                Vector3Wide.Negate(ref simplexClosest, out var sampleDirection);
                SampleMinkowskiDifference(ref a, ref rA, ref supportFinderA, ref b, ref rB, ref supportFinderB, ref offsetB, ref sampleDirection, out var vA, out var v);

                //If the newly sampled is no better than the simplex-identified closest point, the lane is done.
                Vector3Wide.Subtract(ref v, ref simplexClosest, out var progressOffset);
                Vector3Wide.Dot(ref progressOffset, ref sampleDirection, out var progress);
                var noProgressMade = Vector.LessThanOrEqual(progress, Vector<float>.Zero);
                //Any still-executing lane that failed to make progress is now verified to be nonintersecting.
                //Note that the default initialized state is nonintersecting, so we don't have to set the flag explicitly here.
                //Note that lanes which previously terminated are excluded from closest/normal updates.      
                Vector3Wide.ConditionalSelect(ref mask, ref closestA, ref simplexClosestA, out closestA);
                Vector3Wide.ConditionalSelect(ref mask, ref normal, ref simplexClosest, out normal);
                mask = Vector.BitwiseOr(mask, noProgressMade);
                if (Vector.EqualsAll(mask, -Vector<int>.One))
                    break;

                Append(ref mask, ref simplex, ref vA, ref v);
            }
            //The normal gathered during the loop was the unnormalized offset.
            //Note that this normalization will cover lanes which weren't separated- that's fine. Valid lanes are described by the intersected flag.
            Vector3Wide.Normalize(ref normal, out normal);
        }
    }
}
