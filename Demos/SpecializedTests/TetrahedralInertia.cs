using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using BepuPhysics;
using BepuUtilities;

namespace Demos.SpecializedTests
{
    public static class TetrahedralInertia
    {
        struct TetrahedronInertiaTester : IInertiaTester
        {
            public Vector3 A, B, C, D;
            public void ComputeAnalyticInertia(float mass, out BodyInertia inertia)
            {
                //Computing the inertia of a tetrahedron requires integrating across its volume.
                //While it's possible to do so directly given arbitrary plane equations, it's more convenient to integrate over a normalized tetrahedron with coordinates 
                //at (0,0,0), (1,0,0), (0,1,0), and (0,0,1). The integration location can be transformed back to the original frame of reference using the tetrahedral edges.
                //That is, (1,0,0) in normalized space transforms to B-A in world space.
                //To make that explicit, we have an equation:
                // [1,0,0]               [ B - A ]
                // [0,1,0] * Transform = [ C - A ]
                // [0,0,1]               [ D - A ]
                //(Note that you could consider this to be an affine transform with a translation equal to A.)

                //Since the normalized edge directions compose the identity matrix, the transform is just the edge directions.
                //So, given function f that computes a point's contribution to the inertia tensor, the inertia tensor of the normalized tetrahedron with uniform unit density is:
                //Integrate[Integrate[Integrate[f[{i, j, k}], {k, 0, 1-i-j}], {j, 0, 1-i}], {i, 0, 1}];
                //Note the integration bounds- they are a result of the simple shape of the normalized tetrahedron making the plane equations easier to deal with.
                //Now, to integrate over the true tetrahedron's shape, the normalized coordinates are transformed to world coordinates:
                //Integrate[Integrate[Integrate[f[{i, j, k}.Transform + A] * Abs[Det[Transform]], {k, 0, 1-i-j}], {j, 0, 1-i}], {i, 0, 1}];

                //One key difference is the inclusion of the transform's jacobian's determinant, which in this case is just the volume of the tetrahedron times six.
                //For a geometric intuition for why that exists, consider that the normalized integration covers a tetrahedron with a volume of 1/6. The world space tetrahedron
                //has a volume of Abs[Det[Transform]]/6. So, the volume changes by a factor of exactly Abs[Det[Transform]]. 
                //That term compensates for the difference in integration domain.
                //It's also constant over the integration, so you can just pull it out.
                //Similarly, if you had a non-unit uniform density, you would multiply the integration by it too.

                //So, putting that together and assuming the scaling term is pulled out, here's a chunk of code you can plop into wolfram cloud and whatnot to recreate the results:
                //f[{x_, y_, z_}] := {{y^2 + z^2, -x * y, -x * z}, {-x * y, x^2 + z^2, -y * z}, {-x * z, -y * z, x^2 + y^2}}
                //a = { AX, AY, AZ};
                //b = { BX, BY, BZ};
                //c = { CX, CY, CZ};
                //d = { DX, DY, DZ};
                //ab = b - a;
                //ac = c - a;
                //ad = d - a;
                //A = { ab, ac, ad};
                //Integrate[Integrate[Integrate[f[{ i, j, k}.A + a], {k, 0, 1-i-j}], {j, 0, 1-i}],{i, 0, 1}]
                inertia.InverseMass = 1f / mass;
                var ab = B - A;
                var ac = C - A;
                var ad = D - A;
                //Revisiting the determinant, note that:
                //density * abs(determinant) = density * volume * 6 = mass * 6
                //So there's no need to actually compute the determinant/volume since we were given the mass directly.
                var diagonalScaling = mass * (6f / 60f);
                Triangular3x3 inertiaTensor;
                inertiaTensor.XX = diagonalScaling * (
                    A.Y * A.Y + A.Z * A.Z + B.Y * B.Y + B.Z * B.Z + C.Y * C.Y + C.Z * C.Z + D.Y * D.Y + D.Z * D.Z +
                    B.Y * C.Y + B.Z * C.Z +
                    (B.Y + C.Y) * D.Y + (B.Z + C.Z) * D.Z +
                    A.Y * (B.Y + C.Y + D.Y) + A.Z * (B.Z + C.Z + D.Z));
                inertiaTensor.YY = diagonalScaling * (
                    A.X * A.X + A.Z * A.Z + B.X * B.X + B.Z * B.Z + C.X * C.X + C.Z * C.Z + D.X * D.X + D.Z * D.Z +
                    B.X * C.X + B.Z * C.Z +
                    (B.X + C.X) * D.X + (B.Z + C.Z) * D.Z +
                    A.X * (B.X + C.X + D.X) + A.Z * (B.Z + C.Z + D.Z));
                inertiaTensor.ZZ = diagonalScaling * (
                    A.X * A.X + A.Y * A.Y + B.X * B.X + B.Y * B.Y + C.X * C.X + C.Y * C.Y + D.X * D.X + D.Y * D.Y +
                    B.X * C.X + B.Y * C.Y +
                    (B.X + C.X) * D.X + (B.Y + C.Y) * D.Y +
                    A.X * (B.X + C.X + D.X) + A.Y * (B.Y + C.Y + D.Y));
                var offScaling = mass * (6f / 120f);
                inertiaTensor.YX = offScaling * (
                    -2 * B.X * B.Y - 2 * C.X * C.Y - 
                    B.Y * C.X - B.X * C.Y - B.Y * D.X - C.Y * D.X - 
                    A.Y * (B.X + C.X + D.X) - (B.X + C.X + 2 * D.X) * D.Y - A.X * (2 * A.Y + B.Y + C.Y + D.Y));
                inertiaTensor.ZX = offScaling * (
                    -2 * B.X * B.Z - 2 * C.X * C.Z -
                    B.Z * C.X - B.X * C.Z - B.Z * D.X - C.Z * D.X -
                    A.Z * (B.X + C.X + D.X) - (B.X + C.X + 2 * D.X) * D.Z - A.X * (2 * A.Z + B.Z + C.Z + D.Z));
                inertiaTensor.ZY = offScaling * (
                    -2 * B.Y * B.Z - 2 * C.Y * C.Z -
                    B.Z * C.Y - B.Y * C.Z - B.Z * D.Y - C.Z * D.Y -
                    A.Z * (B.Y + C.Y + D.Y) - (B.Y + C.Y + 2 * D.Y) * D.Z - A.Y * (2 * A.Z + B.Z + C.Z + D.Z));
                //TODO: Note that the above implementation isn't exactly optimal. Assuming for now that the performance isn't going to be relevant.
                //That could change given certain convex hull use cases, but in that situation you should probably just jump to vectorizing over multiple tetrahedra at a time.
                //(Plus some basic term caching.)
                Triangular3x3.SymmetricInvert(inertiaTensor, out inertia.InverseInertiaTensor);

            }

            public void ComputeBounds(out Vector3 min, out Vector3 max)
            {
                min = Vector3.Min(Vector3.Min(A, B), Vector3.Min(C, D));
                max = Vector3.Max(Vector3.Max(A, B), Vector3.Max(C, D));
            }

            public bool PointIsContained(ref Vector3 sampleSpacing, ref Vector3 point)
            {
                var ab = B - A;
                var ac = C - A;
                var ad = D - A;
                var bc = C - B;
                var bd = D - B;
                var nabc = Vector3.Cross(ab, ac);
                var nabd = Vector3.Cross(ab, ad);
                var nacd = Vector3.Cross(ac, ad);
                var nbcd = Vector3.Cross(bc, bd);
                var ap = point - A;
                var bp = point - B;
                return
                    Vector3.Dot(nabc, ap) * Vector3.Dot(nabc, ad) >= 0 &&
                    Vector3.Dot(nabd, ap) * Vector3.Dot(nabd, ac) >= 0 &&
                    Vector3.Dot(nacd, ap) * Vector3.Dot(nacd, ab) >= 0 &&
                    Vector3.Dot(nbcd, bp) * Vector3.Dot(nbcd, ab) <= 0;
            }
        }
        
        public static void Test()
        {
            TetrahedronInertiaTester tester;
            tester.A = new Vector3(1, -1, -1);
            tester.B = new Vector3(2, -1, 1);
            tester.C = new Vector3(1, -3, -1);
            tester.D = new Vector3(1, -1, 3);
            InertiaTensorTests.CheckInertia(ref tester);
        }
    }
}
