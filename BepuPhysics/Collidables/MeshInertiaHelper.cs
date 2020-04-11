using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.Collidables
{
    /// <summary>
    /// Defines a type capable of providing a sequence of triangles.
    /// </summary>
    public interface ITriangleSource
    {
        /// <summary>
        /// Gets the next triangle in the sequence, if any.
        /// </summary>
        /// <param name="a">First vertex in the triangle.</param>
        /// <param name="b">Second vertex in the triangle.</param>
        /// <param name="c">Third vertex in the triangle.</param>
        /// <returns>True if there was another triangle, false otherwise.</returns>
        bool GetNextTriangle(out Vector3 a, out Vector3 b, out Vector3 c);
    }

    /// <summary>
    /// Provides helpers for computing the inertia of objects with triangular surfaces.
    /// </summary>
    public static class MeshInertiaHelper
    {
        public static void ComputeTetrahedronContribution(in Vector3 a, in Vector3 b, in Vector3 c, float mass, out Symmetric3x3 inertiaTensor)
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

            //Revisiting the determinant, note that:
            //density * abs(determinant) = density * volume * 6 = mass * 6
            //So there's no need to actually compute the determinant/volume since we were given the mass directly.
            var diagonalScaling = mass * (6f / 60f);
            inertiaTensor.XX = diagonalScaling * (
                a.Y * a.Y + a.Z * a.Z + b.Y * b.Y + b.Z * b.Z + c.Y * c.Y + c.Z * c.Z +
                b.Y * c.Y + b.Z * c.Z +
                a.Y * (b.Y + c.Y) + a.Z * (b.Z + c.Z));
            inertiaTensor.YY = diagonalScaling * (
                a.X * a.X + a.Z * a.Z + b.X * b.X + b.Z * b.Z + c.X * c.X + c.Z * c.Z +
                b.X * c.X + b.Z * c.Z +
                a.X * (b.X + c.X) + a.Z * (b.Z + c.Z));
            inertiaTensor.ZZ = diagonalScaling * (
                a.X * a.X + a.Y * a.Y + b.X * b.X + b.Y * b.Y + c.X * c.X + c.Y * c.Y +
                b.X * c.X + b.Y * c.Y +
                a.X * (b.X + c.X) + a.Y * (b.Y + c.Y));
            var offScaling = mass * (6f / 120f);
            inertiaTensor.YX = offScaling * (
                -2 * b.X * b.Y - 2 * c.X * c.Y -
                b.Y * c.X - b.X * c.Y -
                a.Y * (b.X + c.X) - a.X * (2 * a.Y + b.Y + c.Y));
            inertiaTensor.ZX = offScaling * (
                -2 * b.X * b.Z - 2 * c.X * c.Z -
                b.Z * c.X - b.X * c.Z -
                a.Z * (b.X + c.X) - a.X * (2 * a.Z + b.Z + c.Z));
            inertiaTensor.ZY = offScaling * (
                -2 * b.Y * b.Z - 2 * c.Y * c.Z -
                b.Z * c.Y - b.Y * c.Z -
                a.Z * (b.Y + c.Y) - a.Y * (2 * a.Z + b.Z + c.Z));
            //TODO: Note that the above implementation isn't exactly optimal. Assuming for now that the performance isn't going to be relevant.
            //That could change given certain convex hull use cases, but in that situation you should probably just jump to vectorizing over multiple tetrahedra at a time.
            //(Plus some basic term caching.)

        }

        /// <summary>
        /// Computes the signed volume of a tetrahedron where the fourth vertex is at the origin.
        /// Triangles visible from outside the shape are assumed to have clockwise winding in right handed coordinates or counterclockwise winding in left handed coordinates.
        /// </summary>
        /// <param name="a">First vertex of the tetrahedron.</param>
        /// <param name="b">Second vertex of the tetrahedron.</param>
        /// <param name="c">Third vertex of the tetrahedron.</param>
        /// <returns>Volume of the tetrahedron.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float ComputeTetrahedronVolume(in Vector3 a, in Vector3 b, in Vector3 c)
        {
            return (1f / 6f) * Vector3.Dot(Vector3.Cross(b, a), c);
        }

        /// <summary>
        /// Integrates the inertia contribution of a tetrahedron with vertices at a, b, c, and (0,0,0) assuming a density of 1.
        /// </summary>
        /// <param name="a">First vertex of the tetrahedron.</param>
        /// <param name="b">Second vertex of the tetrahedron.</param>
        /// <param name="c">Third vertex of the tetrahedron.</param>
        /// <param name="volume">Volume of the tetrahedron.</param>
        /// <param name="inertiaTensor">Inertia tensor of this tetrahedron assuming a density of 1.</param>
        public static void ComputeTetrahedronContribution(in Vector3 a, in Vector3 b, in Vector3 c, out float volume, out Symmetric3x3 inertiaTensor)
        {
            volume = ComputeTetrahedronVolume(a, b, c);
            ComputeTetrahedronContribution(a, b, c, volume, out inertiaTensor);
        }

        /// <summary>
        /// Computes the inertia of a closed mesh.
        /// Triangles visible from outside the shape are assumed to have clockwise winding in right handed coordinates or counterclockwise winding in left handed coordinates.
        /// </summary>
        /// <typeparam name="TTriangleSource">Type of the triangle source.</typeparam>
        /// <param name="triangleSource">Source from which to retrieve a sequence of triangles.</param>
        /// <param name="mass">Mass of the mesh to scale the inertia tensor with.</param>
        /// <param name="volume">Volume of the mesh.</param>
        /// <param name="inertia">Inertia tensor of the mesh.</param>
        public static void ComputeClosedInertia<TTriangleSource>(ref TTriangleSource triangleSource, float mass, out float volume, out Symmetric3x3 inertia) where TTriangleSource : ITriangleSource
        {
            volume = 0;
            Symmetric3x3 summedContributions = default;
            while (triangleSource.GetNextTriangle(out var a, out var b, out var c))
            {
                ComputeTetrahedronContribution(a, b, c, out var tVolume, out var tContribution);
                Symmetric3x3.Add(summedContributions, tContribution, out summedContributions);
                volume += tVolume;
            }
            Symmetric3x3.Scale(summedContributions, mass / volume, out inertia);
        }

        /// <summary>
        /// Computes the inertia of a closed mesh.
        /// Triangles visible from outside the shape are assumed to have clockwise winding in right handed coordinates or counterclockwise winding in left handed coordinates.
        /// </summary>
        /// <typeparam name="TTriangleSource">Type of the triangle source.</typeparam>
        /// <param name="triangleSource">Source from which to retrieve a sequence of triangles.</param>
        /// <param name="mass">Mass of the mesh to scale the inertia tensor with.</param>
        /// <param name="volume">Volume of the mesh.</param>
        /// <param name="inertia">Inertia tensor of the mesh.</param>
        /// <param name="center">Center of mass of the mesh.</param>
        public static void ComputeClosedInertia<TTriangleSource>(ref TTriangleSource triangleSource, float mass, out float volume, out Symmetric3x3 inertia, out Vector3 center) where TTriangleSource : ITriangleSource
        {
            volume = 0;
            Symmetric3x3 summedContributions = default;
            center = default;
            while (triangleSource.GetNextTriangle(out var a, out var b, out var c))
            {
                ComputeTetrahedronContribution(a, b, c, out var tVolume, out var tContribution);
                Symmetric3x3.Add(summedContributions, tContribution, out summedContributions);
                volume += tVolume;
                center += (a + b + c) * tVolume;
            }
            var inverseVolume = 1f / volume;
            center *= inverseVolume * 0.25f;
            Symmetric3x3.Scale(summedContributions, mass * inverseVolume, out inertia);
        }

        /// <summary>
        /// Computes the center of mass of a closed mesh.
        /// </summary>
        /// <typeparam name="TTriangleSource">Type of the triangle source.</typeparam>
        /// <param name="triangleSource">Source from which to retrieve a sequence of triangles.</param>
        /// <param name="volume">Volume of the mesh.</param>
        /// <param name="center">Center of mass of the mesh.</param>
        public static void ComputeClosedCenterOfMass<TTriangleSource>(ref TTriangleSource triangleSource, out float volume, out Vector3 center) where TTriangleSource : ITriangleSource
        {
            center = default;
            volume = 0f;
            while (triangleSource.GetNextTriangle(out var a, out var b, out var c))
            {
                var tVolume = ComputeTetrahedronVolume(a, b, c);
                volume += tVolume;
                center += tVolume * (a + b + c);
            }
            center /= volume * 4;
        }

        /// <summary>
        /// Integrates the inertia contribution from a single triangle.
        /// </summary>
        /// <param name="a">First vertex in the triangle.</param>
        /// <param name="b">Second vertex in the triangle.</param>
        /// <param name="c">Third vertex in the triangle.</param>
        /// <param name="mass">Mass of the triangle.</param>
        /// <param name="inertiaTensor">Inertia tensor of the triangle.</param>
        public static void ComputeTriangleContribution(in Vector3 a, in Vector3 b, in Vector3 c, float mass, out Symmetric3x3 inertiaTensor)
        {
            //This follows the same logic as the tetrahedral inertia tensor calculation, but the transform is different.
            //There are only two dimensions of interest, but if we wanted to express it as a 3x3 linear transform:
            // [ B - A ]
            // [   N   ]
            // [ C - A ]
            //where N = (ab x ac) / ||ab x ac||.
            //In other words, this transform maintains the plane normal such that you can compute the scaled triangle area using (ab x ac) * N.
            //In practice, that normal won't actually appear in our calculations because we were given the mass explicitly rather than integrating it from density across the area.
            //So, putting that together and assuming the scaling term is pulled out, here's a chunk of code you can plop into wolfram cloud and whatnot to recreate the results:
            //f[{x_, y_, z_}] := {{y^2 + z^2, -x * y, -x * z}, {-x * y, x^2 + z^2, -y * z}, {-x * z, -y * z, x^2 + y^2}}
            //a = { ax, ay, az };
            //b = { bx, by, bz };
            //c = { cx, cy, cz };
            //ab = b - a;
            //ac = c - a;
            //n = Cross[ab, ac] / Length[Cross[ab, ac]];
            //A = { ab, n, ac };
            //result = Integrate[Integrate[f[{ i, 0, k}.A + a], {k, 0, 1-i}], {i, 0, 1}];
            //Revisiting the determinant, note that:
            //density * abs(determinant) = density * volume * 2 = mass * 2
            //So there's no need to actually compute the determinant/area since we were given the mass directly.
            var diagonalScaling = mass * (2f / 12f);
            inertiaTensor.XX = diagonalScaling * (
                a.Y * a.Y + a.Z * a.Z + b.Y * b.Y + b.Z * b.Z + c.Y * c.Y + c.Z * c.Z +
                a.Y * b.Y + a.Z * b.Z + a.Y * c.Y + b.Y * c.Y + a.Z * c.Z + b.Z * c.Z);
            inertiaTensor.YY = diagonalScaling * (
                a.X * a.X + a.Z * a.Z + b.X * b.X + b.Z * b.Z + c.X * c.X + c.Z * c.Z +
                a.X * b.X + a.Z * b.Z + a.X * c.X + b.X * c.X + a.Z * c.Z + b.Z * c.Z);
            inertiaTensor.ZZ = diagonalScaling * (
                a.X * a.X + a.Y * a.Y + b.X * b.X + b.Y * b.Y + c.X * c.X + c.Y * c.Y +
                a.X * b.X + a.Y * b.Y + a.X * c.X + b.X * c.X + a.Y * c.Y + b.Y * c.Y);
            var offScaling = mass * (2f / 24f);
            inertiaTensor.YX = offScaling * (-a.Y * (b.X + c.X) - b.Y * (2 * b.X + c.X) - (b.X + 2 * c.X) * c.Y - a.X * (2 * a.Y + b.Y + c.Y));
            inertiaTensor.ZX = offScaling * (-a.Z * (b.X + c.X) - b.Z * (2 * b.X + c.X) - (b.X + 2 * c.X) * c.Z - a.X * (2 * a.Z + b.Z + c.Z));
            inertiaTensor.ZY = offScaling * (-a.Z * (b.Y + c.Y) - b.Z * (2 * b.Y + c.Y) - (b.Y + 2 * c.Y) * c.Z - a.Y * (2 * a.Z + b.Z + c.Z));
            //TODO: Note that the above implementation isn't exactly optimal. Assuming for now that the performance isn't going to be relevant.
            //That could change given certain convex hull use cases, but in that situation you should probably just jump to vectorizing over multiple tetrahedra at a time.
            //(Plus some basic term caching.)
        }

        /// <summary>
        /// Computes the area of a triangle.
        /// </summary>
        /// <param name="a">First vertex in the triangle.</param>
        /// <param name="b">Second vertex in the triangle.</param>
        /// <param name="c">Third vertex in the triangle.</param>
        /// <returns>Area of the triangle.</returns>
        public static float ComputeTriangleArea(in Vector3 a, in Vector3 b, in Vector3 c)
        {
            return 0.5f * Vector3.Cross(b - a, c - a).Length(); //Not exactly fast, but again, we're assuming performance is irrelevant for the mesh inertia helper.
        }

        /// <summary>
        /// Integrates the inertia contribution from a single triangle assuming a density of 1.
        /// </summary>
        /// <param name="a">First vertex in the triangle.</param>
        /// <param name="b">Second vertex in the triangle.</param>
        /// <param name="c">Third vertex in the triangle.</param>
        /// <param name="area">Area of the triangle.</param>
        /// <param name="inertiaTensor">Inertia tensor of the triangle assuming that the density is 1.</param>
        public static void ComputeTriangleContribution(in Vector3 a, in Vector3 b, in Vector3 c, out float area, out Symmetric3x3 inertiaTensor)
        {
            area = ComputeTriangleArea(a, b, c);
            ComputeTriangleContribution(a, b, c, area, out inertiaTensor);
        }

        /// <summary>
        /// Computes the inertia of an open mesh, treating it as a triangle soup.
        /// </summary>
        /// <typeparam name="TTriangleSource">Type of the triangle source.</typeparam>
        /// <param name="triangleSource">Source from which to retrieve a sequence of triangles.</param>
        /// <param name="mass">Mass of the mesh to scale the inertia tensor with.</param>
        /// <param name="inertia">Inertia tensor of the mesh.</param>
        public static void ComputeOpenInertia<TTriangleSource>(ref TTriangleSource triangleSource, float mass, out Symmetric3x3 inertia) where TTriangleSource : ITriangleSource
        {
            float area = 0f;
            inertia = default;
            while (triangleSource.GetNextTriangle(out var a, out var b, out var c))
            {
                ComputeTriangleContribution(a, b, c, out var tArea, out var tContribution);
                area += tArea;
                Symmetric3x3.Add(tContribution, inertia, out inertia);
            }
            Symmetric3x3.Scale(inertia, mass / area, out inertia);
        }

        /// <summary>
        /// Computes the inertia of an open mesh, treating it as a triangle soup.
        /// </summary>
        /// <typeparam name="TTriangleSource">Type of the triangle source.</typeparam>
        /// <param name="triangleSource">Source from which to retrieve a sequence of triangles.</param>
        /// <param name="mass">Mass of the mesh to scale the inertia tensor with.</param>
        /// <param name="inertia">Inertia tensor of the mesh.</param>
        /// <param name="center">Center of mass of the mesh.</param>
        public static void ComputeOpenInertia<TTriangleSource>(ref TTriangleSource triangleSource, float mass, out Symmetric3x3 inertia, out Vector3 center) where TTriangleSource : ITriangleSource
        {
            center = default;
            float area = 0f;
            inertia = default;
            while (triangleSource.GetNextTriangle(out var a, out var b, out var c))
            {
                ComputeTriangleContribution(a, b, c, out var tArea, out var tContribution);
                area += tArea;
                center += tArea * (a + b + c);
                Symmetric3x3.Add(tContribution, inertia, out inertia);
            }
            var inverseArea = 1f / area;
            center *= inverseArea * (1f / 3f);
            Symmetric3x3.Scale(inertia, mass * inverseArea, out inertia);
        }

        /// <summary>
        /// Computes the center of mass of an open mesh, treating it as a triangle soup.
        /// </summary>
        /// <typeparam name="TTriangleSource">Type of the triangle source.</typeparam>
        /// <param name="triangleSource">Source from which to retrieve a sequence of triangles.</param>
        /// <returns>Center of mass of the mesh.</returns>
        public static Vector3 ComputeOpenCenterOfMass<TTriangleSource>(ref TTriangleSource triangleSource) where TTriangleSource : ITriangleSource
        {
            Vector3 center = default;
            float area = 0f;
            while (triangleSource.GetNextTriangle(out var a, out var b, out var c))
            {
                var tArea = ComputeTriangleArea(a, b, c);
                area += tArea;
                center += tArea * (a + b + c);
            }
            center /= area * 3;
            return center;
        }

        /// <summary>
        /// Computes an offset for an inertia tensor based on an offset frame of reference.
        /// </summary>
        /// <param name="mass">Mass associated with the inertia tensor being moved.</param>
        /// <param name="offset">Offset from the current inertia frame of reference to the new frame of reference.</param>
        /// <param name="inertiaOffset">Modification to add to the inertia tensor to move it into the new reference frame.</param>
        public static void GetInertiaOffset(float mass, in Vector3 offset, out Symmetric3x3 inertiaOffset)
        {
            //Just the parallel axis theorem.
            var squared = offset * offset;
            var diagonal = squared.X + squared.Y + squared.Z;
            inertiaOffset.XX = mass * (squared.X - diagonal);
            inertiaOffset.YX = mass * (offset.X * offset.Y);
            inertiaOffset.YY = mass * (squared.Y - diagonal);
            inertiaOffset.ZX = mass * (offset.X * offset.Z);
            inertiaOffset.ZY = mass * (offset.Y * offset.Z);
            inertiaOffset.ZZ = mass * (squared.Z - diagonal);
        }

    }
}
