using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
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
        /// <summary>
        /// Integrates the inertia contribution of a tetrahedron with vertices at (0,0,0), v2, v3, and v4.
        /// </summary>
        /// <param name="v2">Second vertex of the tetrahedron.</param>
        /// <param name="v3">Third vertex of the tetrahedron.</param>
        /// <param name="v4">Fourth vertex of the tetrahedron.</param>
        /// <param name="xx">Contribution of this tetrahedron to the XX component of the inertia tensor.</param>
        /// <param name="yy">Contribution of this tetrahedron to the YY component of the inertia tensor.</param>
        /// <param name="zz">Contribution of this tetrahedron to the ZZ component of the inertia tensor.</param>
        /// <param name="xy">Contribution of this tetrahedron to the XY component of the inertia tensor.</param>
        /// <param name="xz">Contribution of this tetrahedron to the XZ component of the inertia tensor.</param>
        /// <param name="yz">Contribution of this tetrahedron to the YZ component of the inertia tensor.</param>
        /// <param name="scaledVolume">Six times the volume of the tetrahedron.</param>
        public static void IntegrateTetrahedron(in Vector3 v2, in Vector3 v3, in Vector3 v4,
            out float xx, out float yy, out float zz, out float xy, out float xz, out float yz,
            out float scaledVolume)
        {
            //This is just taken straight out of v1, derivation from Explicit Exact Formulas for the 3-D Tetrahedron Inertia Tensor in Terms of its Vertex Coordinates.
            //Could do better than this; doesn't bother even trying to vectorize.

            scaledVolume = v2.X * (v3.Z * v4.Y - v3.Y * v4.Z) -
                           v3.X * (v2.Z * v4.Y - v2.Y * v4.Z) +
                           v4.X * (v2.Z * v3.Y - v2.Y * v3.Z);

            xx = scaledVolume * (v2.Y * v2.Y + v2.Y * v3.Y + v3.Y * v3.Y + v2.Y * v4.Y + v3.Y * v4.Y + v4.Y * v4.Y +
                                 v2.Z * v2.Z + v2.Z * v3.Z + v3.Z * v3.Z + v2.Z * v4.Z + v3.Z * v4.Z + v4.Z * v4.Z);
            yy = scaledVolume * (v2.X * v2.X + v2.X * v3.X + v3.X * v3.X + v2.X * v4.X + v3.X * v4.X + v4.X * v4.X +
                                 v2.Z * v2.Z + v2.Z * v3.Z + v3.Z * v3.Z + v2.Z * v4.Z + v3.Z * v4.Z + v4.Z * v4.Z);
            zz = scaledVolume * (v2.X * v2.X + v2.X * v3.X + v3.X * v3.X + v2.X * v4.X + v3.X * v4.X + v4.X * v4.X +
                                 v2.Y * v2.Y + v2.Y * v3.Y + v3.Y * v3.Y + v2.Y * v4.Y + v3.Y * v4.Y + v4.Y * v4.Y);
            yz = scaledVolume * (2 * v2.Y * v2.Z + v3.Y * v2.Z + v4.Y * v2.Z + v2.Y * v3.Z + 2 * v3.Y * v3.Z + v4.Y * v3.Z + v2.Y * v4.Z + v3.Y * v4.Z + 2 * v4.Y * v4.Z);
            xy = scaledVolume * (2 * v2.X * v2.Z + v3.X * v2.Z + v4.X * v2.Z + v2.X * v3.Z + 2 * v3.X * v3.Z + v4.X * v3.Z + v2.X * v4.Z + v3.X * v4.Z + 2 * v4.X * v4.Z);
            xz = scaledVolume * (2 * v2.X * v2.Y + v3.X * v2.Y + v4.X * v2.Y + v2.X * v3.Y + 2 * v3.X * v3.Y + v4.X * v3.Y + v2.X * v4.Y + v3.X * v4.Y + 2 * v4.X * v4.Y);

        }

        /// <summary>
        /// Finalizes the inertia tensor from tetrahedral integration.
        /// </summary>
        /// <param name="xx">Scaled XX component of the inertia tensor.</param>
        /// <param name="yy">Scaled YY component of the inertia tensor.</param>
        /// <param name="zz">Scaled ZZ component of the inertia tensor.</param>
        /// <param name="xy">Scaled XY component of the inertia tensor.</param>
        /// <param name="xz">Scaled XZ component of the inertia tensor.</param>
        /// <param name="yz">Scaled YZ component of the inertia tensor.</param>
        /// <param name="scaledVolume">Scaled volume of the mesh.</param>
        /// <param name="mass">Mass to scale the inertia tensor with.</param>
        /// <param name="volume">Computed volume of the mesh.</param>
        /// <param name="inverseInertia">Computed inverse inertia tensor of the mesh.</param>
        public static void FinalizeInertia(float xx, float yy, float zz, float xy, float xz, float yz, float scaledVolume, float mass,
            out float volume, out Symmetric3x3 inverseInertia)
        {
            volume = scaledVolume / 6;
            float scaledDensity = mass / volume;
            float diagonalFactor = scaledDensity / 60;
            float offFactor = -scaledDensity / 120;
            inverseInertia.XX = xx * diagonalFactor;
            inverseInertia.YX = xy * diagonalFactor;
            inverseInertia.YY = yy * diagonalFactor;
            inverseInertia.ZX = xz * offFactor;
            inverseInertia.ZY = yz * offFactor;
            inverseInertia.ZZ = zz * offFactor;
            Symmetric3x3.Invert(inverseInertia, out inverseInertia);
        }

        /// <summary>
        /// Computes the inertia of a closed mesh. Assumes counterclockwise winding.
        /// </summary>
        /// <typeparam name="TTriangleSource">Type of the triangle source.</typeparam>
        /// <param name="triangleSource">Source from which to retrieve a sequence of triangles.</param>
        /// <param name="mass">Mass of the mesh to scale the inertia tensor with.</param>
        /// <param name="volume">Volume of the mesh.</param>
        /// <param name="inertia">Inertia of the mesh.</param>
        public static void ComputeInertia<TTriangleSource>(ref TTriangleSource triangleSource, float mass, out float volume, out BodyInertia inertia) where TTriangleSource : ITriangleSource
        {
            float xx = 0, yy = 0, zz = 0, xy = 0, xz = 0, yz = 0, scaledVolume = 0;
            while (triangleSource.GetNextTriangle(out var a, out var b, out var c))
            {
                IntegrateTetrahedron(a, b, c, out var txx, out var tyy, out var tzz, out var txy, out var txz, out var tyz, out var tScaledVolume);
                xx += txx;
                yy += tyy;
                zz += tzz;
                xy += txy;
                xz += txz;
                yz += tyz;
                scaledVolume += tScaledVolume;
            }
            inertia.InverseMass = 1f / mass;
            FinalizeInertia(xx, yy, zz, xy, xz, yz, scaledVolume, mass, out volume, out inertia.InverseInertiaTensor);
        }

        /// <summary>
        /// Computes the inertia of a closed mesh. Assumes counterclockwise winding.
        /// </summary>
        /// <typeparam name="TTriangleSource">Type of the triangle source.</typeparam>
        /// <param name="triangleSource">Source from which to retrieve a sequence of triangles.</param>
        /// <param name="mass">Mass of the mesh to scale the inertia tensor with.</param>
        /// <param name="volume">Volume of the mesh.</param>
        /// <param name="inertia">Inertia of the mesh.</param>
        /// <param name="center">Center of mass of the mesh.</param>
        public static void ComputeInertia<TTriangleSource>(ref TTriangleSource triangleSource, float mass, out float volume, out BodyInertia inertia, out Vector3 center) where TTriangleSource : ITriangleSource
        {
            float xx = 0, yy = 0, zz = 0, xy = 0, xz = 0, yz = 0, scaledVolume = 0;
            center = default;
            while (triangleSource.GetNextTriangle(out var a, out var b, out var c))
            {
                IntegrateTetrahedron(a, b, c, out var txx, out var tyy, out var tzz, out var txy, out var txz, out var tyz, out var tScaledVolume);
                xx += txx;
                yy += tyy;
                zz += tzz;
                xy += txy;
                xz += txz;
                yz += tyz;
                scaledVolume += tScaledVolume;
                center += tScaledVolume * (a + b + c);
            }
            center /= scaledVolume * 4;
            inertia.InverseMass = 1f / mass;
            FinalizeInertia(xx, yy, zz, xy, xz, yz, scaledVolume, mass, out volume, out inertia.InverseInertiaTensor);
        }

        /// <summary>
        /// Computes the center of mass of a closed mesh.
        /// </summary>
        /// <typeparam name="TTriangleSource">Type of the triangle source.</typeparam>
        /// <param name="triangleSource">Source from which to retrieve a sequence of triangles.</param>
        /// <param name="volume">Volume of the mesh.</param>
        /// <param name="center">Center of mass of the mesh.</param>
        public static void ComputeCenterOfMass<TTriangleSource>(ref TTriangleSource triangleSource, out float volume, out Vector3 center) where TTriangleSource : ITriangleSource
        {
            center = default;
            var scaledVolume = 0f;
            while (triangleSource.GetNextTriangle(out var a, out var b, out var c))
            {
                Vector3x.Cross(c - a, b - a, out var n);
                var tScaledVolume = Vector3.Dot(n, a);
                scaledVolume += tScaledVolume;
                center += tScaledVolume * (a + b + c);
            }
            center /= scaledVolume * 4;
            volume = scaledVolume / 6;
        }

    }
}
