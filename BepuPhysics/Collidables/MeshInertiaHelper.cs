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
        /// Integrates the inertia contribution of a tetrahedron with vertices at (0,0,0), a, b, and c.
        /// </summary>
        /// <param name="a">Second vertex of the tetrahedron.</param>
        /// <param name="b">Third vertex of the tetrahedron.</param>
        /// <param name="c">Fourth vertex of the tetrahedron.</param>
        /// <param name="scaledContribution">Scaled contribution of this tetrahedron to the inertia tensor.</param>
        /// <param name="scaledVolume">Six times the volume of the tetrahedron.</param>
        public static void IntegrateTetrahedron(in Vector3 a, in Vector3 b, in Vector3 c,
            out Symmetric3x3 scaledContribution,
            out float scaledVolume)
        {
            //This is just taken straight out of v1, derivation from Explicit Exact Formulas for the 3-D Tetrahedron Inertia Tensor in Terms of its Vertex Coordinates.
            //Could do better than this; doesn't bother even trying to vectorize.

            scaledVolume = a.X * (c.Z * b.Y - c.Y * b.Z) -
                           c.X * (a.Z * b.Y - a.Y * b.Z) +
                           b.X * (a.Z * c.Y - a.Y * c.Z);

            scaledContribution.XX = scaledVolume * (a.Y * a.Y + a.Y * c.Y + c.Y * c.Y + a.Y * b.Y + c.Y * b.Y + b.Y * b.Y +
                                                    a.Z * a.Z + a.Z * c.Z + c.Z * c.Z + a.Z * b.Z + c.Z * b.Z + b.Z * b.Z);
            scaledContribution.YY = scaledVolume * (a.X * a.X + a.X * c.X + c.X * c.X + a.X * b.X + c.X * b.X + b.X * b.X +
                                                    a.Z * a.Z + a.Z * c.Z + c.Z * c.Z + a.Z * b.Z + c.Z * b.Z + b.Z * b.Z);
            scaledContribution.ZZ = scaledVolume * (a.X * a.X + a.X * c.X + c.X * c.X + a.X * b.X + c.X * b.X + b.X * b.X +
                                                    a.Y * a.Y + a.Y * c.Y + c.Y * c.Y + a.Y * b.Y + c.Y * b.Y + b.Y * b.Y);
            scaledContribution.YX = scaledVolume * (2 * a.X * a.Y + c.X * a.Y + b.X * a.Y + a.X * c.Y + 2 * c.X * c.Y + b.X * c.Y + a.X * b.Y + c.X * b.Y + 2 * b.X * b.Y);
            scaledContribution.ZX = scaledVolume * (2 * a.X * a.Z + c.X * a.Z + b.X * a.Z + a.X * c.Z + 2 * c.X * c.Z + b.X * c.Z + a.X * b.Z + c.X * b.Z + 2 * b.X * b.Z);
            scaledContribution.ZY = scaledVolume * (2 * a.Y * a.Z + c.Y * a.Z + b.Y * a.Z + a.Y * c.Z + 2 * c.Y * c.Z + b.Y * c.Z + a.Y * b.Z + c.Y * b.Z + 2 * b.Y * b.Z);

        }

        /// <summary>
        /// Finalizes the inertia tensor from tetrahedral integration.
        /// </summary>
        /// <param name="summedContributions">Summed scaled tetrahedral contributions.</param>
        /// <param name="scaledVolume">Scaled volume of the mesh.</param>
        /// <param name="mass">Mass to scale the inertia tensor with.</param>
        /// <param name="volume">Computed volume of the mesh.</param>
        /// <param name="inertia">Computed inertia tensor of the mesh.</param>
        public static void FinalizeInertia(in Symmetric3x3 summedContributions, float scaledVolume, float mass,
            out float volume, out Symmetric3x3 inertia)
        {
            volume = scaledVolume / 6;
            float scaledDensity = mass / volume;
            float diagonalFactor = scaledDensity / 60;
            float offFactor = scaledDensity / -120;
            inertia.XX = summedContributions.XX * diagonalFactor;
            inertia.YX = summedContributions.YX * offFactor;
            inertia.YY = summedContributions.YY * diagonalFactor;
            inertia.ZX = summedContributions.ZX * offFactor;
            inertia.ZY = summedContributions.ZY * offFactor;
            inertia.ZZ = summedContributions.ZZ * diagonalFactor;
        }

        /// <summary>
        /// Computes the inertia of a closed mesh. Assumes counterclockwise winding.
        /// </summary>
        /// <typeparam name="TTriangleSource">Type of the triangle source.</typeparam>
        /// <param name="triangleSource">Source from which to retrieve a sequence of triangles.</param>
        /// <param name="mass">Mass of the mesh to scale the inertia tensor with.</param>
        /// <param name="volume">Volume of the mesh.</param>
        /// <param name="inertia">Inertia tensor of the mesh.</param>
        public static void ComputeInertia<TTriangleSource>(ref TTriangleSource triangleSource, float mass, out float volume, out Symmetric3x3 inertia) where TTriangleSource : ITriangleSource
        {
            float scaledVolume = 0;
            Symmetric3x3 summedContributions = default;
            while (triangleSource.GetNextTriangle(out var a, out var b, out var c))
            {
                IntegrateTetrahedron(a, b, c, out var scaledContribution, out var tScaledVolume);
                Symmetric3x3.Add(scaledContribution, summedContributions, out summedContributions);
                scaledVolume += tScaledVolume;
            }
            FinalizeInertia(summedContributions, scaledVolume, mass, out volume, out inertia);
        }

        /// <summary>
        /// Computes the inertia of a closed mesh. Assumes counterclockwise winding.
        /// </summary>
        /// <typeparam name="TTriangleSource">Type of the triangle source.</typeparam>
        /// <param name="triangleSource">Source from which to retrieve a sequence of triangles.</param>
        /// <param name="mass">Mass of the mesh to scale the inertia tensor with.</param>
        /// <param name="volume">Volume of the mesh.</param>
        /// <param name="inertia">Inertia tensor of the mesh.</param>
        /// <param name="center">Center of mass of the mesh.</param>
        public static void ComputeInertia<TTriangleSource>(ref TTriangleSource triangleSource, float mass, out float volume, out Symmetric3x3 inertia, out Vector3 center) where TTriangleSource : ITriangleSource
        {
            float scaledVolume = 0;
            Symmetric3x3 summedContributions = default;
            center = default;
            while (triangleSource.GetNextTriangle(out var a, out var b, out var c))
            {
                IntegrateTetrahedron(a, b, c, out var scaledContribution, out var tScaledVolume);
                Symmetric3x3.Add(scaledContribution, summedContributions, out summedContributions);
                scaledVolume += tScaledVolume;
                center += tScaledVolume * (a + b + c);
            }
            center /= scaledVolume * 4;
            FinalizeInertia(summedContributions, scaledVolume, mass, out volume, out inertia);
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
