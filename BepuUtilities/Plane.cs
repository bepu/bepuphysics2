using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuUtilities
{
    /// <summary>
    /// Provides XNA-like plane functionality.
    /// </summary>
    public struct Plane
    {
        /// <summary>
        /// Normal of the plane.
        /// </summary>
        public Vector3 Normal;
        /// <summary>
        /// Negative distance to the plane from the origin along the normal.
        /// </summary>
        public float D;


        /// <summary>
        /// Constructs a new plane.
        /// </summary>
        /// <param name="position">A point on the plane.</param>
        /// <param name="normal">The normal of the plane.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Plane(ref Vector3 position, ref Vector3 normal)
        {
            D = -Vector3.Dot(position, normal);
            Normal = normal;
        }


        /// <summary>
        /// Constructs a new plane.
        /// </summary>
        /// <param name="position">A point on the plane.</param>
        /// <param name="normal">The normal of the plane.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Plane(Vector3 position, Vector3 normal)
            : this(ref position, ref normal)
        {

        }


        /// <summary>
        /// Constructs a new plane.
        /// </summary>
        /// <param name="normal">Normal of the plane.</param>
        /// <param name="d">Negative distance to the plane from the origin along the normal.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Plane(Vector3 normal, float d)
            : this(ref normal, d)
        {
        }

        /// <summary>
        /// Constructs a new plane.
        /// </summary>
        /// <param name="normal">Normal of the plane.</param>
        /// <param name="d">Negative distance to the plane from the origin along the normal.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Plane(ref Vector3 normal, float d)
        {
            this.Normal = normal;
            this.D = d;
        }

        /// <summary>
        /// Gets the dot product of the position offset from the plane along the plane's normal.
        /// </summary>
        /// <param name="v">Position to compute the dot product of.</param>
        /// <returns>Dot product.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float DotCoordinate(ref Vector3 v)
        {
            return Vector3.Dot(Normal, v) + D;
        }
    }
}
