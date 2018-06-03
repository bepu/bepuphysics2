

using System.Numerics;

namespace BepuUtilities
{
    ///<summary>
    /// Transform composed of a rotation and translation.
    ///</summary>
    public struct RigidTransform
    {
        ///<summary>
        /// Translation component of the transform.
        ///</summary>
        public Vector3 Position;
        ///<summary>
        /// Rotation component of the transform.
        ///</summary>
        public Quaternion Orientation;

        ///<summary>
        /// Constructs a new rigid transform.
        ///</summary>
        ///<param name="position">Translation component of the transform.</param>
        ///<param name="orientation">Rotation component of the transform.</param>
        public RigidTransform(Vector3 position, Quaternion orientation)
        {
            Position = position;
            Orientation = orientation;
        }

        ///<summary>
        /// Constructs a new rigid transform.
        ///</summary>
        ///<param name="position">Translation component of the transform.</param>
        public RigidTransform(Vector3 position)
        {
            Position = position;
            Orientation = Quaternion.Identity;
        }

        ///<summary>
        /// Constructs a new rigid transform.
        ///</summary>
        ///<param name="orienation">Rotation component of the transform.</param>
        public RigidTransform(Quaternion orienation)
        {
            Position = new Vector3();
            Orientation = orienation;
        }

    

        ///<summary>
        /// Gets the identity rigid transform.
        ///</summary>
        public static RigidTransform Identity
        {
            get
            {
                var t = new RigidTransform {Orientation = Quaternion.Identity, Position = new Vector3()};
                return t;
            }
        }

        /// <summary>
        /// Inverts a rigid transform.
        /// </summary>
        /// <param name="transform">Transform to invert.</param>
        /// <param name="inverse">Inverse of the transform.</param>
        public static void Invert(ref RigidTransform transform, out RigidTransform inverse)
        {
            Quaternion.Conjugate(transform.Orientation, out inverse.Orientation);
            Quaternion.Transform(transform.Position, inverse.Orientation, out inverse.Position);
            inverse.Position = -inverse.Position;
        }

        ///<summary>
        /// Concatenates a rigid transform with another rigid transform. Assumes input and output do not overlap.
        ///</summary>
        ///<param name="a">The first rigid transform.</param>
        ///<param name="b">The second rigid transform.</param>
        ///<param name="combined">Concatenated rigid transform.</param>
        public static void MultiplyWithoutOverlap(ref RigidTransform a, ref RigidTransform b, out RigidTransform combined)
        {
            Vector3 intermediate;
            Quaternion.Transform(a.Position, b.Orientation, out intermediate);
            combined.Position = intermediate + b.Position;
            Quaternion.ConcatenateWithoutOverlap(a.Orientation, b.Orientation, out combined.Orientation);

        }

        ///<summary>
        /// Concatenates a rigid transform with another rigid transform's inverse. Assumes input and output do not overlap.
        ///</summary>
        ///<param name="a">The first rigid transform.</param>
        ///<param name="b">The second rigid transform whose inverse will be concatenated to the first.</param>
        ///<param name="combinedTransform">Combined rigid transform.</param>
        public static void MultiplyByInverseWithoutOverlap(ref RigidTransform a, ref RigidTransform b, out RigidTransform combinedTransform)
        {
            Invert(ref b, out combinedTransform);
            MultiplyWithoutOverlap(ref a, ref combinedTransform, out combinedTransform);
        }

        ///<summary>
        /// Transforms a position by a rigid transform.
        ///</summary>
        ///<param name="position">Position to transform.</param>
        ///<param name="transform">Transform to apply.</param>
        ///<param name="result">Transformed position.</param>
        public static void Transform(ref Vector3 position, ref RigidTransform transform, out Vector3 result)
        {
            Vector3 intermediate;
            Quaternion.Transform(position, transform.Orientation, out intermediate);
            result = intermediate + transform.Position;
        }


        ///<summary>
        /// Transforms a position by a rigid transform's inverse.
        ///</summary>
        ///<param name="position">Position to transform.</param>
        ///<param name="transform">Transform to invert and apply.</param>
        ///<param name="result">Transformed position.</param>
        public static void TransformByInverse(ref Vector3 position, ref RigidTransform transform, out Vector3 result)
        {
            Quaternion orientation;
            Vector3 intermediate = position - transform.Position;
            Quaternion.Conjugate(transform.Orientation, out orientation);
            Quaternion.Transform(intermediate, orientation, out result);
        }


    }
}
