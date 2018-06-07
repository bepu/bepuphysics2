using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuUtilities
{
    ///<summary>
    /// A transformation composed of a linear transformation and a translation.
    ///</summary>
    public struct AffineTransform
    {
        ///<summary>
        /// Translation in the affine transform.
        ///</summary>
        public Vector3 Translation;
        /// <summary>
        /// Linear transform in the affine transform.
        /// </summary>
        public Matrix3x3 LinearTransform;


        ///<summary>
        /// Gets the identity affine transform.
        ///</summary>
        public static AffineTransform Identity
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                var t = new AffineTransform { LinearTransform = Matrix3x3.Identity };
                return t;
            }
        }

        ///<summary>
        /// Constructs a new affine transform.
        ///</summary>
        ///<param name="translation">Translation to use in the transform.</param>
        public AffineTransform(in Vector3 translation)
        {
            LinearTransform = Matrix3x3.Identity;
            Translation = translation;
        }

        ///<summary>
        /// Constructs a new affine tranform.
        ///</summary>
        ///<param name="orientation">Orientation to use as the linear transform.</param>
        ///<param name="translation">Translation to use in the transform.</param>
        public AffineTransform(in Quaternion orientation, in Vector3 translation)
        {
            Matrix3x3.CreateFromQuaternion(orientation, out LinearTransform);
            Translation = translation;
        }

        ///<summary>
        /// Constructs a new affine transform.
        ///</summary>
        ///<param name="scaling">Scaling to apply in the linear transform.</param>
        ///<param name="orientation">Orientation to apply in the linear transform.</param>
        ///<param name="translation">Translation to apply.</param>
        public AffineTransform(in Vector3 scaling, in Quaternion orientation, in Vector3 translation)
        {
            //Create an SRT transform.
            Matrix3x3.CreateScale(scaling, out LinearTransform);
            Matrix3x3.CreateFromQuaternion(orientation, out var rotation);
            Matrix3x3.Multiply(LinearTransform, rotation, out LinearTransform);
            Translation = translation;
        }

         ///<summary>
        /// Constructs a new affine transform.
        ///</summary>
        ///<param name="linearTransform">The linear transform component.</param>
        ///<param name="translation">Translation component of the transform.</param>
        public AffineTransform(in Matrix3x3 linearTransform, in Vector3 translation)
        {
            LinearTransform = linearTransform;
            Translation = translation;

        }

        ///<summary>
        /// Transforms a vector by an affine transform.
        ///</summary>
        ///<param name="position">Position to transform.</param>
        ///<param name="transform">Transform to apply.</param>
        ///<param name="transformed">Transformed position.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Transform(in Vector3 position, in AffineTransform transform, out Vector3 transformed)
        {
            Matrix3x3.Transform(position, transform.LinearTransform, out transformed);
            transformed += transform.Translation;
        }

        ///<summary>
        /// Inverts an affine transform.
        ///</summary>
        ///<param name="transform">Transform to invert.</param>
        /// <param name="inverse">Inverse of the transform.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Invert(in AffineTransform transform, out AffineTransform inverse)
        {
            Matrix3x3.Invert(transform.LinearTransform, out inverse.LinearTransform);
            Matrix3x3.Transform(transform.Translation, inverse.LinearTransform, out inverse.Translation);
            inverse.Translation = -inverse.Translation;
        }

        ///<summary>
        /// Inverts a rigid transform.
        ///</summary>
        ///<param name="transform">Transform to invert.</param>
        /// <param name="inverse">Inverse of the transform.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void InvertRigid(in AffineTransform transform, out AffineTransform inverse)
        {
            Matrix3x3.Transpose(transform.LinearTransform, out inverse.LinearTransform);
            Matrix3x3.Transform(transform.Translation, inverse.LinearTransform, out inverse.Translation);
            inverse.Translation = -inverse.Translation;
        }

        /// <summary>
        /// Multiplies a transform by another transform.
        /// </summary>
        /// <param name="a">First transform.</param>
        /// <param name="b">Second transform.</param>
        /// <param name="transform">Combined transform.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Multiply(in AffineTransform a, in AffineTransform b, out AffineTransform transform)
        {
            Matrix3x3.Transform(a.Translation, b.LinearTransform, out var translation);
            transform.Translation = b.Translation + translation;
            Matrix3x3.Multiply(a.LinearTransform, b.LinearTransform, out transform.LinearTransform);
        }



    }
}