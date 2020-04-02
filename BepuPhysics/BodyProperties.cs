using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics
{
    //TODO: It's a little odd that this exists alongside the BepuUtilities.RigidTransform. The original reasoning was that rigid poses may end up having a non-FP32 representation.
    //We haven't taken advantage of that, so right now it's pretty much a pure duplicate.
    //When/if we take advantage of larger sizes, we'll have to closely analyze every use case of RigidPose to see if we need the higher precision or not.
    /// <summary>
    /// Represents a rigid transformation.
    /// </summary>
    public struct RigidPose
    {
        public Vector3 Position;
        //Note that we store a quaternion rather than a matrix3x3. While this often requires some overhead when performing vector transforms or extracting basis vectors, 
        //systems needing to interact directly with this representation are often terrifically memory bound. Spending the extra ALU time to convert to a basis can actually be faster
        //than loading the extra 5 elements needed to express the full 3x3 rotation matrix. Also, it's marginally easier to keep the rotation normalized over time.
        //There may be an argument for the matrix variant to ALSO be stored for some bandwidth-unconstrained stages, but don't worry about that until there's a reason to worry about it.
        public Quaternion Orientation;

        public static RigidPose Identity { get; } = new RigidPose(new Vector3());

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public RigidPose(in Vector3 position, in Quaternion orientation)
        {
            Position = position;
            Orientation = orientation;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public RigidPose(in Vector3 position)
        {
            Position = position;
            Orientation = Quaternion.Identity;
        }

        /// <summary>
        /// Transforms a vector by the rigid pose: v * pose.Orientation + pose.Position.
        /// </summary>
        /// <param name="v">Vector to transform.</param>
        /// <param name="pose">Pose to transform the vector with.</param>
        /// <param name="result">Transformed vector.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Transform(in Vector3 v, in RigidPose pose, out Vector3 result)
        {
            QuaternionEx.TransformWithoutOverlap(v, pose.Orientation, out var rotated);
            result = rotated + pose.Position;
        }
        /// <summary>
        /// Transforms a vector by the inverse of a rigid pose: (v - pose.Position) * pose.Orientation^-1.
        /// </summary>
        /// <param name="v">Vector to transform.</param>
        /// <param name="pose">Pose to invert and transform the vector with.</param>
        /// <param name="result">Transformed vector.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformByInverse(in Vector3 v, in RigidPose pose, out Vector3 result)
        {
            var translated = v - pose.Position;
            QuaternionEx.Conjugate(pose.Orientation, out var conjugate);
            QuaternionEx.TransformWithoutOverlap(translated, conjugate, out result);
        }
        /// <summary>
        /// Inverts the rigid transformation of the pose.
        /// </summary>
        /// <param name="pose">Pose to invert.</param>
        /// <param name="inverse">Inverse of the pose.</param>
        public static void Invert(in RigidPose pose, out RigidPose inverse)
        {
            QuaternionEx.Conjugate(pose.Orientation, out inverse.Orientation);
            QuaternionEx.Transform(-pose.Position, inverse.Orientation, out inverse.Position);
        }

        /// <summary>
        /// Concatenates one rigid transform with another. The resulting transform is equivalent to performing transform a followed by transform b.
        /// </summary>
        /// <param name="a">First transform to concatenate.</param>
        /// <param name="b">Second transform to concatenate.</param>
        /// <param name="result">Result of the concatenation.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void MultiplyWithoutOverlap(in RigidPose a, in RigidPose b, out RigidPose result)
        {
            QuaternionEx.ConcatenateWithoutOverlap(a.Orientation, b.Orientation, out result.Orientation);
            QuaternionEx.Transform(a.Position, b.Orientation, out var rotatedTranslationA);
            result.Position = rotatedTranslationA + b.Position;
        }
    }

    public struct BodyVelocity
    {
        public Vector3 Linear;
        public Vector3 Angular;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public BodyVelocity(in Vector3 linear)
        {
            Linear = linear;
            Angular = default;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public BodyVelocity(in Vector3 linear, in Vector3 angular)
        {
            Linear = linear;
            Angular = angular;
        }
    }

    public struct BodyInertia
    {
        public Symmetric3x3 InverseInertiaTensor;
        public float InverseMass;
    }

    public struct RigidPoses
    {
        public Vector3Wide Position;
        public QuaternionWide Orientation;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Broadcast(in RigidPose pose, out RigidPoses poses)
        {
            Vector3Wide.Broadcast(pose.Position, out poses.Position);
            QuaternionWide.Broadcast(pose.Orientation, out poses.Orientation);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WriteFirst(in RigidPose pose, ref RigidPoses poses)
        {
            Vector3Wide.WriteFirst(pose.Position, ref poses.Position);
            QuaternionWide.WriteFirst(pose.Orientation, ref poses.Orientation);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ReadFirst(in RigidPoses poses, out RigidPose pose)
        {
            Vector3Wide.ReadFirst(poses.Position, out pose.Position);
            QuaternionWide.ReadFirst(poses.Orientation, out pose.Orientation);
        }
    }

    public struct BodyVelocities
    {
        public Vector3Wide Linear;
        public Vector3Wide Angular;
    }

    public struct BodyInertias
    {
        public Symmetric3x3Wide InverseInertiaTensor;
        //Note that the inverse mass is included in the BodyInertias bundle. InverseMass is rotationally invariant, so it doesn't need to be updated...
        //But it's included alongside the rotated inertia tensor because to split it out would require that constraint presteps suffer another cache miss when they
        //gather the inverse mass in isolation. (From the solver's perspective, inertia/mass gathering is incoherent.)
        public Vector<float> InverseMass;
    }

    public struct BodyActivity
    {
        /// <summary>
        /// Threshold of squared velocity under which the body is allowed to go to sleep. This is compared against dot(linearVelocity, linearVelocity) + dot(angularVelocity, angularVelocity).
        /// Setting this to a negative value guarantees the body cannot go to sleep without user action.
        /// </summary>
        public float SleepThreshold;
        /// <summary>
        /// The number of time steps that the body must be under the sleep threshold before the body becomes a sleeping candidate.
        /// Note that the body is not guaranteed to go to sleep immediately after meeting this minimum.
        /// </summary>
        public byte MinimumTimestepsUnderThreshold;

        //Note that all values beyond this point are runtime set. The user should virtually never need to modify them. 
        //We do not constrain write access by default, instead opting to leave it open for advanced users to mess around with.
        //TODO: If people misuse these, we should internalize them in a case by case basis.

        /// <summary>
        /// If the body is awake, this is the number of time steps that the body has had a velocity below the sleep threshold.
        /// </summary>
        public byte TimestepsUnderThresholdCount;
        //Note that this flag is held alongside the other sleeping data, despite the fact that the traversal only needs the SleepCandidate state.
        //This is primarily for simplicity, but also note that the dominant accessor of this field is actually the sleep candidacy computation. Traversal doesn't visit every
        //body every frame, but sleep candidacy analysis does.
        //The reason why this flag exists at all is just to prevent traversal from being aware of the logic behind candidacy managemnt.
        //It doesn't cost anything extra to store this; it fits within the 8 byte layout.
        /// <summary>
        /// True if this body is a candidate for being slept. If all the bodies that it is connected to by constraints are also candidates, this body may go to sleep.
        /// </summary>
        public bool SleepCandidate;
    }
}
