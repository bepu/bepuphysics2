using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace BepuPhysics
{
    /// <summary>
    /// Describes the pose and velocity of a body.
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Size = 64, Pack = 1)]
    public struct MotionState
    {
        internal const int OffsetToOrientationX = 0;
        internal const int OffsetToOrientationY = 1;
        internal const int OffsetToOrientationZ = 2;
        internal const int OffsetToOrientationW = 3;
        internal const int OffsetToPositionX = 4;
        internal const int OffsetToPositionY = 5;
        internal const int OffsetToPositionZ = 6;
        internal const int OffsetToLinearX = 8;
        internal const int OffsetToLinearY = 9;
        internal const int OffsetToLinearZ = 10;
        internal const int OffsetToAngularX = 12;
        internal const int OffsetToAngularY = 13;
        internal const int OffsetToAngularZ = 14;

        internal const int ByteOffsetToPositionX = OffsetToPositionX * 4;
        internal const int ByteOffsetToPositionY = OffsetToPositionY * 4;
        internal const int ByteOffsetToPositionZ = OffsetToPositionZ * 4;
        internal const int ByteOffsetToOrientationX = OffsetToOrientationX * 4;
        internal const int ByteOffsetToOrientationY = OffsetToOrientationY * 4;
        internal const int ByteOffsetToOrientationZ = OffsetToOrientationZ * 4;
        internal const int ByteOffsetToOrientationW = OffsetToOrientationW * 4;
        internal const int ByteOffsetToLinearX = OffsetToLinearX * 4;
        internal const int ByteOffsetToLinearY = OffsetToLinearY * 4;
        internal const int ByteOffsetToLinearZ = OffsetToLinearZ * 4;
        internal const int ByteOffsetToAngularX = OffsetToAngularX * 4;
        internal const int ByteOffsetToAngularY = OffsetToAngularY * 4;
        internal const int ByteOffsetToAngularZ = OffsetToAngularZ * 4;


        /// <summary>
        /// Pose of the body.
        /// </summary>
        public RigidPose Pose;
        /// <summary>
        /// Linear and angular velocity of the body.
        /// </summary>
        public BodyVelocity Velocity;
    }

    //TODO: It's a little odd that this exists alongside the BepuUtilities.RigidTransform. The original reasoning was that rigid poses may end up having a non-FP32 representation.
    //We haven't taken advantage of that, so right now it's pretty much a pure duplicate.
    //When/if we take advantage of larger sizes, we'll have to closely analyze every use case of RigidPose to see if we need the higher precision or not.
    /// <summary>
    /// Represents a rigid transformation.
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Size = 32, Pack = 1)]
    public struct RigidPose
    {
        //Note that we store a quaternion rather than a matrix3x3. While this often requires some overhead when performing vector transforms or extracting basis vectors, 
        //systems needing to interact directly with this representation are often terrifically memory bound. Spending the extra ALU time to convert to a basis can actually be faster
        //than loading the extra 5 elements needed to express the full 3x3 rotation matrix. Also, it's marginally easier to keep the rotation normalized over time.
        //There may be an argument for the matrix variant to ALSO be stored for some bandwidth-unconstrained stages, but don't worry about that until there's a reason to worry about it.
        /// <summary>
        /// Orientation of the pose.
        /// </summary>
        public Quaternion Orientation;
        /// <summary>
        /// Position of the pose.
        /// </summary>
        public Vector3 Position;

        /// <summary>
        /// Returns a pose with a position at (0,0,0) and identity orientation.
        /// </summary>
        public static RigidPose Identity => new RigidPose(default);

        /// <summary>
        /// Creates a rigid pose with the given position and orientation.
        /// </summary>
        /// <param name="position">Position of the pose.</param>
        /// <param name="orientation">Orientation of the pose.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public RigidPose(Vector3 position, Quaternion orientation)
        {
            Position = position;
            Orientation = orientation;
        }

        /// <summary>
        /// Creates a rigid pose with the given position and identity orientation.
        /// </summary>
        /// <param name="position">Position of the pose.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public RigidPose(Vector3 position)
        {
            Position = position;
            Orientation = Quaternion.Identity;
        }

        /// <summary>
        /// Creates a pose by treating a <see cref="Vector3"/> as a position. Orientation is set to identity.
        /// </summary>
        /// <param name="position">Position to use in the pose.</param>
        public static implicit operator RigidPose(Vector3 position)
        {
            return new RigidPose(position);
        }

        /// <summary>
        /// Creates a pose by treating a <see cref="Quaternion"/> as an orientation in the pose. Position is set to zero.
        /// </summary>
        /// <param name="orientation">Orientation to use in the pose.</param>
        public static implicit operator RigidPose(Quaternion orientation)
        {
            return new RigidPose(default, orientation);
        }

        /// <summary>
        /// Creates a pose from a tuple of a position and orientation.
        /// </summary>
        /// <param name="poseComponents">Position and orientation to use in the pose.</param>
        public static implicit operator RigidPose((Vector3 position, Quaternion orientation) poseComponents)
        {
            return new RigidPose(poseComponents.position, poseComponents.orientation);
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

    /// <summary>
    /// Linear and angular velocity for a body.
    /// </summary>
    [StructLayout(LayoutKind.Explicit, Size = 32)]
    public struct BodyVelocity
    {
        /// <summary>
        /// Linear velocity associated with the body.
        /// </summary>
        [FieldOffset(0)]
        public Vector3 Linear;

        /// <summary>
        /// Angular velocity associated with the body.
        /// </summary>
        [FieldOffset(16)]
        public Vector3 Angular;

        /// <summary>
        /// Creates a new set of body velocities. Angular velocity is set to zero.
        /// </summary>
        /// <param name="linear">Linear velocity to use for the body.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public BodyVelocity(Vector3 linear)
        {
            Linear = linear;
            Angular = default;
        }

        /// <summary>
        /// Creates a new set of body velocities.
        /// </summary>
        /// <param name="linear">Linear velocity to use for the body.</param>
        /// <param name="angular">Angular velocity to use for the body.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public BodyVelocity(Vector3 linear, Vector3 angular)
        {
            Linear = linear;
            Angular = angular;
        }

        /// <summary>
        /// Creates a body velocity by treating a <see cref="Vector3"/> as a linear velocity. Angular velocity is set to zero.
        /// </summary>
        /// <param name="linearVelocity">Linear velocity to use in the body velocity.</param>
        public static implicit operator BodyVelocity(Vector3 linearVelocity)
        {
            return new BodyVelocity(linearVelocity);
        }

        /// <summary>
        /// Creates a body velocity from a tuple of linear and angular velocities..
        /// </summary>
        /// <param name="velocities">Velocities to use in the body velocity.</param>
        public static implicit operator BodyVelocity((Vector3 linearVelocity, Vector3 angularVelocity) velocities)
        {
            return new BodyVelocity(velocities.linearVelocity, velocities.angularVelocity);
        }
    }

    /// <summary>
    /// Stores the inertia for a body.
    /// </summary>
    /// <remarks>This representation stores the inverse mass and inverse inertia tensor. Most of the high frequency use cases in the engine naturally use the inverse.</remarks>
    [StructLayout(LayoutKind.Sequential, Size = 32, Pack = 4)]
    public struct BodyInertia
    {
        /// <summary>
        /// Inverse of the body's inertia tensor.
        /// </summary>
        public Symmetric3x3 InverseInertiaTensor;
        /// <summary>
        /// Inverse of the body's mass.
        /// </summary>
        public float InverseMass;
    }

    /// <summary>
    /// Stores the local and world views of a body's inertia, packed together for efficient access.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct BodyInertias
    {
        /// <summary>
        /// Local inertia of the body.
        /// </summary>
        public BodyInertia Local;
        /// <summary>
        /// Transformed world inertia of the body. Note that this is only valid between the velocity integration that updates it and the pose integration that follows.
        /// Outside of that execution window, this should be considered undefined.
        /// </summary>
        /// <remarks>
        /// We cache this here because velocity integration wants both the local and world inertias, and any integration happening within the solver will do so without the benefit of sequential loads.
        /// In that context, being able to load a single cache line to grab both local and world inertia helps quite a lot.</remarks>
        public BodyInertia World;
    }

    /// <summary>
    /// Stores all body information needed by the solver together.
    /// </summary>
    /// <remarks>
    /// With 2.4's revamp of the solver, every solving stage loads pose, velocity, and inertia for every body in each constraint.
    /// L2 prefetchers often fetch memory in even-odd pairs of cache lines (see https://www.intel.com/content/dam/www/public/us/en/documents/manuals/64-ia-32-architectures-optimization-manual.pdf#page=162).
    /// Since L2 is likely pulling in adjacent cache lines when loading either motion state or inertias, they might as well live together in one block.
    /// Note that this goes along with a change to the buffer pool's default alignment to 128 bytes.
    /// </remarks>
    [StructLayout(LayoutKind.Sequential)]
    public struct SolverState
    {
        /// <summary>
        /// Pose and velocity information for the body.
        /// </summary>
        public MotionState Motion;
        /// <summary>
        /// Inertia information for the body.
        /// </summary>
        public BodyInertias Inertia;
    }


    public struct RigidPoseWide
    {
        public Vector3Wide Position;
        public QuaternionWide Orientation;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Broadcast(in RigidPose pose, out RigidPoseWide poses)
        {
            Vector3Wide.Broadcast(pose.Position, out poses.Position);
            QuaternionWide.Broadcast(pose.Orientation, out poses.Orientation);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WriteFirst(in RigidPose pose, ref RigidPoseWide poses)
        {
            Vector3Wide.WriteFirst(pose.Position, ref poses.Position);
            QuaternionWide.WriteFirst(pose.Orientation, ref poses.Orientation);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ReadFirst(in RigidPoseWide poses, out RigidPose pose)
        {
            Vector3Wide.ReadFirst(poses.Position, out pose.Position);
            QuaternionWide.ReadFirst(poses.Orientation, out pose.Orientation);
        }
    }

    public struct BodyVelocityWide
    {
        public Vector3Wide Linear;
        public Vector3Wide Angular;
    }

    public struct BodyInertiaWide
    {
        public Symmetric3x3Wide InverseInertiaTensor;
        //Note that the inverse mass is included in the bundle. InverseMass is rotationally invariant, so it doesn't need to be updated...
        //But it's included alongside the rotated inertia tensor because to split it out would require that constraint presteps suffer another cache miss when they
        //gather the inverse mass in isolation. (From the solver's perspective, inertia/mass gathering is incoherent.)
        public Vector<float> InverseMass;
    }

    /// <summary>
    /// Describes how a body sleeps, and its current state with respect to sleeping.
    /// </summary>
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
