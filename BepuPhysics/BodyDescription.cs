using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics
{
    /// <summary>
    /// Describes the thresholds for a body going to sleep.
    /// </summary>
    public struct BodyActivityDescription
    {
        /// <summary>
        /// Threshold of squared velocity under which the body is allowed to go to sleep. This is compared against dot(linearVelocity, linearVelocity) + dot(angularVelocity, angularVelocity).
        /// </summary>
        public float SleepThreshold;
        /// <summary>
        /// The number of time steps that the body must be under the sleep threshold before the body becomes a sleep candidate.
        /// Note that the body is not guaranteed to go to sleep immediately after meeting this minimum.
        /// </summary>
        public byte MinimumTimestepCountUnderThreshold;

        /// <summary>
        /// Creates a body activity description.
        /// </summary>
        /// <param name="sleepThreshold">Threshold of squared velocity under which the body is allowed to go to sleep. This is compared against dot(linearVelocity, linearVelocity) + dot(angularVelocity, angularVelocity).</param>
        /// <param name="minimumTimestepCountUnderThreshold">The number of time steps that the body must be under the sleep threshold before the body becomes a sleep candidate.
        /// Note that the body is not guaranteed to go to sleep immediately after meeting this minimum.</param>
        public BodyActivityDescription(float sleepThreshold, byte minimumTimestepCountUnderThreshold = 32)
        {
            SleepThreshold = sleepThreshold;
            MinimumTimestepCountUnderThreshold = minimumTimestepCountUnderThreshold;
        }

        /// <summary>
        /// Creates a body activity description. Uses a <see cref="MinimumTimestepCountUnderThreshold"/> of 32.
        /// </summary>
        /// <param name="sleepThreshold">Threshold of squared velocity under which the body is allowed to go to sleep. This is compared against dot(linearVelocity, linearVelocity) + dot(angularVelocity, angularVelocity).</param>
        /// Note that the body is not guaranteed to go to sleep immediately after meeting this minimum.</param>
        public static implicit operator BodyActivityDescription(float sleepThreshold)
        {
            return new BodyActivityDescription(sleepThreshold);
        }
    }

    /// <summary>
    /// Describes a body's state.
    /// </summary>
    public struct BodyDescription
    {
        /// <summary>
        /// Position and orientation of the body.
        /// </summary>
        public RigidPose Pose;
        /// <summary>
        /// Linear and angular velocity of the body.
        /// </summary>
        public BodyVelocity Velocity;
        /// <summary>
        /// Mass and inertia tensor of the body.
        /// </summary>
        public BodyInertia LocalInertia;
        /// <summary>
        /// Shape and collision detection settings for the body.
        /// </summary>
        public CollidableDescription Collidable;
        /// <summary>
        /// Sleeping settings for the body.
        /// </summary>
        public BodyActivityDescription Activity;

        //Convex shape helpers.
        /// <summary>
        /// Computes a decent default speculative margin for a shape based on its minimum and maximum radii.
        /// </summary>
        /// <typeparam name="TShape">Type of the shape to compute a speculative margin for.</typeparam>
        /// <param name="shape">Shape to compute a speculative margin for.</param>
        /// <returns>Speculative margin for the given shape.</returns>
        public static float GetDefaultSpeculativeMargin<TShape>(in TShape shape) where TShape : struct, IConvexShape
        {
            shape.ComputeAngularExpansionData(out var maximumRadius, out var maximumAngularExpansion);
            var minimumRadius = maximumRadius - maximumAngularExpansion;
            return 0.1f * (float)System.Math.Sqrt(maximumRadius * minimumRadius);
        }

        /// <summary>
        /// Computes a decent default activity description for a shape.
        /// </summary>
        /// <typeparam name="TShape">Type of the shape to create an activity description for.</typeparam>
        /// <param name="shape">Shape to create an activity description for.</param>
        /// <returns>Default activity description for the given shape.</returns>
        public static BodyActivityDescription GetDefaultActivity<TShape>(in TShape shape) where TShape : struct, IConvexShape
        {
            BodyActivityDescription activity;
            activity.MinimumTimestepCountUnderThreshold = 32;
            shape.ComputeAngularExpansionData(out var maximumRadius, out var maximumAngularExpansion);
            var minimumRadius = maximumRadius - maximumAngularExpansion;
            activity.SleepThreshold = minimumRadius * minimumRadius * 0.1f;
            return activity;
        }

        /// <summary>
        /// Creates a dynamic body description.
        /// </summary>
        /// <param name="pose">Pose of the body.</param>
        /// <param name="velocity">Initial velocity of the body.</param>
        /// <param name="inertia">Local inertia of the body.</param>
        /// <param name="collidable">Collidable to associate with the body.</param>
        /// <param name="activity">Activity settings for the body.</param>
        /// <returns>Constructed description for the body.</returns>
        public static BodyDescription CreateDynamic(RigidPose pose, BodyVelocity velocity, BodyInertia inertia, CollidableDescription collidable, BodyActivityDescription activity)
        {
            return new BodyDescription { Pose = pose, Velocity = velocity, LocalInertia = inertia, Activity = activity, Collidable = collidable };
        }

        /// <summary>
        /// Creates a dynamic body description with zero initial velocity.
        /// </summary>
        /// <param name="pose">Pose of the body.</param>
        /// <param name="inertia">Local inertia of the body.</param>
        /// <param name="collidable">Collidable to associate with the body.</param>
        /// <param name="activity">Activity settings for the body.</param>
        /// <returns>Constructed description for the body.</returns>
        public static BodyDescription CreateDynamic(RigidPose pose, BodyInertia inertia, CollidableDescription collidable, BodyActivityDescription activity)
        {
            return new BodyDescription { Pose = pose, LocalInertia = inertia, Activity = activity, Collidable = collidable };
        }

        /// <summary>
        /// Creates a dynamic body description with collidable, inertia, and activity descriptions generated from a convex shape. Adds the shape to the given shape set.
        /// </summary>
        /// <typeparam name="TConvexShape">Type of the shape to create a body for.</typeparam>
        /// <param name="pose">Pose of the body.</param>
        /// <param name="velocity">Initial velocity of the body.</param>
        /// <param name="mass">Mass of the body. The inertia tensor will be calculated based on this mass and the shape.</param>
        /// <param name="shapes">Shape collection to add the shape to.</param>
        /// <param name="shape">Shape to add to the shape set and to create the body from.</param>
        /// <returns>Constructed description for the body.</returns>
        public static BodyDescription CreateConvexDynamic<TConvexShape>(RigidPose pose, BodyVelocity velocity, float mass, Shapes shapes, in TConvexShape shape) where TConvexShape : unmanaged, IConvexShape
        {
            var description = new BodyDescription
            {
                Pose = pose,
                Velocity = velocity,
                Activity = GetDefaultActivity(shape),
                Collidable = shapes.Add(shape)
            };
            description.LocalInertia = shape.ComputeInertia(mass);
            return description;
        }

        /// <summary>
        /// Creates a dynamic body description with zero initial velocity and collidable, inertia, and activity descriptions generated from a convex shape. Adds the shape to the given shape set.
        /// </summary>
        /// <typeparam name="TConvexShape">Type of the shape to create a body for.</typeparam>
        /// <param name="pose">Pose of the body.</param>
        /// <param name="mass">Mass of the body. The inertia tensor will be calculated based on this mass and the shape.</param>
        /// <param name="shapes">Shape collection to add the shape to.</param>
        /// <param name="shape">Shape to add to the shape set and to create the body from.</param>
        /// <returns>Constructed description for the body.</returns>
        public static BodyDescription CreateConvexDynamic<TConvexShape>(RigidPose pose, float mass, Shapes shapes, in TConvexShape shape) where TConvexShape : unmanaged, IConvexShape
        {
            return CreateConvexDynamic(pose, default, mass, shapes, shape);
        }

        /// <summary>
        /// Creates a kinematic body description.
        /// </summary>
        /// <param name="pose">Pose of the body.</param>
        /// <param name="velocity">Initial velocity of the body.</param>
        /// <param name="collidable">Collidable to associate with the body.</param>
        /// <param name="activity">Activity settings for the body.</param>
        /// <returns>Constructed description for the body.</returns>
        public static BodyDescription CreateKinematic(RigidPose pose, BodyVelocity velocity, CollidableDescription collidable, BodyActivityDescription activity)
        {
            return new BodyDescription { Pose = pose, Velocity = velocity, Activity = activity, Collidable = collidable };
        }

        /// <summary>
        /// Creates a kinematic body description with zero initial velocity.
        /// </summary>
        /// <param name="pose">Pose of the body.</param>
        /// <param name="collidable">Collidable to associate with the body.</param>
        /// <param name="activity">Activity settings for the body.</param>
        /// <returns>Constructed description for the body.</returns>
        public static BodyDescription CreateKinematic(RigidPose pose, CollidableDescription collidable, BodyActivityDescription activity)
        {
            return new BodyDescription { Pose = pose, Activity = activity, Collidable = collidable };
        }

        /// <summary>
        /// Creates a kinematic body description with collidable and activity descriptions generated from a convex shape. Adds the shape to the given shape set.
        /// </summary>
        /// <typeparam name="TConvexShape">Type of the shape to create a body for.</typeparam>
        /// <param name="pose">Pose of the body.</param>
        /// <param name="velocity">Initial velocity of the body.</param>
        /// <param name="shapes">Shape collection to add the shape to.</param>
        /// <param name="shape">Shape to add to the shape set and to create the body from.</param>
        /// <returns>Constructed description for the body.</returns>
        public static BodyDescription CreateConvexKinematic<TConvexShape>(RigidPose pose, BodyVelocity velocity, Shapes shapes, in TConvexShape shape) where TConvexShape : unmanaged, IConvexShape
        {
            var description = new BodyDescription
            {
                Pose = pose,
                Velocity = velocity,
                Activity = GetDefaultActivity(shape),
                Collidable = shapes.Add(shape)
            };
            return description;
        }


        /// <summary>
        /// Creates a kinematic body description with zero initial velocity and collidable and activity descriptions generated from a convex shape. Adds the shape to the given shape set.
        /// </summary>
        /// <typeparam name="TConvexShape">Type of the shape to create a body for.</typeparam>
        /// <param name="pose">Pose of the body.</param>
        /// <param name="shapes">Shape collection to add the shape to.</param>
        /// <param name="shape">Shape to add to the shape set and to create the body from.</param>
        /// <returns>Constructed description for the body.</returns>
        public static BodyDescription CreateConvexKinematic<TConvexShape>(RigidPose pose, Shapes shapes, TConvexShape shape) where TConvexShape : unmanaged, IConvexShape
        {
            return CreateConvexKinematic(pose, default, shapes, shape);
        }
    }
}
