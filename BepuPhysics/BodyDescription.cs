using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;
using System.Numerics;
using System.Runtime.CompilerServices;
using Quaternion = BepuUtilities.Quaternion;

namespace BepuPhysics
{
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

    }

    public struct BodyDescription
    {
        public RigidPose Pose;
        public BodyInertia LocalInertia;
        public BodyVelocity Velocity;
        public CollidableDescription Collidable;
        public BodyActivityDescription Activity;

        /// <summary>
        /// Builds a new body description.
        /// </summary>
        /// <param name="position">Position of the body.</param>
        /// <param name="orientation">Orientation of the body.</param>
        /// <param name="velocity">Velocity of the body.</param>
        /// <param name="localInertia">Local inertia of the body. If default initialized (all values set to zero), the body is considered kinematic.</param>
        /// <param name="collidable">Collidable description for the body.</param>
        /// <param name="activity">Activity description for the body.</param>
        public BodyDescription(in Vector3 position, in Quaternion orientation, in BodyVelocity velocity, in BodyInertia localInertia, in CollidableDescription collidable, in BodyActivityDescription activity)
        {
            Pose.Position = position;
            Pose.Orientation = orientation;
            LocalInertia = localInertia;
            Velocity = velocity;
            Collidable = collidable;
            Activity = activity;
        }

        //General direct constructors.

        /// <summary>
        /// Builds a new body description with discrete continuity.
        /// </summary>
        /// <param name="position">Position of the body.</param>
        /// <param name="orientation">Orientation of the body.</param>
        /// <param name="velocity">Velocity of the body.</param>
        /// <param name="localInertia">Local inertia of the body. If default initialized (all values set to zero), the body is considered kinematic.</param>
        /// <param name="shapeIndex">Index of the shape in the simulation shape set.</param>
        /// <param name="speculativeMargin">Distance beyond the surface of the body to allow speculative contacts to be generated.</param>
        /// <param name="activity">Activity description for the body.</param>
        public BodyDescription(in Vector3 position, in Quaternion orientation, in BodyVelocity velocity, in BodyInertia localInertia, TypedIndex shapeIndex, float speculativeMargin, in BodyActivityDescription activity)
            : this(position, orientation, velocity, localInertia, new CollidableDescription { Shape = shapeIndex, SpeculativeMargin = speculativeMargin }, activity)
        {
        }

        /// <summary>
        /// Builds a new body description with discrete continuity..
        /// </summary>
        /// <param name="position">Position of the body.</param>
        /// <param name="velocity">Velocity of the body.</param>
        /// <param name="localInertia">Local inertia of the body. If default initialized (all values set to zero), the body is considered kinematic.</param>
        /// <param name="shapeIndex">Index of the shape in the simulation shape set.</param>
        /// <param name="speculativeMargin">Distance beyond the surface of the body to allow speculative contacts to be generated.</param>
        /// <param name="activity">Activity description for the body.</param>
        public BodyDescription(in Vector3 position, in BodyVelocity velocity, in BodyInertia localInertia, TypedIndex shapeIndex, float speculativeMargin, in BodyActivityDescription activity)
            : this(position, Quaternion.Identity, velocity, localInertia, new CollidableDescription { Shape = shapeIndex, SpeculativeMargin = speculativeMargin }, activity)
        {
        }
        /// <summary>
        /// Builds a new body description with discrete continuity.
        /// </summary>
        /// <param name="position">Position of the body.</param>
        /// <param name="orientation">Orientation of the body.</param>
        /// <param name="localInertia">Local inertia of the body. If default initialized (all values set to zero), the body is considered kinematic.</param>
        /// <param name="shapeIndex">Index of the shape in the simulation shape set.</param>
        /// <param name="speculativeMargin">Distance beyond the surface of the body to allow speculative contacts to be generated.</param>
        /// <param name="activity">Activity description for the body.</param>
        public BodyDescription(in Vector3 position, in Quaternion orientation, in BodyInertia localInertia, TypedIndex shapeIndex, float speculativeMargin, in BodyActivityDescription activity)
            : this(position, orientation, new BodyVelocity(), localInertia, new CollidableDescription { Shape = shapeIndex, SpeculativeMargin = speculativeMargin }, activity)
        {
        }
        /// <summary>
        /// Builds a new body description with discrete continuity.
        /// </summary>
        /// <param name="position">Position of the body.</param>
        /// <param name="localInertia">Local inertia of the body. If default initialized (all values set to zero), the body is considered kinematic.</param>
        /// <param name="shapeIndex">Index of the shape in the simulation shape set.</param>
        /// <param name="speculativeMargin">Distance beyond the surface of the body to allow speculative contacts to be generated.</param>
        /// <param name="activity">Activity description for the body.</param>
        public BodyDescription(in Vector3 position, in BodyInertia localInertia, TypedIndex shapeIndex, float speculativeMargin, in BodyActivityDescription activity)
           : this(position, Quaternion.Identity, new BodyVelocity(), localInertia, new CollidableDescription { Shape = shapeIndex, SpeculativeMargin = speculativeMargin }, activity)
        {
        }

        //Kinematic direct constructors.    
        /// <summary>
        /// Builds a new kinematic body description with discrete continuity.
        /// </summary>
        /// <param name="position">Position of the body.</param>
        /// <param name="orientation">Orientation of the body.</param>
        /// <param name="velocity">Velocity of the body.</param>
        /// <param name="shapeIndex">Index of the shape in the simulation shape set.</param>
        /// <param name="speculativeMargin">Distance beyond the surface of the body to allow speculative contacts to be generated.</param>
        /// <param name="activity">Activity description for the body.</param>
        public BodyDescription(in Vector3 position, in Quaternion orientation, in BodyVelocity velocity, TypedIndex shapeIndex, float speculativeMargin, in BodyActivityDescription activity)
            : this(position, orientation, velocity, new BodyInertia(), new CollidableDescription { Shape = shapeIndex, SpeculativeMargin = speculativeMargin }, activity)
        {
        }

        /// <summary>
        /// Builds a new kinematic body description with discrete continuity.
        /// </summary>
        /// <param name="position">Position of the body.</param>
        /// <param name="velocity">Velocity of the body.</param>
        /// <param name="shapeIndex">Index of the shape in the simulation shape set.</param>
        /// <param name="speculativeMargin">Distance beyond the surface of the body to allow speculative contacts to be generated.</param>
        /// <param name="activity">Activity description for the body.</param>
        public BodyDescription(in Vector3 position, in BodyVelocity velocity, TypedIndex shapeIndex, float speculativeMargin, in BodyActivityDescription activity)
            : this(position, Quaternion.Identity, velocity, new BodyInertia(), new CollidableDescription { Shape = shapeIndex, SpeculativeMargin = speculativeMargin }, activity)
        {
        }

        /// <summary>
        /// Builds a new kinematic body description with discrete continuity.
        /// </summary>
        /// <param name="position">Position of the body.</param>
        /// <param name="orientation">Orientation of the body.</param>
        /// <param name="shapeIndex">Index of the shape in the simulation shape set.</param>
        /// <param name="speculativeMargin">Distance beyond the surface of the body to allow speculative contacts to be generated.</param>
        /// <param name="activity">Activity description for the body.</param>
        public BodyDescription(in Vector3 position, in Quaternion orientation, TypedIndex shapeIndex, float speculativeMargin, in BodyActivityDescription activity)
            : this(position, orientation, new BodyVelocity(), new BodyInertia(), new CollidableDescription { Shape = shapeIndex, SpeculativeMargin = speculativeMargin }, activity)
        {
        }

        /// <summary>
        /// Builds a new kinematic body description with discrete continuity.
        /// </summary>
        /// <param name="position">Position of the body.</param>
        /// <param name="shapeIndex">Index of the shape in the simulation shape set.</param>
        /// <param name="speculativeMargin">Distance beyond the surface of the body to allow speculative contacts to be generated.</param>
        /// <param name="activity">Activity description for the body.</param>
        public BodyDescription(in Vector3 position, TypedIndex shapeIndex, float speculativeMargin, in BodyActivityDescription activity)
           : this(position, Quaternion.Identity, new BodyVelocity(), new BodyInertia(), new CollidableDescription { Shape = shapeIndex, SpeculativeMargin = speculativeMargin }, activity)
        {
        }

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
        /// <param name="activity">Default activity description for the given shape.</param>
        public static void GetDefaultActivity<TShape>(in TShape shape, out BodyActivityDescription activity) where TShape : struct, IConvexShape
        {
            activity.MinimumTimestepCountUnderThreshold = 32;
            shape.ComputeAngularExpansionData(out var maximumRadius, out _);
            activity.SleepThreshold = maximumRadius * maximumRadius * 0.01f;
        }

        /// <summary>
        /// Creates a body description. Adds the given convex shape to the shape set and uses it to initialize inertia.
        /// </summary>
        /// <typeparam name="TShape">Type of the shape to build a description for.</typeparam>
        /// <param name="position">Position of the body.</param>
        /// <param name="orientation">Orientation of the body.</param>
        /// <param name="velocity">Velocity of the body.</param>
        /// <param name="shape">Shape to build a description for.</param>
        /// <param name="shapes">Shape set of the simulation.</param>
        /// <param name="mass">Mass of the body. Use 0 if kinematic.</param>
        /// <param name="speculativeMargin">Distance beyond the surface of the body to allow speculative contacts to be generated.</param>
        /// <param name="continuity">Continuous collision detection settings for the body.</param>
        /// <param name="activity">Activity settings for the body.</param>
        /// <param name="description">Resulting description.</param>
        public static void Create<TShape>(in Vector3 position, in Quaternion orientation, in BodyVelocity velocity,
            in TShape shape, Shapes shapes, float mass, float speculativeMargin,
            in ContinuousDetectionSettings continuity, in BodyActivityDescription activity, out BodyDescription description) where TShape : struct, IConvexShape
        {
            var index = shapes.Add(shape);
            description.Collidable.Shape = index;
            description.Collidable.SpeculativeMargin = speculativeMargin;
            description.Collidable.Continuity = continuity;
            description.Pose.Position = position;
            description.Pose.Orientation = orientation;
            description.Velocity = velocity;
            description.Activity = activity;
            shape.ComputeInertia(mass, out description.LocalInertia);
        }

        /// <summary>
        /// Creates a body description. Adds the given convex shape to the shape set and uses it to initialize inertia, activity, and collidable descriptions. Uses discrete continuity.
        /// </summary>
        /// <typeparam name="TShape">Type of the shape to build a description for.</typeparam>
        /// <param name="position">Position of the body.</param>
        /// <param name="orientation">Orientation of the body.</param>
        /// <param name="velocity">Velocity of the body.</param>
        /// <param name="shape">Shape to build a description for.</param>
        /// <param name="shapes">Shape set of the simulation.</param>
        /// <param name="mass">Mass of the body. Use 0 if kinematic.</param>
        /// <param name="description">Resulting description.</param>
        public static void Create<TShape>(in Vector3 position, in Quaternion orientation, in BodyVelocity velocity,
            in TShape shape, Shapes shapes, float mass, out BodyDescription description) where TShape : struct, IConvexShape
        {
            GetDefaultActivity(shape, out var activity);
            Create(position, orientation, velocity, shape, shapes, mass, GetDefaultSpeculativeMargin(shape), new ContinuousDetectionSettings(), activity, out description);
        }
        /// <summary>
        /// Creates a body description. Adds the given convex shape to the shape set and uses it to initialize inertia, activity, and collidable descriptions. Uses discrete continuity.
        /// </summary>
        /// <typeparam name="TShape">Type of the shape to build a description for.</typeparam>
        /// <param name="position">Position of the body.</param>
        /// <param name="orientation">Orientation of the body.</param>
        /// <param name="shape">Shape to build a description for.</param>
        /// <param name="shapes">Shape set of the simulation.</param>
        /// <param name="mass">Mass of the body. Use 0 if kinematic.</param>
        /// <param name="description">Resulting description.</param>
        public static void Create<TShape>(in Vector3 position, in Quaternion orientation,
            in TShape shape, Shapes shapes, float mass, out BodyDescription description) where TShape : struct, IConvexShape
        {
            Create(position, orientation, new BodyVelocity(), shape, shapes, mass, out description);
        }
        /// <summary>
        /// Creates a body description. Adds the given convex shape to the shape set and uses it to initialize inertia, activity, and collidable descriptions. Uses discrete continuity.
        /// </summary>
        /// <typeparam name="TShape">Type of the shape to build a description for.</typeparam>
        /// <param name="position">Position of the body.</param>
        /// <param name="velocity">Velocity of the body.</param>
        /// <param name="shape">Shape to build a description for.</param>
        /// <param name="shapes">Shape set of the simulation.</param>
        /// <param name="mass">Mass of the body. Use 0 if kinematic.</param>
        /// <param name="description">Resulting description.</param>
        public static void Create<TShape>(in Vector3 position, in BodyVelocity velocity,
            in TShape shape, Shapes shapes, float mass, out BodyDescription description) where TShape : struct, IConvexShape
        {
            Create(position, Quaternion.Identity, velocity, shape, shapes, mass, out description);
        }
        /// <summary>
        /// Creates a body description. Adds the given convex shape to the shape set and uses it to initialize inertia, activity, and collidable descriptions. Uses discrete continuity.
        /// </summary>
        /// <typeparam name="TShape">Type of the shape to build a description for.</typeparam>
        /// <param name="position">Position of the body.</param>
        /// <param name="shape">Shape to build a description for.</param>
        /// <param name="shapes">Shape set of the simulation.</param>
        /// <param name="mass">Mass of the body. Use 0 if kinematic.</param>
        /// <param name="description">Resulting description.</param>
        public static void Create<TShape>(in Vector3 position,
           in TShape shape, Shapes shapes, float mass, out BodyDescription description) where TShape : struct, IConvexShape
        {
            Create(position, Quaternion.Identity, new BodyVelocity(), shape, shapes, mass, out description);
        }

        //Kinematic variants.
        /// <summary>
        /// Creates a kinematic body description. Adds the given convex shape to the shape set and uses it to initialize activity and collidable descriptions. Uses discrete continuity.
        /// </summary>
        /// <typeparam name="TShape">Type of the shape to build a description for.</typeparam>
        /// <param name="position">Position of the body.</param>
        /// <param name="orientation">Orientation of the body.</param>
        /// <param name="velocity">Velocity of the body.</param>
        /// <param name="shape">Shape to build a description for.</param>
        /// <param name="shapes">Shape set of the simulation.</param>
        /// <param name="description">Resulting description.</param>
        public static void Create<TShape>(in Vector3 position, in Quaternion orientation, in BodyVelocity velocity,
            in TShape shape, Shapes shapes, out BodyDescription description) where TShape : struct, IConvexShape
        {
            GetDefaultActivity(shape, out var activity);
            Create(position, orientation, velocity, shape, shapes, 0, GetDefaultSpeculativeMargin(shape), new ContinuousDetectionSettings(), activity, out description);
        }
        /// <summary>
        /// Creates a kinematic body description. Adds the given convex shape to the shape set and uses it to initialize activity and collidable descriptions. Uses discrete continuity.
        /// </summary>
        /// <typeparam name="TShape">Type of the shape to build a description for.</typeparam>
        /// <param name="position">Position of the body.</param>
        /// <param name="orientation">Orientation of the body.</param>
        /// <param name="shape">Shape to build a description for.</param>
        /// <param name="shapes">Shape set of the simulation.</param>
        /// <param name="description">Resulting description.</param>
        public static void Create<TShape>(in Vector3 position, in Quaternion orientation,
            in TShape shape, Shapes shapes, out BodyDescription description) where TShape : struct, IConvexShape
        {
            Create(position, orientation, new BodyVelocity(), shape, shapes, out description);
        }
        /// <summary>
        /// Creates a kinematic body description. Adds the given convex shape to the shape set and uses it to initialize activity and collidable descriptions. Uses discrete continuity.
        /// </summary>
        /// <typeparam name="TShape">Type of the shape to build a description for.</typeparam>
        /// <param name="position">Position of the body.</param>
        /// <param name="velocity">Velocity of the body.</param>
        /// <param name="shape">Shape to build a description for.</param>
        /// <param name="shapes">Shape set of the simulation.</param>
        /// <param name="description">Resulting description.</param>
        public static void Create<TShape>(in Vector3 position, in BodyVelocity velocity,
            in TShape shape, Shapes shapes, out BodyDescription description) where TShape : struct, IConvexShape
        {
            Create(position, Quaternion.Identity, velocity, shape, shapes, out description);
        }
        /// <summary>
        /// Creates a kinematic body description. Adds the given convex shape to the shape set and uses it to initialize activity and collidable descriptions. Uses discrete continuity.
        /// </summary>
        /// <typeparam name="TShape">Type of the shape to build a description for.</typeparam>
        /// <param name="position">Position of the body.</param>
        /// <param name="shape">Shape to build a description for.</param>
        /// <param name="shapes">Shape set of the simulation.</param>
        /// <param name="description">Resulting description.</param>
        public static void Create<TShape>(in Vector3 position,
           in TShape shape, Shapes shapes, out BodyDescription description) where TShape : struct, IConvexShape
        {
            Create(position, Quaternion.Identity, new BodyVelocity(), shape, shapes, out description);
        }


    }


}
