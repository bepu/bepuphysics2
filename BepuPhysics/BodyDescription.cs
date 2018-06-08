using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;
using System.Numerics;
using System.Runtime.CompilerServices;
using Quaternion = BepuUtilities.Quaternion;

namespace BepuPhysics
{
    public struct RigidPose
    {
        public Vector3 Position;
        //Note that we store a quaternion rather than a matrix3x3. While this often requires some overhead when performing vector transforms or extracting basis vectors, 
        //systems needing to interact directly with this representation are often terrifically memory bound. Spending the extra ALU time to convert to a basis can actually be faster
        //than loading the extra 5 elements needed to express the full 3x3 rotation matrix. Also, it's marginally easier to keep the rotation normalized over time.
        //There may be an argument for the matrix variant to ALSO be stored for some bandwidth-unconstrained stages, but don't worry about that until there's a reason to worry about it.
        public BepuUtilities.Quaternion Orientation;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Transform(in Vector3 v, in RigidPose pose, out Vector3 result)
        {
            BepuUtilities.Quaternion.TransformWithoutOverlap(v, pose.Orientation, out var rotated);
            result = rotated + pose.Position;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformByInverse(in Vector3 v, in RigidPose pose, out Vector3 result)
        {
            var translated = v - pose.Position;
            BepuUtilities.Quaternion.Conjugate(pose.Orientation, out var conjugate);
            BepuUtilities.Quaternion.TransformWithoutOverlap(translated, conjugate, out result);
        }
    }

    public struct BodyVelocity
    {
        public Vector3 Linear;
        public Vector3 Angular;
    }

    public struct BodyInertia
    {
        public Triangular3x3 InverseInertiaTensor;
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
    }

    public struct BodyVelocities
    {
        public Vector3Wide Linear;
        public Vector3Wide Angular;
    }

    public struct BodyInertias
    {
        public Triangular3x3Wide InverseInertiaTensor;
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

        //TODO: We may later decide to encode kinematic-ness in the indices held by constraints. That would be somewhat complicated, but if we end up using such an encoding
        //to avoid velocity reads from/writes to kinematics in the solver, we might as well use it for the traversal too.
        /// <summary>
        /// True if this body has effectively infinite mass and inertia, false otherwise. Kinematic bodies may block constraint graph traversals since they cannot propagate impulses.
        /// This value should remain in sync with other systems that make a distinction between kinematic and dynamic bodies. Under normal circumstances, it should never be externally set.
        /// To change an object's kinematic/dynamic state, use Bodies.ChangeLocalInertia or Bodies.ApplyDescription.
        /// </summary>
        public bool Kinematic;
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
        /// Builds a new body description.
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
        /// Builds a new body description.
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
        /// Builds a new body description.
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
        /// Builds a new body description.
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
        /// Builds a new kinematic body description.
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
        /// Builds a new kinematic body description.
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
        /// Builds a new kinematic body description.
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
        /// Builds a new kinematic body description.
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
        /// Creates a body description. Adds the given convex shape to the shape set and uses it to initialize inertia, activity, and collidable descriptions.
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
        /// Creates a body description. Adds the given convex shape to the shape set and uses it to initialize inertia, activity, and collidable descriptions.
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
        /// Creates a body description. Adds the given convex shape to the shape set and uses it to initialize inertia, activity, and collidable descriptions.
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
        /// Creates a body description. Adds the given convex shape to the shape set and uses it to initialize inertia, activity, and collidable descriptions.
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
        /// Creates a kinematic body description. Adds the given convex shape to the shape set and uses it to initialize activity and collidable descriptions.
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
        /// Creates a kinematic body description. Adds the given convex shape to the shape set and uses it to initialize activity and collidable descriptions.
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
        /// Creates a kinematic body description. Adds the given convex shape to the shape set and uses it to initialize activity and collidable descriptions.
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
        /// Creates a kinematic body description. Adds the given convex shape to the shape set and uses it to initialize activity and collidable descriptions.
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

    public struct StaticDescription
    {
        public RigidPose Pose;
        public CollidableDescription Collidable;
    }


}
