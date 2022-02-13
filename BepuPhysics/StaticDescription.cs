using BepuPhysics.Collidables;
using System.Numerics;

namespace BepuPhysics
{
    /// <summary>
    /// Describes the properties of a static object. When added to a simulation, static objects can collide but have no velocity and will not move in response to forces.
    /// </summary>
    public struct StaticDescription
    {
        /// <summary>
        /// Position and orientation of the static.
        /// </summary>
        public RigidPose Pose;
        /// <summary>
        /// Shape of the static.
        /// </summary>
        public TypedIndex Shape;
        /// <summary>
        /// Continuous collision detection settings for the static.
        /// </summary>
        public ContinuousDetection Continuity;

        /// <summary>
        /// Builds a new static description.
        /// </summary>
        /// <param name="pose">Pose of the static collidable.</param>
        /// <param name="shape">Shape of the static.</param>
        /// <param name="continuity">Continuous collision detection settings for the static.</param>
        public StaticDescription(RigidPose pose, TypedIndex shape, ContinuousDetection continuity)
        {
            Pose = pose;
            Shape = shape;
            Continuity = continuity;
        }

        /// <summary>
        /// Builds a new static description with <see cref="ContinuousDetectionMode.Discrete"/> continuity.
        /// </summary>
        /// <param name="pose">Pose of the static collidable.</param>
        /// <param name="shape">Shape of the static.</param>
        public StaticDescription(RigidPose pose, TypedIndex shape)
        {
            Pose = pose;
            Shape = shape;
            Continuity = ContinuousDetection.Discrete;
        }

        /// <summary>
        /// Builds a new static description.
        /// </summary>
        /// <param name="position">Position of the static.</param>
        /// <param name="orientation">Orientation of the static.</param>
        /// <param name="shape">Shape of the static.</param>
        /// <param name="continuity">Continuous collision detection settings for the static.</param>
        public StaticDescription(Vector3 position, Quaternion orientation, TypedIndex shape, ContinuousDetection continuity)
        {
            Pose.Position = position;
            Pose.Orientation = orientation;
            Shape = shape;
            Continuity = continuity;
        }

        /// <summary>
        /// Builds a new static description with <see cref="ContinuousDetectionMode.Discrete"/> continuity.
        /// </summary>
        /// <param name="position">Position of the static.</param>
        /// <param name="orientation">Orientation of the static.</param>
        /// <param name="shape">Shape of the static.</param>
        public StaticDescription(Vector3 position, Quaternion orientation, TypedIndex shape)
        {
            Pose.Position = position;
            Pose.Orientation = orientation;
            Shape = shape;
            Continuity = ContinuousDetection.Discrete;
        }
    }

}
