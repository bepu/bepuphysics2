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
        /// Collidable properties of the static.
        /// </summary>
        public CollidableDescription Collidable;

        /// <summary>
        /// Builds a new static description.
        /// </summary>
        /// <param name="position">Position of the static.</param>
        /// <param name="orientation">Orientation of the static.</param>
        /// <param name="collidable">Collidable description for the static.</param>
        public StaticDescription(in Vector3 position, in Quaternion orientation, in CollidableDescription collidable)
        {
            Pose.Position = position;
            Pose.Orientation = orientation;
            Collidable = collidable;
        }

        /// <summary>
        /// Builds a new static description.
        /// </summary>
        /// <param name="position">Position of the static.</param>
        /// <param name="collidable">Collidable description for the static.</param>
        public StaticDescription(in Vector3 position, in CollidableDescription collidable) : this(position, Quaternion.Identity, collidable)
        {
        }

        /// <summary>
        /// Builds a new static description with discrete continuity.
        /// </summary>
        /// <param name="position">Position of the static.</param>
        /// <param name="orientation">Orientation of the static.</param>
        /// <param name="shapeIndex">Index of the static's shape in the simulation shapes set.</param>
        public StaticDescription(in Vector3 position, in Quaternion orientation, TypedIndex shapeIndex)
        {
            Pose.Position = position;
            Pose.Orientation = orientation;
            Collidable.Continuity = ContinuousDetection.Passive;
            Collidable.Shape = shapeIndex;
        }

        /// <summary>
        /// Builds a new static description with discrete continuity.
        /// </summary>
        /// <param name="position">Position of the static.</param>
        /// <param name="shapeIndex">Index of the static's shape in the simulation shapes set.</param>
        public StaticDescription(in Vector3 position, TypedIndex shapeIndex) : this(position, Quaternion.Identity, shapeIndex)
        {
        }
    }

}
