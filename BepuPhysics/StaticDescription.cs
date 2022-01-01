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
        /// <param name="pose">Pose of the static collidable.</param>
        /// <param name="collidable">Collidable description for the static.</param>
        public StaticDescription(RigidPose pose, CollidableDescription collidable)
        {
            Pose = pose;
            Collidable = collidable;
        }

        /// <summary>
        /// Builds a new static description.
        /// </summary>
        /// <param name="position">Position of the static.</param>
        /// <param name="orientation">Orientation of the static.</param>
        /// <param name="collidable">Collidable description for the static.</param>
        public StaticDescription(Vector3 position, Quaternion orientation, CollidableDescription collidable)
        {
            Pose.Position = position;
            Pose.Orientation = orientation;
            Collidable = collidable;
        }
    }

}
