using BepuPhysics;
using BepuPhysics.Collidables;

namespace Demos.Demos.Tanks
{
    /// <summary>
    /// Describes properties of a piece of a tank.
    /// </summary>
    public struct TankPartDescription
    {
        /// <summary>
        /// Shape index used by this part's collidable.
        /// </summary>
        public TypedIndex Shape;
        /// <summary>
        /// Inertia of this part's body.
        /// </summary>
        public BodyInertia Inertia;
        /// <summary>
        /// Pose of the part in the tank's local space.
        /// </summary>
        public RigidPose Pose;
        /// <summary>
        /// Friction of the body to be used in pair material calculations.
        /// </summary>
        public float Friction;

        public static TankPartDescription Create<TShape>(float mass, in TShape shape, in RigidPose pose, float friction, Shapes shapes) where TShape : unmanaged, IConvexShape
        {
            TankPartDescription description;
            description.Shape = shapes.Add(shape);
            shape.ComputeInertia(mass, out description.Inertia);
            description.Pose = pose;
            description.Friction = friction;
            return description;
        }
    }
}