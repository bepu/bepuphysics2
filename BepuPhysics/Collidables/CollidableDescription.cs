namespace BepuPhysics.Collidables
{
    /// <summary>
    /// Describes a collidable and how it should handle continuous collision detection.
    /// </summary>
    public struct CollidableDescription
    {
        /// <summary>
        /// Shape of the collidable.
        /// </summary>
        public TypedIndex Shape;
        /// <summary>
        /// Continuous collision detection settings used by the collidable.
        /// </summary>
        public ContinuousDetection Continuity;

        /// <summary>
        /// Constructs a new collidable description.
        /// </summary>
        /// <param name="shape">Shape used by the collidable.</param>
        /// <param name="continuity">Continuous collision detection settings for the collidable.</param>
        public CollidableDescription(TypedIndex shape, in ContinuousDetection continuity)
        {
            Shape = shape;
            Continuity = continuity;
        }

        /// <summary>
        /// Constructs a new collidable description with default discrete continuity.
        /// </summary>
        /// <param name="shape">Shape used by the collidable.</param>
        public CollidableDescription(TypedIndex shape) : this(shape, ContinuousDetection.Passive)
        {
        }
    }
}
