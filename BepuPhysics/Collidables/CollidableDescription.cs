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
        /// Constructs a new collidable description with <see cref="ContinuousDetectionMode.Passive"/>. Will use a minimum speculative margin of 0 and a maximum of <see cref="float.MaxValue"/>.
        /// </summary>
        /// <param name="shape">Shape used by the collidable.</param>
        /// <remarks><see cref="ContinuousDetectionMode.Passive"/> and <see cref="ContinuousDetectionMode.Discrete"/> are equivalent in behavior when the maximum speculative margin is <see cref="float.MaxValue"/> since they both result in the same (unbounded) expansion of body bounding boxes in response to velocity.</remarks>
        public CollidableDescription(TypedIndex shape) : this(shape, ContinuousDetection.Passive)
        {
        }

        /// <summary>
        /// Constructs a new collidable description with <see cref="ContinuousDetectionMode.Discrete"/>. Will use a minimum speculative margin of 0 and the given maximumSpeculativeMargin.
        /// </summary>
        /// <param name="shape">Shape used by the collidable.</param>
        /// <param name="maximumSpeculativeMargin">Maximum speculative margin to be used with the discrete continuity configuration.</param>
        public CollidableDescription(TypedIndex shape, float maximumSpeculativeMargin) : this(shape, ContinuousDetection.Discrete(0, maximumSpeculativeMargin))
        {
        }
    }
}
