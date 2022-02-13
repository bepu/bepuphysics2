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
        /// Lower bound on the value of the speculative margin used by the collidable.
        /// </summary>
        /// <remarks>0 tends to be a good default value. Higher values can be chosen if velocity magnitude is a poor proxy for speculative margins, but these cases are rare.
        /// In those cases, try to use the smallest value that still satisfies requirements to avoid creating unnecessary contact constraints.</remarks>
        public float MinimumSpeculativeMargin;
        /// <summary>
        /// Upper bound on the value of the speculative margin used by the collidable.
        /// </summary>
        /// <remarks><see cref="float.MaxValue"/> tends to be a good default value for discrete or passive mode collidables. 
        /// The speculative margin will increase in size proportional to velocity magnitude, so having an unlimited maximum won't cost extra if the body isn't moving fast.
        /// <para>Smaller values can be useful for improving performance in chaotic situations where missing a collision is acceptable. When using <see cref="ContinuousDetectionMode.Continuous"/>, a speculative margin larger than the velocity magnitude will result in the sweep test being skipped, so lowering the maximum margin can help avoid ghost collisions.</para>
        /// </remarks>
        public float MaximumSpeculativeMargin;

        /// <summary>
        /// Constructs a new collidable description.
        /// </summary>
        /// <param name="shape">Shape used by the collidable.</param>
        /// <param name="minimumSpeculativeMargin">Lower bound on the value of the speculative margin used by the collidable.</param>
        /// <param name="maximumSpeculativeMargin">Upper bound on the value of the speculative margin used by the collidable.</param>
        /// <param name="continuity">Continuous collision detection settings for the collidable.</param>
        public CollidableDescription(TypedIndex shape, float minimumSpeculativeMargin, float maximumSpeculativeMargin, ContinuousDetection continuity)
        {
            Shape = shape;
            MinimumSpeculativeMargin = minimumSpeculativeMargin;
            MaximumSpeculativeMargin = maximumSpeculativeMargin;
            Continuity = continuity;
        }

        /// <summary>
        /// Constructs a new collidable description with <see cref="ContinuousDetectionMode.Discrete"/>.
        /// </summary>
        /// <param name="shape">Shape used by the collidable.</param>
        /// <param name="minimumSpeculativeMargin">Lower bound on the value of the speculative margin used by the collidable.</param>
        /// <param name="maximumSpeculativeMargin">Upper bound on the value of the speculative margin used by the collidable.</param>
        public CollidableDescription(TypedIndex shape, float minimumSpeculativeMargin, float maximumSpeculativeMargin)
        {
            Shape = shape;
            MinimumSpeculativeMargin = minimumSpeculativeMargin;
            MaximumSpeculativeMargin = maximumSpeculativeMargin;
            Continuity = ContinuousDetection.Discrete;
        }

        /// <summary>
        /// Constructs a new collidable description. Uses 0 for the <see cref="MinimumSpeculativeMargin"/> .
        /// </summary>
        /// <param name="shape">Shape used by the collidable.</param>
        /// <param name="maximumSpeculativeMargin">Upper bound on the value of the speculative margin used by the collidable.</param>
        /// <param name="continuity">Continuous collision detection settings for the collidable.</param>
        public CollidableDescription(TypedIndex shape, float maximumSpeculativeMargin, ContinuousDetection continuity)
        {
            Shape = shape;
            MinimumSpeculativeMargin = 0;
            MaximumSpeculativeMargin = maximumSpeculativeMargin;
            Continuity = continuity;
        }

        /// <summary>
        /// Constructs a new collidable description. Uses 0 for the <see cref="MinimumSpeculativeMargin"/> and <see cref="float.MaxValue"/> for the <see cref="MaximumSpeculativeMargin"/> .
        /// </summary>
        /// <param name="shape">Shape used by the collidable.</param>
        /// <param name="continuity">Continuous collision detection settings for the collidable.</param>
        public CollidableDescription(TypedIndex shape, ContinuousDetection continuity)
        {
            Shape = shape;
            MinimumSpeculativeMargin = 0;
            MaximumSpeculativeMargin = float.MaxValue;
            Continuity = continuity;
        }

        /// <summary>
        /// Constructs a new collidable description with <see cref="ContinuousDetectionMode.Passive"/>. Will use a <see cref="MinimumSpeculativeMargin"/> of 0 and a <see cref="MaximumSpeculativeMargin"/> of <see cref="float.MaxValue"/>.
        /// </summary>
        /// <param name="shape">Shape used by the collidable.</param>
        /// <remarks><see cref="ContinuousDetectionMode.Passive"/> and <see cref="ContinuousDetectionMode.Discrete"/> are equivalent in behavior when the <see cref="MaximumSpeculativeMargin"/>  is <see cref="float.MaxValue"/> since they both result in the same (unbounded) expansion of body bounding boxes in response to velocity.</remarks>
        public CollidableDescription(TypedIndex shape) : this(shape, 0, float.MaxValue, ContinuousDetection.Passive)
        {
        }

        /// <summary>
        /// Constructs a new collidable description with <see cref="ContinuousDetectionMode.Discrete"/>. Will use a minimum speculative margin of 0 and the given maximumSpeculativeMargin.
        /// </summary>
        /// <param name="shape">Shape used by the collidable.</param>
        /// <param name="maximumSpeculativeMargin">Maximum speculative margin to be used with the discrete continuity configuration.</param>
        public CollidableDescription(TypedIndex shape, float maximumSpeculativeMargin) : this(shape, 0, maximumSpeculativeMargin, ContinuousDetection.Discrete)
        {
        }

        /// <summary>
        /// Constructs a new collidable description with <see cref="ContinuousDetectionMode.Passive"/>. Will use a minimum speculative margin of 0 and a maximum of <see cref="float.MaxValue"/>.
        /// </summary>
        /// <param name="shapeIndex">Shape index to use for the collidable.</param>
        public static implicit operator CollidableDescription(TypedIndex shapeIndex)
        {
            return new CollidableDescription(shapeIndex);
        }
    }
}
