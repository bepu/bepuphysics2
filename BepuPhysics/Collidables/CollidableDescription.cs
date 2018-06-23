namespace BepuPhysics.Collidables
{
    public struct CollidableDescription
    {
        public TypedIndex Shape;
        public float SpeculativeMargin;
        public ContinuousDetectionSettings Continuity;

        /// <summary>
        /// Constructs a new collidable description.
        /// </summary>
        /// <param name="shape">Shape used by the collidable.</param>
        /// <param name="speculativeMargin">Radius of the margin in which to allow speculative contact generation.</param>
        /// <param name="continuity">Continuous collision detection settings for the collidable.</param>
        public CollidableDescription(TypedIndex shape, float speculativeMargin, in ContinuousDetectionSettings continuity)
        {
            Shape = shape;
            SpeculativeMargin = speculativeMargin;
            Continuity = continuity;
        }

        /// <summary>
        /// Constructs a new collidable description with default discrete continuity.
        /// </summary>
        /// <param name="shape">Shape used by the collidable.</param>
        /// <param name="speculativeMargin">Radius of the margin in which to allow speculative contact generation.</param>
        public CollidableDescription(TypedIndex shape, float speculativeMargin) : this(shape, speculativeMargin, default)
        {
        }
    }
}
