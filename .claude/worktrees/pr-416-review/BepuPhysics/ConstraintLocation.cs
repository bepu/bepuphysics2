namespace BepuPhysics
{
    /// <summary>
    /// Location in memory where a constraint is stored.
    /// </summary>
    public struct ConstraintLocation
    {
        //Note that the type id is included, even though we can extract it from a type parameter.
        //This is required for body memory swap induced reference changes- it is not efficient to include type metadata in the per-body connections,
        //so instead we keep a type id cached.
        //(You could pack these a bit- it's pretty reasonable to say you can't have more than 2^24 constraints of a given type and 2^8 constraint types...
        //It's just not that valuable, unless proven otherwise.)
        /// <summary>
        /// Index of the constraint set that owns the constraint. If zero, the constraint is attached to bodies that are awake.
        /// </summary>
        public int SetIndex;
        /// <summary>
        /// Index of the constraint batch the constraint belongs to.
        /// </summary>
        public int BatchIndex;
        /// <summary>
        /// Type id of the constraint. Used to look up the type batch index in a constraint batch's type id to type batch index table.
        /// </summary>
        public int TypeId;
        /// <summary>
        /// Index of the constraint in a type batch.
        /// </summary>
        public int IndexInTypeBatch;
    }
}
