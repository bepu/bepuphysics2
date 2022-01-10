namespace BepuPhysics
{
    /// <summary>
    /// The common set of allocation sizes for a simulation.
    /// </summary>
    public struct SimulationAllocationSizes
    {
        /// <summary>
        /// The number of bodies to allocate space for.
        /// </summary>
        public int Bodies;
        /// <summary>
        /// The number of statics to allocate space for.
        /// </summary>
        public int Statics;
        /// <summary>
        /// The number of inactive islands to allocate space for.
        /// </summary>
        public int Islands;
        /// <summary>
        /// Minimum number of shapes to allocate space for in each shape type batch.
        /// </summary>
        public int ShapesPerType;
        /// <summary>
        /// The number of constraints to allocate bookkeeping space for. This does not affect actual type batch allocation sizes, only the solver-level constraint handle storage.
        /// </summary>
        public int Constraints;
        /// <summary>
        /// The minimum number of constraints to allocate space for in each individual type batch.
        /// New type batches will be given enough memory for this number of constraints, and any compaction will not reduce the allocations below it.
        /// The number of constraints can vary greatly across types- there are usually far more contacts than ragdoll constraints.
        /// Per type estimates can be assigned within the Solver.TypeBatchAllocation if necessary. This value acts as a lower bound for all types.
        /// </summary>
        public int ConstraintsPerTypeBatch;
        /// <summary>
        /// The minimum number of constraints to allocate space for in each body's constraint list.
        /// New bodies will be given enough memory for this number of constraints, and any compaction will not reduce the allocations below it.
        /// </summary>
        public int ConstraintCountPerBodyEstimate;

        /// <summary>
        /// Constructs a description of simulation allocations.
        /// </summary>
        /// <param name="bodies">The number of bodies to allocate space for.</param>
        /// <param name="statics">The number of inactive islands to allocate space for.</param>
        /// <param name="islands">The number of inactive islands to allocate space for.</param>
        /// <param name="shapesPerType">Minimum number of shapes to allocate space for in each shape type batch.</param>
        /// <param name="constraints">The number of constraints to allocate bookkeeping space for. This does not affect actual type batch allocation sizes, only the solver-level constraint handle storage.</param>
        /// <param name="constraintsPerTypeBatch">The minimum number of constraints to allocate space for in each individual type batch.
        /// New type batches will be given enough memory for this number of constraints, and any compaction will not reduce the allocations below it.
        /// The number of constraints can vary greatly across types- there are usually far more contacts than ragdoll constraints.
        /// Per type estimates can be assigned within the Solver.TypeBatchAllocation if necessary. This value acts as a lower bound for all types.</param>
        /// <param name="constraintCountPerBodyEstimate">The minimum number of constraints to allocate space for in each body's constraint list.
        /// New bodies will be given enough memory for this number of constraints, and any compaction will not reduce the allocations below it.</param>
        public SimulationAllocationSizes(int bodies, int statics, int islands, int shapesPerType, int constraints, int constraintsPerTypeBatch, int constraintCountPerBodyEstimate)
        {
            Bodies = bodies;
            Statics = statics;
            Islands = islands;
            ShapesPerType = shapesPerType;
            Constraints = constraints;
            ConstraintsPerTypeBatch = constraintsPerTypeBatch;
            ConstraintCountPerBodyEstimate = constraintCountPerBodyEstimate;
        }
    }
}
