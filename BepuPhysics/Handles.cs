using System;
using System.Collections.Generic;
using System.Text;

namespace BepuPhysics
{
    /// <summary>
    /// Unique identifier of a body belonging to a simulation's Bodies collection.
    /// </summary>
    public struct BodyHandle
    {
        /// <summary>
        /// Index in the handle-to-memory mapping table used to look up the current memory location of the body. 
        /// This value will not (and must not) change during the body's lifespan, but the memory that the table points to could change.
        /// </summary>
        public readonly int Value;

        public BodyHandle(int index)
        {
            Value = index;
        }
    }

    /// <summary>
    /// Unique identifier of a constraint belonging to a simulation's Solver.
    /// </summary>
    public struct ConstraintHandle
    {
        /// <summary>
        /// Index in the handle-to-memory mapping table used to look up the current memory location of the constraint. 
        /// This value will not (and must not) change during the constraint's lifespan, but the memory that the table points to could change.
        /// </summary>
        public readonly int Value;

        public ConstraintHandle(int index)
        {
            Value = index;
        }
    }
}
