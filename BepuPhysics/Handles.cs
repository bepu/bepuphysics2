using System;
using System.Collections.Generic;
using System.Text;

namespace BepuPhysics
{
    //Wrapping the handle integer in a typed container makes it somewhat less likely that people will confuse a handle for an in-memory index, or confuse static handles/body handles/constraint handles for each other.
    //It's worth noting that these types really don't do anything at all- they're strictly there so avoid some common pitfalls.
    //So, if a user has an advanced use case where treating these as integers is helpful, they can do so without any problem.

    /// <summary>
    /// Unique identifier of a body belonging to a simulation's Bodies collection.
    /// </summary>
    public struct BodyHandle
    {
        /// <summary>
        /// Index in the handle-to-memory mapping table used to look up the current memory location of the body. 
        /// This value will not (and must not) change during the body's lifespan, but the memory that the table points to could change.
        /// </summary>
        public int Value;

        public BodyHandle(int value)
        {
            Value = value;
        }

        public override string ToString()
        {
            return Value.ToString();
        }
    }

    /// <summary>
    /// Unique identifier of a static belonging to a simulation's Statics collection.
    /// </summary>
    public struct StaticHandle
    {
        /// <summary>
        /// Index in the handle-to-memory mapping table used to look up the current memory location of the body. 
        /// This value will not (and must not) change during the body's lifespan, but the memory that the table points to could change.
        /// </summary>
        public int Value;

        public StaticHandle(int value)
        {
            Value = value;
        }

        public override string ToString()
        {
            return Value.ToString();
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
        public int Value;

        public ConstraintHandle(int index)
        {
            Value = index;
        }

        public override string ToString()
        {
            return Value.ToString();
        }
    }
}
