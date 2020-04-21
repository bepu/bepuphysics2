using BepuUtilities.Collections;
using System;
using System.Runtime.CompilerServices;

namespace BepuPhysics
{
    //Wrapping the handle integer in a typed container makes it somewhat less likely that people will confuse a handle for an in-memory index, or confuse static handles/body handles/constraint handles for each other.
    //It's worth noting that these types really don't do anything at all- they're strictly there so avoid some common pitfalls.
    //So, if a user has an advanced use case where treating these as integers is helpful, they can do so without any problem.

    /// <summary>
    /// Unique identifier of a body belonging to a simulation's Bodies collection.
    /// </summary>
    public struct BodyHandle : IEquatable<BodyHandle>, IEqualityComparerRef<BodyHandle>
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(BodyHandle other)
        {
            return Value == other.Value;
        }
        public override bool Equals(object obj)
        {
            return Equals((BodyHandle)obj);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(BodyHandle a, BodyHandle b)
        {
            return a.Value == b.Value;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(BodyHandle a, BodyHandle b)
        {
            return a.Value != b.Value;
        }
        public override int GetHashCode()
        {
            return Value.GetHashCode();
        }

        public override string ToString()
        {
            return Value.ToString();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Hash(ref BodyHandle item)
        {
            return item.Value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(ref BodyHandle a, ref BodyHandle b)
        {
            return a.Value == b.Value;
        }
    }

    /// <summary>
    /// Unique identifier of a static belonging to a simulation's Statics collection.
    /// </summary>
    public struct StaticHandle : IEquatable<StaticHandle>, IEqualityComparerRef<StaticHandle>
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(StaticHandle other)
        {
            return Value == other.Value;
        }
        public override bool Equals(object obj)
        {
            return Equals((StaticHandle)obj);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(StaticHandle a, StaticHandle b)
        {
            return a.Value == b.Value;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(StaticHandle a, StaticHandle b)
        {
            return a.Value != b.Value;
        }
        public override int GetHashCode()
        {
            return Value.GetHashCode();
        }

        public override string ToString()
        {
            return Value.ToString();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Hash(ref StaticHandle item)
        {
            return item.Value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(ref StaticHandle a, ref StaticHandle b)
        {
            return a.Value == b.Value;
        }
    }

    /// <summary>
    /// Unique identifier of a constraint belonging to a simulation's Solver.
    /// </summary>
    public struct ConstraintHandle : IEquatable<ConstraintHandle>, IEqualityComparerRef<ConstraintHandle>
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


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(ConstraintHandle other)
        {
            return Value == other.Value;
        }
        public override bool Equals(object obj)
        {
            return Equals((ConstraintHandle)obj);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(ConstraintHandle a, ConstraintHandle b)
        {
            return a.Value == b.Value;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(ConstraintHandle a, ConstraintHandle b)
        {
            return a.Value != b.Value;
        }
        public override int GetHashCode()
        {
            return Value.GetHashCode();
        }

        public override string ToString()
        {
            return Value.ToString();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Hash(ref ConstraintHandle item)
        {
            return item.Value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(ref ConstraintHandle a, ref ConstraintHandle b)
        {
            return a.Value == b.Value;
        }
    }
}
