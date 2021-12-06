using BepuPhysics.Constraints;
using System.Runtime.CompilerServices;

namespace BepuPhysics
{
    /// <summary>
    /// Reference to a constraint's memory location in the solver.
    /// </summary>
    public unsafe struct ConstraintReference
    {
        internal TypeBatch* typeBatchPointer;
        /// <summary>
        /// Gets a reference to the type batch holding the constraint.
        /// </summary>
        public ref TypeBatch TypeBatch
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return ref *typeBatchPointer;
            }
        }
        /// <summary>
        /// Index in the type batch where the constraint is allocated.
        /// </summary>
        public readonly int IndexInTypeBatch;

        /// <summary>
        /// Creates a new constraint reference from a constraint memory location.
        /// </summary>
        /// <param name="typeBatchPointer">Pointer to the type batch where the constraint lives.</param>
        /// <param name="indexInTypeBatch">Index in the type batch where the constraint is allocated.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ConstraintReference(TypeBatch* typeBatchPointer, int indexInTypeBatch)
        {
            this.typeBatchPointer = typeBatchPointer;
            IndexInTypeBatch = indexInTypeBatch;
        }
    }
}
