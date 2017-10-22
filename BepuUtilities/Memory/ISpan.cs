using BepuUtilities.Collections;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace BepuUtilities.Memory
{
    /// <summary>
    /// Defines a type that can act as an indexable typed wrapper for a block of memory.
    /// </summary>
    /// <remarks>
    /// The motivation here is allow tons of pool-based collections in an array without requiring GC inspection for underlying pool references.
    /// The primary use case for the ISpan{T} is heap-stored pinned pointers, but this layer of abstraction is needed for Quick* collections to also work for managed types.
    /// This is distinct from the .NET Span{T} type, which is a stack-only construct that has to be 'rehydrated' from another type.
    /// Ideally, you could use a true Span{T} wrapped in a supporting ISpan{T} struct with all of the quick collections too,
    /// but the stack-only property (and lack of a generalized language-level stack-only concept) currently disallows this. Maybe later!
    /// </remarks>
    /// <typeparam name="T">Type of the elements in the memory block.</typeparam>
    public interface ISpan<T>
    {
        //Note that this type does not require slicing support. While it could be supported by the implementations of the interface, we don't require it.

        /// <summary>
        /// Gets a reference to the element at the given index.
        /// </summary>
        /// <param name="index">Index of the element to get the reference of.</param>
        /// <returns>Reference of the element at the given index.</returns>
        ref T this[int index] { get; }

        /// <summary>
        /// Gets the number of elements in the span.
        /// </summary>
        int Length { get; }

        /// <summary>
        /// Gets whether the span has any backing memory.
        /// </summary>
        bool Allocated { get;  }
        
        /// <summary>
        /// Copies elements from one span region to another.
        /// </summary>
        /// <remarks>
        /// If the span regions overlap, the result is the same as if the original values of the source were preserved in a temporary location before writing to the target.
        /// </remarks>
        /// <typeparam name="TTargetSpan">The type of the span to copy into.</typeparam>
        /// <param name="sourceStart">Start index of the copy source region.</param>
        /// <param name="targetSpan">Span to copy into.</param>
        /// <param name="targetStart">Start index of the copy target region.</param>
        /// <param name="count">Number of elements to copy.</param>
        void CopyTo<TTargetSpan>(int sourceStart, ref TTargetSpan targetSpan, int targetStart, int count) where TTargetSpan : ISpan<T>;

        /// <summary>
        /// Clears the memory in a subset of the span.
        /// </summary>
        /// <param name="start">Inclusive starting index of the clear operation.</param>
        /// <param name="count">Number of indices to clear.</param>
        void Clear(int start, int count);

        /// <summary>
        /// Clears any references included in the span. Memory that isn't a managed reference may or may not be left untouched.
        /// </summary>
        /// <remarks>
        /// In many cases, we can use a pointer-backed span or an otherwise pure valuetype span that cannot include managed references. In those cases,
        /// this does absolutely nothing- which is good!
        /// </remarks>
        /// <param name="start">Inclusive starting index of the clear operation.</param>
        /// <param name="count">Number of indices to clear.</param>
        void ClearManagedReferences(int start, int count);

        /// <summary>
        /// Gets the index of the first occurrence of an element in the span as determined by the default comparison. Search proceeds from low to high indices.
        /// </summary>
        /// <param name="element">Element to search for.</param>
        /// <param name="start">Inclusive start index of the search.</param>
        /// <param name="count">Number of elements to search.</param>
        /// <returns>Index of the element in the span if it is present; -1 otherwise.</returns>
        int IndexOf(T element, int start, int count);

        /// <summary>
        /// Gets the index of the first occurrence of an element in the span as determined by the default comparison. Search proceeds from low to high indices.
        /// </summary>
        /// <param name="element">Element to search for.</param>
        /// <param name="start">Inclusive start index of the search.</param>
        /// <param name="count">Number of elements to search.</param>
        /// <returns>Index of the element in the span if it is present; -1 otherwise.</returns>
        int IndexOf(ref T element, int start, int count);

        /// <summary>
        /// Gets the index of the first match of a predicate. Search proceeds from low to high indices.
        /// </summary>
        /// <param name="predicate">Predicate to test against the elements in the span region.</param>
        /// <param name="start">Inclusive start index of the search.</param>
        /// <param name="count">Number of elements to search.</param>
        /// <returns>Index of the element in the span if it is present; -1 otherwise.</returns>
        int IndexOf<TPredicate>(ref TPredicate predicate, int start, int count) where TPredicate : IPredicate<T>;

        /// <summary>
        /// Pins the span's backing memory if it is managed.
        /// </summary>
        /// <param name="handle">Handle to the pinned memory.</param>
        /// <returns>True if the span is backed by managed memory that is now pinned, false otherwise.</returns>
        bool TryPin(out GCHandle handle);
    }


}
