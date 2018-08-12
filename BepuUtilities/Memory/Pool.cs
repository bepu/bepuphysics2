using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;

namespace BepuUtilities.Memory
{
    /// <summary>
    /// Manages a cache of a type of resource.
    /// </summary>
    /// <typeparam name="T">Type of object to pool.</typeparam>
    public class Pool<T>
    {
        Stack<T> stack = new Stack<T>();

        /// <summary>
        /// Gets the number of existing elements in the pool. This number of elements can be requested without creating any new ones.
        /// </summary>
        /// <remarks> 
        /// Does not do any locking. Should not be used while elements may still be getting added to or removed from the pool.
        /// </remarks>
        public int PooledElementCount
        {
            get
            {
                return stack.Count;
            }
        }

        /// <summary>
        /// Gets or sets the function used to create new objects when the pool has no existing objects available.
        /// </summary>
        public Func<T> Creator
        {
            get; set;
        }

        /// <summary>
        /// Gets or sets the function used to initialize objects taken from the pool. Runs even if the object was just created by the Creator delegate.
        /// </summary>
        public Action<T> Initializer
        {
            get; set;
        }

        /// <summary>
        /// Gets or sets the action applied to an element when it is returned to the pool.
        /// </summary>
        public Action<T> Cleaner
        {
            get; set;
        }

        public Pool(Func<T> creator, Action<T> initializer = null, Action<T> cleaner = null)
        {
            Creator = creator ?? throw new ArgumentException("Creator must not be null.");
            Initializer = initializer;
            Cleaner = cleaner;
        }

#if DEBUG
        public HashSet<T> OutstandingElements = new HashSet<T>();
#endif

        /// <summary>
        /// Clears all elements from the pool.
        /// </summary>
        public void Clear()
        {
            stack.Clear();
        }

        [Conditional("DEBUG")]
        public void CheckIfOutstandingElementsExist()
        {
#if DEBUG
            Debug.Assert(OutstandingElements.Count == 0);
#endif
        }

        /// <summary>
        /// Takes an element from the pool. If the pool is empty, a new resource is created and returned.
        /// </summary>
        /// <returns>Element from the pool.</returns>
        public T Take()
        {
            T item;
            if (stack.Count > 0)
            {
                item = stack.Pop();
            }
            else
            {
                item = Creator();
            }
            Initializer?.Invoke(item);
#if DEBUG
            OutstandingElements.Add(item);
#endif
            return item;
        }


        /// <summary>
        /// Returns the specified item to the pool. If a cleaner delegate is set, the item is cleaned.
        /// </summary>
        /// <param name="item">Item to give back to the pool.</param>
        public void Return(T item)
        {
#if DEBUG
            if (!OutstandingElements.Remove(item))
                throw new InvalidOperationException("Cannot return an item that did not originate from this pool.");
#endif

            Cleaner?.Invoke(item);

            stack.Push(item);
        }
    }
}