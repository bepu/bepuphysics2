using System.Runtime.CompilerServices;

namespace BepuUtilities.Memory
{
    /// <summary>
    /// Naive implementation of a 'pool' that actually just creates fresh arrays whenever they are requested.
    /// Returning arrays to the pool does nothing. This is useful in some limited cases where you don't really care about reuse, like persistent structures where resizing should be rare.
    /// </summary>
    /// <remarks>Note that there is no unmanaged equivalent to this.
    /// Unmanaged spans absolutely have to be returned to avoid memory leaks since they either involve unmanaged allocations (e.g. Marshal.AllocateHGlobal) or
    /// are based on pinned managed arrays. An unmanaged span could be created that uses a reference type's finalizer to return the resource, but a big part
    /// of the point of unmanaged spans is referenceless storage. Given the common use cases, there's not much point in offering a third 'self-returning' type.</remarks>
    /// <typeparam name="T">Type of the arrays returned by the pool.</typeparam>
    public struct PassthroughArrayPool<T> : IMemoryPool<T, Array<T>>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Return(ref Array<T> span)
        {
            //Drop it on the floor and let the GC deal with it.
            span = new Array<T>();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Take(int count, out Array<T> span)
        {
            TakeForPower(SpanHelper.GetContainingPowerOf2(count), out span);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void TakeForPower(int power, out Array<T> span)
        {
            span = new Array<T>(new T[1 << power]);
        }
    }


}
