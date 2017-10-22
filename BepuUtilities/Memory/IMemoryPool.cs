namespace BepuUtilities.Memory
{
    /// <summary>
    /// Defines a type that is capable of pooling blocks of memory in the form of ISpan{T}.
    /// The backing memory may be untyped.
    /// </summary>
    public interface IMemoryPool<T, TSpan> where TSpan : ISpan<T>
    {
        void Take(int count, out TSpan span);
        void TakeForPower(int power, out TSpan span);
        void Return(ref TSpan span);
    }

}