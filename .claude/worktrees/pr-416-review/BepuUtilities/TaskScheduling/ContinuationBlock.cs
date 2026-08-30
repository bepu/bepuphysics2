using BepuUtilities.Memory;

namespace BepuUtilities.TaskScheduling;

/// <summary>
/// Stores a block of task continuations that maintains a pointer to previous blocks. 
/// </summary>
internal unsafe struct ContinuationBlock
{
    public ContinuationBlock* Previous;

    public int Count;
    public Buffer<TaskContinuation> Continuations;

    public static ContinuationBlock* Create(int continuationCapacity, BufferPool pool)
    {
        pool.Take<byte>(sizeof(TaskContinuation) * continuationCapacity + sizeof(ContinuationBlock), out var rawBuffer);
        ContinuationBlock* block = (ContinuationBlock*)rawBuffer.Memory;
        block->Continuations = new Buffer<TaskContinuation>(rawBuffer.Memory + sizeof(ContinuationBlock), continuationCapacity, rawBuffer.Id);
        block->Count = 0;
        block->Previous = null;
        return block;
    }

    public bool TryAllocateContinuation(out TaskContinuation* continuation)
    {
        if (Count < Continuations.length)
        {
            continuation = Continuations.Memory + Count++;
            return true;
        }
        continuation = null;
        return false;
    }

    public void Dispose(BufferPool pool)
    {
        var id = Continuations.Id;
        pool.ReturnUnsafely(id);
        if (Previous != null)
            Previous->Dispose(pool);
        this = default;
    }
}
