using BepuUtilities.Memory;

namespace BepuPhysics
{
    public class Deactivator
    {
        public Buffer<Island> Islands;
        IdPool<Buffer<int>> islandIdPool;
        BufferPool pool;
        public Deactivator(BufferPool pool)
        {
            this.pool = pool;
            IdPool<Buffer<int>>.Create(pool.SpecializeFor<int>(), 16, out var islandIdPool);
        }

        public void Dispose()
        {
            islandIdPool.Dispose(pool.SpecializeFor<int>());
        }
    }
}
