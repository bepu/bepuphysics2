using BepuUtilities.Memory;

namespace BepuPhysics.Collidables
{
	public interface IDisposableShape : IShape
	{
		/// <summary>
		/// Returns all resources used by the shape instance to the given pool.
		/// </summary>
		/// <param name="pool">Pool to return shape resources to.</param>
		void Dispose(BufferPool pool);
	}
}
