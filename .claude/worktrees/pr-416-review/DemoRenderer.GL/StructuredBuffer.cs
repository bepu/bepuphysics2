using System;
using System.Runtime.CompilerServices;
using OpenTK.Graphics.OpenGL4;

namespace DemoRenderer
{
    public class StructuredBuffer<T> : Disposable where T : struct
    {
        /// <summary>
        /// Gets the size of individual elements within the buffer.
        /// </summary>
        public static readonly int Stride = Unsafe.SizeOf<T>();

        private int buffer;
        private readonly BufferTarget target;
        /// <summary>
        /// Gets the capacity of the buffer.
        /// </summary>
        public int Capacity { get; private set; }
        /// <summary>
        /// Gets the debug name associated with the buffer and its views.
        /// </summary>
        public readonly string DebugName;

        private void Allocate(int capacity)
        {
            buffer = GL.GenBuffer();
            Capacity = capacity;
            GL.BindBuffer(target, buffer);
            GL.BufferStorage(target, Capacity * Stride, IntPtr.Zero, BufferStorageFlags.DynamicStorageBit);
            GL.BindBuffer(target, 0);
        }

        /// <summary>
        /// Creates an immutable buffer filled with the given values.
        /// </summary>
        /// <param name="target">Target to which the buffer is bound.</param>
        /// <param name="initialCapacity">Number of elements that the buffer can hold by default.</param>
        /// <param name="debugName">Name to associate with the buffer.</param>
        public StructuredBuffer(BufferTarget target, int initialCapacity, string debugName = "UNNAMED")
        {
            this.target = target;
            Allocate(initialCapacity);
            DebugName = debugName;
        }

        /// <summary>
        /// Recreates the buffer without retaining old data.
        /// </summary>
        /// <param name="newCapacity">New capacity of the buffer.</param>
        public void SetCapacityWithoutCopy(int newCapacity)
        {
            DoDispose();
            Allocate(newCapacity);
        }

        /// <summary>
        /// Updates the buffer with the given values.
        /// </summary>
        /// <param name="newValues">Values to load into the buffer.</param>
        public void Update(Span<T> newValues) =>
            GL.NamedBufferSubData(buffer, IntPtr.Zero, newValues.Length * Stride, ref newValues[0]);

        /// <summary>
        /// Updates the buffer with the given values.
        /// </summary>
        /// <param name="newValues">Values to load into the buffer.</param>
        /// <param name="count">Number of elements from the source array to read.</param>
        /// <param name="sourceOffset">Offset from which to begin reading the new values.</param>
        /// <param name="destinationOffset">Index at which the stored values should start in the buffer.</param>
        public void Update(Span<T> newValues, int count, int sourceOffset = 0, int destinationOffset = 0) =>
            GL.NamedBufferSubData(buffer, (IntPtr)(destinationOffset * Stride), count * Stride, ref newValues[sourceOffset]);

        public void Bind(int index) => GL.BindBufferBase((BufferRangeTarget)target, index, buffer);
        protected override void DoDispose() => GL.DeleteBuffer(buffer);
    }
}
