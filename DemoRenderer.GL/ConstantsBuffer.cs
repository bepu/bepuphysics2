using System;
using System.Runtime.CompilerServices;
using OpenTK.Graphics.OpenGL4;

namespace DemoRenderer
{
    public class ConstantsBuffer<T> : Disposable where T : struct
    {
        private static readonly int alignedSize;

        static ConstantsBuffer()
        {
            var size = Unsafe.SizeOf<T>();
            alignedSize = (size >> 4) << 4;
            if (alignedSize < size)
                alignedSize += 16;
        }

        private readonly int buffer = GL.GenBuffer();
        private readonly BufferTarget target;
        public readonly string DebugName;

        /// <summary>
        /// Creates a constants buffer.
        /// </summary>
        /// <param name="target">Target to which the buffer is bound.</param>
        /// <param name="debugName">Name to associate with the buffer.</param>
        public ConstantsBuffer(BufferTarget target, string debugName = "UNNAMED")
        {
            this.target = target;
            GL.BindBuffer(target, buffer);
            GL.BufferStorage(target, alignedSize, IntPtr.Zero, BufferStorageFlags.DynamicStorageBit);
            GL.BindBuffer(target, 0);
            DebugName = debugName;
        }
        /// <summary>
        /// Updates the buffer with the given data.
        /// </summary>
        /// <param name="bufferData">Data to load into the buffer.</param>
        public unsafe void Update(ref T bufferData) =>
            GL.NamedBufferSubData(buffer, IntPtr.Zero, alignedSize, ref bufferData);
        public void Bind(int index) => GL.BindBufferBase((BufferRangeTarget)target, index, buffer);
        protected override void DoDispose() => GL.DeleteBuffer(buffer);
    }
}
