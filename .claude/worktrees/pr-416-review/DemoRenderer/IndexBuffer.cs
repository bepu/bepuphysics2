
using System;
using SharpDX.DXGI;
using SharpDX.Direct3D11;
using Buffer = SharpDX.Direct3D11.Buffer;
using Device = SharpDX.Direct3D11.Device;

namespace DemoRenderer
{
    public class IndexBuffer : IDisposable
    {
        public Buffer Buffer { get; private set; }

        /// <summary>
        /// Gets the format used by the index bufer.
        /// </summary>
        public Format Format { get; private set; }

        /// <summary>
        /// Gets the number of indices in the index buffer.
        /// </summary>
        public int Length { get; private set; }

        /// <summary>
        /// Gets the number of bytes in the index buffer.
        /// </summary>
        public int SizeInBytes
        {
            get { return Length * (Format == Format.R32_UInt ? sizeof(uint) : sizeof(ushort)); }
        }

        public IndexBuffer(uint[] indices, Device device, string debugName = "UNNAMED INDEX BUFFER")
        {
            Format = Format.R32_UInt;
            Length = indices.Length;
            Buffer = Buffer.Create(device, indices, new BufferDescription
            {
                BindFlags = BindFlags.IndexBuffer,
                CpuAccessFlags = CpuAccessFlags.None,
                OptionFlags = ResourceOptionFlags.None,
                SizeInBytes = sizeof(uint) * indices.Length,
                StructureByteStride = sizeof(uint),
                Usage = ResourceUsage.Immutable
            });
            Buffer.DebugName = debugName;
        }

        public IndexBuffer(ushort[] indices, Device device, string debugName = "UNNAMED INDEX BUFFER")
        {
            Format = Format.R16_UInt;
            Length = indices.Length;
            Buffer = Buffer.Create(device, indices, new BufferDescription
            {
                BindFlags = BindFlags.IndexBuffer,
                CpuAccessFlags = CpuAccessFlags.None,
                OptionFlags = ResourceOptionFlags.None,
                SizeInBytes = sizeof(ushort) * indices.Length,
                StructureByteStride = sizeof(ushort),
                Usage = ResourceUsage.Immutable
            });
            Buffer.DebugName = debugName;
        }



        private bool disposed;
        public void Dispose()
        {
            if (!disposed)
            {
                disposed = true;
                Buffer.Dispose();
            }
        }

#if DEBUG
        ~IndexBuffer()
        {
            Helpers.CheckForUndisposed(disposed, this);
        }
#endif

    }
}
