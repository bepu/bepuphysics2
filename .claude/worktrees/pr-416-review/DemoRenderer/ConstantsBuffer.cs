using System;
using SharpDX;
using SharpDX.Direct3D11;
using Buffer = SharpDX.Direct3D11.Buffer;
using BepuUtilities.Memory;

namespace DemoRenderer
{
    public class ConstantsBuffer<T> : IDisposable where T : struct
    {
        private Buffer buffer;
        public Buffer Buffer { get { return buffer; } }

        private bool mappable;

        /// <summary>
        /// Gets the capacity of the buffer.
        /// </summary>
        public int Capacity { get; private set; }


        /// <summary>
        /// Creates a constants buffer.
        /// </summary>
        /// <param name="device">Device used to create the buffer.</param>
        /// <param name="debugName">Name to associate with the buffer.</param>
        /// <param name="mappable">If true, the buffer will be mapped with WriteDiscard when updated. If false, UpdateSubresource will be used.</param>
        public ConstantsBuffer(Device device, bool mappable = true, string debugName = "UNNAMED")
        {
            this.mappable = mappable;

            var size = Utilities.SizeOf<T>();
            var alignedSize = (size >> 4) << 4;
            if (alignedSize < size)
                alignedSize += 16;
            buffer = new Buffer(device, new BufferDescription
            {
                BindFlags = BindFlags.ConstantBuffer,
                CpuAccessFlags = mappable ? CpuAccessFlags.Write : CpuAccessFlags.None,
                OptionFlags = ResourceOptionFlags.None,
                SizeInBytes = alignedSize,
                StructureByteStride = alignedSize,
                Usage = mappable ? ResourceUsage.Dynamic : ResourceUsage.Default
            });
            buffer.DebugName = debugName;
        }


        /// <summary>
        /// Updates the buffer with the given data.
        /// </summary>
        /// <param name="context">Device context used to update the buffer.</param>
        /// <param name="bufferData">Data to load into the buffer.</param>
        public void Update(DeviceContext context, ref T bufferData)
        {
            if (mappable)
            {
                var dataBox = context.MapSubresource(Buffer, 0, MapMode.WriteDiscard, MapFlags.None);
                Utilities.Write(dataBox.DataPointer, ref bufferData);
                context.UnmapSubresource(buffer, 0);
            }
            else
            {
                context.UpdateSubresource(ref bufferData, Buffer);
            }
        }



        private bool disposed;
        public void Dispose()
        {
            if (!disposed)
            {
                disposed = true;
                buffer.Dispose();
            }
        }

#if DEBUG
        ~ConstantsBuffer()
        {
            Helpers.CheckForUndisposed(disposed, this);
        }
#endif
    }
}
