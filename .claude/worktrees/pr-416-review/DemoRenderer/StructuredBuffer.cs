using System;
using SharpDX;
using SharpDX.Direct3D11;
using Buffer = SharpDX.Direct3D11.Buffer;

namespace DemoRenderer
{
    public class StructuredBuffer<T> : IDisposable where T : struct
    {
        private Buffer buffer;
        private ShaderResourceView srv;
        private UnorderedAccessView uav;
        private Device device;

        /// <summary>
        /// Gets the underlying structured buffer resource.
        /// </summary>
        public Buffer Buffer
        {
            get { return buffer; }
        }

        /// <summary>
        /// Gets an SRV of the structured buffer.
        /// </summary>
        public ShaderResourceView SRV
        {
            get { return srv; }
        }

        /// <summary>
        /// Gets a UAV of the structured buffer.
        /// </summary>
        public UnorderedAccessView UAV
        {
            get { return uav; }
        }

        /// <summary>
        /// Gets the capacity of the buffer.
        /// </summary>
        public int Capacity { get; private set; }

        private string debugName;
        /// <summary>
        /// Gets or sets the debug name associated with the buffer and its views.
        /// </summary>
        public string DebugName
        {
            get { return debugName; }
            set
            {
                debugName = value;
                buffer.DebugName = debugName;
                srv.DebugName = debugName + " SRV";
                uav.DebugName = debugName + " UAV";
            }
        }

        /// <summary>
        /// Gets the size of individual elements within the buffer.
        /// </summary>
        public int Stride { get; private set; }

        /// <summary>
        /// Creates an immutable buffer filled with the given values.
        /// </summary>
        /// <param name="device">Device used to create the buffer.</param>
        /// <param name="initialCapacity">Number of elements that the buffer can hold by default.</param>
        /// <param name="debugName">Name to associate with the buffer.</param>
        public StructuredBuffer(Device device, int initialCapacity, string debugName = "UNNAMED")
        {
            this.device = device;
            this.debugName = debugName;
            Stride = Utilities.SizeOf<T>();
            CreateBufferForSize(initialCapacity, out buffer, out srv, out uav);
            Capacity = initialCapacity;
        }

        private void CreateBufferForSize(int size, out Buffer newBuffer, out ShaderResourceView newSRV, out UnorderedAccessView newUAV)
        {
            newBuffer = new Buffer(device, new BufferDescription
            {
                BindFlags = BindFlags.ShaderResource | BindFlags.UnorderedAccess,
                CpuAccessFlags = CpuAccessFlags.None,
                OptionFlags = ResourceOptionFlags.BufferStructured,
                SizeInBytes = size * Stride,
                StructureByteStride = Stride,
                Usage = ResourceUsage.Default
            });
            newBuffer.DebugName = debugName;
            newSRV = new ShaderResourceView(device, newBuffer);
            newSRV.DebugName = debugName + " SRV";
            newUAV = new UnorderedAccessView(device, newBuffer);
            newUAV.DebugName = debugName + " UAV";
        }

        /// <summary>
        /// Sets the capacity of the vertex buffer, retaining old data. If the new buffer is smaller than the old one, the remainder will be truncated.
        /// </summary>
        /// <param name="context">Context used to perform the copy operation.</param>
        /// <param name="newCapacity">New capacity of the buffer.</param>
        public void SetCapacityWithCopy(DeviceContext context, int newCapacity)
        {
            CreateBufferForSize(newCapacity, out Buffer newBuffer, out ShaderResourceView newSRV, out UnorderedAccessView newUAV);

            //Copies data from the previous buffer to the new buffer, truncating anything which does not fit.
            context.CopySubresourceRegion(buffer, 0, new ResourceRegion(0, 0, 0, Math.Min(Capacity, newCapacity) * Stride, 1, 1), newBuffer, 0);
            buffer.Dispose();
            srv.Dispose();
            uav.Dispose();
            buffer = newBuffer;
            srv = newSRV;
            uav = newUAV;
            Capacity = newCapacity;
        }

        /// <summary>
        /// Recreates the buffer without retaining old data.
        /// </summary>
        /// <param name="newCapacity">New capacity of the buffer.</param>
        public void SetCapacityWithoutCopy(int newCapacity)
        {
            buffer.Dispose();
            srv.Dispose();
            uav.Dispose();
            CreateBufferForSize(newCapacity, out buffer, out srv, out uav);
            Capacity = newCapacity;
        }

        /// <summary>
        /// Updates the buffer with the given values.
        /// </summary>
        /// <param name="context">Device context used to update the buffer.</param>
        /// <param name="newValues">Values to load into the buffer.</param>
        public void Update(DeviceContext context, Span<T> newValues)
        {
            context.UpdateBuffer(buffer, newValues, newValues.Length);
        }

        /// <summary>
        /// Updates the buffer with the given values.
        /// </summary>
        /// <param name="context">Device context used to update the buffer.</param>
        /// <param name="newValues">Values to load into the buffer.</param>
        /// <param name="sourceOffset">Offset from which to begin reading the new values.</param>
        /// <param name="count">Number of elements from the source array to read.</param>
        /// <param name="destinationOffset">Index at which the stored values should start in the buffer.</param>
        public void Update(DeviceContext context, Span<T> newValues, int count, int sourceOffset = 0, int destinationOffset = 0)
        {
            context.UpdateBuffer(buffer, newValues, count, sourceOffset, destinationOffset);
        }


        private bool disposed;
        public void Dispose()
        {
            if (!disposed)
            {
                disposed = true;
                uav.Dispose();
                srv.Dispose();
                buffer.Dispose();
            }
        }

#if DEBUG
        ~StructuredBuffer()
        {
            Helpers.CheckForUndisposed(disposed, this);
        }
#endif
    }
}
