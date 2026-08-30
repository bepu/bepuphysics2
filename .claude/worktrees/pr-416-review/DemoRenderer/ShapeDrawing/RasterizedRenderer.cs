using BepuUtilities;
using DemoContentLoader;
using SharpDX.Direct3D11;
using System;
using System.Numerics;
using System.Runtime.InteropServices;

namespace DemoRenderer.ShapeDrawing
{
    [StructLayout(LayoutKind.Explicit)]
    struct RasterizedVertexConstants
    {
        [FieldOffset(0)]
        public Matrix Projection;
        [FieldOffset(64)]
        public Vector3 CameraPosition;
        [FieldOffset(80)]
        public Vector3 CameraRight;
        [FieldOffset(96)]
        public Vector3 CameraUp;
        [FieldOffset(112)]
        public Vector3 CameraBackward;
    }
    public class RasterizedRenderer<TInstance> : IDisposable where TInstance : struct
    {       
        ConstantsBuffer<RasterizedVertexConstants> vertexConstants;

        StructuredBuffer<TInstance> instances;

        VertexShader vertexShader;
        PixelShader pixelShader;

        public RasterizedRenderer(Device device, ShaderCache cache, string shaderPath, int maximumInstancesPerDraw = 2048)
        {
            string instanceTypeName = typeof(TInstance).Name;
            instances = new StructuredBuffer<TInstance>(device, maximumInstancesPerDraw, $"{instanceTypeName} Instances");
            
            vertexConstants = new ConstantsBuffer<RasterizedVertexConstants>(device, debugName: $"{instanceTypeName} Renderer Vertex Constants");

            vertexShader = new VertexShader(device, cache.GetShader($"{shaderPath}.vshader"));
            pixelShader = new PixelShader(device, cache.GetShader($"{shaderPath}.pshader"));
        }

        protected virtual void OnDrawSetup(DeviceContext context)
        {

        }

        protected virtual void OnBatchDraw(DeviceContext context, int batchCount)
        {

        }

        public void Render(DeviceContext context, Camera camera, Int2 screenResolution, Span<TInstance> instances, int start, int count)
        {
            var vertexConstantsData = new RasterizedVertexConstants
            {
                Projection = Matrix.Transpose(camera.Projection), //compensate for the shader packing.
                CameraPosition = camera.Position,
                CameraRight = camera.Right,
                CameraUp = camera.Up,
                CameraBackward = camera.Backward,
            };
            vertexConstants.Update(context, ref vertexConstantsData);

            //This assumes that render states have been set appropriately for opaque rendering.
            context.InputAssembler.InputLayout = null;
            context.InputAssembler.PrimitiveTopology = SharpDX.Direct3D.PrimitiveTopology.TriangleList;
            OnDrawSetup(context);
            context.VertexShader.Set(vertexShader);
            context.VertexShader.SetConstantBuffer(0, vertexConstants.Buffer);
            context.VertexShader.SetShaderResource(0, this.instances.SRV);
            context.PixelShader.Set(pixelShader);

            while (count > 0)
            {
                var batchCount = Math.Min(this.instances.Capacity, count);
                this.instances.Update(context, instances, batchCount, start);
                OnBatchDraw(context, batchCount);
                count -= batchCount;
                start += batchCount;
            }
        }


        protected virtual void OnDispose()
        {

        }
        bool disposed;
        public void Dispose()
        {
            if (!disposed)
            {
                disposed = true;
                vertexShader.Dispose();
                pixelShader.Dispose();
                instances.Dispose();
                vertexConstants.Dispose();
                OnDispose();
            }
        }

#if DEBUG
        ~RasterizedRenderer()
        {
            Helpers.CheckForUndisposed(disposed, this);
        }
#endif
    }
}
