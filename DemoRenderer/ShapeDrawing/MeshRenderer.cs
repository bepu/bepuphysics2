using BepuUtilities;
using BepuUtilities.Collections;
using DemoContentLoader;
using SharpDX.Direct3D11;
using System;
using System.Numerics;
using System.Runtime.InteropServices;
using Quaternion = BepuUtilities.Quaternion;

namespace DemoRenderer.ShapeDrawing
{
    [StructLayout(LayoutKind.Explicit, Size = 48)]
    public struct MeshInstance
    {
        [FieldOffset(0)]
        public Vector3 Position;
        [FieldOffset(12)]
        public uint PackedColor;
        [FieldOffset(16)]
        public ulong PackedOrientation;
        [FieldOffset(24)]
        public int VertexStart;
        [FieldOffset(28)]
        public int VertexCount;
        [FieldOffset(32)]
        public Vector3 Scale;
    }
    public class MeshRenderer : IDisposable
    {
        MeshCache meshCache;
   
        ConstantsBuffer<RasterizedVertexConstants> vertexConstants;

        StructuredBuffer<MeshInstance> instances;

        VertexShader vertexShader;
        PixelShader pixelShader;

        public MeshRenderer(Device device, MeshCache meshCache, ShaderCache cache, int maximumInstancesPerDraw = 2048)
        {
            this.meshCache = meshCache;
            instances = new StructuredBuffer<MeshInstance>(device, maximumInstancesPerDraw, $"Mesh Instances");

            vertexConstants = new ConstantsBuffer<RasterizedVertexConstants>(device, debugName: $"Mesh Renderer Vertex Constants");

            vertexShader = new VertexShader(device, cache.GetShader(@"ShapeDrawing\RenderMeshes.hlsl.vshader"));
            pixelShader = new PixelShader(device, cache.GetShader(@"ShapeDrawing\RenderMeshes.hlsl.pshader"));
        }

        public void Render(DeviceContext context, Camera camera, Int2 screenResolution, MeshInstance[] instances, int start, int count)
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
            context.VertexShader.Set(vertexShader);
            context.VertexShader.SetConstantBuffer(0, vertexConstants.Buffer);
            context.VertexShader.SetShaderResource(0, this.instances.SRV);
            context.VertexShader.SetShaderResource(1, meshCache.TriangleBuffer.SRV);
            context.PixelShader.Set(pixelShader);
            

            while (count > 0)
            {
                var batchCount = Math.Min(this.instances.Capacity, count);
                this.instances.Update(context, instances, batchCount, start);
                //context.DrawInstanced()
                count -= batchCount;
                start += batchCount;
            }
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
            }
        }

#if DEBUG
        ~MeshRenderer()
        {
            Helpers.CheckForUndisposed(disposed, this);
        }
#endif
    }
}
