using BepuUtilities;
using DemoContentLoader;
using SharpDX.Direct3D11;
using System;
using System.Numerics;
using System.Runtime.InteropServices;
using Quaternion = BepuUtilities.Quaternion;

namespace DemoRenderer.ShapeDrawing
{
    //These are out here because generic types (including nongeneric types held within generic classes) cannot have explicit layouts.
    [StructLayout(LayoutKind.Explicit)]
    struct RayTracedVertexConstants
    {
        [FieldOffset(0)]
        public Matrix Projection;
        [FieldOffset(64)]
        public Vector3 CameraPosition;
        [FieldOffset(76)]
        public float NearClip;
        [FieldOffset(80)]
        public Vector3 CameraRight;
        [FieldOffset(96)]
        public Vector3 CameraUp;
        [FieldOffset(112)]
        public Vector3 CameraBackward;
    }
    [StructLayout(LayoutKind.Explicit, Size = 64)]
    struct RayTracedPixelConstants
    {
        [FieldOffset(0)]
        public Vector3 CameraRight;
        [FieldOffset(12)]
        public float NearClip;
        [FieldOffset(16)]
        public Vector3 CameraUp;
        [FieldOffset(28)]
        public float FarClip;
        [FieldOffset(32)]
        public Vector3 CameraBackward;
        [FieldOffset(48)]
        public Vector2 PixelSizeAtUnitPlane;

    }
    public class RayTracedRenderer<TInstance> : IDisposable where TInstance : struct
    {
        //While multiple ray traced renderers will end up with redundant constants and some other details, it hardly matters. Doing it this way is super simple and low effort.


        ConstantsBuffer<RayTracedVertexConstants> vertexConstants;

        ConstantsBuffer<RayTracedPixelConstants> pixelConstants;

        StructuredBuffer<TInstance> instances;
        IndexBuffer indices;

        VertexShader vertexShader;
        PixelShader pixelShader;

        public RayTracedRenderer(Device device, ShaderCache cache, string shaderPath, int maximumInstancesPerDraw = 2048)
        {
            var instanceTypeName = typeof(TInstance).Name;
            instances = new StructuredBuffer<TInstance>(device, maximumInstancesPerDraw, $"{instanceTypeName} Instances");

            indices = new IndexBuffer(Helpers.GetBoxIndices(maximumInstancesPerDraw), device, $"{instanceTypeName} AABB Indices");

            vertexConstants = new ConstantsBuffer<RayTracedVertexConstants>(device, debugName: $"{instanceTypeName} Renderer Vertex Constants");
            pixelConstants = new ConstantsBuffer<RayTracedPixelConstants>(device, debugName: $"{instanceTypeName} Renderer Pixel Constants");

            vertexShader = new VertexShader(device, cache.GetShader($"{shaderPath}.vshader"));
            pixelShader = new PixelShader(device, cache.GetShader($"{shaderPath}.pshader"));
        }

        public void Render(DeviceContext context, Camera camera, Int2 screenResolution, Span<TInstance> instances, int start, int count)
        {
            var vertexConstantsData = new RayTracedVertexConstants
            {
                Projection = Matrix.Transpose(camera.Projection), //compensate for the shader packing.
                CameraPosition = camera.Position,
                CameraRight = camera.Right,
                NearClip = camera.NearClip,
                CameraUp = camera.Up,
                CameraBackward = camera.Backward,
            };
            vertexConstants.Update(context, ref vertexConstantsData);
            var viewportHeight = 2 * (float)Math.Tan(camera.FieldOfView / 2);
            var viewportWidth = viewportHeight * camera.AspectRatio;
            var pixelConstantsData = new RayTracedPixelConstants
            {
                CameraRight = camera.Right,
                NearClip = camera.NearClip,
                CameraUp = camera.Up,
                FarClip = camera.FarClip,
                CameraBackward = camera.Backward,
                PixelSizeAtUnitPlane = new Vector2(viewportWidth / screenResolution.X, viewportHeight / screenResolution.Y)
            };
            pixelConstants.Update(context, ref pixelConstantsData);

            //This assumes that render states have been set appropriately for opaque rendering.
            context.InputAssembler.InputLayout = null;
            context.InputAssembler.SetIndexBuffer(indices);
            context.InputAssembler.PrimitiveTopology = SharpDX.Direct3D.PrimitiveTopology.TriangleList;
            context.VertexShader.Set(vertexShader);
            context.VertexShader.SetConstantBuffer(0, vertexConstants.Buffer);
            context.VertexShader.SetShaderResource(0, this.instances.SRV);
            context.PixelShader.Set(pixelShader);
            context.PixelShader.SetConstantBuffer(1, pixelConstants.Buffer);

            while (count > 0)
            {
                var batchCount = Math.Min(this.instances.Capacity, count);
                this.instances.Update(context, instances, batchCount, start);
                context.DrawIndexed(batchCount * 36, 0, 0);
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
                indices.Dispose();
                vertexConstants.Dispose();
                pixelConstants.Dispose();
            }
        }

#if DEBUG
        ~RayTracedRenderer()
        {
            Helpers.CheckForUndisposed(disposed, this);
        }
#endif
    }
}
