using BepuUtilities;
using DemoContentLoader;
using SharpDX.Direct3D11;
using System;
using System.Numerics;
using System.Runtime.InteropServices;
using Quaternion = BepuUtilities.Quaternion;

namespace DemoRenderer.ShapeDrawing
{
    /// <summary>
    /// GPU-relevant information for the rendering of a single sphere instance.
    /// </summary>
    public struct SphereInstance
    {
        public Vector3 Position;
        public float Radius;
        public Vector3 PackedOrientation;
        public uint PackedColor;
    }

    public class SphereRenderer : IDisposable
    {
        [StructLayout(LayoutKind.Explicit)]
        struct VertexConstants
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
        ConstantsBuffer<VertexConstants> vertexConstants;
        [StructLayout(LayoutKind.Explicit, Size = 64)]
        struct PixelConstants
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
        ConstantsBuffer<PixelConstants> pixelConstants;

        StructuredBuffer<SphereInstance> instances;
        IndexBuffer indices;

        VertexShader vertexShader;
        PixelShader pixelShader;

        public SphereRenderer(Device device, ShaderCache cache, int maximumInstancesPerDraw = 2048)
        {
            instances = new StructuredBuffer<SphereInstance>(device, maximumInstancesPerDraw, "Sphere Instances");

            indices = new IndexBuffer(Helpers.GetBoxIndices(maximumInstancesPerDraw), device, "Sphere AABB Indices");

            vertexConstants = new ConstantsBuffer<VertexConstants>(device, debugName: "Sphere Renderer Vertex Constants");
            pixelConstants = new ConstantsBuffer<PixelConstants>(device, debugName: "Sphere Renderer Pixel Constants");

            vertexShader = new VertexShader(device, cache.GetShader(@"ShapeDrawing\RenderSpheres.hlsl.vshader"));
            pixelShader = new PixelShader(device, cache.GetShader(@"ShapeDrawing\RenderSpheres.hlsl.pshader"));
        }

        public void Render(DeviceContext context, Camera camera, Int2 screenResolution, SphereInstance[] instances, int start, int count)
        {
            var vertexConstantsData = new VertexConstants
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
            var pixelConstantsData = new PixelConstants
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
            }
        }

#if DEBUG
        ~SphereRenderer()
        {
            Helpers.CheckForUndisposed(disposed, this);
        }
#endif
    }
}
