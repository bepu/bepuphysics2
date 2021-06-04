using BepuUtilities;
using DemoContentLoader;
using System;
using System.Numerics;
using System.Runtime.InteropServices;
using OpenTK.Graphics.OpenGL4;

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
    public class RayTracedRenderer<TInstance> : Shader where TInstance : struct
    {
        //While multiple ray traced renderers will end up with redundant constants and some other details, it hardly matters. Doing it this way is super simple and low effort.

        private readonly ConstantsBuffer<RayTracedVertexConstants> vertexConstants;
        private readonly ConstantsBuffer<RayTracedPixelConstants> pixelConstants;
        private readonly StructuredBuffer<TInstance> instances;
        private readonly IndexBuffer indices;

        public RayTracedRenderer(ContentArchive content, string shaderPath, int maximumInstancesPerDraw = 2048) : base(
            content.Load<GLSLContent>($"{shaderPath}.glvs").Source,
            content.Load<GLSLContent>($"{shaderPath}.glfs").Source
        )
        {
            var instanceTypeName = typeof(TInstance).Name;
            instances = new StructuredBuffer<TInstance>(BufferTarget.ShaderStorageBuffer, maximumInstancesPerDraw, $"{instanceTypeName} Instances");
            indices = new IndexBuffer(Helpers.GetBoxIndices(maximumInstancesPerDraw), $"{instanceTypeName} AABB Indices");
            vertexConstants = new ConstantsBuffer<RayTracedVertexConstants>(BufferTarget.UniformBuffer, debugName: $"{instanceTypeName} Renderer Vertex Constants");
            pixelConstants = new ConstantsBuffer<RayTracedPixelConstants>(BufferTarget.UniformBuffer, debugName: $"{instanceTypeName} Renderer Pixel Constants");
        }

        public void Render(Camera camera, Int2 screenResolution, Span<TInstance> instances, int start, int count)
        {
            Use();
            indices.Bind();
            vertexConstants.Bind(0);
            this.instances.Bind(0);
            pixelConstants.Bind(1);

            var vertexConstantsData = new RayTracedVertexConstants {
                Projection = camera.Projection,
                CameraPosition = camera.Position,
                CameraRight = camera.Right,
                NearClip = camera.NearClip,
                CameraUp = camera.Up,
                CameraBackward = camera.Backward,
            };
            vertexConstants.Update(ref vertexConstantsData);
            var viewportHeight = 2 * (float)Math.Tan(camera.FieldOfView / 2);
            var viewportWidth = viewportHeight * camera.AspectRatio;
            var pixelConstantsData = new RayTracedPixelConstants {
                CameraRight = camera.Right,
                NearClip = camera.NearClip,
                CameraUp = camera.Up,
                FarClip = camera.FarClip,
                CameraBackward = camera.Backward,
                PixelSizeAtUnitPlane = new Vector2(viewportWidth / screenResolution.X, viewportHeight / screenResolution.Y)
            };
            pixelConstants.Update(ref pixelConstantsData);

            while (count > 0)
            {
                var batchCount = Math.Min(this.instances.Capacity, count);
                this.instances.Update(instances, batchCount, start);
                GL.DrawElements(PrimitiveType.Triangles, 36 * batchCount, indices.Type, 0);
                count -= batchCount;
                start += batchCount;
            }
        }

        protected override void DoDispose()
        {
            instances.Dispose();
            indices.Dispose();
            vertexConstants.Dispose();
            pixelConstants.Dispose();
            base.DoDispose();
        }
    }
}
