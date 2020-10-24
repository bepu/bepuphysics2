using BepuUtilities;
using DemoContentLoader;
using System;
using System.Numerics;
using System.Runtime.InteropServices;
using OpenTK.Graphics.OpenGL4;

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
    public class RasterizedRenderer<TInstance> : Shader where TInstance : struct
    {
        private readonly StructuredBuffer<TInstance> instances;
        private readonly ConstantsBuffer<RasterizedVertexConstants> vertexConstants;

        public RasterizedRenderer(ContentArchive content, string shaderPath, int maximumInstancesPerDraw = 2048) : base(
            content.Load<GLSLContent>($"{shaderPath}.glvs").Source,
            content.Load<GLSLContent>($"{shaderPath}.glfs").Source
        )
        {
            string instanceTypeName = typeof(TInstance).Name;
            instances = new StructuredBuffer<TInstance>(BufferTarget.ShaderStorageBuffer, maximumInstancesPerDraw, $"{instanceTypeName} Instances");
            vertexConstants = new ConstantsBuffer<RasterizedVertexConstants>(BufferTarget.UniformBuffer, debugName: $"{instanceTypeName} Renderer Vertex Constants");
        }

        protected virtual void OnDrawSetup() { }
        protected virtual void OnBatchDraw(int batchCount) { }

        public void Render(Camera camera, Int2 screenResolution, Span<TInstance> instances, int start, int count)
        {
            Use();
            vertexConstants.Bind(0);
            this.instances.Bind(0);
            OnDrawSetup();

            var vertexConstantsData = new RasterizedVertexConstants
            {
                Projection = camera.Projection,
                CameraPosition = camera.Position,
                CameraRight = camera.Right,
                CameraUp = camera.Up,
                CameraBackward = camera.Backward,
            };
            vertexConstants.Update(ref vertexConstantsData);

            while (count > 0)
            {
                var batchCount = Math.Min(this.instances.Capacity, count);
                this.instances.Update(instances, batchCount, start);
                OnBatchDraw(batchCount);
                count -= batchCount;
                start += batchCount;
            }
        }

        protected override void DoDispose()
        {
            instances.Dispose();
            vertexConstants.Dispose();
            base.DoDispose();
        }
    }
}
