using BepuUtilities;
using DemoContentLoader;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using OpenTK.Graphics.OpenGL4;

namespace DemoRenderer.Constraints
{
    /// <summary>
    /// GPU-relevant information for the rendering of a single line instance.
    /// </summary>
    [StructLayout(LayoutKind.Explicit)]
    public struct LineInstance
    {
        [FieldOffset(0)]
        public Vector3 Start;
        [FieldOffset(12)]
        public uint PackedBackgroundColor;
        [FieldOffset(16)]
        public Vector3 End;
        [FieldOffset(28)]
        public uint PackedColor;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public LineInstance(in Vector3 start, in Vector3 end, in Vector3 color, in Vector3 backgroundColor)
        {
            Start = start;
            PackedBackgroundColor = Helpers.PackColor(backgroundColor);
            End = end;
            PackedColor = Helpers.PackColor(color);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public LineInstance(in Vector3 start, in Vector3 end, uint packedColor, uint packedBackgroundColor)
        {
            Start = start;
            PackedBackgroundColor = packedBackgroundColor;
            End = end;
            PackedColor = packedColor;
        }
    }

    public class LineRenderer : Shader
    {
        [StructLayout(LayoutKind.Explicit)]
        struct VertexConstants
        {
            [FieldOffset(0)]
            public Matrix ViewProjection;
            [FieldOffset(64)]
            public Vector2 NDCToScreenScale;
            [FieldOffset(80)]
            public Vector3 CameraForward;
            [FieldOffset(92)]
            public float TanAnglePerPixel;
            [FieldOffset(96)]
            public Vector3 CameraRight;
            [FieldOffset(112)]
            public Vector3 CameraPosition;
        }

        private readonly ConstantsBuffer<VertexConstants> vertexConstants;
        private readonly StructuredBuffer<LineInstance> instances;
        private readonly IndexBuffer indices;

        public LineRenderer(ContentArchive content, int maximumInstancesPerDraw = 16384) : base(
            content.Load<GLSLContent>(@"Constraints\RenderLines.glvs").Source,
            content.Load<GLSLContent>(@"Constraints\RenderLines.glfs").Source
        )
        {
            instances = new StructuredBuffer<LineInstance>(BufferTarget.ShaderStorageBuffer, maximumInstancesPerDraw, "Line Instances");
            indices = new IndexBuffer(Helpers.GetBoxIndices(maximumInstancesPerDraw), "Line Quad Indices");

            vertexConstants = new ConstantsBuffer<VertexConstants>(BufferTarget.UniformBuffer, debugName: "Line Renderer Vertex Constants");
        }

        public void Render(Camera camera, Int2 resolution, Span<LineInstance> instances, int start, int count)
        {
            Use();
            indices.Bind();
            vertexConstants.Bind(0);
            this.instances.Bind(0);

            var vertexConstantsData = new VertexConstants {
                ViewProjection = camera.ViewProjection,
                NDCToScreenScale = new Vector2(resolution.X / 2f, resolution.Y / 2f),
                CameraForward = camera.Forward,
                TanAnglePerPixel = (float)Math.Tan(camera.FieldOfView / resolution.Y),
                CameraRight = camera.Right,
                CameraPosition = camera.Position,
            };
            vertexConstants.Update(ref vertexConstantsData);

            while (count > 0)
            {
                var batchCount = Math.Min(this.instances.Capacity, count);
                this.instances.Update(instances, batchCount, start);
                GL.DrawElements(PrimitiveType.Triangles, batchCount * 36, indices.Type, 0);
                count -= batchCount;
                start += batchCount;
            }
        }

        protected override void DoDispose()
        {
            instances.Dispose();
            indices.Dispose();
            vertexConstants.Dispose();
            base.DoDispose();
        }
    }
}
