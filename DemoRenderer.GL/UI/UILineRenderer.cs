using BepuUtilities;
using DemoContentLoader;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using OpenTK.Graphics.OpenGL4;

namespace DemoRenderer.UI
{
    /// <summary>
    /// GPU-relevant information for the rendering of a single screenspace line instance.
    /// </summary>
    public struct UILineInstance
    {
        /// <summary>
        /// Start location. X is stored in the lower 16 bits, Y in the upper 16. Should be scaled by the PackedToScreenScale.
        /// </summary>
        public uint PackedStart;
        /// <summary>
        /// End location. X is stored in the lower 16 bits, Y in the upper 16. Should be scaled by the PackedToScreenScale.
        /// </summary>
        public uint PackedEnd;
        /// <summary>
        /// Radius of the line in screen pixels.
        /// </summary>
        public float Radius;
        /// <summary>
        /// Color, packed in a UNORM manner such that bits 0 through 10 are R, bits 11 through 21 are G, and bits 22 through 31 are B.
        /// </summary>
        public uint PackedColor;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public UILineInstance(in Vector2 start, in Vector2 end, float radius, in Vector3 color, in Vector2 screenToPackedScale)
        {
            // screenspace of OpenGL +1 is up.
            PackedStart = (uint)(start.X * screenToPackedScale.X) | ((uint)(65535f - start.Y * screenToPackedScale.Y) << 16);
            PackedEnd = (uint)(end.X * screenToPackedScale.X) | ((uint)(65535f - end.Y * screenToPackedScale.Y) << 16);
            Radius = radius;
            PackedColor = Helpers.PackColor(color);
        }
    }

    public class UILineRenderer : Shader
    {
        struct VertexConstants
        {
            //Splitting these two scales (Packed->Screen followed by Screen->NDC) makes handling the radius easy, since it's in uniform screen pixels.
            public Vector2 PackedToScreenScale;
            public Vector2 ScreenToNDCScale;
        }

        private readonly ConstantsBuffer<VertexConstants> vertexConstants;
        private readonly StructuredBuffer<UILineInstance> instances;
        private readonly IndexBuffer indices;

        public UILineRenderer(ContentArchive content, int maximumLinesPerDraw = 2048) : base(
            content.Load<GLSLContent>(@"UI\RenderUILines.glvs").Source,
            content.Load<GLSLContent>(@"UI\RenderUILines.glfs").Source
        )
        {
            instances = new StructuredBuffer<UILineInstance>(BufferTarget.ShaderStorageBuffer, maximumLinesPerDraw, "UI Line Instances");
            indices = new IndexBuffer(Helpers.GetQuadIndices(maximumLinesPerDraw), "UI Line Indices");
            vertexConstants = new ConstantsBuffer<VertexConstants>(BufferTarget.UniformBuffer, debugName: "UI Line Renderer Vertex Constants");
        }

        public void Render(Int2 screenResolution, UILineInstance[] lines, int start, int count)
        {
            Use();
            indices.Bind();
            vertexConstants.Bind(0);
            instances.Bind(0);
            var vertexConstantsData = new VertexConstants {
                //The packed minimum must permit subpixel locations. So, distribute the range 0 to 65535 over the pixel range 0 to resolution.
                PackedToScreenScale = new Vector2(screenResolution.X / 65535f, screenResolution.Y / 65535f),
                ScreenToNDCScale = new Vector2(2f / screenResolution.X, 2f / screenResolution.Y),
            };
            vertexConstants.Update(ref vertexConstantsData);
            while (count > 0)
            {
                var batchCount = Math.Min(instances.Capacity, count);
                instances.Update(lines, batchCount, start);
                GL.DrawElements(PrimitiveType.Triangles, batchCount * 6, indices.Type, 0);
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
