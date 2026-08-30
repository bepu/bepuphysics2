using BepuUtilities;
using DemoContentLoader;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using OpenTK.Graphics.OpenGL4;

namespace DemoRenderer.UI
{
    /// <summary>
    /// GPU-relevant information for the rendering of a single character glyph instance.
    /// </summary>
    public struct GlyphInstance
    {
        /// <summary>
        /// Packed location of the minimum corner of the glyph. Lower 16 bits is X, upper 16 bits is Y. Should be scaled by PackedToScreen.
        /// </summary>
        public uint PackedMinimum;
        /// <summary>
        /// Packed horizontal axis used by the glyph. Lower 16 bits is X, upper 16 bits is Y. UNORM packed across a range from -1.0 at 0 to 1.0 at 65534.
        /// </summary>
        public uint PackedHorizontalAxis;
        /// <summary>
        /// The combination of two properties: scale to apply to the source glyph. UNORM packed across a range of 0.0 at 0 to 16.0 at 65535, stored in the lower 16 bits,
        /// and the id of the glyph type in the font stored in the upper 16 bits.
        /// </summary>
        public uint PackedScaleAndSourceId;
        /// <summary>
        /// RGBA color, packed in a UNORM manner such that bits 0 through 7 are R, bits 8 through 15 are G, bits 16 through 23 are B, and bits 24 through 31 are A.
        /// </summary>
        public uint PackedColor;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public GlyphInstance(ref Vector2 start, ref Vector2 horizontalAxis, float scale, int sourceId, ref Vector4 color, ref Vector2 screenToPackedScale)
        {
            //Note that this can do some weird stuff if the position is outside of the target range. For the sake of the demos, we just assume everything's in frame.
            //If you want to use this for a game where you can't guarantee that everything's in frame, this packing range would need to be modified.
            //One simple option is to just set the mapped region to extend beyond the rendered target. It reduces the precision density a bit, but that's not too important.
            PackedMinimum = (uint)(start.X * screenToPackedScale.X) | ((uint)(start.Y * screenToPackedScale.Y) << 16);
            var scaledAxisX = (uint)(horizontalAxis.X * 32767f + 32767f);
            var scaledAxisY = (uint)(horizontalAxis.Y * 32767f + 32767f);
            Debug.Assert(scaledAxisX <= 65534);
            Debug.Assert(scaledAxisY <= 65534);
            PackedHorizontalAxis = scaledAxisX | (scaledAxisY << 16);
            var packScaledScale = scale * (65535f / 16f);
            Debug.Assert(packScaledScale >= 0);
            if (packScaledScale > 65535f)
                packScaledScale = 65535f;
            Debug.Assert(sourceId >= 0 && sourceId < 65536);
            PackedScaleAndSourceId = (uint)packScaledScale | (uint)(sourceId << 16);
            PackedColor = Helpers.PackColor(color);
        }
    }

    public class GlyphRenderer : Shader
    {
        struct VertexConstants
        {
            public Vector2 PackedToScreenScale;
            public Vector2 ScreenToNDCScale;
            public Vector2 InverseAtlasResolution;
        }

        private readonly ConstantsBuffer<VertexConstants> vertexConstants;
        private readonly StructuredBuffer<GlyphInstance> instances;
        private readonly IndexBuffer indices;

        public GlyphRenderer(ContentArchive content, int maximumGlyphsPerDraw = 2048) : base(
            content.Load<GLSLContent>(@"UI\RenderGlyphs.glvs").Source,
            content.Load<GLSLContent>(@"UI\RenderGlyphs.glfs").Source
        )
        {
            instances = new StructuredBuffer<GlyphInstance>(BufferTarget.ShaderStorageBuffer, maximumGlyphsPerDraw, "Glyph Instances");
            indices = new IndexBuffer(Helpers.GetQuadIndices(maximumGlyphsPerDraw), "Glyph Indices");
            vertexConstants = new ConstantsBuffer<VertexConstants>(BufferTarget.UniformBuffer, debugName: "Glyph Renderer Vertex Constants");
        }

        /// <summary>
        /// Sets up the rendering pipeline with any glyph rendering specific render state that can be shared across all glyph batches drawn using the GlyphRenderer.Render function.
        /// </summary>
        public void PreparePipeline()
        {
            Use();
            indices.Bind();
            vertexConstants.Bind(0);
            instances.Bind(0);
        }

        public void Render(Font font, Int2 screenResolution, Span<GlyphInstance> glyphs)
        {
            font.Sources.Bind(1);
            GL.BindTexture(TextureTarget.Texture2D, font.Atlas);
            var vertexConstantsData = new VertexConstants {
                //These first two scales could be uploaded once, but it would require another buffer. Not important enough.
                //The packed minimum must permit subpixel locations. So, distribute the range 0 to 65535 over the pixel range 0 to resolution.
                PackedToScreenScale = new Vector2(screenResolution.X / 65535f, screenResolution.Y / 65535f),
                ScreenToNDCScale = new Vector2(2f / screenResolution.X, -2f / screenResolution.Y),
                InverseAtlasResolution = new Vector2(1f / font.Content.Atlas.Width, 1f / font.Content.Atlas.Height)
            };
            vertexConstants.Update(ref vertexConstantsData);
            var count = glyphs.Length;
            var start = 0;
            while (count > 0)
            {
                var batchCount = Math.Min(instances.Capacity, count);
                instances.Update(glyphs.Slice(start, batchCount));
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
