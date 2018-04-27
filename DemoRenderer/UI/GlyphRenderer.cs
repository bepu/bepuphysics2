using BepuUtilities;
using BepuUtilities.Memory;
using DemoContentLoader;
using SharpDX.Direct3D11;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

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
        /// Color, packed in a UNORM manner such that bits 0 through 10 are R, bits 11 through 21 are G, and bits 22 through 31 are B.
        /// </summary>
        public uint PackedColor;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public GlyphInstance(ref Vector2 start, ref Vector2 horizontalAxis, float scale, int sourceId, ref Vector3 color, ref Vector2 screenToPackedScale)
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

    public class GlyphRenderer : IDisposable
    {
        struct VertexConstants
        {
            public Vector2 PackedToScreenScale;
            public Vector2 ScreenToNDCScale;
            public Vector2 InverseAtlasResolution;
        }
        ConstantsBuffer<VertexConstants> vertexConstants;

        StructuredBuffer<GlyphInstance> instances;
        IndexBuffer indices;

        SamplerState sampler;
        VertexShader vertexShader;
        PixelShader pixelShader;
        public GlyphRenderer(Device device, DeviceContext context, ShaderCache cache, int maximumGlyphsPerDraw = 2048)
        {
            instances = new StructuredBuffer<GlyphInstance>(device, maximumGlyphsPerDraw, "Glyph Instances");
            indices = new IndexBuffer(Helpers.GetQuadIndices(maximumGlyphsPerDraw), device, "Glyph Indices");

            var samplerDescription = SamplerStateDescription.Default();
            samplerDescription.Filter = Filter.MinMagMipLinear;
            sampler = new SamplerState(device, samplerDescription);

            vertexConstants = new ConstantsBuffer<VertexConstants>(device, debugName: "Glyph Renderer Vertex Constants");

            vertexShader = new VertexShader(device, cache.GetShader(@"UI\RenderGlyphs.hlsl.vshader"));
            pixelShader = new PixelShader(device, cache.GetShader(@"UI\RenderGlyphs.hlsl.pshader"));
        }

        /// <summary>
        /// Sets up the rendering pipeline with any glyph rendering specific render state that can be shared across all glyph batches drawn using the GlyphRenderer.Render function.
        /// </summary>
        /// <param name="context">Context to configure.</param>
        public void PreparePipeline(DeviceContext context)
        {
            //This assumes that rasterizer, blend, and depth states have been set appropriately for screenspace transparent rendering.
            context.InputAssembler.InputLayout = null;
            context.InputAssembler.SetIndexBuffer(indices);
            context.InputAssembler.PrimitiveTopology = SharpDX.Direct3D.PrimitiveTopology.TriangleList;
            context.VertexShader.Set(vertexShader);
            context.VertexShader.SetConstantBuffer(0, vertexConstants.Buffer);
            context.VertexShader.SetShaderResource(0, instances.SRV);
            context.PixelShader.Set(pixelShader);
            context.PixelShader.SetSampler(0, sampler);
        }

        public void Render(DeviceContext context, Font font, Int2 screenResolution, GlyphInstance[] glyphs, int start, int count)
        {
            var vertexConstantsData = new VertexConstants
            {
                //These first two scales could be uploaded once, but it would require another buffer. Not important enough.
                //The packed minimum must permit subpixel locations. So, distribute the range 0 to 65535 over the pixel range 0 to resolution.
                PackedToScreenScale = new Vector2(screenResolution.X / 65535f, screenResolution.Y / 65535f),
                ScreenToNDCScale = new Vector2(2f / screenResolution.X, -2f / screenResolution.Y),
                InverseAtlasResolution = new Vector2(1f / font.Content.Atlas.Width, 1f / font.Content.Atlas.Height)
            };
            vertexConstants.Update(context, ref vertexConstantsData);
            context.VertexShader.SetShaderResource(1, font.Sources.SRV);
            context.PixelShader.SetShaderResource(0, font.AtlasSRV);

            while (count > 0)
            {
                var batchCount = Math.Min(instances.Capacity, count);
                instances.Update(context, glyphs, batchCount, start);
                context.DrawIndexed(batchCount * 6, 0, 0);
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
                sampler.Dispose();
                vertexConstants.Dispose();
            }
        }

#if DEBUG
        ~GlyphRenderer()
        {
            Helpers.CheckForUndisposed(disposed, this);
        }
#endif
    }
}
