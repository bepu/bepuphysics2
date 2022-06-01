﻿using BepuUtilities;
using BepuUtilities.Memory;
using DemoContentLoader;
using DemoRenderer.Attributes;
using SharpDX.Direct3D11;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

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
            PackedStart = (uint)(start.X * screenToPackedScale.X) | ((uint)(start.Y * screenToPackedScale.Y) << 16);
            PackedEnd = (uint)(end.X * screenToPackedScale.X) | ((uint)(end.Y * screenToPackedScale.Y) << 16);
            Radius = radius;
            PackedColor = Helpers.PackColor(color);
        }
    }

    public class UILineRenderer : IDisposable
    {
        struct VertexConstants
        {
            //Splitting these two scales (Packed->Screen followed by Screen->NDC) makes handling the radius easy, since it's in uniform screen pixels.
            public Vector2 PackedToScreenScale;
            public Vector2 ScreenToNDCScale;
        }
        const int maximumLinesPerDraw = 2048;
#pragma warning disable 0649
        ConstantsBuffer<VertexConstants> vertexConstants;

        [InitialCapacity(maximumLinesPerDraw)]
        StructuredBuffer<UILineInstance> instances;
        [QuadIndices(maximumLinesPerDraw)]
        IndexBuffer indices;

        [Resource(@"UI\RenderUILines.hlsl.vshader")]
        VertexShader vertexShader;
        [Resource(@"UI\RenderUILines.hlsl.pshader")]
        PixelShader pixelShader;
#pragma warning restore 0649
        public UILineRenderer()
        {
        }

        public void Render(DeviceContext context, Int2 screenResolution, UILineInstance[] lines, int start, int count)
        {
            //This assumes that rasterizer, blend, and depth states have been set appropriately for screenspace transparent rendering.
            context.InputAssembler.InputLayout = null;
            context.InputAssembler.SetIndexBuffer(indices);
            context.InputAssembler.PrimitiveTopology = SharpDX.Direct3D.PrimitiveTopology.TriangleList;
            context.VertexShader.Set(vertexShader);
            context.VertexShader.SetConstantBuffer(0, vertexConstants.Buffer);
            context.VertexShader.SetShaderResource(0, instances.SRV);
            var vertexConstantsData = new VertexConstants
            {
                //The packed minimum must permit subpixel locations. So, distribute the range 0 to 65535 over the pixel range 0 to resolution.
                PackedToScreenScale = new Vector2(screenResolution.X / 65535f, screenResolution.Y / 65535f),
                ScreenToNDCScale = new Vector2(2f / screenResolution.X, -2f / screenResolution.Y),
            };
            vertexConstants.Update(context, ref vertexConstantsData);
            context.PixelShader.Set(pixelShader);

            while (count > 0)
            {
                var batchCount = Math.Min(instances.Capacity, count);
                instances.Update(context, lines, batchCount, start);
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
                vertexConstants.Dispose();
            }
        }

#if DEBUG
        ~UILineRenderer()
        {
            Helpers.CheckForUndisposed(disposed, this);
        }
#endif
    }
}
