﻿using BepuUtilities;
using DemoContentLoader;
using SharpDX.Direct3D11;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using DemoRenderer.Attributes;

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

    public class LineRenderer : IDisposable
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

        const int maximumInstancesPerDraw = 2048;
#pragma warning disable 0649
        ConstantsBuffer<VertexConstants> vertexConstants;

        [InitialCapacity(maximumInstancesPerDraw)]
        StructuredBuffer<LineInstance> instances;
        [BoxIndices(1)]
        IndexBuffer indices;

        [Resource(@"Constraints\RenderLines.hlsl.vshader")]
        VertexShader vertexShader;
        [Resource(@"Constraints\RenderLines.hlsl.pshader")]
        PixelShader pixelShader;
#pragma warning restore 0649

        public LineRenderer()
        {
        }

        public void Render(DeviceContext context, Camera camera, Int2 resolution, Span<LineInstance> instances, int start, int count)
        {
            var vertexConstantsData = new VertexConstants
            {
                ViewProjection = Matrix.Transpose(camera.ViewProjection), //compensate for the shader packing.
                NDCToScreenScale = new Vector2(resolution.X / 2f, resolution.Y / 2f),
                CameraForward = camera.Forward,
                TanAnglePerPixel = (float)Math.Tan(camera.FieldOfView / resolution.Y),
                CameraRight = camera.Right,
                CameraPosition = camera.Position,
            };
            vertexConstants.Update(context, ref vertexConstantsData);

            //This assumes that render states have been set appropriately for opaque rendering.
            context.InputAssembler.InputLayout = null;
            context.InputAssembler.SetIndexBuffer(indices);
            context.InputAssembler.PrimitiveTopology = SharpDX.Direct3D.PrimitiveTopology.TriangleList;
            context.VertexShader.Set(vertexShader);
            context.VertexShader.SetConstantBuffer(0, vertexConstants.Buffer);
            context.VertexShader.SetShaderResource(0, this.instances.SRV);
            context.PixelShader.Set(pixelShader);

            while (count > 0)
            {
                var batchCount = Math.Min(this.instances.Capacity, count);
                this.instances.Update(context, instances, batchCount, start);
                context.DrawIndexedInstanced(36, batchCount, 0, 0, 0);
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
        ~LineRenderer()
        {
            Helpers.CheckForUndisposed(disposed, this);
        }
#endif
    }
}
