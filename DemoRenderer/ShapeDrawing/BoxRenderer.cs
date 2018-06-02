using BepuUtilities;
using DemoContentLoader;
using SharpDX.Direct3D11;
using System;
using System.Numerics;
using System.Runtime.InteropServices;
using Quaternion = BepuUtilities.Quaternion;

namespace DemoRenderer.ShapeDrawing
{
    //Could get this down to 32 bytes with some extra packing (e.g. 4 byte quaternion), but it would require some effort with really, really questionable gains.
    [StructLayout(LayoutKind.Explicit, Size = 48)]
    public struct BoxInstance
    {
        [FieldOffset(0)]
        public Vector3 Position;
        [FieldOffset(12)]
        public uint PackedColor;
        [FieldOffset(16)]
        public Quaternion Orientation;
        [FieldOffset(32)]
        public float HalfWidth;
        [FieldOffset(36)]
        public float HalfHeight;
        [FieldOffset(40)]
        public float HalfLength;
    }
    public class BoxRenderer : RasterizedRenderer<BoxInstance>
    {
        IndexBuffer indices;

        public BoxRenderer(Device device, ShaderCache cache, int maximumInstancesPerDraw = 2048) 
            : base(device, cache, @"ShapeDrawing\RenderBoxes.hlsl", maximumInstancesPerDraw)
        {
            indices = new IndexBuffer(Helpers.GetBoxIndices(maximumInstancesPerDraw), device, "Box Indices");
        }

        protected override void OnDrawSetup(DeviceContext context)
        {
            context.InputAssembler.SetIndexBuffer(indices);
        }
        protected override void OnBatchDraw(DeviceContext context, int batchCount)
        {
            context.DrawIndexed(36 * batchCount, 0, 0);
        }
        protected override void OnDispose()
        {
            indices.Dispose();
        }
    }
}
