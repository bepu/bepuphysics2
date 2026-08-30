using DemoContentLoader;
using System.Numerics;
using System.Runtime.InteropServices;
using OpenTK.Graphics.OpenGL4;

namespace DemoRenderer.ShapeDrawing
{
    //This isn't exactly an efficient representation, but isolated triangle rendering is mainly just for small scale testing anyway.
    [StructLayout(LayoutKind.Explicit, Size = 64)]
    public struct TriangleInstance
    {
        [FieldOffset(0)]
        public Vector3 A;
        [FieldOffset(12)]
        public uint PackedColor;
        [FieldOffset(16)]
        public Vector3 B;
        [FieldOffset(28)]
        public float X;
        [FieldOffset(32)]
        public Vector3 C;
        [FieldOffset(44)]
        public float Y;
        [FieldOffset(48)]
        public ulong PackedOrientation;
        [FieldOffset(56)]
        public float Z;
    }
    public class TriangleRenderer : RasterizedRenderer<TriangleInstance>
    {
        public TriangleRenderer(ContentArchive content, int maximumInstancesPerDraw = 2048) : base(content, @"ShapeDrawing\RenderTriangles", maximumInstancesPerDraw)
        { }
        protected override void OnBatchDraw(int batchCount) =>
            GL.DrawArrays(PrimitiveType.Triangles, 0, 3 * batchCount);
    }
}
