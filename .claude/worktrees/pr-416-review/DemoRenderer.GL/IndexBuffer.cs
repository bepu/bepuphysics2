using OpenTK.Graphics.OpenGL4;

namespace DemoRenderer
{
    public class IndexBuffer : Disposable
    {
        private readonly int buffer = GL.GenBuffer();
        public DrawElementsType Type => DrawElementsType.UnsignedInt;
        public readonly string DebugName;

        public IndexBuffer(uint[] indices, string debugName = "UNNAMED INDEX BUFFER")
        {
            GL.BindBuffer(BufferTarget.ElementArrayBuffer, buffer);
            GL.BufferStorage(BufferTarget.ElementArrayBuffer, sizeof(uint) * indices.Length, indices, default);
            GL.BindBuffer(BufferTarget.ElementArrayBuffer, 0);
            DebugName = debugName;
        }
        public void Bind() => GL.BindBuffer(BufferTarget.ElementArrayBuffer, buffer);
        protected override void DoDispose() => GL.DeleteBuffer(buffer);
    }
}
