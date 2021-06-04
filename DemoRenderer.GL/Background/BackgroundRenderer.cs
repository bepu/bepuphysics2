using BepuUtilities;
using DemoContentLoader;
using OpenTK.Graphics.OpenGL4;

namespace DemoRenderer.Background
{
    public class BackgroundRenderer : Shader
    {
        private readonly ConstantsBuffer<Matrix> constants;

        public BackgroundRenderer(ContentArchive content) : base(
            content.Load<GLSLContent>(@"Background\RenderBackground.glvs").Source,
            content.Load<GLSLContent>(@"Background\RenderBackground.glfs").Source
        ) =>
            constants = new ConstantsBuffer<Matrix>(BufferTarget.UniformBuffer, debugName: "BackgroundRenderer Constants");
        public void Render(Camera camera)
        {
            Use();
            constants.Bind(0);
            var constantsData = Matrix.Invert(camera.View * Matrix.CreatePerspectiveFieldOfView(camera.FieldOfView, camera.AspectRatio, camera.FarClip, camera.NearClip));
            constants.Update(ref constantsData);
            GL.DrawArrays(PrimitiveType.Triangles, 0, 3);
        }
        protected override void DoDispose()
        {
            constants.Dispose();
            base.DoDispose();
        }
    }
}
