using DemoContentLoader;
using OpenTK.Graphics.OpenGL4;

namespace DemoRenderer.PostProcessing
{
    /// <summary>
    /// Applies a gamma curve, anti-banding dithering, and outputs the result into the lower precision target.
    /// </summary>
    public class CompressToSwap : Shader
    {
        /// <summary>
        /// Gets or sets the display gamma. This isn't SRGB, but it'll do.
        /// </summary>
        public float Gamma;
        private readonly ConstantsBuffer<float> constants; //alas, lack of root constants

        public CompressToSwap(ContentArchive content, float gamma = 2.2f) : base(
            content.Load<GLSLContent>(@"PostProcessing\CompressToSwap.glvs").Source,
            content.Load<GLSLContent>(@"PostProcessing\CompressToSwap.glfs").Source
        )
        {
            Gamma = gamma;
            constants = new ConstantsBuffer<float>(BufferTarget.UniformBuffer, debugName: "CompressToSwap Constants");
        }

        public void Render(int source)
        {
            Use();
            constants.Bind(0);
            GL.BindTexture(TextureTarget.Texture2D, source);
            float inverseGamma = 1f / Gamma;
            constants.Update(ref inverseGamma);
            GL.DrawArrays(PrimitiveType.Triangles, 0, 3);
        }

        protected override void DoDispose()
        {
            constants.Dispose();
            base.DoDispose();
        }
    }
}
