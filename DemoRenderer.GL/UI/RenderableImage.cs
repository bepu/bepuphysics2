using DemoContentLoader;
using System;
using System.Diagnostics;
using OpenTK.Graphics.OpenGL4;

namespace DemoRenderer.UI
{
    /// <summary>
    /// Runtime type containing GPU-related information necessary to render a specific font type.
    /// </summary>
    public class RenderableImage : Disposable
    {
        public readonly int Texture = GL.GenTexture();
        public readonly Texture2DContent Content;
        private readonly bool srgb;
        public readonly string DebugName;

        public RenderableImage(int width, int height, bool srgb = false, string debugName = null)
        {
            Content = new Texture2DContent(width, height, 1, 4);
            this.srgb = srgb;
            DebugName = debugName;
        }

        public RenderableImage(Texture2DContent imageContent, bool srgb = false, string debugName = null)
        {
            if (imageContent.TexelSizeInBytes != 4)
            {
                throw new ArgumentException("The renderable image assumes an R8G8B8A8_UNorm or  texture.");
            }
            Debug.Assert(imageContent.MipLevels == 1, "We ignore any mip levels stored in the content; if the content pipeline output them, something's likely mismatched.");
            Content = imageContent;
            this.srgb = srgb;
            DebugName = debugName;
            UploadContentToTexture();
        }

        /// <summary>
        /// Uploads the mip0 stored in the Content to the Texture2D and generates new mips.
        /// </summary>
        public unsafe void UploadContentToTexture()
        {
            GL.BindTexture(TextureTarget.Texture2D, Texture);
            var data = Content.Pin();
            GL.TexImage2D(TextureTarget.Texture2D, 0,
                srgb ? PixelInternalFormat.Rgba8Snorm : PixelInternalFormat.Rgba8,
                Content.Width, Content.Height, 0,
                PixelFormat.Rgba, PixelType.UnsignedByte,
                new IntPtr(data + Content.GetMipStartIndex(0))
            );
            Content.Unpin();
            GL.GenerateMipmap(GenerateMipmapTarget.Texture2D);
            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, (int)TextureMinFilter.LinearMipmapLinear);
            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, (int)TextureMagFilter.Linear);
            GL.BindTexture(TextureTarget.Texture2D, 0);
        }

        protected override void DoDispose() => GL.DeleteTexture(Texture);
    }
}
