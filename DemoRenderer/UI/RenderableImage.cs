using DemoContentLoader;
using SharpDX;
using SharpDX.Direct3D11;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Text;

namespace DemoRenderer.UI
{
    /// <summary>
    /// Convenience type bundling a GPU resident texture, its view, and a CPU side buffer for uploading to the GPU.
    /// </summary>
    public class RenderableImage : IDisposable
    {
        public Texture2D Texture { get; private set; }
        public ShaderResourceView SRV { get; private set; }

        public Texture2DContent Content { get; private set; }

        void Initialize(Device device, int width, int height, bool srgb = false, string debugName = null)
        {
            Texture = new Texture2D(device, new Texture2DDescription
            {
                ArraySize = 1,
                BindFlags = BindFlags.ShaderResource | BindFlags.RenderTarget,
                CpuAccessFlags = CpuAccessFlags.None,
                Format = srgb ? SharpDX.DXGI.Format.R8G8B8A8_UNorm_SRgb : SharpDX.DXGI.Format.R8G8B8A8_UNorm,
                Height = height,
                Width = width,
                MipLevels = (int)MathF.Floor(MathF.Log(MathF.Min(width, height), 2)) + 1,
                OptionFlags = ResourceOptionFlags.GenerateMipMaps,
                SampleDescription = new SharpDX.DXGI.SampleDescription(1, 0),
                Usage = ResourceUsage.Default
            });
            Texture.DebugName = debugName;
            SRV = new ShaderResourceView(device, Texture);
            SRV.DebugName = debugName + " SRV";
        }


        public unsafe RenderableImage(Device device, int width, int height, bool srgb = false, string debugName = null)
        {
            Initialize(device, width, height, srgb, debugName);
            Content = new Texture2DContent(width, height, 1, 4);
        }

        public unsafe RenderableImage(Device device, DeviceContext context, Texture2DContent imageContent, bool srgb = false, string debugName = null)
        {
            if (imageContent.TexelSizeInBytes != 4)
            {
                throw new ArgumentException("The renderable image assumes an R8G8B8A8_UNorm or  texture.");
            }
            Debug.Assert(imageContent.MipLevels == 1, "We ignore any mip levels stored in the content; if the content pipeline output them, something's likely mismatched.");
            Initialize(device, imageContent.Width, imageContent.Height, srgb, debugName);
            Content = imageContent;
            UploadContentToTexture(context);
        }

        /// <summary>
        /// Uploads the mip0 stored in the Content to the Texture2D and generates new mips.
        /// </summary>
        /// <param name="context">Context to use to update the texture.</param>
        public unsafe void UploadContentToTexture(DeviceContext context)
        {
            var data = Content.Pin();
            var databox = new DataBox(new IntPtr(data + Content.GetMipStartIndex(0)), Content.GetRowPitch(0) * 4, 0);
            context.UpdateSubresource(databox, Texture, 0);
            Content.Unpin();
            context.GenerateMips(SRV);
        }


        bool disposed;
        public void Dispose()
        {
            if (!disposed)
            {
                disposed = true;
                Texture.Dispose();
                SRV.Dispose();
            }
        }

#if DEBUG
        ~RenderableImage()
        {
            Helpers.CheckForUndisposed(disposed, this);
        }
#endif
    }
}
