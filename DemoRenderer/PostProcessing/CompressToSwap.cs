using DemoContentLoader;
using DemoRenderer.Attributes;
using SharpDX.Direct3D11;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;

namespace DemoRenderer.PostProcessing
{
    /// <summary>
    /// Applies a gamma curve, anti-banding dithering, and outputs the result into the lower precision target.
    /// </summary>
    public class CompressToSwap : IDisposable
    {
        /// <summary>
        /// Gets or sets the display gamma. This isn't SRGB, but it'll do.
        /// </summary>
        public float Gamma { get; set; }
#pragma warning disable 0649
        ConstantsBuffer<float> constants; //alas, lack of root constants
        [Resource(@"PostProcessing\CompressToSwap.hlsl.vshader")]
        VertexShader vertexShader;
        [Resource(@"PostProcessing\CompressToSwap.hlsl.pshader")]
        PixelShader pixelShader;
#pragma warning restore 0649


        //At the moment, this is the only form of post processing in the pipeline. We'll isolate the state changes needed in here rather than outside.
        DepthStencilState depthState;

        public CompressToSwap(Device device, float gamma = 2.2f)
        {
            Gamma = gamma;
            var depthStateDescription = DepthStencilStateDescription.Default();
            depthStateDescription.DepthWriteMask = DepthWriteMask.Zero;
            depthStateDescription.IsDepthEnabled = false;
            depthState = new DepthStencilState(device, depthStateDescription);
        }


        public void Render(DeviceContext context, ShaderResourceView source, RenderTargetView target)
        {
            float inverseGamma = 1f / Gamma;
            constants.Update(context, ref inverseGamma);

            context.OutputMerger.SetRenderTargets(null, target);
            context.OutputMerger.SetDepthStencilState(depthState);
            context.VertexShader.Set(vertexShader);
            context.PixelShader.Set(pixelShader);
            context.PixelShader.SetConstantBuffer(0, constants.Buffer);
            context.PixelShader.SetShaderResource(0, source);
            context.Draw(3, 0);
            context.PixelShader.SetShaderResource(0, null); //Unhook the SRV to allow the underlying resource to be used as an RTV next time around.
        }

        bool disposed;
        public void Dispose()
        {
            if (!disposed)
            {
                disposed = true;
                constants.Dispose();
                vertexShader.Dispose();
                pixelShader.Dispose();
                depthState.Dispose();
            }
        }

#if DEBUG
        ~CompressToSwap()
        {
            Helpers.CheckForUndisposed(disposed, this);
        }
#endif
    }
}
