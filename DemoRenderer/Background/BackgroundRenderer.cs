using BepuUtilities;
using DemoContentLoader;
using SharpDX.Direct3D11;
using System;
using System.Numerics;

namespace DemoRenderer.Background
{
    public class BackgroundRenderer : IDisposable
    {
        VertexShader vertexShader;
        PixelShader pixelShader;
        ConstantsBuffer<Matrix> constants;
        public BackgroundRenderer(Device device, ShaderCache cache)
        {
            vertexShader = new VertexShader(device, cache.GetShader(@"Background\RenderBackground.hlsl.vshader"));
            vertexShader.DebugName = "BackgroundVS";
            pixelShader = new PixelShader(device, cache.GetShader(@"Background\RenderBackground.hlsl.pshader"));
            pixelShader.DebugName = "BackgroundPS";
            constants = new ConstantsBuffer<Matrix>(device, debugName: "BackgroundRenderer Constants");
        }


        public void Render(DeviceContext context, Camera camera)
        {
            var constantsData = Matrix.Transpose(Matrix.Invert(camera.ViewProjection)); //Compensate for the shader packing we use.
            constants.Update(context, ref constantsData);

            context.InputAssembler.InputLayout = null;
            context.InputAssembler.PrimitiveTopology = SharpDX.Direct3D.PrimitiveTopology.TriangleList;
            context.VertexShader.Set(vertexShader);
            context.VertexShader.SetConstantBuffer(0, constants.Buffer);
            context.PixelShader.Set(pixelShader);
            context.Draw(3, 0);
        }

        bool disposed;
        public void Dispose()
        {
            if (!disposed)
            {
                disposed = true;
                vertexShader.Dispose();
                pixelShader.Dispose();
                constants.Dispose();
            }
        }
    }
}
