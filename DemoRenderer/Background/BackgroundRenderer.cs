using BepuUtilities;
using DemoContentLoader;
using DemoRenderer.Attributes;
using SharpDX.Direct3D11;
using System;
using System.Numerics;

namespace DemoRenderer.Background
{
    public class BackgroundRenderer : IDisposable
    {
#pragma warning disable 0649
        [Resource(@"Background\RenderBackground.hlsl.vshader")]
        VertexShader vertexShader;
        [Resource(@"Background\RenderBackground.hlsl.pshader")]
        PixelShader pixelShader;
        ConstantsBuffer<Matrix> constants;
#pragma warning restore 0649
        public BackgroundRenderer()
        {
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
