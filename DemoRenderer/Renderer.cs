using DemoContentLoader;
using DemoRenderer.Background;
using DemoRenderer.UI;
using DemoRenderer.PostProcessing;
using SharpDX.Direct3D11;
using SharpDX.DXGI;
using System;
using DemoRenderer.ShapeDrawing;
using DemoRenderer.Constraints;
using BepuUtilities.Memory;
using DemoUtilities;

namespace DemoRenderer
{
    public class Renderer : IDisposable
    {
        public RenderSurface Surface { get; private set; }
        public ShaderCache ShaderCache { get; private set; }
        public BackgroundRenderer Background { get; private set; }
        //TODO: Down the road, the sphere renderer will be joined by a bunch of other types. 
        //They'll likely be stored in an array indexed by a shape type rather than just being a swarm of properties.
        public RayTracedRenderer<SphereInstance> SphereRenderer { get; private set; }
        public RayTracedRenderer<CapsuleInstance> CapsuleRenderer { get; private set; }
        public RayTracedRenderer<CylinderInstance> CylinderRenderer { get; private set; }
        public BoxRenderer BoxRenderer { get; private set; }
        public TriangleRenderer TriangleRenderer { get; private set; }
        public MeshRenderer MeshRenderer { get; private set; }
        public ShapesExtractor Shapes { get; private set; }
        public LineRenderer LineRenderer { get; private set; }
        public LineExtractor Lines { get; private set; }
        public GlyphRenderer GlyphRenderer { get; private set; }
        public UILineRenderer UILineRenderer { get; private set; }
        public CompressToSwap CompressToSwap { get; private set; }

        public TextBatcher TextBatcher { get; private set; }
        public UILineBatcher UILineBatcher { get; private set; }


        ParallelLooper looper;
        BufferPool pool;

        Texture2D depthBuffer;
        DepthStencilView dsv;
        //Technically we could get away with rendering directly to the backbuffer, but a dedicated color buffer simplifies some things- 
        //you aren't bound by the requirements of the swapchain's buffer during rendering, and post processing is nicer.
        //Not entirely necessary for the demos, but hey, you could add tonemapping if you wanted?
        Texture2D colorBuffer;
        RenderTargetView rtv;
        Texture2D resolvedColorBuffer;
        ShaderResourceView resolvedSRV;
        RenderTargetView resolvedRTV;

        RasterizerState rasterizerState;
        DepthStencilState opaqueDepthState;
        BlendState opaqueBlendState;
        BlendState a2cBlendState;
        DepthStencilState uiDepthState;
        BlendState uiBlendState;


        public Renderer(RenderSurface surface)
        {
            looper = new ParallelLooper();
            Surface = surface;
            using (var stream = GetType().Assembly.GetManifestResourceStream("DemoRenderer.DemoRenderer.shaderarchive"))
            {
                ShaderCache = ShaderCache.Load(stream);
            }
            pool = new BufferPool();
            Shapes = new ShapesExtractor(Surface.Device, looper, pool);
            SphereRenderer = new RayTracedRenderer<SphereInstance>(surface.Device, ShaderCache, @"ShapeDrawing\RenderSpheres.hlsl");
            CapsuleRenderer = new RayTracedRenderer<CapsuleInstance>(surface.Device, ShaderCache, @"ShapeDrawing\RenderCapsules.hlsl");
            CylinderRenderer = new RayTracedRenderer<CylinderInstance>(surface.Device, ShaderCache, @"ShapeDrawing\RenderCylinders.hlsl");
            BoxRenderer = new BoxRenderer(surface.Device, ShaderCache);
            TriangleRenderer = new TriangleRenderer(surface.Device, ShaderCache);
            MeshRenderer = new MeshRenderer(surface.Device, Shapes.MeshCache, ShaderCache);
            Lines = new LineExtractor(pool, looper);
            LineRenderer = new LineRenderer(surface.Device, ShaderCache);
            Background = new BackgroundRenderer(surface.Device, ShaderCache);
            CompressToSwap = new CompressToSwap(surface.Device, ShaderCache);

            GlyphRenderer = new GlyphRenderer(surface.Device, surface.Context, ShaderCache);
            TextBatcher = new TextBatcher();
            UILineRenderer = new UILineRenderer(surface.Device, ShaderCache);
            UILineBatcher = new UILineBatcher();

            OnResize();
            var rasterizerStateDescription = RasterizerStateDescription.Default();
            rasterizerStateDescription.IsFrontCounterClockwise = true;
            rasterizerStateDescription.CullMode = CullMode.Back;
            rasterizerState = new RasterizerState(Surface.Device, rasterizerStateDescription);
            rasterizerState.DebugName = "Default Rasterizer State";

            var opaqueDepthStencilDescription = new DepthStencilStateDescription
            {
                IsDepthEnabled = true,
                DepthWriteMask = DepthWriteMask.All,
                //Note depth reversal.
                DepthComparison = Comparison.Greater,
                IsStencilEnabled = false
            };

            opaqueDepthState = new DepthStencilState(Surface.Device, opaqueDepthStencilDescription);
            opaqueDepthState.DebugName = "Opaque Depth State";

            var opaqueBlendStateDescription = BlendStateDescription.Default();
            opaqueBlendState = new BlendState(Surface.Device, opaqueBlendStateDescription);
            opaqueBlendState.DebugName = "Opaque Blend State";

            var a2cBlendStateDescription = BlendStateDescription.Default();
            a2cBlendStateDescription.AlphaToCoverageEnable = true;
            a2cBlendState = new BlendState(Surface.Device, a2cBlendStateDescription);
            a2cBlendState.DebugName = "A2C Blend State";

            var uiDepthStateDescription = new DepthStencilStateDescription
            {
                IsDepthEnabled = false,
                DepthWriteMask = DepthWriteMask.Zero,
                //Note depth reversal.
                DepthComparison = Comparison.Greater,
                IsStencilEnabled = false
            };

            uiDepthState = new DepthStencilState(Surface.Device, uiDepthStateDescription);
            uiDepthState.DebugName = "UI Depth State";

            //The UI will use premultiplied alpha.
            var uiBlendStateDescription = BlendStateDescription.Default();
            uiBlendStateDescription.RenderTarget[0].IsBlendEnabled = true;
            uiBlendStateDescription.RenderTarget[0].SourceBlend = BlendOption.One;
            uiBlendStateDescription.RenderTarget[0].SourceAlphaBlend = BlendOption.One;
            uiBlendStateDescription.RenderTarget[0].DestinationBlend = BlendOption.InverseSourceAlpha;
            uiBlendStateDescription.RenderTarget[0].DestinationAlphaBlend = BlendOption.InverseSourceAlpha;
            uiBlendStateDescription.RenderTarget[0].BlendOperation = BlendOperation.Add;
            uiBlendStateDescription.RenderTarget[0].AlphaBlendOperation = BlendOperation.Add;
            uiBlendStateDescription.RenderTarget[0].RenderTargetWriteMask = ColorWriteMaskFlags.All;
            uiBlendState = new BlendState(Surface.Device, uiBlendStateDescription);
            uiBlendState.DebugName = "UI Blend State";
        }

        void OnResize()
        {
            Helpers.Dispose(ref depthBuffer);
            Helpers.Dispose(ref dsv);
            Helpers.Dispose(ref colorBuffer);
            Helpers.Dispose(ref rtv);

            var resolution = Surface.Resolution;

            TextBatcher.Resolution = resolution;
            UILineBatcher.Resolution = resolution;

            var sampleDescription = new SampleDescription(4, 0);
            depthBuffer = new Texture2D(Surface.Device, new Texture2DDescription
            {
                Format = Format.R32_Typeless,
                ArraySize = 1,
                MipLevels = 1,
                Width = resolution.X,
                Height = resolution.Y,
                SampleDescription = sampleDescription,
                Usage = ResourceUsage.Default,
                BindFlags = BindFlags.DepthStencil,
                CpuAccessFlags = CpuAccessFlags.None,
                OptionFlags = ResourceOptionFlags.None
            });
            depthBuffer.DebugName = "Depth Buffer";

            var depthStencilViewDescription = new DepthStencilViewDescription
            {
                Flags = DepthStencilViewFlags.None,
                Dimension = DepthStencilViewDimension.Texture2DMultisampled,
                Format = Format.D32_Float,
                Texture2D = { MipSlice = 0 }
            };
            dsv = new DepthStencilView(Surface.Device, depthBuffer, depthStencilViewDescription);
            dsv.DebugName = "Depth DSV";

            //Using a 64 bit texture in the demos for lighting is pretty silly. But we gon do it.
            var description = new Texture2DDescription
            {
                Format = Format.R16G16B16A16_Float,
                ArraySize = 1,
                MipLevels = 1,
                Width = resolution.X,
                Height = resolution.Y,
                SampleDescription = sampleDescription,
                Usage = ResourceUsage.Default,
                BindFlags = BindFlags.RenderTarget | BindFlags.ShaderResource,
                CpuAccessFlags = CpuAccessFlags.None,
                OptionFlags = ResourceOptionFlags.None
            };
            colorBuffer = new Texture2D(Surface.Device, description);
            colorBuffer.DebugName = "Color Buffer";

            rtv = new RenderTargetView(Surface.Device, colorBuffer);
            rtv.DebugName = "Color RTV";
            
            description.SampleDescription = new SampleDescription(1, 0);
            resolvedColorBuffer = new Texture2D(Surface.Device, description);
            resolvedColorBuffer.DebugName = "Resolved Color Buffer";

            resolvedSRV = new ShaderResourceView(Surface.Device, resolvedColorBuffer);
            resolvedSRV.DebugName = "Resolved Color SRV";

            resolvedRTV = new RenderTargetView(Surface.Device, resolvedColorBuffer);
            resolvedRTV.DebugName = "Resolved Color RTV";

        }

        public void Render(Camera camera)
        {
            if (Surface.Resolution.X != depthBuffer.Description.Width || Surface.Resolution.Y != depthBuffer.Description.Height)
            {
                OnResize();
            }
            var context = Surface.Context;
            Shapes.MeshCache.FlushPendingUploads(context);

            context.Rasterizer.SetViewport(0, 0, Surface.Resolution.X, Surface.Resolution.Y, 0.0f, 1.0f);

            //Note reversed depth.
            context.ClearDepthStencilView(dsv, DepthStencilClearFlags.Depth, 0, 0);
            context.ClearRenderTargetView(rtv, new SharpDX.Mathematics.Interop.RawColor4());
            context.OutputMerger.SetRenderTargets(dsv, rtv);
            context.Rasterizer.State = rasterizerState;
            context.OutputMerger.SetDepthStencilState(opaqueDepthState);

            //All ray traced shapes use analytic coverage writes to get antialiasing.
            context.OutputMerger.SetBlendState(a2cBlendState);
            SphereRenderer.Render(context, camera, Surface.Resolution, SpanConverter.AsSpan(Shapes.spheres.Span), 0, Shapes.spheres.Count);
            CapsuleRenderer.Render(context, camera, Surface.Resolution, SpanConverter.AsSpan(Shapes.capsules.Span), 0, Shapes.capsules.Count);
            CylinderRenderer.Render(context, camera, Surface.Resolution, SpanConverter.AsSpan(Shapes.cylinders.Span), 0, Shapes.cylinders.Count);

            //Non-raytraced shapes just use regular opaque rendering.
            context.OutputMerger.SetBlendState(opaqueBlendState);
            BoxRenderer.Render(context, camera, Surface.Resolution, SpanConverter.AsSpan(Shapes.boxes.Span), 0, Shapes.boxes.Count);
            TriangleRenderer.Render(context, camera, Surface.Resolution, SpanConverter.AsSpan(Shapes.triangles.Span), 0, Shapes.triangles.Count);
            MeshRenderer.Render(context, camera, Surface.Resolution, SpanConverter.AsSpan(Shapes.meshes.Span), 0, Shapes.meshes.Count);
            LineRenderer.Render(context, camera, Surface.Resolution, SpanConverter.AsSpan(Lines.lines.Span), 0, Lines.lines.Count);

            Background.Render(context, camera);

            //Resolve MSAA rendering down to a single sample buffer for screenspace work.
            //Note that we're not bothering to properly handle tonemapping during the resolve. That's going to hurt quality a little, but the demos don't make use of very wide ranges.
            //(If for some reason you end up expanding the demos to make use of wider HDR, you can make this a custom resolve pretty easily.)
            context.ResolveSubresource(colorBuffer, 0, resolvedColorBuffer, 0, Format.R16G16B16A16_Float);
            context.OutputMerger.SetRenderTargets(resolvedRTV);

            //Glyph and screenspace line drawing rely on the same premultiplied alpha blending transparency. We'll handle their state out here.
            context.OutputMerger.SetBlendState(uiBlendState);
            context.OutputMerger.SetDepthStencilState(uiDepthState);
            UILineBatcher.Flush(context, Surface.Resolution, UILineRenderer);
            GlyphRenderer.PreparePipeline(context);
            TextBatcher.Flush(context, Surface.Resolution, GlyphRenderer);

            //Note that, for now, the compress to swap handles its own depth state since it's the only post processing stage.
            context.OutputMerger.SetBlendState(opaqueBlendState);
            context.Rasterizer.State = rasterizerState;
            CompressToSwap.Render(context, resolvedSRV, Surface.RTV);
        }

        bool disposed;
        public void Dispose()
        {
            if (!disposed)
            {
                disposed = true;
                Background.Dispose();
                CompressToSwap.Dispose();

                Lines.Dispose();

                SphereRenderer.Dispose();
                CapsuleRenderer.Dispose();
                CylinderRenderer.Dispose();
                BoxRenderer.Dispose();
                TriangleRenderer.Dispose();
                MeshRenderer.Dispose();

                UILineRenderer.Dispose();
                GlyphRenderer.Dispose();

                dsv.Dispose();
                depthBuffer.Dispose();
                rtv.Dispose();
                colorBuffer.Dispose();
                resolvedSRV.Dispose();
                resolvedRTV.Dispose();
                resolvedColorBuffer.Dispose();

                rasterizerState.Dispose();
                opaqueDepthState.Dispose();
                opaqueBlendState.Dispose();
                a2cBlendState.Dispose();
                uiDepthState.Dispose();
                uiBlendState.Dispose();

                Shapes.Dispose();
            }
        }

#if DEBUG
        ~Renderer()
        {
            Helpers.CheckForUndisposed(disposed, this);
        }
#endif
    }
}
