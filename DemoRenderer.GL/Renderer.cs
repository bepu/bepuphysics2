using System;
using DemoContentLoader;
using DemoRenderer.Background;
using DemoRenderer.UI;
using DemoRenderer.PostProcessing;
using DemoRenderer.ShapeDrawing;
using DemoRenderer.Constraints;
using BepuUtilities.Memory;
using DemoUtilities;
using OpenTK.Graphics.OpenGL4;

namespace DemoRenderer
{
    public class Renderer : Disposable
    {
        public readonly RenderSurface Surface;
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
        public ImageRenderer ImageRenderer { get; private set; }
        public GlyphRenderer GlyphRenderer { get; private set; }
        public UILineRenderer UILineRenderer { get; private set; }
        public CompressToSwap CompressToSwap { get; private set; }

        public ImageBatcher ImageBatcher { get; private set; }
        public TextBatcher TextBatcher { get; private set; }
        public UILineBatcher UILineBatcher { get; private set; }

        private readonly ParallelLooper looper = new ParallelLooper();
        private readonly BufferPool pool = new BufferPool();

        private readonly int depthBuffer = GL.GenTexture();
        //Technically we could get away with rendering directly to the backbuffer, but a dedicated color buffer simplifies some things- 
        //you aren't bound by the requirements of the swapchain's buffer during rendering, and post processing is nicer.
        //Not entirely necessary for the demos, but hey, you could add tonemapping if you wanted?
        private readonly int colorBuffer = GL.GenTexture();
        private readonly int framebuffer = GL.GenFramebuffer();
        private readonly int resolvedColorBuffer = GL.GenTexture();
        private readonly int resolvedFramebuffer = GL.GenFramebuffer();
        private int width = 0;
        private int height = 0;

        public Renderer(RenderSurface surface)
        {
            Surface = surface;
            ContentArchive content;
            using (var stream = GetType().Assembly.GetManifestResourceStream("DemoRenderer.DemoRenderer.contentarchive"))
            {
                content = ContentArchive.Load(stream);
            }
            Shapes = new ShapesExtractor(looper, pool);
            SphereRenderer = new RayTracedRenderer<SphereInstance>(content, @"ShapeDrawing\RenderSpheres");
            CapsuleRenderer = new RayTracedRenderer<CapsuleInstance>(content, @"ShapeDrawing\RenderCapsules");
            CylinderRenderer = new RayTracedRenderer<CylinderInstance>(content, @"ShapeDrawing\RenderCylinders");
            BoxRenderer = new BoxRenderer(content);
            TriangleRenderer = new TriangleRenderer(content);
            MeshRenderer = new MeshRenderer(Shapes.MeshCache, content);
            Lines = new LineExtractor(pool, looper);
            LineRenderer = new LineRenderer(content);
            Background = new BackgroundRenderer(content);
            CompressToSwap = new CompressToSwap(content);

            ImageRenderer = new ImageRenderer(content);
            ImageBatcher = new ImageBatcher(pool);
            GlyphRenderer = new GlyphRenderer(content);
            TextBatcher = new TextBatcher();
            UILineRenderer = new UILineRenderer(content);
            UILineBatcher = new UILineBatcher();

            OnResize();
        }

        void OnResize()
        {
            var resolution = Surface.Resolution;
            width = resolution.X;
            height = resolution.Y;

            TextBatcher.Resolution = resolution;
            ImageBatcher.Resolution = resolution;
            UILineBatcher.Resolution = resolution;

            GL.BindFramebuffer(FramebufferTarget.Framebuffer, framebuffer);
            GL.BindTexture(TextureTarget.Texture2DMultisample, depthBuffer);
            GL.TexImage2DMultisample(TextureTargetMultisample.Texture2DMultisample, 4, PixelInternalFormat.DepthComponent, width, height, false);
            GL.FramebufferTexture2D(FramebufferTarget.Framebuffer, FramebufferAttachment.DepthAttachment, TextureTarget.Texture2DMultisample, depthBuffer, 0);
            //Using a 64 bit texture in the demos for lighting is pretty silly. But we gon do it.
            GL.BindTexture(TextureTarget.Texture2DMultisample, colorBuffer);
            GL.TexImage2DMultisample(TextureTargetMultisample.Texture2DMultisample, 4, PixelInternalFormat.Rgba16f, width, height, false);
            GL.FramebufferTexture2D(FramebufferTarget.Framebuffer, FramebufferAttachment.ColorAttachment0, TextureTarget.Texture2DMultisample, colorBuffer, 0);
            GL.BindTexture(TextureTarget.Texture2DMultisample, 0);

            GL.BindFramebuffer(FramebufferTarget.Framebuffer, resolvedFramebuffer);
            GL.BindTexture(TextureTarget.Texture2D, resolvedColorBuffer);
            GL.TexImage2D(TextureTarget.Texture2D, 0, PixelInternalFormat.Rgba16f, width, height, 0, PixelFormat.Rgba, PixelType.UnsignedByte, IntPtr.Zero);
            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, (int)TextureMinFilter.Nearest);
            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, (int)TextureMagFilter.Nearest);
            GL.FramebufferTexture2D(FramebufferTarget.Framebuffer, FramebufferAttachment.ColorAttachment0, TextureTarget.Texture2D, resolvedColorBuffer, 0);
            GL.BindTexture(TextureTarget.Texture2D, 0);

            GL.BindFramebuffer(FramebufferTarget.Framebuffer, 0);
        }

        public void Render(Camera camera)
        {
            if (Surface.Resolution.X != width || Surface.Resolution.Y != height)
            {
                OnResize();
            }
            Shapes.MeshCache.FlushPendingUploads();

            GL.BindFramebuffer(FramebufferTarget.Framebuffer, framebuffer);
            //Note reversed depth.
            GL.ClearDepth(0.0f);
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);
            GL.Enable(EnableCap.CullFace);
            GL.Enable(EnableCap.DepthTest);
            GL.DepthFunc(DepthFunction.Greater);

            //All ray traced shapes use analytic coverage writes to get antialiasing.
            GL.Enable(EnableCap.SampleAlphaToCoverage);
            SphereRenderer.Render(camera, Surface.Resolution, SpanConverter.AsSpan(Shapes.spheres.Span), 0, Shapes.spheres.Count);
            CapsuleRenderer.Render(camera, Surface.Resolution, SpanConverter.AsSpan(Shapes.capsules.Span), 0, Shapes.capsules.Count);
            CylinderRenderer.Render(camera, Surface.Resolution, SpanConverter.AsSpan(Shapes.cylinders.Span), 0, Shapes.cylinders.Count);

            //Non-raytraced shapes just use regular opaque rendering.
            GL.Disable(EnableCap.SampleAlphaToCoverage);
            BoxRenderer.Render(camera, Surface.Resolution, SpanConverter.AsSpan(Shapes.boxes.Span), 0, Shapes.boxes.Count);
            TriangleRenderer.Render(camera, Surface.Resolution, SpanConverter.AsSpan(Shapes.triangles.Span), 0, Shapes.triangles.Count);
            MeshRenderer.Render(camera, Surface.Resolution, SpanConverter.AsSpan(Shapes.meshes.Span), 0, Shapes.meshes.Count);
            LineRenderer.Render(camera, Surface.Resolution, SpanConverter.AsSpan(Lines.lines.Span), 0, Lines.lines.Count);

            Background.Render(camera);
            GL.Disable(EnableCap.CullFace);
            GL.Disable(EnableCap.DepthTest);

            //Resolve MSAA rendering down to a single sample buffer for screenspace work.
            //Note that we're not bothering to properly handle tonemapping during the resolve. That's going to hurt quality a little, but the demos don't make use of very wide ranges.
            //(If for some reason you end up expanding the demos to make use of wider HDR, you can make this a custom resolve pretty easily.)
            GL.BlitNamedFramebuffer(framebuffer, resolvedFramebuffer, 0, 0, width, height, 0, 0, width, height, ClearBufferMask.ColorBufferBit, BlitFramebufferFilter.Nearest);
            GL.BindFramebuffer(FramebufferTarget.Framebuffer, resolvedFramebuffer);

            //Glyph and screenspace line drawing rely on the same premultiplied alpha blending transparency. We'll handle their state out here.
            GL.Enable(EnableCap.Blend);
            GL.BlendFunc(BlendingFactorSrc.One, BlendingFactorDest.OneMinusSrcAlpha);
            GL.BlendEquation(BlendEquationMode.FuncAdd);
            ImageRenderer.PreparePipeline();
            ImageBatcher.Flush(Surface.Resolution, ImageRenderer);
            UILineBatcher.Flush(Surface.Resolution, UILineRenderer);
            GlyphRenderer.PreparePipeline();
            TextBatcher.Flush(Surface.Resolution, GlyphRenderer);
            GL.Disable(EnableCap.Blend);

            GL.BindFramebuffer(FramebufferTarget.Framebuffer, 0);
            CompressToSwap.Render(resolvedColorBuffer);
        }

        protected override void DoDispose()
        {
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

            GL.DeleteFramebuffer(framebuffer);
            GL.DeleteTexture(depthBuffer);
            GL.DeleteTexture(colorBuffer);
            GL.DeleteFramebuffer(resolvedFramebuffer);
            GL.DeleteTexture(resolvedColorBuffer);

            Shapes.Dispose();
        }
    }
}
