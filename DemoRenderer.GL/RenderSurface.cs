using System;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL4;
using OpenTK.Platform;
using BepuUtilities;

namespace DemoRenderer
{
    /// <summary>
    /// The swap chain, drawing surfaces, and the device contexts associated with a window.
    /// </summary>
    public class RenderSurface : Disposable
    {
        private readonly IWindowInfo window;
        private readonly IGraphicsContext context;

        /// <summary>
        /// Gets the current resolution of the render surface. To change the resolution, use Resize.
        /// </summary>
        public Int2 Resolution { get; private set; }

        /// <summary>
        /// Constructs a new swap surface.
        /// </summary> 
        /// <param name="window">Window to build a swap chain and drawing surface for.</param>
        /// <param name="resolution">Resolution of the rendering surface.</param>
        /// <param name="enableDeviceDebugLayer">Whether to use the debug layer for this window's graphics device.</param>
        public RenderSurface(IWindowInfo window, Int2 resolution, bool fullScreen = false, bool enableDeviceDebugLayer = false)
        {
            this.window = window;
            context = new GraphicsContext(new GraphicsMode(new ColorFormat(8, 8, 8, 8), 24, 0, 4), window, 4, 6, GraphicsContextFlags.Default);
            context.LoadAll();
            if (enableDeviceDebugLayer)
            {
                GL.Enable(EnableCap.DebugOutput);
                GL.DebugMessageCallback((source, type, id, severity, length, message, userParam) =>
                {
                    Console.Error.WriteLine($"{source}, {type}, {id}, {severity}, {System.Runtime.InteropServices.Marshal.PtrToStringAnsi(message)}");
                    if (type == DebugType.DebugTypeError) throw new Exception();
                }, IntPtr.Zero);
            }
            Resolution = resolution;
            GL.Viewport(0, 0, Resolution.X, Resolution.Y);
        }

        public void Resize(Int2 resolution, bool fullScreen)
        {
            Resolution = resolution;
            context.MakeCurrent(window);
            GL.Viewport(0, 0, Resolution.X, Resolution.Y);
        }

        public void Present() => context.SwapBuffers();

        protected override void DoDispose() => context.Dispose();
    }
}
