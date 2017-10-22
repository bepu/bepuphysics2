using System;
using System.Diagnostics;
using SharpDX.DXGI;
using Device = SharpDX.Direct3D11.Device;
using Resource = SharpDX.Direct3D11.Resource;
using SharpDX.Direct3D11;
using SharpDX.Direct3D;
using BepuUtilities;

namespace DemoRenderer
{
    /// <summary>
    /// The swap chain, drawing surfaces, and the device contexts associated with a window.
    /// </summary>
    public class RenderSurface : IDisposable
    {
        public Device Device { get; private set; }
        public DeviceContext Context { get; private set; }

        private SwapChain swapChain;

        private Texture2D drawingSurfaceBuffer;
        private RenderTargetView drawingSurfaceRTV;
        /// <summary>
        /// Gets the render target view for the backbuffer of the window.
        /// </summary>
        public RenderTargetView RTV
        {
            get { return drawingSurfaceRTV; }
        }

        /// <summary>
        /// Gets the current resolution of the render surface. To change the resolution, use Resize.
        /// </summary>
        public Int2 Resolution { get; private set; }

        /// <summary>
        /// Constructs a new swap surface.
        /// </summary> 
        /// <param name="windowHandle">Handle of the window to build a swap chain and drawing surface for.</param>
        /// <param name="resolution">Resolution of the rendering surface.</param>
        /// <param name="enableDeviceDebugLayer">Whether to use the debug layer for this window's graphics device.</param>
        public RenderSurface(IntPtr windowHandle, Int2 resolution, bool fullScreen = false, bool enableDeviceDebugLayer = false)
        {
            var swapChainDescription = new SwapChainDescription
            {
                BufferCount = 2,
                ModeDescription =
                    new ModeDescription(resolution.X, resolution.Y, new Rational(), Format.R8G8B8A8_UNorm),
                IsWindowed = !fullScreen,
                OutputHandle = windowHandle,
                SampleDescription = new SampleDescription(1, 0),
                SwapEffect = SwapEffect.FlipDiscard,
                Usage = Usage.RenderTargetOutput
            };

            Device.CreateWithSwapChain(DriverType.Hardware, enableDeviceDebugLayer ? DeviceCreationFlags.Debug : DeviceCreationFlags.None, swapChainDescription,
                out Device device, out swapChain);
            var context = device.ImmediateContext;

            Device = device;
            Context = context;

            Resolution = resolution;

        }

        public void Resize(Int2 resolution, bool fullScreen)
        {
            Helpers.Dispose(ref drawingSurfaceRTV);
            Helpers.Dispose(ref drawingSurfaceBuffer);
            swapChain.ResizeBuffers(2, resolution.X, resolution.Y, Format.Unknown, SwapChainFlags.None);
            swapChain.IsFullScreen = fullScreen;

            // Get the backbuffer from the swapchain.
            drawingSurfaceBuffer = Resource.FromSwapChain<Texture2D>(swapChain, 0);
            drawingSurfaceBuffer.DebugName = "Window Backbuffer";
            drawingSurfaceRTV = new RenderTargetView(Device, drawingSurfaceBuffer) { DebugName = "Window Backbuffer RTV" };

            Resolution = resolution;
        }


        public void Present()
        {
            swapChain.Present(1, PresentFlags.None);
        }

        private bool disposed;
        public void Dispose()
        {
            if (!disposed)
            {
                drawingSurfaceRTV.Dispose();
                drawingSurfaceBuffer.Dispose();
                Device.Dispose();
                Context.Dispose();
                swapChain.Dispose();

                disposed = true;
            }
        }

#if DEBUG
        ~RenderSurface()
        {
            Helpers.CheckForUndisposed(disposed, this);
        }
#endif
    }
}
