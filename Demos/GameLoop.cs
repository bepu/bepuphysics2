using DemoRenderer;
using DemoUtilities;
using System;
using System.Collections.Generic;
using System.Text;
using BepuUtilities;
using OpenTK;

namespace Demos
{
    public class GameLoop : IDisposable
    {
        public Window Window { get; private set; }
        public Input Input { get; private set; }
        public Camera Camera { get; private set; }
        public RenderSurface Surface { get; private set; }
        public Renderer Renderer { get; private set; }
        public DemoHarness DemoHarness { get; set; }

        public GameLoop(Window window)
        {
            Window = window;
            Input = new Input(window);
            var useDebugLayer =
#if DEBUG
                true;
#else
                false;
#endif
            Surface = new RenderSurface(window.Handle, window.Resolution, enableDeviceDebugLayer: useDebugLayer);
            Renderer = new Renderer(Surface);
            Camera = new Camera(window.Resolution.X / (float)window.Resolution.Y, (float)Math.PI / 2, 0.01f, 100000);            
        }

        void Update(float dt)
        {
            Input.Start();
            if (DemoHarness != null)
            {
                //We'll let the delgate's logic handle the variable time steps.
                DemoHarness.Update(dt);
                //At the moment, rendering just follows sequentially. Later on we might want to distinguish it a bit more with fixed time stepping or something. Maybe.
                DemoHarness.Render(Renderer);
            }
            Renderer.Render(Camera);
            Surface.Present();
            Input.End();
        }

        public void Run(DemoHarness harness)
        {
            DemoHarness = harness;
            Window.Run(Update, OnResize);
        }

        private void OnResize(Int2 resolution)
        {
            //We just don't support true fullscreen in the demos. Would be pretty pointless.
            Renderer.Surface.Resize(resolution, false);
            Camera.AspectRatio = resolution.X / (float)resolution.Y;
            DemoHarness?.OnResize(resolution);
        }

        bool disposed;
        public void Dispose()
        {
            if (!disposed)
            {
                disposed = true;
                Input.Dispose();
                Renderer.Dispose();
                //Note that we do not own the window.
            }
        }
    }
}
