using System;
using System.Diagnostics;
using OpenTK;
using BepuUtilities;
using OpenTK.Graphics;
using System.Threading;
using Vector2 = System.Numerics.Vector2;

namespace DemoUtilities
{
    public enum WindowMode
    {
        FullScreen,
        Windowed
    }

    /// <summary>
    /// Simple and not-very-general-purpose window management.
    /// </summary>
    public class Window : IDisposable
    {
        internal NativeWindow window;

        private bool resized;

        private WindowMode windowMode;
        WindowMode WindowMode
        {
            get
            {
                return windowMode;
            }
            set
            {
                switch (value)
                {
                    case WindowMode.FullScreen:
                        if (windowMode != WindowMode.FullScreen)
                        {
                            windowMode = value;
                            window.WindowState = WindowState.Fullscreen;
                            window.WindowBorder = WindowBorder.Hidden;
                            window.Location = new Point(0, 0);
                            var primaryBounds = DisplayDevice.Default.Bounds;
                            window.Size = new Size(primaryBounds.Width, primaryBounds.Height);
                            resized = true;
                        }
                        break;
                    case WindowMode.Windowed:
                        if (windowMode != WindowMode.Windowed)
                        {
                            windowMode = value;
                            window.WindowState = WindowState.Normal;
                            window.WindowBorder = WindowBorder.Resizable;
                            resized = true;
                        }
                        break;
                }
            }
        }

        /// <summary>
        /// Gets or sets the resolution of the window's body.
        /// </summary>
        public Int2 Resolution
        {
            get
            {
                return new Int2(window.ClientSize.Width, window.ClientSize.Height);
            }
            set
            {
                window.ClientSize = new Size(value.X, value.Y);
                resized = true;
            }
        }

        public IntPtr Handle { get { return window.WindowInfo.Handle; } }

        /// <summary>
        /// Gets whether the window is currently focused.
        /// </summary>
        public bool Focused { get { return window.Focused; } }


        /// <summary>
        /// Constructs a new rendering-capable window.
        /// </summary>
        /// <param name="title">Title of the window.</param>
        /// <param name="resolution">Initial size in pixels of the window's drawable surface.</param>
        /// <param name="windowMode">Initial window mode.</param>
        public Window(string title, Int2 resolution, WindowMode windowMode)
        {
            window = new NativeWindow(resolution.X, resolution.Y, title, GameWindowFlags.FixedWindow, GraphicsMode.Default, DisplayDevice.Default);
            Debug.Assert(window.ClientSize.Width == resolution.X);
            Debug.Assert(window.ClientSize.Height == resolution.Y);
            window.Visible = true;
            Resolution = resolution;
            window.Resize += (form, args) => resized = true;
            window.Closing += OnClosing;

            window.WindowBorder = WindowBorder.Resizable;

            WindowMode = windowMode;

        }

        public Vector2 GetNormalizedMousePosition(Int2 mousePosition)
        {
            return new Vector2((float)mousePosition.X / Resolution.X, (float)mousePosition.Y / Resolution.Y);
        }

        private void OnClosing(object sender, EventArgs e)
        {
            //This will redundantly call window.Close, but that's fine.
            tryToClose = true;
        }

        private bool windowUpdateLoopRunning;
        private bool tryToClose;
        /// <summary>
        /// Closes the window at the next available opportunity.
        /// </summary>
        public void Close()
        {
            if (windowUpdateLoopRunning)
                tryToClose = true;
            else
                window.Close();
        }


        /// <summary>
        /// Launches the update loop for the window. Processes events before every invocation of the update handler.
        /// </summary>
        /// <param name="updateHandler">Delegate to be invoked within the loop repeatedly.</param>
        public void Run(Action<float> updateHandler, Action<Int2> onResize)
        {
            long previousTime = Stopwatch.GetTimestamp();
            windowUpdateLoopRunning = true;
            while (true)
            {
                if (disposed)
                    break;
                if (resized)
                {
                    //Note that minimizing or resizing the window to invalid sizes don't result in actual resize attempts. Zero width rendering surfaces aren't allowed.
                    if (window.Width > 0 && window.Height > 0)
                    {
                        onResize(new Int2(window.Width, window.Height));
                    }
                    resized = false;
                }
                window.ProcessEvents();
                if (tryToClose)
                {
                    window.Close();
                    break;
                }
                long time = Stopwatch.GetTimestamp();
                var dt = (float)((time - previousTime) / (double)Stopwatch.Frequency);
                previousTime = time;

                if (window.WindowState != WindowState.Minimized)
                {
                    updateHandler(dt);
                }
                else
                {
                    //If the window is minimized, take a breather.
                    Thread.Sleep(1);
                }
            }
            windowUpdateLoopRunning = false;
        }

        private bool disposed;
        public void Dispose()
        {
            if (!disposed)
            {
                window.Dispose();
                disposed = true;
            }
        }

    }
}
