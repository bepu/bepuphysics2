using System;
using System.Diagnostics;
using OpenTK;
using BepuUtilities;
using OpenTK.Graphics;
using OpenTK.Platform;
using System.Threading;
using Vector2 = System.Numerics.Vector2;
using System.Configuration;
using System.Runtime.Intrinsics.X86;
using System.Runtime.InteropServices;


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
        public IWindowInfo WindowInfo => window.WindowInfo;

        /// <summary>
        /// Gets whether the window is currently focused.
        /// </summary>
        public bool Focused { get { return window.Focused; } }

            //TODO TEST THIS
        [DllImport("Shcore.dll")] static extern void SetProcessDpiAwareness(PROCESS_DPI_AWARENESS awareness);  //  //This code contains a dll import statement for the SetProcessDpiAwareness function from Shcore.dll. Please note that you will need to import the namespace for PROCESS_DPI_AWARENESS in order to use this code properly.

        enum PROCESS_DPI_AWARENESS : uint
        {
            Process_DPI_Unaware = 0,
            Process_System_DPI_Aware = 1,
            Process_Per_Monitor_DPI_Aware = 2
        }


        /// <summary>
        /// Constructs a new rendering-capable window.
        /// </summary>
        /// <param name="title">Title of the window.</param>
        /// <param name="resolution">Initial size in pixels of the window's drawable surface.</param>
        /// <param name="location">Initial location of the window's drawable surface.</param>
        /// <param name="windowMode">Initial window mode.</param>
        public Window(string title, Int2 resolution, Int2 location, WindowMode windowMode)
        {

        window=new NativeWindow(location.X, location.Y, resolution.X, resolution.Y, title, GameWindowFlags.FixedWindow, GraphicsMode.Default, DisplayDevice.Default);
     
            window.Visible=true;
            Resolution=resolution;


            //save the size to the user settings so that the window will be the same size next time the program is run
            window.Resize+=(form, args) =>
            {
                resized=true;

              
                var width = window.ClientSize.Width;
                var height = window.ClientSize.Height;
                var config = ConfigurationManager.OpenExeConfiguration(ConfigurationUserLevel.None);
                config.AppSettings.Settings.Remove("width");
                config.AppSettings.Settings.Add("width", width.ToString());
                config.AppSettings.Settings.Remove("height");
                config.AppSettings.Settings.Add("height", height.ToString());
                config.Save(ConfigurationSaveMode.Modified);
                ConfigurationManager.RefreshSection("appSettings");
                resized=false;
                   
               
            };
            //try to get the size from the user settings
            var widthString = ConfigurationManager.AppSettings["width"];
            var heightString = ConfigurationManager.AppSettings["height"];
            if (!string.IsNullOrEmpty(widthString)&&!string.IsNullOrEmpty(heightString))
            {
                int width, height;
                if (int.TryParse(widthString, out width)&&int.TryParse(heightString, out height))
                {
                    window.ClientSize=new Size(width, height);
                }

            }

            //2 lines added by chatbot to fix window resize bug??
            windowUpdateLoopRunning=false;
            tryToClose=false;


            window.Closing+=OnClosing;
            WindowMode=windowMode;       //todo save this alos and seee if portable or if woks on optrk or win openGL as platfrom independedn.
                                         //wet as window not console to rid the console box

            //TODO  sset dpi             //TID sset dpi awarneness          if windows or if the api is there?
        //    SetProcessDpiAwareness(PROCESS_DPI_AWARENESS.Process_Per_Monitor_DPI_Aware);
            //awarneness


        }
        /// <summary>
        /// Constructs a new rendering-capable window.
        /// </summary>
        /// <param name="title">Title of the window.</param>
        /// <param name="resolution">Initial size in pixels of the window's drawable surface.</param>
        /// <param name="windowMode">Initial window mode.</param>
        public Window(string title, Int2 resolution, WindowMode windowMode)
            : this(title, resolution, new Int2((DisplayDevice.Default.Width - resolution.X) / 2, (DisplayDevice.Default.Height - resolution.Y) / 2), windowMode)
        {
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
