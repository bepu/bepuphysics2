using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using OpenTK;
using OpenTK.Input;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Text;

namespace DemoUtilities
{
    using KeySet = QuickSet<Key, KeyComparer>;
    using MouseButtonSet = QuickSet<MouseButton, MouseButtonComparer>;
    struct KeyComparer : IEqualityComparerRef<Key>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(ref Key a, ref Key b)
        {
            return a == b;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Hash(ref Key item)
        {
            return (int)item;
        }
    }
    struct MouseButtonComparer : IEqualityComparerRef<MouseButton>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(ref MouseButton a, ref MouseButton b)
        {
            return a == b;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Hash(ref MouseButton item)
        {
            return (int)item;
        }
    }
    public class Input : IDisposable
    {
        NativeWindow window;


        //You could use GetState-like stuff to avoid the need to explicitly grab these, but shrug. This keeps events localized to just the window, and we can do a little logic of our own.
        KeySet anyDownedKeys;
        KeySet downedKeys;
        KeySet previousDownedKeys;
        MouseButtonSet anyDownedButtons;
        MouseButtonSet downedButtons;
        MouseButtonSet previousDownedButtons;
        BufferPool pool;
        public QuickList<char> TypedCharacters;

        /// <summary>
        /// Forces the mouse to stay at the center of the screen by recentering it on every flush.
        /// </summary>
        public bool MouseLocked
        {
            get; set;
        }
        Int2 WindowCenter { get { return new Int2(window.Width / 2, window.Height / 2); } }

        /// <summary>
        /// Gets or sets the mouse position in window coordinates without changing the net mouse delta.
        /// </summary>
        public Int2 MousePosition
        {
            get
            {
                var state = Mouse.GetCursorState();
                var clientPosition = window.PointToClient(new Point(state.X, state.Y));
                return new Int2 { X = clientPosition.X, Y = clientPosition.Y };
            }
            set
            {
                //Note that changing the cursor position does not change the raw mouse x/y.
                var screen = window.PointToScreen(new Point(value.X, value.Y));
                Mouse.SetPosition(screen.X, screen.Y);
            }
        }

        /// <summary>
        /// Gets the change in mouse position since the previous flush.
        /// </summary>
        public Int2 MouseDelta
        {
            get
            {
                return mouseDelta;
            }
        }

        /// <summary>
        /// Gets the amount of upward mouse wheel scrolling since the last flush regardless of how much downward scrolling occurred.
        /// </summary>
        public float ScrolledUp { get; private set; }
        /// <summary>
        /// Gets the amount of downward mouse wheel scrolling since the last flush regardless of how much upward scrolling occurred.
        /// </summary>
        public float ScrolledDown { get; private set; }
        /// <summary>
        /// Gets the mouse wheel scroll delta since the last flush.
        /// </summary>
        public float ScrollDelta { get { return ScrolledUp + ScrolledDown; } }

        public Input(Window window, BufferPool pool)
        {
            this.window = window.window;
            this.window.KeyDown += KeyDown;
            this.window.KeyUp += KeyUp;
            this.window.MouseDown += MouseDown;
            this.window.MouseUp += MouseUp;
            this.window.MouseWheel += MouseWheel;
            this.window.KeyPress += KeyPress;
            this.pool = pool;
            anyDownedButtons = new MouseButtonSet(8, pool);
            downedButtons = new MouseButtonSet(8, pool);
            previousDownedButtons = new MouseButtonSet(8, pool);
            anyDownedKeys = new KeySet(8, pool);
            downedKeys = new KeySet(8, pool);
            previousDownedKeys = new KeySet(8, pool);
            TypedCharacters = new QuickList<char>(32, pool);
        }

        private void KeyPress(object sender, KeyPressEventArgs e)
        {
            TypedCharacters.Add(e.KeyChar, pool);
        }

        private void MouseWheel(object sender, MouseWheelEventArgs e)
        {
            if (e.DeltaPrecise > 0)
                ScrolledUp += e.DeltaPrecise;
            else
                ScrolledDown += e.DeltaPrecise;
        }

        private void MouseDown(object sender, MouseButtonEventArgs e)
        {
            anyDownedButtons.Add(e.Button, pool);
            downedButtons.Add(e.Button, pool);
        }
        private void MouseUp(object sender, MouseButtonEventArgs e)
        {
            downedButtons.FastRemove(e.Button);
        }

        private void KeyDown(object sender, KeyboardKeyEventArgs e)
        {
            anyDownedKeys.Add(e.Key, pool);
            downedKeys.Add(e.Key, pool);
            //Unfortunately, backspace isn't reported by keypress, so we do it manually.
            if (e.Key == Key.BackSpace)
                TypedCharacters.Add('\b', pool);
        }
        private void KeyUp(object sender, KeyboardKeyEventArgs e)
        {
            downedKeys.FastRemove(e.Key);
        }



        /// <summary>
        /// Gets whether a key is currently pressed according to the latest event processing call.
        /// </summary>
        /// <param name="key">Key to check.</param>
        /// <returns>True if the key was pressed in the latest event processing call, false otherwise.</returns>
        public bool IsDown(Key key)
        {
            return downedKeys.Contains(key);
        }

        /// <summary>
        /// Gets whether a key was down at the time of the previous flush.
        /// </summary>
        /// <param name="key">Key to check.</param>
        /// <returns>True if the key was down at the time of the previous flush, false otherwise.</returns>
        public bool WasDown(Key key)
        {
            return previousDownedKeys.Contains(key);
        }

        /// <summary>
        /// Gets whether a down event occurred at any point between the previous flush and up to the last event process call for a key that was not down in the previous flush.
        /// </summary>
        /// <param name="key">Key to check.</param>
        /// <returns>True if the key was pressed in the latest event processing call, false otherwise.</returns>
        public bool WasPushed(Key key)
        {
            return !previousDownedKeys.Contains(key) && anyDownedKeys.Contains(key);
        }


        /// <summary>
        /// Gets whether a button is currently pressed according to the latest event processing call.
        /// </summary>
        /// <param name="button">Button to check.</param>
        /// <returns>True if the button was pressed in the latest event processing call, false otherwise.</returns>
        public bool IsDown(MouseButton button)
        {
            return downedButtons.Contains(button);
        }

        /// <summary>
        /// Gets whether a button was down at the time of the previous flush.
        /// </summary>
        /// <param name="mouseButton">Button to check.</param>
        /// <returns>True if the button was down at the time of the previous flush, false otherwise.</returns>
        public bool WasDown(MouseButton mouseButton)
        {
            return previousDownedButtons.Contains(mouseButton);
        }

        /// <summary>
        /// Gets whether a down event occurred at any point between the previous flush and up to the last event process call for a button that was not down in the previous flush.
        /// </summary>
        /// <param name="button">Button to check.</param>
        /// <returns>True if the button was pressed in the latest event processing call, false otherwise.</returns>
        public bool WasPushed(MouseButton button)
        {
            return !previousDownedButtons.Contains(button) && anyDownedButtons.Contains(button);
        }

        Int2 mouseDelta;
        Int2 previousRawMouse;
        public void Start()
        {
            var currentState = Mouse.GetState();
            //Given a long enough time, this could theoretically hit overflow.
            //But that would require hours of effort with a high DPI mouse, and this is a demo application...
            mouseDelta.X = currentState.X - previousRawMouse.X;
            mouseDelta.Y = currentState.Y - previousRawMouse.Y;
            previousRawMouse = new Int2(currentState.X, currentState.Y);
            if (MouseLocked)
            {
                //This is pretty doofy, but it works reasonably well and we don't have easy access to the windows-provided capture stuff through opentk (that I'm aware of?).
                //Could change it later if it matters, but realistically it won't matter.
                MousePosition = WindowCenter;
                window.CursorVisible = false;
            }
            else
            {
                window.CursorVisible = true;
            }

        }
        public void End()
        {
            anyDownedKeys.Clear();
            anyDownedButtons.Clear();
            previousDownedKeys.Clear();
            previousDownedButtons.Clear();
            for (int i = 0; i < downedKeys.Count; ++i)
                previousDownedKeys.Add(downedKeys[i], pool);
            for (int i = 0; i < downedButtons.Count; ++i)
                previousDownedButtons.Add(downedButtons[i], pool);
            ScrolledDown = 0;
            ScrolledUp = 0;
            TypedCharacters.Count = 0;
        }

        /// <summary>
        /// Unhooks the input management from the window.
        /// </summary>
        public void Dispose()
        {
            window.KeyDown -= KeyDown;
            window.KeyUp -= KeyUp;
            window.MouseDown -= MouseDown;
            window.MouseUp -= MouseUp;

            anyDownedKeys.Dispose(pool);
            downedKeys.Dispose(pool);
            previousDownedKeys.Dispose(pool);
            anyDownedButtons.Dispose(pool);
            downedButtons.Dispose(pool);
            previousDownedButtons.Dispose(pool);
        }
    }
}
