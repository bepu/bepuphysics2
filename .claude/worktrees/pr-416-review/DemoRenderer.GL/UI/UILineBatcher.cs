using BepuUtilities;
using System;
using System.Diagnostics;
using System.Numerics;

namespace DemoRenderer.UI
{
    /// <summary>
    /// Accumulates UI lines for rendering.
    /// </summary>
    public class UILineBatcher
    {
        Vector2 screenToPackedScale;
        internal Int2 Resolution
        {
            set
            {
                screenToPackedScale = new Vector2(65535f / value.X, 65535f / value.Y);
            }
        }
        UILineInstance[] lines;
        public int LineCount { get; private set; }
        public UILineBatcher(int initialCapacity = 512)
        {
            lines = new UILineInstance[initialCapacity];
        }

        public void Draw(in Vector2 start, in Vector2 end, float radius, in Vector3 color)
        {
            if (LineCount == lines.Length)
            {
                Debug.Assert(lines.Length > 0);
                Array.Resize(ref lines, LineCount * 2);
            }
            lines[LineCount++] = new UILineInstance(start, end, radius, color, screenToPackedScale);
        }

        public void Flush(Int2 screenResolution, UILineRenderer renderer)
        {
            renderer.Render(screenResolution, lines, 0, LineCount);
            LineCount = 0;
        }
    }
}
