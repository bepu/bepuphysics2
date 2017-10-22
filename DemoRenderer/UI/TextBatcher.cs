using BepuUtilities;
using BepuUtilities.Memory;
using DemoUtilities;
using SharpDX.Direct3D11;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace DemoRenderer.UI
{
    /// <summary>
    /// Accumulates text for rendering.
    /// </summary>
    public class TextBatcher
    {
        Pool<GlyphBatch> batchPool = new Pool<GlyphBatch>(() => new GlyphBatch(), cleaner: batch => batch.Clear());

        Dictionary<Font, GlyphBatch> batches = new Dictionary<Font, GlyphBatch>();
        
        Vector2 screenToPackedScale;
        internal Int2 Resolution
        {
            set
            {
                screenToPackedScale = new Vector2(65535f / value.X, 65535f / value.Y);
            }
        }
        
        public void Write(TextBuilder characters, int start, int count, Vector2 targetPosition, float height,
            Vector2 horizontalAxis, Vector3 color, Font font)
        {
            if (!batches.TryGetValue(font, out var glyphBatch))
            {
                glyphBatch = batchPool.Take();
                batches.Add(font, glyphBatch);
            }

            glyphBatch.Add(characters, start, count, screenToPackedScale, targetPosition, horizontalAxis, color, height, font);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Write(TextBuilder characters, Vector2 targetPosition, float height, Vector3 color, Font font)
        {
            Write(characters, 0, characters.Length, targetPosition, height, new Vector2(1, 0), color, font);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Write(TextBuilder characters, Vector2 targetPosition, float height, Vector2 horizontalAxis, Vector3 color, Font font)
        {
            Write(characters, 0, characters.Length, targetPosition, height, horizontalAxis, color, font);
        }

        public void Flush(DeviceContext context, Int2 screenResolution, GlyphRenderer renderer)
        {
            foreach (var batch in batches)
            {
                renderer.Render(context, batch.Key, screenResolution, batch.Value.Glyphs, 0, batch.Value.GlyphCount);
                batchPool.Return(batch.Value);
            }
            batches.Clear();

        }

    }
}
