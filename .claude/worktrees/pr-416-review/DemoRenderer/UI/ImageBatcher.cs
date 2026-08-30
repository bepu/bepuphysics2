using BepuUtilities;
using BepuUtilities.Collections;
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
    /// Accumulates image instances for rendering.
    /// </summary>
    public class ImageBatcher
    {
        //Explicitly box the instance list. Marginally easier than making renderable image unmanaged.
        class ImageBatch
        {
            public QuickList<ImageInstance> Instances;
        }
        Pool<ImageBatch> batchPool;

        Dictionary<RenderableImage, ImageBatch> batches = new Dictionary<RenderableImage, ImageBatch>();

        BufferPool pool;
        public ImageBatcher(BufferPool pool)
        {
            this.pool = pool;
            batchPool = new Pool<ImageBatch>(
                () => new ImageBatch(),
                (batch) => batch.Instances = new QuickList<ImageInstance>(16, pool),
                (batch) => batch.Instances.Dispose(pool));
        }

        Vector2 screenToPackedScale;
        internal Int2 Resolution
        {
            set
            {
                screenToPackedScale = new Vector2(65535f / value.X, 65535f / value.Y);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Draw(RenderableImage image, in Vector2 targetPosition, in Vector2 size, in Vector2 horizontalAxis, in Vector4 color)
        {
            if (!batches.TryGetValue(image, out var batch))
            {
                batch = batchPool.Take();
                batches.Add(image, batch);
            }
            batch.Instances.Allocate(pool) = new ImageInstance(targetPosition, horizontalAxis, size, color, screenToPackedScale);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Draw(RenderableImage image, in Vector2 targetPosition, in Vector2 size, in Vector4 color)
        {
            Draw(image, targetPosition, size, new Vector2(1, 0), color);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Draw(RenderableImage image, in Vector2 targetPosition, in Vector2 size)
        {
            Draw(image, targetPosition, size, new Vector2(1, 0), new Vector4(1));
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Draw(RenderableImage image, in Vector2 targetPosition, float height)
        {
            var scale = height / image.Content.Height;
            Draw(image, targetPosition, scale * new Vector2(image.Content.Width, image.Content.Height), new Vector2(1, 0), new Vector4(1));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Draw(RenderableImage image, in Vector2 targetPosition, float height, in Vector4 color)
        {
            var scale = height / image.Content.Height;
            Draw(image, targetPosition, scale * new Vector2(image.Content.Width, image.Content.Height), new Vector2(1, 0), color);
        }


        public unsafe void Flush(DeviceContext context, Int2 screenResolution, ImageRenderer renderer)
        {
            foreach (var batch in batches)
            {
                renderer.Render(context, batch.Key, screenResolution, new Span<ImageInstance>(batch.Value.Instances.Span.Memory, batch.Value.Instances.Count));
                batchPool.Return(batch.Value);
            }
            batches.Clear();

        }

    }
}
