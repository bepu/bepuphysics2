using BepuUtilities;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace Demos
{
    public class RolloverInfo
    {
        struct RolloverDescription
        {
            public Vector3 Position;
            public string Description;
            public float PreviewOffset;
            public string Preview;
        }

        List<RolloverDescription> descriptions;

        public RolloverInfo()
        {
            descriptions = new List<RolloverDescription>();
        }
        
        public void Add(in Vector3 position, string description, float previewOffset = -1.2f, string previewText = "Info...")
        {
            this.descriptions.Add(new RolloverDescription { Position = position, Description = description, PreviewOffset = previewOffset, Preview = previewText });
        }


        public unsafe void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            var resolution = new Vector2(renderer.Surface.Resolution.X, renderer.Surface.Resolution.Y);
            var screenLocations = stackalloc Vector2[descriptions.Count];
            int closestIndex = -1;
            float closestDistance = MathF.Max(resolution.X, resolution.Y) * 0.1f;
            for (int i = 0; i < descriptions.Count; ++i)
            {
                var textPosition = descriptions[i].Position;
                Matrix.Transform(new Vector4(textPosition, 1), camera.ViewProjection, out var projected);
                projected /= projected.W;
                if (projected.Z <= 0 || MathF.Abs(projected.X) > 1 || MathF.Abs(projected.Y) > 1)
                    continue;
                var ndc = new Vector2(projected.X, projected.Y);
                screenLocations[i] = (ndc * new Vector2(0.5f, -0.5f) + new Vector2(0.5f)) * resolution;
                var mouse = input.MousePosition;
                var distance = Vector2.Distance(new Vector2(mouse.X, mouse.Y), screenLocations[i]);
                if (distance < closestDistance)
                {
                    closestDistance = distance;
                    closestIndex = i;
                }
            }

            const float infoHeight = 8;
            const float descriptionHeight = 16;
            for (int i = 0; i < descriptions.Count; ++i)
            {
                if (i != closestIndex)
                {
                    text.Clear().Append(descriptions[i].Preview);
                    var infoLength = GlyphBatch.MeasureLength(text, font, infoHeight);
                    renderer.TextBatcher.Write(text, screenLocations[i] + new Vector2(-infoLength * 0.5f, descriptions[i].PreviewOffset * descriptionHeight), infoHeight, new Vector3(1), font);
                }
            }
            if (closestIndex >= 0)
            {
                text.Clear().Append(descriptions[closestIndex].Description);
                var descriptionLength = GlyphBatch.MeasureLength(text, font, descriptionHeight);
                renderer.TextBatcher.Write(text, screenLocations[closestIndex] - new Vector2(descriptionLength * 0.5f, 0), 16, new Vector3(1), font);
            }

        }
    }
}
