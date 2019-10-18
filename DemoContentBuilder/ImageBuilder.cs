using DemoContentLoader;
using ObjLoader.Loader.Loaders;
using SixLabors.ImageSharp.Advanced;
using SixLabors.ImageSharp.PixelFormats;
using System;
using System.Collections.Generic;
using System.IO;
using System.Numerics;

namespace DemoContentBuilder
{
    public static class ImageBuilder
    {
        public unsafe static ImageContent Build(Stream dataStream)
        {
            using (var rawImage = SixLabors.ImageSharp.Image.Load(dataStream))
            using (var image = rawImage.CloneAs<Rgba32>())
            {
                var pixels = image.GetPixelSpan();
                var imageData = new int[pixels.Length];
                fixed (int* imageDataPointer = imageData)
                {
                    var casted = new Span<Rgba32>(imageDataPointer, imageData.Length);
                    pixels.CopyTo(casted);
                }
                return new ImageContent(imageData, image.Width, image.Height);
            }
        }
    }
}
