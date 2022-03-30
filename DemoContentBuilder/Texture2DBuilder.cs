using DemoContentLoader;
using ObjLoader.Loader.Loaders;
using SixLabors.ImageSharp.Advanced;
using SixLabors.ImageSharp.PixelFormats;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace DemoContentBuilder
{
    public static class Texture2DBuilder
    {
        public unsafe static Texture2DContent Build(Stream dataStream)
        {
            using var rawImage = SixLabors.ImageSharp.Image.Load(dataStream);
            using var image = rawImage.CloneAs<Rgba32>();
            //We're only supporting R8G8B8A8 right now, so texel size in bytes is always 4.
            //We don't compute mips during at content time. We could, but... there's not much reason to.
            //The font builder does because it uses a nonstandard mip process, but this builder is expected to be used to with normal data.
            var content = new Texture2DContent(image.Width, image.Height, 1, sizeof(Rgba32));
            var data = (Rgba32*)content.Pin();
            //Copy the image data into the Texture2DContent.
            for (int rowIndex = 0; rowIndex < image.Height; ++rowIndex)
            {
                var sourceRow = image.GetPixelRowSpan(rowIndex);
                var targetRow = data + content.GetRowOffsetForMip0(rowIndex);
                Unsafe.CopyBlockUnaligned(ref *(byte*)targetRow, ref Unsafe.As<Rgba32, byte>(ref sourceRow[0]), (uint)(sizeof(Rgba32) * image.Width));
            }
            content.Unpin();
            return content;
        }
    }
}