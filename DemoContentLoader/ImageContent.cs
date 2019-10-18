using BepuUtilities;
using DemoContentLoader;
using System;
using System.Collections.Generic;
using System.Numerics;

namespace DemoContentLoader
{
    /// <summary>
    /// Simple RGBA32 image data.
    /// </summary>
    public class ImageContent : IContent
    {
        public ContentType ContentType { get { return ContentType.Image; } }

        public int Width;
        public int Height;
        public int[] ImageData;

        public ImageContent(int[] imageData, int width, int height)
        {
            ImageData = imageData;
            Width = width;
            Height = height;
        }
    }
}
