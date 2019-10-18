using System;
using System.Collections.Generic;
using System.IO;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace DemoContentLoader
{
    public class ImageIO
    {
        public static ImageContent Load(BinaryReader reader)
        {
            var width = reader.ReadInt32();
            var height = reader.ReadInt32();
            //Lotta unnecessary copies here. Doesn't matter for the demos!
            var imageData = new int[width * height];
            var imageBytes = reader.ReadBytes(imageData.Length * 4);
            Unsafe.CopyBlockUnaligned(ref Unsafe.As<int, byte>(ref imageData[0]), ref imageBytes[0], (uint)imageBytes.Length);
            return new ImageContent(imageData, width, height);
        }

        public unsafe static void Save(ImageContent content, BinaryWriter writer)
        {
            writer.Write(content.Width);
            writer.Write(content.Height);
            var imageBytes = new byte[content.Width * content.Height * 4];
            Unsafe.CopyBlockUnaligned(ref imageBytes[0], ref Unsafe.As<int, byte>(ref content.ImageData[0]), (uint)imageBytes.Length);
            writer.Write(imageBytes);
        }
    }
}