using System.IO;

namespace DemoContentLoader
{
    public class Texture2DIO
    {
        public unsafe static Texture2DContent Load(BinaryReader reader)
        {
            var width = reader.ReadInt32();
            var height = reader.ReadInt32();
            var mipLevels = reader.ReadInt32();
            var texelSizeInBytes = reader.ReadInt32();
            var content = new Texture2DContent(width, height, mipLevels, texelSizeInBytes);
            reader.Read(content.Data, 0, content.Data.Length);
            return content;
        }
        public static void Save(Texture2DContent content, BinaryWriter writer)
        {
            writer.Write(content.Width);
            writer.Write(content.Height);
            writer.Write(content.MipLevels);
            writer.Write(content.TexelSizeInBytes);
            writer.Write(content.Data);
        }
    }
}