using System;
using System.Collections.Generic;
using System.IO;
using SharpDX.D3DCompiler;
using System.Linq;
using DemoContentLoader;

namespace DemoContentBuilder
{
    public struct ContentElement
    {
        public long LastModifiedTimestamp;
        public IContent Content;
    }

    /// <summary>
    /// Stores built content and associated timestamps needed to know which content needs to be freshly built.
    /// </summary>
    public static class ContentBuildCacheIO
    {
        public static void Save(Dictionary<string, ContentElement> cache, string path)
        {
            using (var stream = new FileStream(path, FileMode.Create, FileAccess.Write, FileShare.None))
            {
                Save(cache, stream);
            }
        }

        public static void Save(Dictionary<string, ContentElement> cache, Stream outputStream)
        {
            //Save the number of shaders.
            using (var writer = new BinaryWriter(outputStream))
            {
                writer.Write(cache.Count);

                //Save every element in sequence.
                foreach (var element in cache)
                {
                    writer.Write(element.Key);
                    writer.Write(element.Value.LastModifiedTimestamp);
                    writer.Write((int)element.Value.Content.ContentType);
                    ContentArchive.Save(element.Value.Content, writer);
                }
            }
        }

        public static bool Load(string path, out Dictionary<string, ContentElement> cache)
        {
            if (!File.Exists(path))
            {
                cache = new Dictionary<string, ContentElement>();
                return false;
            }
            using (var stream = File.OpenRead(path))
            {
                cache = Load(stream);
                return true;
            }
        }



        public static Dictionary<string, ContentElement> Load(Stream stream)
        {
            using (var reader = new BinaryReader(stream))
            {
                try
                {
                    var cache = new Dictionary<string, ContentElement>();
                    var contentCount = reader.ReadInt32();

                    for (int i = 0; i < contentCount; ++i)
                    {
                        var path = reader.ReadString();
                        var lastModifiedTimestamp = reader.ReadInt64();
                        var contentType = (ContentType)reader.ReadInt32();
                        var content = ContentArchive.Load(contentType, reader);
                        cache.Add(path, new ContentElement { LastModifiedTimestamp = lastModifiedTimestamp, Content = content });
                    }
                    return cache;
                }
                catch (Exception e)
                {
                    Console.WriteLine($"Content build cache load failed; may be corrupted. Assuming fresh build. Message:");
                    Console.WriteLine(e.Message);
                    return new Dictionary<string, ContentElement>();
                }
            }
        }
    }
}
