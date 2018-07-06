using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace DemoContentLoader
{
    public enum ContentType
    {
        Font = 1,
        Mesh = 2
    }
    public interface IContent
    {
        ContentType ContentType { get; }
    }

    public class ContentArchive
    {
        private Dictionary<string, IContent> pathsToContent = new Dictionary<string, IContent>();

        public T Load<T>(string path)
        {
            if (!pathsToContent.TryGetValue(path, out var untypedContent))
            {
                throw new ArgumentException($"{path} not found in the content archive.");
            }
            if (untypedContent is T content)
            {
                return content;
            }
            else
            {
                throw new ArgumentException($"Content associated with {path} does not match the given type {typeof(T).Name}.");
            }
        }

        //We have a very limited set of content types. This isn't a general purpose engine. Rather than having a dictionary of type->loader or something, we can do a quick hack.
        public static IContent Load(ContentType type, BinaryReader reader)
        {
            switch (type)
            {
                case ContentType.Font:
                    return FontIO.Load(reader);
                case ContentType.Mesh:
                    return MeshIO.Load(reader);
            }
            throw new ArgumentException($"Given content type {type} cannot be loaded; no loader is specified. Is the archive corrupted?");
        }

        public static void Save(IContent content, BinaryWriter writer)
        {
            switch (content.ContentType)
            {
                case ContentType.Font:
                    FontIO.Save((FontContent)content, writer);
                    return;
                case ContentType.Mesh:
                    MeshIO.Save((MeshContent)content, writer);
                    return;
            }
            throw new ArgumentException("Given content type cannot be saved; no archiver is specified.");
        }

        /// <summary>
        /// Loads a content archive, previously saved using ContentArchive.Save, from a stream.
        /// </summary>
        /// <param name="stream">Stream to load from.</param>
        /// <returns>Archive of loaded content.</returns>
        public static ContentArchive Load(Stream stream)
        {
            //Read each piece of content in sequence.
            //Format follows:
            //Entry count
            //[Entry 1]
            //[Entry 2]
            //...
            //[Entry N]

            //where Entry:
            //[pathLength : int32]
            //[pathBytes : byte[]]
            //[contentType : int32]
            //[contentLengthInBytes : int32]
            //[content : serializer specific]

            var archive = new ContentArchive();
            using (var reader = new BinaryReader(stream))
            {
                var entryCount = reader.ReadInt32();
                for (int i = 0; i < entryCount; ++i)
                {
                    var pathLengthInBytes = reader.ReadInt32();
                    byte[] pathBytes = new byte[pathLengthInBytes];
                    reader.Read(pathBytes, 0, pathLengthInBytes);
                    var path = Encoding.Unicode.GetString(pathBytes, 0, pathBytes.Length);

                    var contentType = (ContentType)reader.ReadInt32();
                    archive.pathsToContent.Add(path, Load(contentType, reader));
                }
            }
            return archive;
        }

        /// <summary>
        /// Saves out a set of path-content pairs in a format loadable as a ContentArchive.
        /// </summary>
        /// <param name="pathsToContent">Path-content pairs to save.</param>
        /// <param name="stream">Output stream to save to.</param>
        public static void Save(Dictionary<string, IContent> pathsToContent, Stream stream)
        {
            using (var writer = new BinaryWriter(stream))
            {
                writer.Write(pathsToContent.Count);
                foreach (var pair in pathsToContent)
                {
                    var path = pair.Key;
                    var content = pair.Value;

                    var pathBytes = Encoding.Unicode.GetBytes(path);
                    writer.Write(pathBytes.Length);
                    writer.Write(pathBytes);

                    writer.Write((int)content.ContentType);
                    Save(content, writer);
                }
            }
        }
    }
}
