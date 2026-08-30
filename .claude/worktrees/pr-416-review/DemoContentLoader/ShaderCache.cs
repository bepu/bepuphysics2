using System;
using System.Collections.Generic;
using System.IO;

namespace DemoContentLoader
{
    public class ShaderCache
    {
        private Dictionary<SourceShader, byte[]> shaders;
        public ShaderCache(Dictionary<SourceShader, byte[]> shaders)
        {
            this.shaders = shaders;
        }

        public byte[] GetShader(string resourceName)
        {
            return GetShader(new SourceShader { Name = resourceName });
        }
        public byte[] GetShader(string resourceName, params string[] defines)
        {
            return GetShader(new SourceShader { Name = resourceName, Defines = defines });
        }


        public byte[] GetShader(SourceShader sourceShader)
        {
            if (shaders.TryGetValue(sourceShader, out var shader))
            {
                return shader;
            }
            throw new Exception("Shader " + sourceShader + " was not in the shader cache.");
        }

        //* * * * NONE OF THIS CLASS IS THREAD SAFE * * * * 

        //FORMAT OF SHADER CONTENT ARCHIVE:
        //Count of elements stored in int32
        //{List of name-bytecode pairs}
        //
        //Where each pair is represented as:
        //Unique shader name
        //Element size in bytes (int64)
        //{Chunk of element-defined bytes}
        //
        //Where each Unique element name is represented by a UTF-16 string, stored as:
        //Character count (int32)
        //{Characters}


        public static void Save(Dictionary<SourceShader, byte[]> archive, Stream stream)
        {
            //Save the number of elements.
            using (var writer = new BinaryWriter(stream))
            {
                writer.Write(archive.Count);

                //Save every element in sequence.
                foreach (var pathShaderPair in archive)
                {
                    SourceShader.Write(writer, pathShaderPair.Key);

                    //Write the size of the element.
                    var size = pathShaderPair.Value.Length;
                    writer.Write(size);

                    //Write the element's data.
                    writer.Write(pathShaderPair.Value, 0, size);
                }
            }
        }

        public static ShaderCache Load(Stream stream)
        {
            var archive = new Dictionary<SourceShader, byte[]>();
            using (var reader = new BinaryReader(stream))
            {
                var count = reader.ReadInt32();

                for (int i = 0; i < count; ++i)
                {
                    //Read the name length.
                    var shader = SourceShader.Read(reader);

                    //Read the size in bytes of the content element data itself.
                    int sizeInBytes = reader.ReadInt32();
                    var data = new byte[sizeInBytes];
                    reader.Read(data, 0, sizeInBytes);
                    archive.Add(shader, data);

                }
            }
            return new ShaderCache(archive);
        }

    }
}
