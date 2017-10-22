using System;
using System.Collections.Generic;
using System.IO;
using SharpDX.D3DCompiler;
using System.Linq;
using DemoContentLoader;

namespace DemoContentBuilder
{
    /// <summary>
    /// Stores compiled shaders and their associated timestamps needed to know which shaders need to be freshly compiled.
    /// </summary>
    /// <remarks>This is a bit overkill for the bepuphysics demos, but... I just pulled it over from the graphics engine's pipeline so shrug.</remarks>
    public class ShaderCompilationCache
    {
        /// <summary>
        /// Gets the mapping of file names to time stamps representing the last compiled time of a file.
        /// This includes files which appear solely as dependencies.
        /// </summary>
        public Dictionary<string, long> TimeStamps { get; private set; }

        /// <summary>
        /// Gets the mapping of file names to their previous compilation results.
        /// Only includes shader files with entry points.
        /// </summary>
        public Dictionary<SourceShader, ShaderBytecode> CompiledShaders { get; private set; }

        /// <summary>
        /// Gets the mapping of file names to the files which they depend on.
        /// </summary>
        public Dictionary<string, HashSet<string>> Dependencies { get; private set; }

        /// <summary>
        /// Gets the set of flags that this cache was compiled with.
        /// </summary>
        public ShaderFlags ShaderFlags { get; private set; }

        public ShaderCompilationCache(ShaderFlags flags)
        {
            ShaderFlags = flags;
            TimeStamps = new Dictionary<string, long>();
            CompiledShaders = new Dictionary<SourceShader, ShaderBytecode>();
            Dependencies = new Dictionary<string, HashSet<string>>();
        }

        //FORMAT OF SHADER CONTENT ARCHIVE:
        //int32 holding shader flags
        //Count of path-shaders stored in int32
        //{Contiguous list of path-shaders}
        //
        //Where each element is represented as:
        //Path name
        //Shader bytecode size in bytes (int64)
        //Shader bytecode
        //        
        //Count of path-timestamps stored in int32
        //{Contiguous list of path-timestamps}
        //
        //Where each element is represented as:
        //Path name
        //timestamp
        //
        //Count of path-dependencies stored in int32
        //{Contiguous list of path-dependencies}
        //
        //Where each element is represented as:
        //Path name
        //number of dependencies
        //[list of dependency paths]
        //
        //Where each Unique element name is represented by a UTF-16 string, stored as:
        //Character count (int32)
        //{Characters}


        public static void Save(ShaderCompilationCache cache, string path)
        {
            using (var archiveFileStream = new FileStream(path, FileMode.Create, FileAccess.Write, FileShare.None))
            {
                Save(cache, archiveFileStream);
            }
        }

        public static void Save(ShaderCompilationCache cache, Stream outputStream)
        {
            //Save the number of shaders.
            using (var writer = new BinaryWriter(outputStream))
            {
                writer.Write((int)cache.ShaderFlags);
                writer.Write(cache.CompiledShaders.Count);

                //Save every path-shader in sequence.
                foreach (var element in cache.CompiledShaders)
                {
                    //Write the element's name.
                    writer.Write(element.Key.Name);
                    writer.Write(element.Key.Defines.Length);
                    for (int i = 0; i < element.Key.Defines.Length; ++i)
                    {
                        writer.Write(element.Key.Defines[i]);
                    }

                    //Write the size of the shader bytecode.
                    writer.Write(element.Value.Data.Length);
                    //Write the bytecode itself.
                    writer.Write(element.Value.Data, 0, element.Value.Data.Length);


                }

                //Save the number of timestamped references.
                //Note that timestamps and dependencies do not have define permutations. Timestamps and dependencies do not change with respect to permutations.
                writer.Write(cache.TimeStamps.Count);

                foreach (var element in cache.TimeStamps)
                {
                    writer.Write(element.Key);

                    //Write the current time stamp onto the element.
                    writer.Write(element.Value);
                }

                writer.Write(cache.Dependencies.Count);
                foreach (var pathDependenciesPair in cache.Dependencies)
                {
                    writer.Write(pathDependenciesPair.Key);
                    writer.Write(pathDependenciesPair.Value.Count);
                    foreach (var dependency in pathDependenciesPair.Value)
                    {
                        writer.Write(dependency);
                    }
                }
            }
        }

        public static bool TryLoad(string path, out ShaderCompilationCache cache)
        {
            if (!File.Exists(path))
            {
                cache = null;
                return false;
            }
            using (var stream = File.OpenRead(path))
            {
                return TryLoad(stream, out cache);
            }
        }



        public static bool TryLoad(Stream stream, out ShaderCompilationCache cache)
        {
            using (var reader = new BinaryReader(stream))
            {
                try
                {
                    cache = new ShaderCompilationCache((ShaderFlags)reader.ReadInt32());

                    byte[] data = new byte[16384];

                    var shaderDataCount = reader.ReadInt32();
                    for (int i = 0; i < shaderDataCount; ++i)
                    {
                        var shaderSource = SourceShader.Read(reader);

                        //Read the size in bytes of the content element data itself.
                        int sizeInBytes = reader.ReadInt32();

                        if (data.Length < sizeInBytes)
                            data = new byte[sizeInBytes];

                        reader.Read(data, 0, sizeInBytes);

                        ShaderBytecode bytecode;
                        unsafe
                        {
                            fixed (byte* buffer = data)
                            {
                                bytecode = new ShaderBytecode(new IntPtr(buffer), sizeInBytes);
                            }
                        }
                        cache.CompiledShaders.Add(shaderSource, bytecode);
                    }

                    var timeStampCount = reader.ReadInt32();
                    for (int i = 0; i < timeStampCount; ++i)
                    {
                        var shaderSource = reader.ReadString();

                        //Read the time stamp.
                        long timeStamp = reader.ReadInt64();
                        cache.TimeStamps.Add(shaderSource, timeStamp);
                    }

                    var dependenciesCount = reader.ReadInt32();
                    for (int i = 0; i < dependenciesCount; ++i)
                    {
                        var shaderSource = reader.ReadString();
                        var pathDependenciesCount = reader.ReadInt32();
                        var dependencies = new HashSet<string>();
                        for (int j = 0; j < pathDependenciesCount; ++j)
                        {
                            dependencies.Add(reader.ReadString());
                        }
                        cache.Dependencies.Add(shaderSource, dependencies);
                    }
                }
                catch
                {
                    cache = null;
                    return false;
                }
                return true;
            }
        }


        public void CopyFrom(string source, ShaderCompilationCache loadedCache)
        {
            lock (CompiledShaders)
            {
                //Just try every stage. Could be smarter about this, but nah.
                foreach (var stage in MetadataParsing.Stages)
                {
                    //The old cached data is still valid. Copy it over into the new cache and skip.
                    //Note that there may be multiple compiled shaders due to shader permutations.
                    string sourceWithExtension = source + stage.Extension;
                    var existingShaders = from pair in loadedCache.CompiledShaders
                                          where pair.Key.Name == sourceWithExtension
                                          select pair;
                    foreach (var entry in existingShaders)
                    {
                        CompiledShaders.Add(entry.Key, entry.Value);
                    }
                }
                TimeStamps[source] = loadedCache.TimeStamps[source];
                var dependencies = new HashSet<string>(loadedCache.Dependencies[source]);
                foreach (var dependency in dependencies)
                {
                    TimeStamps[dependency] = loadedCache.TimeStamps[dependency];
                }
                Dependencies[source] = dependencies;
            }
        }

        public void AddCompiledShader(ShaderCompilationTarget target, List<string> dependencyPaths, CompilationResult result)
        {
            string[] defines = new string[target.ShaderMacros.Length];
            for (int i = 0; i < target.ShaderMacros.Length; ++i)
            {
                defines[i] = target.ShaderMacros[i].Name;
            }
            var sourceShader = new SourceShader { Name = target.Path + target.Stage.Extension, Defines = defines };
            lock (CompiledShaders)
            {
                CompiledShaders[sourceShader] = result;
                TimeStamps[target.Path] = target.CurrentTimeStamp;

                foreach (var dependency in dependencyPaths)
                {
                    TimeStamps[dependency] = File.GetLastWriteTime(dependency).Ticks;
                }
                if (!Dependencies.TryGetValue(target.Path, out HashSet<string> cacheDependencyPaths))
                {
                    cacheDependencyPaths = new HashSet<string>();
                    Dependencies.Add(target.Path, cacheDependencyPaths);
                }
                foreach (var dependencyPath in dependencyPaths)
                {
                    cacheDependencyPaths.Add(dependencyPath);
                }
            }
        }

    }
}
