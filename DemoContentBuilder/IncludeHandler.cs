using System;
using System.Collections.Generic;
using System.IO;
using SharpDX;
using System.Text;
using SharpDX.D3DCompiler;

namespace DemoContentBuilder
{
    struct MetadataParsingError
    {
        public string Path;
        public string Message;
    }

    struct MacroGroup : IEquatable<MacroGroup>
    {
        public string[] Names;

        public bool Equals(MacroGroup other)
        {
            if (Names.Length != other.Names.Length)
            {
                return false;
            }
            foreach (var name in Names)
            {
                if (Array.IndexOf(other.Names, name) == -1)
                    return false;
            }
            return true;
        }
    }
    /// <summary>
    /// Standard shader include handler that pulls out dependencies and metadata.
    /// </summary>
    class IncludeHandler : CallbackBase, Include
    {
        public List<string> DependencyPaths { get; private set; }
        public List<ShaderStage> Stages { get; private set; }
        public List<MacroGroup> MacroGroups { get; private set; }
        public List<MetadataParsingError> IncludeParsingErrors { get; private set; }
        public ShaderFileCache ShaderFileCache { get; private set; }

        public string ProjectWorkingPath { get; private set; }
        public string LocalWorkingPath { get; private set; }

        public IncludeHandler(ShaderFileCache shaderFileCache, string projectWorkingPath, string localWorkingPath)
        {
            ShaderFileCache = shaderFileCache;
            ProjectWorkingPath = projectWorkingPath;
            LocalWorkingPath = localWorkingPath;
            DependencyPaths = new List<string>();
            Stages = new List<ShaderStage>();
            MacroGroups = new List<MacroGroup>();
            IncludeParsingErrors = new List<MetadataParsingError>();
        }

        public void Close(Stream stream)
        {
            stream.Dispose();
        }

        MemoryStream ExtractMetadataAndStream(string path)
        {
            if (ShaderFileCache.TryLoad(path, out string text))
            {
                DependencyPaths.Add(path);
                if (MetadataParsing.Parse(path, text, Stages, MacroGroups, IncludeParsingErrors))
                {
                    //We write ascii into the bytes regardless of the source text so that fxc is kept happy.
                    var bytes = Encoding.ASCII.GetBytes(text);
                    return new MemoryStream(bytes);
                }
            }
            return null;
        }

        public Stream Open(IncludeType type, string fileName, Stream parentStream)
        {
            if (type == IncludeType.Local)
            {
                var localPath = Path.GetFullPath(LocalWorkingPath + Path.DirectorySeparatorChar + fileName);
                var stream = ExtractMetadataAndStream(localPath);
                if (stream == null)
                {
                    //Console.WriteLine($"Failed opening local path: {localPath}. Trying project working path.");
                    var projectPath = Path.GetFullPath(ProjectWorkingPath + Path.DirectorySeparatorChar + fileName);
                    stream = ExtractMetadataAndStream(projectPath);
                    if (stream == null)
                    {
                        //Console.WriteLine($"Failed opening project relative path: {projectPath}.");
                    }
                }
                return stream;
            }
            return null;
        }


    }
}
