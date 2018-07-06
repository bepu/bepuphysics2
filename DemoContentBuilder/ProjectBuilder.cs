using DemoContentLoader;
using System;
using System.Collections.Generic;
using System.IO;

namespace DemoContentBuilder
{
    static class ProjectBuilder
    {
        public static string GetRelativePathFromDirectory(string path, string baseDirectory)
        {
            //(Borrowed this from a ye olde Marc Gravell post on SO.)
            var pathUri = new Uri(path);
            //Guarantee that the folder ends with a slash.
            if (!baseDirectory.EndsWith(Path.DirectorySeparatorChar.ToString()))
            {
                baseDirectory += Path.DirectorySeparatorChar;
            }
            var directoryUri = new Uri(baseDirectory);
            return Uri.UnescapeDataString(directoryUri.MakeRelativeUri(pathUri).ToString().Replace('/', Path.DirectorySeparatorChar));
        }


        unsafe static void CollectContentPaths(string projectPath, out string workingPath,
            out List<string> shaderPaths,
            out List<ContentBuildInput> contentToBuild)
        {
            projectPath = Path.GetFullPath(projectPath);
            workingPath = Path.GetDirectoryName(projectPath);
            shaderPaths = new List<string>();
            contentToBuild = new List<ContentBuildInput>();
            try
            {
                using (var stream = new StreamReader(File.OpenRead(projectPath)))
                {
                    string line;
                    while ((line = stream.ReadLine()) != null)
                    {
                        line = line.Trim();
                        if (line.Length > 0)
                        {
                            var path = Path.Combine(workingPath, line);
                            if (File.Exists(path))
                            {
                                var extension = Path.GetExtension(line);
                                switch (extension)
                                {
                                    case ".hlsl":
                                        shaderPaths.Add(path);
                                        break;
                                    case ".ttf":
                                    case ".otf":
                                        contentToBuild.Add(new ContentBuildInput { Path = path, Type = ContentType.Font });
                                        break;
                                    case ".obj":
                                        contentToBuild.Add(new ContentBuildInput { Path = path, Type = ContentType.Mesh });
                                        break;
                                }
                            }
                            else
                            {
                                Console.WriteLine($"{projectPath}: error: Content list contains path to nonexistent file: {line}");
                            }
                        }
                    }
                }
            }
            catch (Exception e)
            {
                Console.WriteLine($"{projectPath}: error: Content list read exception: {e.Message}");
            }
        }


        public static void Main(string[] args)
        {
            bool debug = false;
            int optimizationLevel = 3;
            var targetPaths = new List<string>();
            for (int i = 0; i < args.Length; ++i)
            {
                //The argument should be either a compilation flag or a project path
                if (args[i].Length > 0 && args[i][0] == '-')
                {
                    switch (args[i])
                    {
                        case "-debug":
                            debug = true;
                            break;
                        case "-O0":
                            optimizationLevel = 0;
                            break;
                        case "-O1":
                            optimizationLevel = 1;
                            break;
                        case "-O2":
                            optimizationLevel = 2;
                            break;
                        case "-O3":
                            optimizationLevel = 3;
                            break;
                    }
                }
                else
                {
                    if (File.Exists(args[i]))
                    {
                        targetPaths.Add(args[i]);
                    }
                    else
                    {
                        Console.WriteLine($"No file exists for argument \"{args[i]}\".");
                    }
                }
            }
            foreach (var targetPath in targetPaths)
            {
                CollectContentPaths(targetPath, out var workingPath, out var shaderPaths, out var contentPaths);
                var cachePathStart = Path.Combine(workingPath, Path.GetFileNameWithoutExtension(targetPath));
                //Shaders are stored a little differently than the rest of content. This is partially for legacy reasons.
                //You could make the argument for bundling them together, but shaders do have some unique macro and dependency management that other kinds of content lack.

                ShaderCompiler.Compile(workingPath,
                    cachePathStart + ".shaderbuildcache",
                    cachePathStart + ".shaderarchive", shaderPaths.ToArray(), out var shaderWarnings, out var shaderErrors, debug: debug, optimizationLevel: optimizationLevel);
                foreach (var error in shaderErrors)
                {
                    Console.WriteLine($"{error.File} ({error.LineNumber},{error.ColumnNumber}): error {error.Code}: {error.Message}");
                }
                foreach (var warning in shaderWarnings)
                {
                    Console.WriteLine($"{warning.File} ({warning.LineNumber},{warning.ColumnNumber}): warning {warning.Code}: {warning.Message}");
                }
                ContentBuilder.BuildContent(workingPath,
                    cachePathStart + ".contentbuildcache",
                    cachePathStart + ".contentarchive", contentPaths, out var contentWarnings, out var contentErrors);
                foreach (var error in contentErrors)
                {
                    Console.WriteLine($"{error.File}: error: {error.Message}");
                }
                foreach (var warning in contentWarnings)
                {
                    Console.WriteLine($"{warning.File}: warning: {warning.Message}");
                }
            }
        }
    }
}
