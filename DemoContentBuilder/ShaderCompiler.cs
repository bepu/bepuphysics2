using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Text;
using System.Text.RegularExpressions;
using SharpDX;
using SharpDX.D3DCompiler;
using SharpDX.Direct3D;
using System.Threading.Tasks;
using DemoContentLoader;
using System.Threading;

namespace DemoContentBuilder
{
    public struct ShaderCompilationResult
    {
        public readonly string Subcategory;
        public readonly string Code;
        public readonly string HelpKeyword;
        public readonly string File;
        public readonly int LineNumber;
        public readonly int ColumnNumber;
        public readonly int EndLineNumber;
        public readonly int EndColumnNumber;
        public readonly string Message;

        public ShaderCompilationResult(string subcategory, string code, string helpKeyword, string file, int lineNumber, int columnNumber, int endLineNumber, int endColumnNumber, string message)
        {
            Subcategory = subcategory;
            Code = code;
            HelpKeyword = helpKeyword;
            File = file;
            LineNumber = lineNumber;
            ColumnNumber = columnNumber;
            EndLineNumber = endLineNumber;
            EndColumnNumber = endColumnNumber;
            Message = message;
        }

    }
    public struct ShaderCompilationTarget
    {
        public string Path;
        public ShaderStage Stage;
        public ShaderMacro[] ShaderMacros;
        public long CurrentTimeStamp;

        public override string ToString()
        {
            var builder = new StringBuilder();
            builder.Append(Path);
            builder.Append(Stage.Extension);
            if (ShaderMacros.Length > 0)
            {
                builder.Append(": ");
                for (int i = 0; i < ShaderMacros.Length; ++i)
                {
                    builder.Append(ShaderMacros[i].Name);
                    if (i < ShaderMacros.Length - 1)
                    {
                        builder.Append(",");
                    }
                }
            }
            return builder.ToString();
        }
    }

    /// <summary>
    /// Takes raw shader sources and turns them into bytecode, reporting any errors and warnings encountered.
    /// </summary>
    public static class ShaderCompiler
    {
        public static bool ParseCompilerResult(string error, out string filePath, out int lineBegin, out int columnBegin, out int lineEnd, out int columnEnd, out string description)
        {
            //TODO: This error parsing is not particularly robust- quite a few errors will confuse it, 
            //and I've never bothered to fix it. But hey if you want to wink wink nudge nudge ;)            

            //There are four possible forms for the error to take:
            //(x,y)
            //(x1-x2,y)
            //(x,y1-y2)
            //(x1-x2,y1-y2)

            //We'll try each in sequence rather than trying to construct a regex which captures all of them at once because OUGH.
            //(x,y)
            var errorRegex = new Regex(@"(?<filePath>.*)\((?<line>[0-9]+),(?<column>[0-9]+)\): (?<description>.*)");
            var m = errorRegex.Match(error);
            if (m.Success)
            {
                var lineString = m.Groups["line"].Value;
                var columnString = m.Groups["column"].Value;
                filePath = m.Groups["filePath"].Value;
                description = m.Groups["description"].Value;
                if (int.TryParse(lineString, out lineBegin) && int.TryParse(columnString, out columnBegin))
                {
                    lineEnd = lineBegin;
                    columnEnd = columnBegin;
                    return true;
                }
            }
            else
            {
                //(x1-x2,y)
                errorRegex = new Regex(@"(?<filePath>.*)\((?<lineBegin>[0-9]+)\-(?<lineEnd>[0-9]+),(?<column>[0-9]+)\): (?<description>.*)");
                m = errorRegex.Match(error);
                if (m.Success)
                {

                    var lineBeginString = m.Groups["lineBegin"].Value;
                    var lineEndString = m.Groups["lineEnd"].Value;
                    var columnString = m.Groups["column"].Value;
                    filePath = m.Groups["filePath"].Value;
                    description = m.Groups["description"].Value;
                    if (int.TryParse(lineBeginString, out lineBegin) && int.TryParse(lineEndString, out lineEnd) && int.TryParse(columnString, out columnBegin))
                    {
                        columnEnd = columnBegin;
                        return true;
                    }
                }
                else
                {
                    //(x,y1-y2)
                    errorRegex = new Regex(@"(?<filePath>.*)\((?<line>[0-9]+),(?<columnBegin>[0-9]+)\-(?<columnEnd>[0-9]+)\): (?<description>.*)");
                    m = errorRegex.Match(error);

                    if (m.Success)
                    {
                        var lineString = m.Groups["line"].Value;
                        var columnBeginString = m.Groups["columnBegin"].Value;
                        var columnEndString = m.Groups["columnEnd"].Value;
                        filePath = m.Groups["filePath"].Value;
                        description = m.Groups["description"].Value;

                        if (int.TryParse(lineString, out lineBegin) && int.TryParse(columnBeginString, out columnBegin) && int.TryParse(columnEndString, out columnEnd))
                        {
                            lineEnd = lineBegin;
                            return true;
                        }
                    }
                    else
                    {
                        //(x1-x2,y1-y2)
                        errorRegex = new Regex(@"(?<filePath>.*)\((?<lineBegin>[0-9]+)\-(?<lineEnd>[0-9]+),(?<columnBegin>[0-9]+)\-(?<columnEnd>[0-9]+)\): (?<description>.*)");
                        m = errorRegex.Match(error);

                        if (m.Success)
                        {
                            var lineBeginString = m.Groups["lineBegin"].Value;
                            var lineEndString = m.Groups["lineEnd"].Value;
                            var columnBeginString = m.Groups["columnBegin"].Value;
                            var columnEndString = m.Groups["columnEnd"].Value;
                            filePath = m.Groups["filePath"].Value;
                            description = m.Groups["description"].Value;

                            if (int.TryParse(lineBeginString, out lineBegin) && int.TryParse(lineEndString, out lineEnd) &&
                                int.TryParse(columnBeginString, out columnBegin) && int.TryParse(columnEndString, out columnEnd))
                            {
                                return true;
                            }
                        }
                    }
                }
            }

            filePath = null;
            description = null;
            lineBegin = lineEnd = columnBegin = columnEnd = 0;
            return false;
        }


        private static string GetRootedPath(string filePath, string workingPath, string localWorkingPath)
        {
            //Make sure that the path is a full path. Includes will have relative paths.
            if (!Path.IsPathRooted(filePath))
            {
                //Try the local version first.
                var fullPath = Path.GetFullPath(localWorkingPath + Path.DirectorySeparatorChar + filePath);
                if (!File.Exists(fullPath))
                {
                    fullPath = Path.GetFullPath(workingPath + Path.DirectorySeparatorChar + filePath);
                }
                return fullPath;
            }
            return filePath;
        }

        private static ShaderMacro[][] GetShaderMacroPermutations(List<MacroGroup> macroGroups)
        {
            if (macroGroups.Count == 0)
            {
                var emptyPermutations = new ShaderMacro[1][];
                emptyPermutations[0] = new ShaderMacro[] { };
                return emptyPermutations;
            }

            int permutationCount = macroGroups[0].Names.Length;
            for (int i = 1; i < macroGroups.Count; ++i)
            {
                permutationCount *= macroGroups[i].Names.Length;
            }
            var permutations = new ShaderMacro[permutationCount][];
            for (int permutationIndex = 0; permutationIndex < permutationCount; ++permutationIndex)
            {
                var permutation = permutations[permutationIndex] = new ShaderMacro[macroGroups.Count];
                int index = permutationIndex;
                for (int i = 0; i < macroGroups.Count; ++i)
                {
                    var defineCount = macroGroups[i].Names.Length;
                    var newIndex = index / defineCount;
                    var nameIndex = index - defineCount * newIndex;
                    permutation[i] = new ShaderMacro { Name = macroGroups[i].Names[nameIndex] };
                    index = newIndex;
                }
            }
            return permutations;

        }

        private static void CollectCompilationTargets(string source, string workingPath,
            ShaderFileCache shaderFileCache, ShaderCompilationCache loadedCache, ShaderCompilationCache cache,
            List<ShaderCompilationResult> errors, List<ShaderCompilationTarget> compilationTargets)
        {
            if (!shaderFileCache.TryLoad(source, out string shaderCode))
            {
                lock (errors)
                {
                    errors.Add(new ShaderCompilationResult("Shader", "", "", source, 0, 0, 0, 0, "Could not find file " + source));
                }
                return;
            }


            //Only compile this shader if the shader has been modified since the previous compile.
            var currentTimeStamp = File.GetLastWriteTime(source).Ticks;
            if (loadedCache.ShaderFlags == cache.ShaderFlags && loadedCache.TimeStamps.TryGetValue(source, out long previousTimeStamp)) //If this file was contained before, we MIGHT be able to skip its compilation.
            {
                if (currentTimeStamp <= previousTimeStamp) //If the file hasn't been updated, we MIGHT be able to skip.
                {
                    //Check the timestamps associated with the dependencies of this file.
                    //If any are newer than the previous snapshot, skip.
                    bool allowSkip = true;
                    if (loadedCache.Dependencies.TryGetValue(source, out HashSet<string> loadedDependencyPaths))
                    {
                        foreach (var dependency in loadedDependencyPaths)
                        {
                            //The loadedCache is guaranteed to contain a time stamp for any dependency referenced,
                            //because the only time any dependency is added to the list the dependency is ALSO put into the timestamps list.
                            var previousDependencyTimeStamp = loadedCache.TimeStamps[dependency];
                            var currentDependencyTimeStamp = File.GetLastWriteTime(dependency).Ticks;
                            if (currentDependencyTimeStamp > previousDependencyTimeStamp)
                            {
                                //One of the dependencies has been updated. This shader must be compiled.
                                allowSkip = false;
                                break;
                            }
                        }
                    }
                    else
                    {
                        //One of the dependencies could not be found in the old cache. Implies a new dependency was added; must compile.
                        allowSkip = false;
                    }

                    if (allowSkip)
                    {
                        Console.WriteLine($"Shader up to date: {Path.GetFileName(source)}");
                        cache.CopyFrom(source, loadedCache);
                        return;
                    }
                }
            }


            var localWorkingPath = Path.GetDirectoryName(source);
            var include = new IncludeHandler(shaderFileCache, workingPath, localWorkingPath);


            var stages = new List<ShaderStage>();
            var macroGroups = new List<MacroGroup>();
            var metadataParsingErrors = new List<MetadataParsingError>();
            MetadataParsing.Parse(source, shaderCode, stages, macroGroups, metadataParsingErrors);
            //Prepass to collect include metadata. Seems hacky, oh well.
            try
            {
                ShaderBytecode.Preprocess(shaderCode, null, include);
            }
            catch (CompilationException e)
            {
                if (ParseCompilerResult(e.Message, out string filePath, out int lineBegin, out int columnBegin, out int lineEnd, out int columnEnd, out string description))
                {
                    var errorChecker = new Regex("error X(?<errorCode>[0-9]{4}): ");
                    var errorCode = errorChecker.Match(e.Message).Groups["errorCode"].Value;
                    lock (errors)
                    {
                        errors.Add(new ShaderCompilationResult("Shader", errorCode, "", source, lineBegin, columnBegin, lineEnd, columnEnd, description));
                    }
                }
                else
                {
                    lock (errors)
                    {
                        //This error does not match the regular error pattern, but it should still be caught!
                        errors.Add(new ShaderCompilationResult("Shader", "", "", source, 0, 0, 0, 0,
                            "Preprocessing error: " + e.Message));
                    }
                }
                return;
            }
            stages.AddRange(include.Stages);
            macroGroups.AddRange(include.MacroGroups);
            //Remove duplicates.
            for (int i = stages.Count - 2; i >= 0; --i)
            {
                var index = stages.IndexOf(stages[i], i + 1);
                if (index != -1)
                {
                    stages.RemoveAt(i);
                }
            }
            for (int i = macroGroups.Count - 2; i >= 0; --i)
            {
                var index = macroGroups.IndexOf(macroGroups[i], i + 1);
                if (index != -1)
                {
                    macroGroups.RemoveAt(i);
                }
            }
            metadataParsingErrors.AddRange(include.IncludeParsingErrors);
            var dependencyPaths = include.DependencyPaths;
            var shaderMacroPermutations = GetShaderMacroPermutations(macroGroups);

            include.Dispose();


            if (metadataParsingErrors.Count > 0)
            {
                foreach (var error in metadataParsingErrors)
                {
                    //Make sure that the path is a full path. Includes will have relative paths.
                    var path = GetRootedPath(error.Path, workingPath, localWorkingPath);

                    lock (errors)
                        errors.Add(new ShaderCompilationResult("Shader", "", "", path, 0, 0, 0, 0, error.Message));
                }
                //No point in continuing the compilation if our metadata, which drives the compilation, is flawed.
                return;
            }

            lock (compilationTargets)
            {
                foreach (var stage in stages)
                {
                    foreach (var shaderMacros in shaderMacroPermutations)
                    {
                        compilationTargets.Add(new ShaderCompilationTarget
                        {
                            Path = source,
                            ShaderMacros = shaderMacros,
                            Stage = stage,
                            CurrentTimeStamp = currentTimeStamp
                        });
                    }
                }
            }
        }

        private static void CompileShaderTarget(ShaderCompilationTarget compilationTarget, string workingPath, ShaderFlags shaderFlags,
            ShaderFileCache shaderFileCache, ShaderCompilationCache loadedCache, ShaderCompilationCache cache,
            List<ShaderCompilationResult> warnings, List<ShaderCompilationResult> errors)
        {
            var permutationErrors = new List<ShaderCompilationResult>();
            var permutationWarnings = new List<ShaderCompilationResult>();
            var shaderCompileStartTime = Stopwatch.GetTimestamp();
            var localWorkingPath = Path.GetDirectoryName(compilationTarget.Path);
            var include = new IncludeHandler(shaderFileCache, workingPath, localWorkingPath);
            shaderFileCache.TryLoad(compilationTarget.Path, out string shaderCode);
            CompilationResult result;

            result = ShaderBytecode.Compile(shaderCode, compilationTarget.Stage.EntryPoint, compilationTarget.Stage.Profile, shaderFlags,
                EffectFlags.None, compilationTarget.ShaderMacros, include, compilationTarget.Path);


            //Note that dependency paths can vary between defines (so long as the new defines aren't expected to vary define permutations themselves),
            //so we can't trust the ones identified by the preprocessor.
            var dependencyPaths = include.DependencyPaths;
            include.Dispose();

            var fullSourcePath = Path.GetFullPath(compilationTarget.Path);
            if (result.Message != null)
            {
                var fileErrors = result.Message.Trim().Split('\n');

                var errorChecker = new Regex("error X(?<errorCode>[0-9]{4}): ");
                foreach (var error in fileErrors)
                {
                    if (!ParseCompilerResult(error, out string filePath, out int lineBegin, out int columnBegin, out int lineEnd, out int columnEnd, out string description))
                    {
                        //This error does not match the regular error pattern, but it should still be caught!
                        permutationErrors.Add(new ShaderCompilationResult("Shader", "", "", compilationTarget.Path, 0, 0, 0, 0,
                            "UNEXPECTED ERROR: " + error));
                        continue;
                    }


                    filePath = GetRootedPath(filePath, workingPath, localWorkingPath);

                    //Is this a warning, or an error?
                    var match = errorChecker.Match(description);
                    if (match.Success)
                    {

                        //It's an error!
                        var errorCode = errorChecker.Match(error).Groups["errorCode"].Value;
                        permutationErrors.Add(new ShaderCompilationResult("Shader", errorCode, "", filePath, lineBegin, columnBegin, lineEnd, columnEnd, description));

                    }
                    else
                    {
                        //It's a warning!
                        var warningChecker = new Regex("warning X(?<errorCode>[0-9]{4}): ");
                        match = warningChecker.Match(description);

                        var errorCode = match.Groups["errorCode"].Value;
                        permutationWarnings.Add(new ShaderCompilationResult("Shader", errorCode, "", filePath, lineBegin, columnBegin, lineEnd, columnEnd, description));

                    }


                }
            }

            if (permutationErrors.Count == 0)
            {
                //Place the compiled result into the successfully compiled list.                            
                cache.AddCompiledShader(compilationTarget, dependencyPaths, result);
                Console.WriteLine($"Compiled {compilationTarget}");
            }
            else
            {
                Console.WriteLine($"Failed to compile {compilationTarget}");
                lock (errors)
                    errors.AddRange(permutationErrors);
            }
            lock (warnings)
                warnings.AddRange(permutationWarnings);
            long shaderCompileEndTime = Stopwatch.GetTimestamp();
        }

        public static void Compile(string workingPath, string compilationCachePath, string runtimeCachePath, string[] sources,
            out List<ShaderCompilationResult> outWarnings, out List<ShaderCompilationResult> outErrors,
            bool debug = false, bool packMatrixRowMajor = false, int optimizationLevel = 3)
        {
            var shaderFlags = new ShaderFlags();
            if (debug)
                shaderFlags |= ShaderFlags.Debug | ShaderFlags.SkipOptimization;
            if (packMatrixRowMajor)
                shaderFlags |= ShaderFlags.PackMatrixRowMajor;
            else
                shaderFlags |= ShaderFlags.PackMatrixColumnMajor;
            switch (optimizationLevel)
            {
                case 0:
                    shaderFlags |= ShaderFlags.OptimizationLevel0;
                    break;
                case 1:
                    shaderFlags |= ShaderFlags.OptimizationLevel1;
                    break;
                case 2:
                    shaderFlags |= ShaderFlags.OptimizationLevel2;
                    break;
                default:
                    shaderFlags |= ShaderFlags.OptimizationLevel3;
                    break;
            }

            long totalStartTime = Stopwatch.GetTimestamp();

            //Load any preexisting compilation cache to compare against.
            if (!ShaderCompilationCache.TryLoad(compilationCachePath, out ShaderCompilationCache loadedCache))
            {
                loadedCache = new ShaderCompilationCache(shaderFlags);
            }
            var cache = new ShaderCompilationCache(shaderFlags);
            Configuration.ThrowOnShaderCompileError = false;


            //Collect the set of compilation targets from the sources.
            var compilationTargets = new List<ShaderCompilationTarget>();
            var shaderFileCache = new ShaderFileCache();
            var warnings = outWarnings = new List<ShaderCompilationResult>();
            var errors = outErrors = new List<ShaderCompilationResult>();
            Parallel.ForEach(sources, source =>
            {
                CollectCompilationTargets(source, workingPath, shaderFileCache, loadedCache, cache, errors, compilationTargets);
            });
            Parallel.ForEach(compilationTargets, compilationTarget =>
            {
                CompileShaderTarget(compilationTarget, workingPath, shaderFlags, shaderFileCache, loadedCache, cache, warnings, errors);
            });

            if (compilationTargets.Count > 0)
            {
                //Something was compiled; prepare to save stuff.   
                var prunedShaders = new Dictionary<SourceShader, byte[]>();
                foreach (var pathShaderPair in cache.CompiledShaders)
                {
                    //Prune out all of the extra path bits and save it.
                    var relativePath = ProjectBuilder.GetRelativePathFromDirectory(pathShaderPair.Key.Name, workingPath);
                    prunedShaders.Add(new SourceShader { Name = relativePath, Defines = pathShaderPair.Key.Defines }, pathShaderPair.Value.Data);
                }

                const int retryCount = 10;
                const int retryDelay = 200;
                for (int i = 0; i < retryCount; ++i)
                {
                    try
                    {
                        ShaderCompilationCache.Save(cache, compilationCachePath);
                        using (var stream = File.OpenWrite(runtimeCachePath))
                        {
                            ShaderCache.Save(prunedShaders, stream);
                        }
                        break;
                    }
                    catch (IOException e)
                    {
                        Console.WriteLine($"Failed to write shader cache (attempt {i}): {e.Message}, retrying...");
                        Thread.Sleep(retryDelay);
                    }
                }
            }

        }

    }
}
