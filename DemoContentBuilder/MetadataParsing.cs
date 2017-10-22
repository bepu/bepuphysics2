using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DemoContentBuilder
{
    public enum ShaderType
    {
        VS = 0,
        GS = 1,
        HS = 2,
        DS = 3,
        PS = 4,
        CS = 5
    }

    public struct ShaderStage
    {
        public ShaderType Type;
        public string EntryPoint;
        public string Profile;
        public string Extension;
    }
    public struct StageProperties
    {
        public string DefaultEntryPoint;
        public string Profile;
        public string Extension;
    }

    class MetadataParsing
    {
        public static StageProperties[] Stages = new[]
            {
                new StageProperties {DefaultEntryPoint = "VSMain", Profile = "vs_5_0", Extension = ".vshader"},
                new StageProperties {DefaultEntryPoint = "GSMain", Profile = "gs_5_0", Extension = ".gshader"},
                new StageProperties {DefaultEntryPoint = "HSMain", Profile = "hs_5_0", Extension = ".hshader"},
                new StageProperties {DefaultEntryPoint = "DSMain", Profile = "ds_5_0", Extension = ".dshader"},
                new StageProperties {DefaultEntryPoint = "PSMain", Profile = "ps_5_0", Extension = ".pshader"},
                new StageProperties {DefaultEntryPoint = "CSMain", Profile = "cs_5_0", Extension = ".cshader"},
            };

        internal static bool Parse(string path, string text,
            List<ShaderStage> stages,
            List<MacroGroup> macroGroups,
            List<MetadataParsingError> parsingErrors)
        {
            text = text.Trim();
            using (StringReader reader = new StringReader(text))
            {
                //Check for metadata.
                var line = reader.ReadLine();
                if (line.Contains("/*[META]"))
                {
                    while (true)
                    {
                        line = reader.ReadLine();
                        if (line == null)
                        {
                            //We reached the end of the file before finding a metadata terminator.
                            parsingErrors.Add(new MetadataParsingError
                            {
                                Path = path,
                                Message = "Metadata does not have a closing tag- ensure that [META]*/ appears at the end of the metadata block."
                            });
                            return false;
                        }
                        line = line.Trim();
                        if (line == "[META]*/")
                            break;
                        string defineString = "define";
                        if (line.StartsWith(defineString))
                        {
                            var open = line.IndexOf('{');
                            var close = line.IndexOf('}', open + 1);
                            if (open == -1 || close == -1)
                            {
                                parsingErrors.Add(new MetadataParsingError
                                {
                                    Path = path,
                                    Message = "Metadata define has no following definition permutations. Are you missing {brackets?}"
                                });
                                return false;
                            }
                            var values = line.Substring(open + 1, close - open - 1).Split(',');
                            for (int i = 0; i < values.Length; ++i)
                            {
                                values[i] = values[i].Trim(); //This could be more efficient. Oh well!
                            }
                            macroGroups.Add(new MacroGroup
                            {
                                Names = values
                            });
                        }
                        else if (line.StartsWith("vs"))
                        {
                            //Vertex shader.
                            var entryPoint = line.Substring(2).Trim();
                            var stage = Stages[(int)ShaderType.VS];
                            stages.Add(new ShaderStage { EntryPoint = entryPoint.Length > 0 ? entryPoint : stage.DefaultEntryPoint, Extension = stage.Extension, Profile = stage.Profile, Type = ShaderType.VS });
                        }
                        else if (line.StartsWith("gs"))
                        {
                            //Geometry shader.
                            var entryPoint = line.Substring(2).Trim();
                            var stage = Stages[(int)ShaderType.GS];
                            stages.Add(new ShaderStage { EntryPoint = entryPoint.Length > 0 ? entryPoint : stage.DefaultEntryPoint, Extension = stage.Extension, Profile = stage.Profile, Type = ShaderType.GS });
                        }
                        else if (line.StartsWith("hs"))
                        {
                            //Hull shader.
                            var entryPoint = line.Substring(2).Trim();
                            var stage = Stages[(int)ShaderType.HS];
                            stages.Add(new ShaderStage { EntryPoint = entryPoint.Length > 0 ? entryPoint : stage.DefaultEntryPoint, Extension = stage.Extension, Profile = stage.Profile, Type = ShaderType.HS });
                        }
                        else if (line.StartsWith("ds"))
                        {
                            //Domain shader.
                            var entryPoint = line.Substring(2).Trim();
                            var stage = Stages[(int)ShaderType.DS];
                            stages.Add(new ShaderStage { EntryPoint = entryPoint.Length > 0 ? entryPoint : stage.DefaultEntryPoint, Extension = stage.Extension, Profile = stage.Profile, Type = ShaderType.DS });
                        }
                        else if (line.StartsWith("ps"))
                        {
                            //Pixel shader.
                            var entryPoint = line.Substring(2).Trim();
                            var stage = Stages[(int)ShaderType.PS];
                            stages.Add(new ShaderStage { EntryPoint = entryPoint.Length > 0 ? entryPoint : stage.DefaultEntryPoint, Extension = stage.Extension, Profile = stage.Profile, Type = ShaderType.PS });
                        }
                        else if (line.StartsWith("cs"))
                        {
                            //Compute shader.
                            var entryPoint = line.Substring(2).Trim();
                            var stage = Stages[(int)ShaderType.CS];
                            stages.Add(new ShaderStage { EntryPoint = entryPoint.Length > 0 ? entryPoint : stage.DefaultEntryPoint, Extension = stage.Extension, Profile = stage.Profile, Type = ShaderType.CS });
                        }
                    }
                }
            }
            return true;
        }
    }
}
