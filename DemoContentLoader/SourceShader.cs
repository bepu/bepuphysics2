using System;
using System.IO;
using System.Linq;
using System.Text;

namespace DemoContentLoader
{
    /// <summary>
    /// A single shader along with any defines it was compiled with.
    /// </summary>
    public struct SourceShader : IEquatable<SourceShader>
    {
        public string Name;
        public string[] Defines;


        public static void Write(BinaryWriter writer, SourceShader sourceShader)
        {
            writer.Write(sourceShader.Name);
            writer.Write(sourceShader.Defines.Length);
            for (int i = 0; i < sourceShader.Defines.Length; ++i)
            {
                writer.Write(sourceShader.Defines[i]);
            }
        }

        public static SourceShader Read(BinaryReader reader)
        {
            var name = reader.ReadString();
            var defineCount = reader.ReadInt32();
            var defines = new string[defineCount];
            for (int i = 0; i < defineCount; ++i)
            {
                defines[i] = reader.ReadString();
            }
            return new SourceShader { Name = name, Defines = defines };
        }


        public bool Equals(SourceShader other)
        {
            if (Name != other.Name)
                return false;

            bool localHasDefines = Defines != null && Defines.Length > 0;
            bool otherHasDefines = other.Defines != null && other.Defines.Length > 0;
            if (localHasDefines && otherHasDefines)
            {
                if (Defines.Length != other.Defines.Length)
                    return false;
                for (int i = 0; i < Defines.Length; ++i)
                {
                    if (!other.Defines.Contains(Defines[i]))
                        return false;
                }
            }
            else
            {
                //At least one of the two has no defines, but not both. Both must have no defines to be equal; if one has defines, they're not equal.
                if (localHasDefines ^ otherHasDefines)
                {
                    return false;
                }
            }
            return true;
        }
        public override bool Equals(object obj)
        {
            return Equals((SourceShader)obj);
        }

        public override int GetHashCode()
        {
            int hash = Name.GetHashCode();
            if (Defines != null)
            {
                for (int i = 0; i < Defines.Length; ++i)
                {
                    hash ^= Defines[i].GetHashCode();
                }
            }
            return hash;
        }

        public override string ToString()
        {
            if (Defines == null || Defines.Length == 0)
                return $"{Name}";
            StringBuilder builder = new StringBuilder();
            builder.Append($"{Name}: ");
            for (int i = 0; i < Defines.Length - 1; ++i)
            {
                builder.Append($"{Defines[i]}, ");
            }
            builder.Append($"{Defines[Defines.Length - 1]}");
            return builder.ToString();
        }
    }
}
