using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace DemoContentLoader
{
    public class FontIO
    {
        public static FontContent Load(BinaryReader reader)
        {
            var name = reader.ReadString();
            var inverseSizeInTexels = reader.ReadSingle();
            var glyphCount = reader.ReadInt32();
            var characterData = new Dictionary<char, CharacterData>();
            for (int i = 0; i < glyphCount; ++i)
            {
                var character = reader.ReadChar();
                CharacterData data;
                data.SourceMinimum.X = reader.ReadInt32();
                data.SourceMinimum.Y = reader.ReadInt32();
                data.SourceSpan.X = reader.ReadInt32();
                data.SourceSpan.Y = reader.ReadInt32();
                data.Bearing.X = reader.ReadInt32();
                data.Bearing.Y = reader.ReadInt32();
                data.Advance = reader.ReadInt32();
                data.DistanceScale = reader.ReadSingle();
                characterData.Add(character, data);
            }
            var kerningRelationshipCount = reader.ReadInt32();
            var kerningTable = new Dictionary<CharacterPair, int>();
            for (int i = 0; i < kerningRelationshipCount; ++i)
            {
                var a = reader.ReadChar();
                var b = reader.ReadChar();
                var amount = reader.ReadInt32();
                kerningTable.Add(new CharacterPair(a, b), amount);
            }
            var atlas = Texture2DIO.Load(reader);
            return new FontContent(atlas, name, inverseSizeInTexels, characterData, kerningTable);
        }

        public static void Save(FontContent content, BinaryWriter writer)
        {
            writer.Write(content.Name);
            writer.Write(content.InverseSizeInTexels);
            writer.Write(content.Characters.Count);
            foreach (var pair in content.Characters)
            {
                writer.Write(pair.Key);
                writer.Write(pair.Value.SourceMinimum.X);
                writer.Write(pair.Value.SourceMinimum.Y);
                writer.Write(pair.Value.SourceSpan.X);
                writer.Write(pair.Value.SourceSpan.Y);
                writer.Write(pair.Value.Bearing.X);
                writer.Write(pair.Value.Bearing.Y);
                writer.Write(pair.Value.Advance);
                writer.Write(pair.Value.DistanceScale);
            }
            writer.Write(content.kerning.Count);
            foreach (var pair in content.kerning)
            {
                writer.Write(pair.Key.A);
                writer.Write(pair.Key.B);
                writer.Write(pair.Value);
            }
            Texture2DIO.Save(content.Atlas, writer);
        }
    }
}