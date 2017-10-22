using BepuUtilities;
using DemoContentLoader;
using System;
using System.Collections.Generic;
using System.Numerics;

namespace DemoContentLoader
{
    public struct CharacterData
    {
        /// <summary>
        /// Minimum position of a character glyph in the font distance atlas, measured in atlas texels.
        /// (0,0) corresponds to the upper-left corner of the atlas, not the center of the upper left texel.
        /// </summary>
        public Int2 SourceMinimum;
        /// <summary>
        /// Width and height of the glyph in the font distance atlas, measured in atlas texels.
        /// </summary>
        public Int2 SourceSpan;
        /// <summary>
        /// Offset from a starting pen position to the upper left corner of a glyph's target render position in atlas texels.
        /// </summary>
        public Int2 Bearing;
        /// <summary>
        /// Change in horizontal pen position when moving across this character, measured in atlas texels. Does not include any kerning.
        /// </summary>
        public int Advance;
        /// <summary>
        /// The scaling factor to apply to the distance sampled from the glyph's texture data to convert it to texel units.
        /// </summary>
        public float DistanceScale;
    }

    public struct CharacterPair : IEquatable<CharacterPair>
    {
        public readonly char A;
        public readonly char B;

        public CharacterPair(char a, char b)
        {
            A = a;
            B = b;
        }

        //Order matters!
        public bool Equals(CharacterPair other)
        {
            return A == other.A && B == other.B;
        }

        public override int GetHashCode()
        {
            return (A * 7919) ^ (B * 6263);
        }

        public override string ToString()
        {
            return "{" + A + ", " + B + "}";
        }
    }
    public class FontContent : IContent
    {
        public int GlyphCount { get; private set; }
        public Texture2DContent Atlas { get; private set; }
        public string Name { get; private set; }
        public float InverseSizeInTexels { get; private set; }
        public Dictionary<char, CharacterData> Characters { get; private set; }

        public ContentType ContentType {  get { return ContentType.Font; } }
        
        internal Dictionary<CharacterPair, int> kerning;

        public FontContent(Texture2DContent atlas, string name, float inverseSizeInTexels,
            Dictionary<char, CharacterData> characterData, Dictionary<CharacterPair, int> kerningTable)
        {
            GlyphCount = GlyphCount;
            Atlas = atlas;
            Name = name;
            InverseSizeInTexels = inverseSizeInTexels;
            Characters = characterData;
            kerning = kerningTable;
        }
        
        public int GetKerningInTexels(char a, char b)
        {
            if (kerning.TryGetValue(new CharacterPair(a, b), out var pairKerning))
            {
                return pairKerning;
            }
            return 0;
        }
    }
}
