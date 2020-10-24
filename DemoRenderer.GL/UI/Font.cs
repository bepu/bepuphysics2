using DemoContentLoader;
using System;
using System.Collections.Generic;
using System.Numerics;
using OpenTK.Graphics.OpenGL4;

namespace DemoRenderer.UI
{
    /// <summary>
    /// Location of a glyph in the atlas.
    /// </summary>
    public struct GlyphSource
    {
        public Vector2 Minimum;
        public int PackedSpan;  //Lower 16 bits X, upper 16 bits Y. In texels.
        public float DistanceScale;
    }
    /// <summary>
    /// Runtime type containing GPU-related information necessary to render a specific font type.
    /// </summary>
    public class Font : Disposable
    {
        public readonly StructuredBuffer<GlyphSource> Sources;
        public readonly int Atlas = GL.GenTexture();
        public readonly FontContent Content;

        //Technically you could establish the char-source relationship within the font content itself, and that would eliminate one dictionary lookup.
        //However, source ids don't really exist outside of the runtime type, and establishing a consistent order for them would require a little more complexity.
        //Just doing it here is a little simpler. You can change this up if glyph setup is somehow ever a performance concern.
        private readonly Dictionary<char, int> sourceIds = new Dictionary<char, int>();

        public unsafe Font(FontContent font)
        {
            Content = font;
            Sources = new StructuredBuffer<GlyphSource>(BufferTarget.ShaderStorageBuffer, font.Characters.Count, font.Name + " Glyph Sources");

            GL.BindTexture(TextureTarget.Texture2D, Atlas);
            var data = font.Atlas.Pin();
            for (int mipLevel = 0; mipLevel < font.Atlas.MipLevels; ++mipLevel)
            {
                GL.TexImage2D(TextureTarget.Texture2D, mipLevel, PixelInternalFormat.R8Snorm, font.Atlas.Width >> mipLevel, font.Atlas.Height >> mipLevel, 0, PixelFormat.Red, PixelType.Byte, new IntPtr(data + font.Atlas.GetMipStartIndex(mipLevel)));
            }
            font.Atlas.Unpin();
            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMaxLevel, font.Atlas.MipLevels - 1);
            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, (int)TextureMinFilter.LinearMipmapLinear);
            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, (int)TextureMagFilter.Linear);
            GL.BindTexture(TextureTarget.Texture2D, 0);

            int nextSourceId = 0;
            var sourcesData = new GlyphSource[font.Characters.Count];
            foreach (var character in font.Characters)
            {
                sourceIds.Add(character.Key, nextSourceId);
                sourcesData[nextSourceId] = new GlyphSource {
                    Minimum = new Vector2(character.Value.SourceMinimum.X, character.Value.SourceMinimum.Y),
                    PackedSpan = character.Value.SourceSpan.X | (character.Value.SourceSpan.Y << 16),
                    DistanceScale = character.Value.DistanceScale
                };
                ++nextSourceId;
            }
            Sources.Update(sourcesData);
        }

        public int GetSourceId(char character) =>
            sourceIds.TryGetValue(character, out var sourceId) ? sourceId : -1;

        protected override void DoDispose()
        {
            Sources.Dispose();
            GL.DeleteTexture(Atlas);
        }
    }
}
