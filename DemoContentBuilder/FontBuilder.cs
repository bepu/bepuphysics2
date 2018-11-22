using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using DemoContentLoader;
using SharpFont;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;

namespace DemoContentBuilder
{
    public static class FontBuilder
    {
        private static Dictionary<CharacterPair, int> ComputeKerningTable(Face face, string characterSet)
        {
            var kerningTable = new Dictionary<CharacterPair, int>();
            for (int i = 0; i < characterSet.Length; ++i)
            {
                var glyphIndex = face.GetCharIndex(characterSet[i]);
                for (int j = 0; j < characterSet.Length; ++j)
                {
                    var kerning = face.GetKerning(glyphIndex, face.GetCharIndex(characterSet[i]), KerningMode.Default);
                    if (kerning.X != 0)
                    {
                        kerningTable.Add(new CharacterPair(characterSet[i], characterSet[j]), kerning.X.ToInt32());
                    }
                }
            }
            return kerningTable;
        }


        private const string characterSet = @"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz1234567890`-=[]\;',./~!@#$%^&*()_+{}|:""<>? ";

        //Text below 128 / 2^(5-1) = 8 pixels is not going to be terribly common, so generating more mips for it has pretty low value.
        //Technically, by outputting at a really high resolution and mipping it down, we miss out on low resolution hinting.
        //This could be addressed by actually rasterizing glyphs at each mip resolution and redoing the distance calculation.
        //But in practice, we'll probably just be using larger glyphs.
        private const int FontSizeInPixels = 128;
        private const int MipLevels = 5;
        private const int AtlasWidth = 2048;

        class CharacterHeightComparer : IComparer<CharacterData>
        {
            public int Compare(CharacterData a, CharacterData b)
            {
                return a.SourceSpan.Y < b.SourceSpan.Y ? 1 : a.SourceSpan.Y > b.SourceSpan.Y ? -1 : 0;
            }
        }

        public unsafe static FontContent Build(Stream fontDataStream)
        {
            var faceBytes = new byte[fontDataStream.Length];
            fontDataStream.Read(faceBytes, 0, faceBytes.Length);
            using (var library = new Library())
            {
                using (var face = new Face(library, faceBytes, 0))
                {

                    //Collect glyph boundings information.
                    face.SetPixelSizes(FontSizeInPixels, FontSizeInPixels);
                    var sortedCharacterSet = new char[characterSet.Length];
                    var sortedCharacterData = new CharacterData[characterSet.Length];

                    for (int i = 0; i < characterSet.Length; ++i)
                    {
                        sortedCharacterSet[i] = characterSet[i];
                        face.LoadGlyph(face.GetCharIndex(characterSet[i]), LoadFlags.Default, LoadTarget.Normal);
                        ref var characterData = ref sortedCharacterData[i];
                        characterData.SourceSpan.X = face.Glyph.Metrics.Width.ToInt32();
                        characterData.SourceSpan.Y = face.Glyph.Metrics.Height.ToInt32();

                        characterData.Bearing.X = face.Glyph.Metrics.HorizontalBearingX.ToInt32();
                        characterData.Bearing.Y = face.Glyph.Metrics.HorizontalBearingY.ToInt32();

                        characterData.Advance = face.Glyph.Metrics.HorizontalAdvance.ToInt32();
                    }

                    //Next, allocate space in the atlas for each character.
                    //Sort the characters by height, and then scan from one side of the atlas to the other placing characters.
                    //Once the other side is reached, flip directions and repeat. Continue until no more characters remain.
                    Array.Sort(sortedCharacterData, sortedCharacterSet, new CharacterHeightComparer());

                    const int padding = 1 << MipLevels;
                    var characters = new Dictionary<char, CharacterData>();
                    var packer = new FontPacker(AtlasWidth, MipLevels, padding, characterSet.Length);
                    for (int i = 0; i < sortedCharacterSet.Length; ++i)
                    {
                        //The packer class handles the placement logic and sets the SourceMinimum in the character data, too.
                        packer.Add(ref sortedCharacterData[i]);
                    }

                    //Now that every glyph has been positioned within the sheet, we can actually rasterize the glyph alphas into a bitmap proto-atlas.
                    //We're building the rasterized set sequentially first so we don't have to worry about threading issues in the underlying library.
                    var rasterizedAlphas = new Texture2DContent(AtlasWidth, packer.Height, 1, 1);
                    for (int i = 0; i < sortedCharacterSet.Length; ++i)
                    {
                        //Rasterize the glyph.
                        var character = sortedCharacterSet[i];
                        face.LoadGlyph(face.GetCharIndex(character), LoadFlags.Default, LoadTarget.Normal);
                        face.Glyph.RenderGlyph(RenderMode.Normal);

                        //Copy the alphas into the pixel alpha buffer at the appropriate position.
                        int glyphWidth = face.Glyph.Bitmap.Width;
                        int glyphHeight = face.Glyph.Bitmap.Rows;
                        var glyphBuffer = (byte*)face.Glyph.Bitmap.Buffer;


                        Int2 location;
                        location.X = sortedCharacterData[i].SourceMinimum.X;
                        location.Y = sortedCharacterData[i].SourceMinimum.Y;
                        for (int glyphRow = 0; glyphRow < glyphHeight; ++glyphRow)
                        {
                            Unsafe.CopyBlockUnaligned(
                                ref rasterizedAlphas.Data[rasterizedAlphas.GetRowOffsetForMip0(glyphRow + location.Y) + location.X],
                                ref glyphBuffer[glyphRow * glyphWidth], (uint)glyphWidth);
                        }
                    }

                    //Preallocate memory for full single precision float version of the atlas. This will be used as scratch memory (admittedly, more than is necessary)
                    //which will be encoded into the final single byte representation after the mips are calculated. The full precision stage makes the mips a little more accurate.
                    var preciseAtlas = new Texture2DContent(AtlasWidth, packer.Height, MipLevels, 4);
                    var atlas = new Texture2DContent(AtlasWidth, packer.Height, MipLevels, 1);
                    //Compute the distances for every character-covered texel in the atlas.
                    var atlasData = atlas.Pin();
                    var preciseData = (float*)preciseAtlas.Pin();
                    var alphaData = rasterizedAlphas.Pin();
                    //for (int i = 0; i < sortedCharacterData.Length; ++i)
                    Parallel.For(0, sortedCharacterData.Length, i =>
                    {
                        //Note that the padding around characters should also have its distances filled in. That way, the less detailed mips can pull from useful data.
                        ref var charData = ref sortedCharacterData[i];

                        var min = new Int2(charData.SourceMinimum.X, charData.SourceMinimum.Y);
                        var max = new Int2(charData.SourceMinimum.X + charData.SourceSpan.X, charData.SourceMinimum.Y + charData.SourceSpan.Y);
                        var paddedMin = new Int2(min.X - padding, min.Y - padding);
                        var paddedMax = new Int2(max.X + padding, max.Y + padding);
                        //Initialize every character texel to max distance. The following BFS only ever reduces distances, so it has to start high.
                        var maxDistance = Math.Max(AtlasWidth, packer.Height);
                        for (int rowIndex = paddedMin.Y; rowIndex < paddedMax.Y; ++rowIndex)
                        {
                            var rowOffset = preciseAtlas.GetRowOffsetForMip0(rowIndex);
                            var distancesRow = preciseData + rowOffset;
                            for (int columnIndex = paddedMin.X; columnIndex < paddedMax.X; ++columnIndex)
                            {
                                distancesRow[columnIndex] = maxDistance;
                            }
                        }


                        //Scan the alphas. Add border texels of the glyph to the point set. We collect both the nonzero alpha outline and the 'negative space' zero alpha outline.
                        //While scanning distances, nonzero alpha texels will look for the shortest distance to a zero alpha texel, while zero alpha texels will look for the shortest
                        //distance to a nonzero alpha texel.
                        var glyphOutline = new List<Int2>((max.X - min.X) * (max.Y - min.Y));
                        int coverageThreshold = 127;
                        for (int rowIndex = min.Y; rowIndex < max.Y; ++rowIndex)
                        {
                            //Alphas and atlas have same dimensions, so sharing row offset is safe.
                            Debug.Assert(padding > 0, "This assumes at least one padding; no boundary checking is performed on the alpha accesses.");
                            var rowOffset = preciseAtlas.GetRowOffsetForMip0(rowIndex);
                            var alphasRow0 = alphaData + rowOffset - preciseAtlas.Width;
                            var alphasRow1 = alphaData + rowOffset;
                            var alphasRow2 = alphaData + rowOffset + preciseAtlas.Width;
                            for (int columnIndex = min.X; columnIndex < max.X; ++columnIndex)
                            {
                                if (alphasRow1[columnIndex] >= coverageThreshold)
                                {
                                    //This texel is considered covered.
                                    //Only add this to the point set if there is at least one adjacent uncovered texel.
                                    //If there isn't an uncovered texel next to this one, then it can't be on the surface.
                                    if (alphasRow0[columnIndex] < coverageThreshold ||
                                        alphasRow1[columnIndex - 1] < coverageThreshold ||
                                        alphasRow1[columnIndex + 1] < coverageThreshold ||
                                        alphasRow2[columnIndex] < coverageThreshold)
                                    {
                                        Int2 texelCoordinates;
                                        texelCoordinates.X = columnIndex;
                                        texelCoordinates.Y = rowIndex;
                                        glyphOutline.Add(texelCoordinates);
                                    }
                                }
                            }
                        }

                        //For every texel in the character's region, scan the glyph point set for the nearest texel.
                        //Cache the largest distance as we go so that we can maximize precision within this character.
                        float largestDistanceMagnitude = 0;
                        for (int rowIndex = paddedMin.Y; rowIndex < paddedMax.Y; ++rowIndex)
                        {
                            var rowOffset = preciseAtlas.GetRowOffsetForMip0(rowIndex); //Same dimensions; can be shared.
                            var distancesRow = preciseData + rowOffset;
                            var alphasRow = alphaData + rowOffset;
                            for (int columnIndex = paddedMin.X; columnIndex < paddedMax.X; ++columnIndex)
                            {
                                //This is an uncovered texel. Look for a glyph outline.
                                float lowestDistance = float.MaxValue;
                                for (int pointIndex = 0; pointIndex < glyphOutline.Count; ++pointIndex)
                                {
                                    var point = glyphOutline[pointIndex];
                                    var offsetX = point.X - columnIndex;
                                    var offsetY = point.Y - rowIndex;
                                    var candidateDistance = (float)Math.Sqrt(offsetX * offsetX + offsetY * offsetY);
                                    if (candidateDistance < lowestDistance)
                                        lowestDistance = candidateDistance;
                                }
                                //If it's uncovered, use a positive distance. If it's covered, use a negative distance.
                                distancesRow[columnIndex] = alphasRow[columnIndex] < coverageThreshold ? lowestDistance : -lowestDistance;
                                if (lowestDistance > largestDistanceMagnitude)
                                    largestDistanceMagnitude = lowestDistance;
                            }
                        }

                        //Build the mips. We already have all the data in cache on this core; 256KiB L2 can easily hold the processing context of a 128x128 glyph.
                        //(Though worrying about performance in the content builder too much is pretty silly. We aren't going to be building fonts often.)
                        //Note that we aligned and padded each glyph during packing. For a given texel in mip(n), the four parent texels in mip(n-1) can be safely sampled.
                        for (int mipLevel = 1; mipLevel < preciseAtlas.MipLevels; ++mipLevel)
                        {
                            var mipMin = new Int2(paddedMin.X >> mipLevel, paddedMin.Y >> mipLevel);
                            var mipMax = new Int2(paddedMax.X >> mipLevel, paddedMax.Y >> mipLevel);

                            //Yes, these do some redundant calculations, but no it doesn't matter.
                            var parentMipStart = preciseData + preciseAtlas.GetMipStartIndex(mipLevel - 1);
                            var parentMipRowPitch = preciseAtlas.GetRowPitch(mipLevel - 1);
                            var mipStart = preciseData + preciseAtlas.GetMipStartIndex(mipLevel);
                            var mipRowPitch = preciseAtlas.GetRowPitch(mipLevel);
                            for (int mipRowIndex = mipMin.Y; mipRowIndex < mipMax.Y; ++mipRowIndex)
                            {
                                var mipRow = mipStart + mipRowIndex * mipRowPitch;
                                var parentRowIndex = mipRowIndex << 1;
                                var parentMipRow0 = parentMipStart + parentRowIndex * parentMipRowPitch;
                                var parentMipRow1 = parentMipStart + (parentRowIndex + 1) * parentMipRowPitch;
                                for (int mipColumnIndex = mipMin.X; mipColumnIndex < mipMax.X; ++mipColumnIndex)
                                {
                                    var parentMipColumnIndex0 = mipColumnIndex << 1;
                                    var parentMipColumnIndex1 = parentMipColumnIndex0 + 1;
                                    mipRow[mipColumnIndex] = 0.25f * (
                                        parentMipRow0[parentMipColumnIndex0] + parentMipRow0[parentMipColumnIndex1] +
                                        parentMipRow1[parentMipColumnIndex0] + parentMipRow1[parentMipColumnIndex1]);
                                }

                            }
                        }

                        //Now that all mips have been filled, bake the data into the final single byte encoding.
                        //Use the largest absolute distance as the encoding multiplier to maximize precision.
                        charData.DistanceScale = largestDistanceMagnitude;
                        var encodingMultiplier = 1f / largestDistanceMagnitude;
                        for (int mipLevel = 0; mipLevel < atlas.MipLevels; ++mipLevel)
                        {
                            var mipMin = new Int2(paddedMin.X >> mipLevel, paddedMin.Y >> mipLevel);
                            var mipMax = new Int2(paddedMax.X >> mipLevel, paddedMax.Y >> mipLevel);

                            //Note signed bytes. We're building an R8_SNORM texture, not UNORM.
                            var encodedStart = (sbyte*)atlasData + atlas.GetMipStartIndex(mipLevel);
                            var preciseStart = preciseData + preciseAtlas.GetMipStartIndex(mipLevel);
                            var rowPitch = atlas.GetRowPitch(mipLevel);
                            for (int rowIndex = mipMin.Y; rowIndex < mipMax.Y; ++rowIndex)
                            {
                                var preciseRow = preciseStart + rowIndex * rowPitch;
                                var encodedRow = encodedStart + rowIndex * rowPitch;
                                for (int columnIndex = mipMin.X; columnIndex < mipMax.X; ++columnIndex)
                                {
                                    encodedRow[columnIndex] = (sbyte)(127 * Math.Max(-1, Math.Min(1, encodingMultiplier * preciseRow[columnIndex])));
                                }

                            }
                        }
                    });

                    //const int savedMip = 0;// MipLevels - 1;
                    //var bitmap = new Bitmap(atlas.Width >> savedMip, atlas.Height >> savedMip);
                    //var bitmapData = bitmap.LockBits(new Rectangle(new Point(), new Size(bitmap.Width, bitmap.Height)),
                    //    ImageLockMode.WriteOnly, PixelFormat.Format32bppRgb);

                    //var scan0 = (byte*)bitmapData.Scan0;
                    //var sourceStart = atlasData + atlas.GetMipStartIndex(savedMip);
                    //for (int rowIndex = 0; rowIndex < bitmapData.Height; ++rowIndex)
                    //{
                    //    var row = (int*)(scan0 + bitmapData.Stride * rowIndex);
                    //    var sourceRow = sourceStart + atlas.GetRowOffsetFromMipStart(savedMip, rowIndex);
                    //    for (int columnIndex = 0; columnIndex < bitmapData.Width; ++columnIndex)
                    //    {
                    //        row[columnIndex] = sourceRow[columnIndex] | (sourceRow[columnIndex] << 8) | (sourceRow[columnIndex] << 16) | (sourceRow[columnIndex] << 24);
                    //    }
                    //}
                    //bitmap.UnlockBits(bitmapData);
                    //bitmap.Save($"{face.FamilyName}Test.bmp", ImageFormat.Bmp);

                    atlas.Unpin();
                    preciseAtlas.Unpin();
                    rasterizedAlphas.Unpin();


                    //Build the kerning table.
                    var kerning = ComputeKerningTable(face, characterSet);

                    //Now that the character data contains the normalization factor, we can add it to the final dictionary.
                    for (int i = 0; i < sortedCharacterData.Length; ++i)
                    {
                        characters.Add(sortedCharacterSet[i], sortedCharacterData[i]);
                    }

                    return new FontContent(atlas, face.FamilyName, 1f / FontSizeInPixels, characters, kerning);
                }
            }
        }
    }
}
