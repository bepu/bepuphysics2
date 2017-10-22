using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using DemoContentLoader;
using System.Diagnostics;

namespace DemoContentBuilder
{
    public class FontPacker
    {
        int atlasWidth;
        int alignmentMask;
        int padding;
        int paddingx2;

        int start;
        int rowIndex;
        public int Height { get; private set; }

        struct Interval
        {
            public int Start;
            public int End; //Technically redundant, but it simplifies the implementation a little. Performance doesn't matter.
            public int Height;

            public override string ToString()
            {
                return $"[{Start}, {End}): {Height}";
            }
        }
        List<Interval> intervals;

        public FontPacker(int width, int mipLevels, int padding, int characterCount)
        {
            this.atlasWidth = width;
            this.alignmentMask = (1 << mipLevels) - 1;
            this.padding = padding;
            this.paddingx2 = padding * 2;

            intervals = new List<Interval>(characterCount);
            intervals.Add(new Interval { Start = 0, End = atlasWidth, Height = 0 });
        }


        int AddAndGetBaseHeight(int queryStart, int queryEnd, int newGlyphHeight)
        {
            Debug.Assert(queryStart >= 0 && queryStart < atlasWidth && queryEnd > 0 && queryEnd <= atlasWidth);
            //Performance doesn't matter here. Just scan through the interval set until the first interval that is fully overlapped by the query interval.
            int firstOverlappedIndex = intervals.Count;
            int baseHeight = 0;
            for (int i = 0; i < intervals.Count; ++i)
            {
                var interval = intervals[i];
                if (interval.End > queryStart)
                {
                    firstOverlappedIndex = i;
                    baseHeight = interval.Height;
                    break;
                }
            }
            //Now scan until the first interval after that that doesn't overlap.
            int lastOverlappedIndex = firstOverlappedIndex;
            for (int i = firstOverlappedIndex + 1; i < intervals.Count; ++i)
            {
                var interval = intervals[i];
                if (interval.Start < queryEnd)
                {
                    if (interval.Height > baseHeight)
                        baseHeight = interval.Height;
                    lastOverlappedIndex = i;
                }
                else
                    break;
            }
            //Align and round up base height.
            baseHeight = (baseHeight + alignmentMask) & (~alignmentMask);

            var firstInterval = intervals[firstOverlappedIndex];
            Interval queryInterval;
            queryInterval.Start = queryStart;
            queryInterval.End = queryEnd;
            queryInterval.Height = baseHeight + newGlyphHeight;
            if (queryInterval.Height > Height)
                Height = queryInterval.Height;
            Debug.Assert(queryInterval.End > queryInterval.Start);

            if (firstOverlappedIndex == lastOverlappedIndex && firstInterval.Start <= queryStart && firstInterval.End >= queryEnd)
            {
                if(firstInterval.Start == queryStart && firstInterval.End == queryEnd)
                {
                    //Perfect replacement.
                    intervals[firstOverlappedIndex] = queryInterval;
                }
                else if (firstInterval.Start == queryStart)
                {
                    //The new interval should be inserted before the firstInterval. Modify the first interval.
                    firstInterval.Start = queryEnd;
                    Debug.Assert(firstInterval.End > firstInterval.Start);
                    intervals[firstOverlappedIndex] = firstInterval;
                    intervals.Insert(firstOverlappedIndex , queryInterval);
                }
                else if (firstInterval.End == queryEnd)
                {
                    //The new interval should be inserted after the firstInterval. Modify the first interval.
                    firstInterval.End = queryStart;
                    Debug.Assert(firstInterval.End > firstInterval.Start);
                    intervals[firstOverlappedIndex] = firstInterval;
                    intervals.Insert(firstOverlappedIndex + 1, queryInterval);
                }
                else
                {
                    //The query interval is inside of an interval, with space available on either side.
                    //Add two more intervals- the query interval, and the interval on the other side.
                    //We treat the existing interval as the left side.
                    var otherSideInterval = firstInterval;
                    otherSideInterval.Start = queryEnd;
                    firstInterval.End = queryStart;
                    Debug.Assert(firstInterval.End > firstInterval.Start);
                    intervals[firstOverlappedIndex] = firstInterval;

                    intervals.Insert(firstOverlappedIndex + 1, queryInterval);
                    Debug.Assert(otherSideInterval.End > otherSideInterval.Start);
                    intervals.Insert(firstOverlappedIndex + 2, otherSideInterval);
                }
            }
            else
            {
                //The query covers more than one interval.
                int removalStartIndex;
                if (firstInterval.Start == queryStart)
                {
                    //The first overlapped index is contained by the query interval. It should be removed.
                    removalStartIndex = firstOverlappedIndex;
                }
                else
                {
                    //The first overlapped index isn't contained; modify its end to match the query start.
                    removalStartIndex = firstOverlappedIndex + 1;
                    firstInterval.End = queryStart;
                    Debug.Assert(firstInterval.End > firstInterval.Start);
                    intervals[firstOverlappedIndex] = firstInterval;
                }

                var lastInterval = intervals[lastOverlappedIndex];
                int removalEndIndex;
                if (lastInterval.End == queryEnd)
                {
                    //The last overlapped interval is contained by the query interval. It should be removed.
                    removalEndIndex = lastOverlappedIndex;
                }
                else
                {
                    //The last overlapped interval isn't contained; modify its start to match the query end.
                    removalEndIndex = lastOverlappedIndex - 1;
                    lastInterval.Start = queryEnd;
                    Debug.Assert(lastInterval.End > lastInterval.Start);
                    intervals[lastOverlappedIndex] = lastInterval;
                }
                //Note that the end is an inclusive bound. The total number of contained intervals is removalEndIndex - removalStartIndex + 1,
                //but reusing one of them avoids an unnecessary insert.
                var removedCount = removalEndIndex - removalStartIndex;
                if (removedCount >= 0)
                {
                    intervals.RemoveRange(removalStartIndex, removalEndIndex - removalStartIndex);
                    intervals[removalStartIndex] = queryInterval;
                }
                else
                {
                    intervals.Insert(removalStartIndex, queryInterval);
                }
            }
            return baseHeight;
        }

        private void FillCharacterMinimum(ref CharacterData characterData, int end)
        {
            characterData.SourceMinimum.X = padding + start;
            characterData.SourceMinimum.Y = padding + AddAndGetBaseHeight(start, end, (int)characterData.SourceSpan.Y + paddingx2);
        }


        public void Add(ref CharacterData characterData)
        {
            int allocationWidth = (int)(paddingx2 + characterData.SourceSpan.X);
            if (allocationWidth > atlasWidth)
            {
                throw new ArgumentException(
                    "A single character that's wider than the entire atlas isn't gonna work. Is the FontPacker incorrectly initialized? Is the rasterized font size ridiculously huge?");
            }
            if ((rowIndex & 1) == 0)
            {
                //Place glyphs from left to right.
                start = (start + alignmentMask) & (~alignmentMask);
                var end = start + allocationWidth;

                if (end <= atlasWidth)
                {
                    FillCharacterMinimum(ref characterData, end);
                    start = end;
                }
                else
                {
                    //This glyph doesn't fit; need to move to the next row.
                    ++rowIndex;
                    start = atlasWidth;
                    Add(ref characterData);
                }
            }
            else
            {
                //Place glyphs from right to left.
                start -= allocationWidth;
                if (start >= 0)
                {
                    //Delayed alignment; alignment will never make this negative.
                    start = start & (~alignmentMask);
                    var end = start + allocationWidth;
                    FillCharacterMinimum(ref characterData, end);
                }
                else
                {
                    //This glyph doesn't fit; need to move to the next row.
                    ++rowIndex;
                    start = 0;
                    Add(ref characterData);
                }

            }
        }

    }
}
