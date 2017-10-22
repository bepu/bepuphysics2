using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Text;

namespace DemoUtilities
{
    /// <summary>
    /// Simple StringBuilder-like class designed for reuse with minimal allocations.
    /// </summary>
    public class TextBuilder
    {
        QuickList<char, Array<char>> characters;

        public ref char this[int index] { get { return ref characters[index]; } }

        public int Length
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return characters.Count; }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set
            {
                Debug.Assert(value >= 0); characters.EnsureCapacity(value, new PassthroughArrayPool<char>()); characters.Count = value;
            }
        }

        public TextBuilder(int initialCapacity = 32)
        {
            QuickList<char, Array<char>>.Create(new PassthroughArrayPool<char>(), initialCapacity, out characters);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public TextBuilder Clear()
        {
            characters.Count = 0;
            return this;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public TextBuilder Append(string text, int start, int count)
        {
            characters.EnsureCapacity(characters.Count + text.Length, new PassthroughArrayPool<char>());
            int end = start + count;
            for (int i = start; i < end; ++i)
            {
                characters.AddUnsafely(text[i]);
            }
            return this;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public TextBuilder Append(string text)
        {
            return Append(text, 0, text.Length);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        char GetCharForDigit(int digit)
        {
            return (char)(digit + 48);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void AddDigit(ref double value, ref double multiplier, ref PassthroughArrayPool<char> pool)
        {
            var digit = (int)((value * multiplier) % 10);
            characters.Add(GetCharForDigit(digit), pool);
            value -= digit / multiplier;
            multiplier *= 10;
        }
        
        public TextBuilder Append(double value, int decimalCount)
        {
            //This is a bit of a throwaway implementation and is far from the fastest or numerically best implementation,
            //but it is fairly simple and it doesn't matter very much.

            const double minimumDoubleMagnitude = 2.22507385850720138309023271733240406421921598046233e-308;
            bool negative = value < 0;
            if (negative)
                value = -value;
            var pool = new PassthroughArrayPool<char>();
            if (value <= minimumDoubleMagnitude)
            {
                //Don't bother with signed zeroes.
                characters.Add('0', pool);
                return this;
            }
            if (negative)
            {
                characters.Add('-', pool);
            }
            value = Math.Round(value, decimalCount);
            var place = (int)Math.Floor(Math.Log10(value));
            var multiplier = Math.Pow(0.1, place);
            var epsilon = Math.Pow(0.1, decimalCount);

            for (int i = place; i >= 0; --i)
            {
                AddDigit(ref value, ref multiplier, ref pool);
            }
            if (value > epsilon)
            {
                characters.Add('.', pool);
                for (int i = -1; i > place; --i)
                {
                    characters.Add('0', pool);
                }
                do
                {
                    AddDigit(ref value, ref multiplier, ref pool);
                } while (value > epsilon);
            }
            return this;

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public TextBuilder Append(int value)
        {
            return Append(value, 0);
        }

        public override string ToString()
        {
            return new string(characters.Span.Memory, 0, characters.Count);
        }
    }
}
