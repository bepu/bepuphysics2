using BepuUtilities.Memory;
using System;

namespace DemoUtilities
{
    public static class SpanConverter
    {
        public static unsafe Span<T> AsSpan<T>(in Buffer<T> buffer) where T : unmanaged
        {
            return new Span<T>(buffer.Memory, buffer.Length);
        }
    }
}
