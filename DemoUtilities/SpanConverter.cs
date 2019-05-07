using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Text;

namespace DemoUtilities
{
    public static class SpanConverter
    {
        public static unsafe Span<T> AsSpan<T>(in Buffer<T> buffer) where T : struct
        {
            return new Span<T>(buffer.Memory, buffer.Length);
        }
    }
}
