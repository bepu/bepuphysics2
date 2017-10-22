using BEPUutilities2.Collections;
using BEPUutilities2.Memory;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace BEPUutilitiesTests
{
    public static class CodeGenTests
    {
        [MethodImpl(MethodImplOptions.NoInlining)]
        static void TestPrimitiveComparer()
        {
            var comparer = default(PrimitiveComparer<int>);
            int a = 2;
            int b = 4;

            var equal = comparer.Equals(ref a, ref b);
            var hashcode = comparer.Hash(ref a);
            var isPrimitive = SpanHelper.IsPrimitive<int>();
            if (SpanHelper.IsPrimitive<Boolean>())
            {
                Console.WriteLine("prim prim");
            }

            Console.WriteLine($"Equality: {equal}, hash: {hashcode}");
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        static void TestDefaultComparer<T>(T a, T b)
        {
            WrapperEqualityComparer<T>.CreateDefault(out var comparer);

            var equal = comparer.Equals(ref a, ref b);
            var hashcode = comparer.Hash(ref a);
            if (SpanHelper.IsPrimitive<T>())
            {
                Console.WriteLine("prim prom");
            }

            Console.WriteLine($"Equality: {equal}, hash: {hashcode}");
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        unsafe static void TestPointerSpans<T>()
        {
            var memory = new byte[2048];
            fixed (byte* memoryPointer = memory)
            {
                var span = new Buffer<T>(memoryPointer, memory.Length);
                var def = default(T);
                var index = span.IndexOf(ref def, 0, 128);

                span.CopyTo(0, ref span, 0, 4);
                var arraySpan = new Array<T>(new T[1024]);
                span.CopyTo(0, ref arraySpan, 0, 4);
                arraySpan.CopyTo(0, ref span, 0, 4);
                arraySpan.CopyTo(0, ref arraySpan, 0, 4);

                Console.WriteLine($"index: {index}");
            }
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        unsafe static void TestArraySpans<T>()
        {
            var span = new Array<T>(new T[1024]);
            var def = default(T);
            var index = span.IndexOf(ref def, 0, 128);

            span.CopyTo(0, ref span, 0, 4);
            span.ClearManagedReferences(0, 4);

            Console.WriteLine($"index: {index}");
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        unsafe static void TestQuickInlining()
        {
            {
                var pool = new PassthroughArrayPool<double>();
                var intPool = new PassthroughArrayPool<int>();
                QuickSet<double, Array<double>, Array<int>, PrimitiveComparer<double>>.Create(pool, intPool, 2, 3, out var set);
                set.AddUnsafely(5);
                var item = set[0];
                Console.WriteLine($"Managed Item: {item}");

                var comparer = default(PrimitiveComparer<double>);
                var hash = comparer.Hash(ref item);

                Console.WriteLine($"Hash: {hash}");
            }

            {

                var pool = new BufferPool().SpecializeFor<int>();
                QuickSet<int, Buffer<int>, Buffer<int>, PrimitiveComparer<int>>.Create(pool, pool, 2, 3, out var set);
                set.AddUnsafely(5);
                var item = set[0];
                pool.Raw.Clear();
            }

        }

        public static void Test()
        {
            TestPrimitiveComparer();

            TestDefaultComparer(2, 4);
            TestDefaultComparer(2L, 2L);
            TestDefaultComparer("hey", "sup");

            TestPointerSpans<ulong>();
            TestPointerSpans<decimal>();
            TestArraySpans<object>();
            TestArraySpans<int>();

            TestQuickInlining();
            
        }
    }
}

