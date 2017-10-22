using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace Demos.SpecializedTests
{
    class ReinterpretVectorRepro
    {
        [MethodImpl(MethodImplOptions.NoInlining)]
        static int UnsafeGrab()
        {
            var vector = new Vector<int>(4);
            return Unsafe.As<Vector<int>, int>(ref vector);
        }
        [MethodImpl(MethodImplOptions.NoInlining)]
        static int IndexerGrab()
        {
            var vector = new Vector<int>(4);
            return vector[0];
        }
        [MethodImpl(MethodImplOptions.NoInlining)]
        static int GeneralStructGrab()
        {
            var count = new GeneralStruct { A = 4, B = 4, C = 4, D = 4 };
            return Unsafe.As<GeneralStruct, int>(ref count);
        }

        struct GeneralStruct
        {
            public int A;
            public int B;
            public int C;
            public int D;
        }


        public static void Test()
        {
            Console.WriteLine($"Unsafe: {UnsafeGrab()}");
            Console.WriteLine($"Indexer: {IndexerGrab()}");
            Console.WriteLine($"General Struct: {GeneralStructGrab()}");
        }
    }
}
