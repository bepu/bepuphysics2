using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Text;

namespace Demos.SpecializedTests
{
    public static class TypeIdCodeGenTests
    {
        interface ITyped
        {
            int TypeId { get; }
        }

        struct Fixed : ITyped
        {
            public long Doop;
            public int Bidoop;
            public short Tridoop;
            public byte Quadoop;
            public bool Pentadoop;

            public Fixed(int p)
            {
                Doop = p;
                Bidoop = p;
                Tridoop = (short)p;
                Quadoop = (byte)p;
                Pentadoop = false;
            }

            public int TypeId
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get { return 2; }
            }
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        static int GetId<T>() where T : ITyped
        {
            //Despite fixed having fields, this should not do anything except move TypeId's constant value into a return register. No locals init or anything.
            return default(T).TypeId;
        }

        public static void Test()
        {
            Console.WriteLine($"Id: {GetId<Fixed>()}");
        }
    }
}
