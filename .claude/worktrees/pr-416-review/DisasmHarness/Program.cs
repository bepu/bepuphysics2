using System;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using System.Runtime.CompilerServices;

class Program
{
    //Mirrors the CollisionBatcher completion paths: property getter invoked on an 'in' parameter.
    //On master, PairContinuation is not a readonly struct, so the IL contains a defensive copy before the Type getter.
    [MethodImpl(MethodImplOptions.NoInlining)]
    static int ConsumeContinuation(in PairContinuation continuation)
    {
        if (continuation.Type == CollisionContinuationType.Direct)
            return continuation.PairId;
        return continuation.ChildA + continuation.ChildB + continuation.ChildIndex;
    }

    //Mirrors the Exists getters: master computes (Packed & (1 << 31)) with int/uint->long promotion.
    [MethodImpl(MethodImplOptions.NoInlining)]
    static bool Exists(TypedIndex index)
    {
        return index.Exists;
    }

    //Mirrors the packing constructors: master computes (uint)((type << 24) | index | (1u << 31)) in 64-bit.
    [MethodImpl(MethodImplOptions.NoInlining)]
    static TypedIndex Make(int type, int index)
    {
        return new TypedIndex(type, index);
    }

    static void Main()
    {
        long accumulator = 0;
        for (int i = 0; i < 1000; ++i)
        {
            var continuation = new PairContinuation(i, i & 3, i & 7, (CollisionContinuationType)(i & 1), i, i & 15);
            accumulator += ConsumeContinuation(continuation);
            var typedIndex = Make(i & 63, i);
            if (Exists(typedIndex))
                accumulator += typedIndex.Index;
        }
        Console.WriteLine(accumulator);
    }
}
