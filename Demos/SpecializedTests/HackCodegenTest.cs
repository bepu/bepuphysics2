using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace Demos.SpecializedTests
{
    static class HackCodegenTest
    {
        [StructLayout(LayoutKind.Sequential, Size = 512)]
        struct SomeStructWithDataAndFunctions
        {

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public int BasicallyStaticFunction(int a, int b)
            {
                return a * b;
            }
        }

        [StructLayout(LayoutKind.Sequential, Size = 0)] //Won't actually be zero, but it'll use a minimal size.
        struct SmallThing
        {
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        public static int Oi(int a, int b)
        {
            //Very small size -> very cheap locals init.
            SmallThing dummy;
            //Despite having near zero extent, and thus all data in the casted struct being invalid, we can still use the type information to invoke the function.
            //Since the function has no references to the data itself- it's basically a static factory function- there is no invalid memory access.
            return Unsafe.As<SmallThing, SomeStructWithDataAndFunctions>(ref dummy).BasicallyStaticFunction(a, b);
        }
        [MethodImpl(MethodImplOptions.NoInlining)]
        public static int Oi2(int a, int b)
        {
            //Good news it actually doesn't do the localsinit if the jit can know that the 'static' function isn't actually accessing any of the data!
            //var dummy = default(SomeStructWithDataAndFunctions);
            var dummy = new SomeStructWithDataAndFunctions();
            a -= 4;
            b -= a / 2;
            var result = dummy.BasicallyStaticFunction(a, b);

            Console.WriteLine("what happens if we do a bunch of other stuff?" + a + b + result);
            return result;
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        public static void Test()
        {
            Console.WriteLine(Oi(3, 8));
            Console.WriteLine(Oi2(3, 8));
        }
    }
}
