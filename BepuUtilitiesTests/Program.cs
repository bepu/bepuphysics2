using System;

namespace BEPUutilitiesTests
{
    class Program
    {
        static void Main(string[] args)
        {
            CodeGenTests.Test();
            AllocatorTests.TestChurnStability();
            QuickCollectionTests.Test();
            //BoundingTests.Test();
            Console.WriteLine();
            AffineTests.Test();
            Console.WriteLine();
            Vector3Tests.Test();
            Console.WriteLine();
            Matrix3x3Tests.Test();
            Console.WriteLine();
            Matrix4x4Tests.Test();

        }

    }
}
