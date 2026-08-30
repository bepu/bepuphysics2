using BepuUtilities;
using System.Numerics;

//using bVector3 = BEPUutilities.Vector3;

namespace BEPUutilitiesTests
{
    public static class Vector3Tests
    {
        //public static float TestTransformScalarOld(int iterationCount)
        //{
        //    bVector3 v1 = new bVector3(1, 2, 3);
        //    bVector3 v2 = new bVector3(1, 2, 3);
        //    float accumulator = 0;
        //    for (int i = 0; i < iterationCount; ++i)
        //    {
        //        bVector3 r0, r1;
        //        bVector3.Cross(ref v1, ref v2, out r0);
        //        bVector3.Cross(ref r0, ref v2, out r1);
        //        bVector3.Cross(ref r1, ref v2, out r0);
        //        bVector3.Cross(ref r0, ref v2, out r1);
        //        bVector3.Cross(ref r1, ref v2, out r0);
        //        bVector3.Cross(ref r0, ref v2, out r1);
        //        bVector3.Cross(ref r1, ref v2, out r0);
        //        bVector3.Cross(ref r0, ref v2, out r1);
        //        bVector3.Cross(ref r1, ref v2, out r0);
        //        bVector3.Cross(ref r0, ref v2, out r1);
        //        accumulator += 0.000001f * r1.X;
        //    }
        //    return accumulator;
        //}

        public static float TestTransformScalar(int iterationCount)
        {
            Vector3 v1 = new Vector3(1, 2, 3);
            Vector3 v2 = new Vector3(1, 2, 3);
            float accumulator = 0;
            for (int i = 0; i < iterationCount; ++i)
            {
                Vector3 r0, r1;
                r0 = Vector3.Cross(v1, v2);
                r1 = Vector3.Cross(r0, v2);
                r0 = Vector3.Cross(r1, v2);
                r1 = Vector3.Cross(r0, v2);
                r0 = Vector3.Cross(r1, v2);
                r1 = Vector3.Cross(r0, v2);
                r0 = Vector3.Cross(r1, v2);
                r1 = Vector3.Cross(r0, v2);
                r0 = Vector3.Cross(r1, v2);
                r1 = Vector3.Cross(r0, v2);
                accumulator += 0.000001f * r1.X;
            }
            return accumulator;
        }

        public static void Test()
        {
            const int iterationCount = 10000000;
            //Helper.Test("Cross Scalar Old", TestTransformScalarOld, iterationCount);
            Helper.Test("Cross Scalar", TestTransformScalar, iterationCount);
        }
    }
}
