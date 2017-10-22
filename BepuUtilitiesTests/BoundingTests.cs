//using Microsoft.VisualStudio.TestTools.UnitTesting;
//using System;

//using bVector3 = BEPUutilities.Vector3;
//using bRay = BEPUutilities.Ray;
//using bBoundingBox = BEPUutilities.BoundingBox;
//using BEPUutilities2;
//using System.Numerics;

//namespace BEPUutilitiesTests
//{
//    [TestClass]
//    class BoundingTests
//    {
//        public static float TestBoxRaySIMD(int iterationCount)
//        {
//            Ray ray;
//            ray.Direction = new Vector3(1, 1, 1);
//            ray.Position = new Vector3(-3);
//            BoundingBox box;
//            box.Min = new Vector3(-1);
//            box.Max = new Vector3(1);


//            float accumulator = 0;
//            for (int i = 0; i < iterationCount; ++i)
//            {
//                float t;
//                Ray.Intersects(ref ray, ref box, out t);
//                Ray.Intersects(ref ray, ref box, out t);
//                Ray.Intersects(ref ray, ref box, out t);
//                Ray.Intersects(ref ray, ref box, out t);
//                Ray.Intersects(ref ray, ref box, out t);
//                Ray.Intersects(ref ray, ref box, out t);
//                Ray.Intersects(ref ray, ref box, out t);
//                Ray.Intersects(ref ray, ref box, out t);
//                Ray.Intersects(ref ray, ref box, out t);
//                Ray.Intersects(ref ray, ref box, out t);
//                accumulator += t;
//            }
//            return accumulator;
//        }

//        public static float TestBoxRayScalar(int iterationCount)
//        {

//            bRay ray;
//            ray.Direction = new bVector3(1, 1, 1);
//            ray.Position = new bVector3(-3, -3, -3);
//            bBoundingBox box;
//            box.Min = new bVector3(-1, -1, -1);
//            box.Max = new bVector3(1, 1, 1);


//            float accumulator = 0;
//            for (int i = 0; i < iterationCount; ++i)
//            {
//                float t;
//                ray.Intersects(ref box, out t);
//                ray.Intersects(ref box, out t);
//                ray.Intersects(ref box, out t);
//                ray.Intersects(ref box, out t);
//                ray.Intersects(ref box, out t);
//                ray.Intersects(ref box, out t);
//                ray.Intersects(ref box, out t);
//                ray.Intersects(ref box, out t);
//                ray.Intersects(ref box, out t);
//                ray.Intersects(ref box, out t);
//                accumulator += t;
//            }
//            return accumulator;
//        }

//        public static bVector3 Convert(Vector3 v)
//        {
//            return new bVector3(v.X, v.Y, v.Z);
//        }

//        public static void TestIntersection(Ray simdRay, BoundingBox simdBox, ref int intersectionCount)
//        {
//            bRay scalarRay;
//            bBoundingBox scalarBox;

//            scalarRay.Position = Convert(simdRay.Position);
//            scalarRay.Direction = Convert(simdRay.Direction);
//            scalarBox.Min = Convert(simdBox.Min);
//            scalarBox.Max = Convert(simdBox.Max);

//            float simdT;
//            var simdIntersects = Ray.Intersects(ref simdRay, ref simdBox, out simdT);

//            float scalarT;
//            var scalarIntersects = scalarRay.Intersects(ref scalarBox, out scalarT);
//            //Console.WriteLine($"Simd says: {simdIntersects}, scalar says {scalarIntersects}");
//            Assert.IsTrue(simdIntersects == scalarIntersects);
//            if (simdIntersects)
//            {
//                var error = Math.Abs(simdT - scalarT);
//                Assert.IsTrue(error <= 1e-5f);
//                intersectionCount++;
//            }
//            //Just treat T as undefined if it's not intersecting. No need to check it.
//        }

//        [TestMethod]
//        public unsafe static void TestBoxRayCorrectness()
//        {
//            //Test random rays.
//            Random random = new Random(5);
//            int intersectionCount = 0;
//            for (int i = 0; i < 1000000; ++i)
//            {
//                Ray simdRay;
//                BoundingBox simdBox;

//                simdRay.Position = 20 * (new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) - new Vector3(0.5f));
//                var boxPosition = 5 * (new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) - new Vector3(0.5f));
//                var target = 5 * (new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) - new Vector3(0.5f));
//                simdRay.Direction = target - simdRay.Position;

//                var width = new Vector3(3 * (float)random.NextDouble());
//                simdBox.Min = boxPosition - width;
//                simdBox.Max = boxPosition + width;


//                TestIntersection(simdRay, simdBox, ref intersectionCount);


//            }

//            //Test corner cases.
//            {
//                Ray simdRay;
//                BoundingBox simdBox;
//                simdBox.Min = new Vector3(0);
//                simdBox.Max = new Vector3(1);

//                //Inside
//                simdRay.Position = new Vector3(0.5f);
//                simdRay.Direction = new Vector3(1, 0, 0);
//                TestIntersection(simdRay, simdBox, ref intersectionCount);

//                simdRay.Position = new Vector3(0.5f);
//                simdRay.Direction = new Vector3(-1, 0, 0);
//                TestIntersection(simdRay, simdBox, ref intersectionCount);

//                //Perpendicular to surface
//                simdRay.Position = new Vector3(0.5f, 0.5f, 0);
//                simdRay.Direction = new Vector3(-1, 0, 0);
//                TestIntersection(simdRay, simdBox, ref intersectionCount);

//                simdRay.Position = new Vector3(0.5f, 0.5f, 1);
//                simdRay.Direction = new Vector3(1, 0, 0);
//                TestIntersection(simdRay, simdBox, ref intersectionCount);

//                //Corner and edge
//                simdRay.Position = new Vector3(0);
//                simdRay.Direction = new Vector3(1, 0, 0);
//                TestIntersection(simdRay, simdBox, ref intersectionCount);

//                simdRay.Position = new Vector3(-1, 0, 0);
//                simdRay.Direction = new Vector3(1, 0, 0);
//                TestIntersection(simdRay, simdBox, ref intersectionCount);

//                simdRay.Position = new Vector3(0, 0.5f, 0);
//                simdRay.Direction = new Vector3(1, 0, 0);
//                TestIntersection(simdRay, simdBox, ref intersectionCount);

//                simdRay.Position = new Vector3(0, 0.5f, 0.5f);
//                simdRay.Direction = new Vector3(1, 0, 0);
//                TestIntersection(simdRay, simdBox, ref intersectionCount);

//                //Reversed
//                simdRay.Position = new Vector3(1);
//                simdRay.Direction = new Vector3(-1, 0, 0);
//                TestIntersection(simdRay, simdBox, ref intersectionCount);

//                simdRay.Position = new Vector3(2, 0, 0);
//                simdRay.Direction = new Vector3(-1, 0, 0);
//                TestIntersection(simdRay, simdBox, ref intersectionCount);

//                simdRay.Position = new Vector3(1, 0.5f, 0);
//                simdRay.Direction = new Vector3(-1, 0, 0);
//                TestIntersection(simdRay, simdBox, ref intersectionCount);

//                simdRay.Position = new Vector3(1, 0.5f, 0.5f);
//                simdRay.Direction = new Vector3(-1, 0, 0);
//                TestIntersection(simdRay, simdBox, ref intersectionCount);



//            }
//            Console.WriteLine($"intersectionCount: {intersectionCount}");
//        }
        

//        public static void Test()
//        {
//            TestBoxRayCorrectness();

//            const int iterations = 1000000;
//            Helper.Test("Box-Ray SIMD", TestBoxRaySIMD, iterations);
//            Helper.Test("Box-Ray Scalar", TestBoxRayScalar, iterations);
//        }
//    }
//}
