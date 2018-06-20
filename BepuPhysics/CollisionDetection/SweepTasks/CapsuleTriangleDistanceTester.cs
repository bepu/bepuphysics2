using BepuPhysics.Collidables;
using BepuUtilities;
using System.Numerics;

namespace BepuPhysics.CollisionDetection.SweepTasks
{
    using GJKTester = GJKDistanceTester<Capsule, CapsuleWide, CapsuleSupportFinder, Triangle, TriangleWide, TriangleSupportFinder>;
    public struct CapsuleTriangleDistanceTester : IPairDistanceTester<CapsuleWide, TriangleWide>
    {
        public void Test(ref CapsuleWide a, ref TriangleWide b, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
            out Vector<int> intersected, out Vector<float> distance, out Vector3Wide closestA, out Vector3Wide normal)
        {
            //While a capsule-triangle dedicated case would likely be ~20-40% faster than using GJK, it's not really worth the effort without some motivating use case.
            //Instead, we'll just use GJK, but our capsule support function only considers the internal line segment (plus GJK's epsilon).
            //So, here, we'll modify the distance and closestA to account for the disparity.
            GJKTester tester = default;
            tester.Test(ref a, ref b, ref offsetB, ref orientationA, ref orientationB, out intersected, out distance, out closestA, out normal);
            //The containment epsilon expands the minkowski sum a little bit. Get rid of that so that the distance corresponds exactly to the capsule's surface.
            //(We don't have this luxury for box-box, but we might as well handle it when we're able.)
            var distanceChange = new Vector<float>(GJKTester.ContainmentEpsilonDefault) - a.Radius;
            distance += distanceChange;
            Vector3Wide.Scale(normal, distanceChange, out var closestPointOffset);
            Vector3Wide.Subtract(closestPointOffset, closestA, out closestA);
        }
    }


}
