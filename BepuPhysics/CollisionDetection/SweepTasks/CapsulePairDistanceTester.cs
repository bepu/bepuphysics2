using BepuPhysics.Collidables;
using BepuUtilities;
using System.Numerics;

namespace BepuPhysics.CollisionDetection.SweepTasks
{
    public struct CapsulePairDistanceTester : IPairDistanceTester<CapsuleWide, CapsuleWide>
    {
        public void Test(ref CapsuleWide a, ref CapsuleWide b, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB,
            out Vector<int> intersected, out Vector<float> distance, out Vector3Wide closestA, out Vector3Wide normal)
        {
            //Compute the closest points between the two line segments. No clamping to begin with.
            //We want to minimize distance = ||(a + da * ta) - (b + db * tb)||.
            //Taking the derivative with respect to ta and doing some algebra (taking into account ||da|| == ||db|| == 1) to solve for ta yields:
            //ta = (da * (b - a) + (db * (a - b)) * (da * db)) / (1 - ((da * db) * (da * db))        
            QuaternionWide.TransformUnitXY(ref orientationA, out var xa, out var da);
            QuaternionWide.TransformUnitY(ref orientationB, out var db);
            Vector3Wide.Dot(ref da, ref offsetB, out var daOffsetB);
            Vector3Wide.Dot(ref db, ref offsetB, out var dbOffsetB);
            Vector3Wide.Dot(ref da, ref db, out var dadb);
            //Note potential division by zero when the axes are parallel. Arbitrarily clamp; near zero values will instead produce extreme values which get clamped to reasonable results.
            var ta = (daOffsetB - dbOffsetB * dadb) / Vector.Max(new Vector<float>(1e-15f), Vector<float>.One - dadb * dadb);
            //tb = ta * (da * db) - db * (b - a)
            var tb = ta * dadb - dbOffsetB;

            //We cannot simply clamp the ta and tb values to the capsule line segments. Instead, project each line segment onto the other line segment, clamping against the target's interval.
            //That new clamped projected interval is the valid solution space on that line segment. We can clamp the t value by that interval to get the correctly bounded solution.
            //The projected intervals are:
            //B onto A: +-BHalfLength * (da * db) + da * offsetB
            //A onto B: +-AHalfLength * (da * db) - db * offsetB
            var absdadb = Vector.Abs(dadb);
            var bOntoAOffset = b.HalfLength * absdadb;
            var aOntoBOffset = a.HalfLength * absdadb;
            var aMin = Vector.Max(-a.HalfLength, Vector.Min(a.HalfLength, daOffsetB - bOntoAOffset));
            var aMax = Vector.Min(a.HalfLength, Vector.Max(-a.HalfLength, daOffsetB + bOntoAOffset));
            var bMin = Vector.Max(-b.HalfLength, Vector.Min(b.HalfLength, -aOntoBOffset - dbOffsetB));
            var bMax = Vector.Min(b.HalfLength, Vector.Max(-b.HalfLength, aOntoBOffset - dbOffsetB));
            ta = Vector.Min(Vector.Max(ta, aMin), aMax);
            tb = Vector.Min(Vector.Max(tb, bMin), bMax);

            Vector3Wide.Scale(ref da, ref ta, out closestA);
            Vector3Wide.Scale(ref db, ref tb, out var closestB);
            Vector3Wide.Add(ref closestB, ref offsetB, out closestB);

            Vector3Wide.Subtract(ref closestA, ref closestB, out normal);
            Vector3Wide.Length(ref normal, out distance);
            var inverseDistance = Vector<float>.One / distance;
            Vector3Wide.Scale(ref normal, ref inverseDistance, out normal);
            Vector3Wide.Scale(ref normal, ref a.Radius, out var aOffset);
            Vector3Wide.Subtract(ref closestA, ref aOffset, out closestA);
            distance = distance - a.Radius - b.Radius;
            intersected = Vector.LessThanOrEqual(distance, Vector<float>.Zero);
        }
    }

}
