using System.Numerics;

namespace BepuPhysics
{
    internal static class Helpers
    {
        public static void BuildOrthnormalBasis(ref Vector3Wide normal, out Vector3Wide t1, out Vector3Wide t2)
        {
            //This could probably be improved.
            var sign = Vector.ConditionalSelect(Vector.LessThan(normal.Z, Vector<float>.Zero), -Vector<float>.One, Vector<float>.One);

            //This has a discontinuity at z==0. Raw frisvad has only one discontinuity, though that region is more unpredictable than the revised version.
            var scale = -Vector<float>.One / (sign + normal.Z);
            t1.X = normal.X * normal.Y * scale;
            t1.Y = sign + normal.Y * normal.Y * scale;
            t1.Z = -normal.Y;

            t2.X = Vector<float>.One + sign * normal.X * normal.X * scale;
            t2.Y = sign * t1.X;
            t2.Z = -sign * normal.X;
        }
    }
}
