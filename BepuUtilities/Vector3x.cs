using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuUtilities
{
    /// <summary>
    /// Extra functionality for the Vector3 struct.
    /// </summary>
    /// <remarks>Hopefully, all of this should eventually go away as the System.Numerics.Vectors improves.</remarks>
    public struct Vector3x
    {
        //TODO: Should definitely double check usages of this. It's been years; the numerics version is probably fine now.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Cross(in Vector3 a, in Vector3 b, out Vector3 result)
        {
            result = new Vector3(
                a.Y * b.Z - a.Z * b.Y,
                a.Z * b.X - a.X * b.Z,
                a.X * b.Y - a.Y * b.X);
        }
    }
}
