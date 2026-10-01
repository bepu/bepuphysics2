using System;
using System.Numerics;

namespace BepuPhysicsTests;

internal static class QuaternionExtensions
{
    extension(Quaternion)
    {
        public static Quaternion CreateFromXAxisAngle(float angle) => Quaternion.CreateFromAxisAngle(Vector3.UnitX, angle);
        public static Quaternion CreateFromYAxisAngle(float angle) => Quaternion.CreateFromAxisAngle(Vector3.UnitY, angle);
        public static Quaternion CreateFromZAxisAngle(float angle) => Quaternion.CreateFromAxisAngle(Vector3.UnitZ, angle);
        public static Quaternion CreateRandom(Random random) => Quaternion.Normalize(new Quaternion(random.NextSingle() - 0.5f, random.NextSingle() - 0.5f, random.NextSingle() - 0.5f, random.NextSingle() + 0.1f));
    }
}
