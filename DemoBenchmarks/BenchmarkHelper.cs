using BepuPhysics;
using BepuUtilities;
using System.Numerics;

namespace DemoBenchmarks;

public static class BenchmarkHelper
{
    public static RigidPose CreateRandomPose(Random random, BoundingBox positionBounds)
    {
        RigidPose pose;
        var span = positionBounds.Max - positionBounds.Min;
        pose.Position = positionBounds.Min + span * new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle());
        var axis = new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle());
        var length = axis.Length();
        if (length > 0)
            axis /= length;
        else
            axis = new Vector3(0, 1, 0);
        pose.Orientation = QuaternionEx.CreateFromAxisAngle(axis, 1203f * random.NextSingle());
        return pose;
    }
}
