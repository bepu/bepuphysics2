﻿using BepuUtilities.Collections;
using BepuPhysics;
using BepuPhysics.Constraints;
using System.Numerics;
using BepuUtilities;

namespace DemoRenderer.Constraints
{
    struct DistanceServoLineExtractor : IConstraintLineExtractor<DistanceServoPrestepData>
    {
        public static int LinesPerConstraint => 4;

        public static unsafe void ExtractLines(ref DistanceServoPrestepData prestepBundle, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance> lines)
        {
            //Could do bundles of constraints at a time, but eh.
            ref var poseA = ref bodies.Sets[setIndex].DynamicsState[bodyIndices[0]].Motion.Pose;
            ref var poseB = ref bodies.Sets[setIndex].DynamicsState[bodyIndices[1]].Motion.Pose;
            Vector3Wide.ReadFirst(prestepBundle.LocalOffsetA, out var localOffsetA);
            Vector3Wide.ReadFirst(prestepBundle.LocalOffsetB, out var localOffsetB);
            var targetDistance = GatherScatter.GetFirst(ref prestepBundle.TargetDistance);
            QuaternionEx.Transform(localOffsetA, poseA.Orientation, out var worldOffsetA);
            QuaternionEx.Transform(localOffsetB, poseB.Orientation, out var worldOffsetB);
            var endA = poseA.Position + worldOffsetA;
            var endB = poseB.Position + worldOffsetB;
            var color = new Vector3(0.2f, 0.2f, 1f) * tint;
            var packedColor = Helpers.PackColor(color);
            var backgroundColor = new Vector3(0f, 0f, 1f) * tint;
            lines.AllocateUnsafely() = new LineInstance(poseA.Position, endA, packedColor, 0);
            lines.AllocateUnsafely() = new LineInstance(poseB.Position, endB, packedColor, 0);
            //Draw a line from A to B. If the true distance is longer than the target distance, draw a red line to complete the gap.
            //If the true distance is shorter than the target distance, draw an overshooting red line.
            var offset = endB - endA;
            var length = offset.Length();
            var direction = length < 1e-9f ? new Vector3(1, 0, 0) : offset / length;
            var errorColor = new Vector3(1, 0, 0) * tint;
            var packedErrorColor = Helpers.PackColor(errorColor);
            var packedDistanceColor = Helpers.PackColor(color * 0.5f);
            var targetEnd = endA + direction * targetDistance;
            if (length < targetDistance)
            {
                lines.AllocateUnsafely() = new LineInstance(endA, endB, packedDistanceColor, 0);
                lines.AllocateUnsafely() = new LineInstance(endB, targetEnd, packedErrorColor, 0);
            }
            else
            {
                lines.AllocateUnsafely() = new LineInstance(endA, targetEnd, packedDistanceColor, 0);
                lines.AllocateUnsafely() = new LineInstance(targetEnd, endB, packedErrorColor, 0);
            }

        }
    }
}
