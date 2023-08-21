﻿using BepuUtilities.Collections;
using BepuPhysics;
using BepuPhysics.Constraints;
using System.Numerics;
using BepuUtilities;

namespace DemoRenderer.Constraints
{
    struct PointOnLineLineExtractor : IConstraintLineExtractor<PointOnLineServoPrestepData>
    {
        public static int LinesPerConstraint => 4;

        public static unsafe void ExtractLines(ref PointOnLineServoPrestepData prestepBundle, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance> lines)
        {
            //Could do bundles of constraints at a time, but eh.
            ref var poseA = ref bodies.Sets[setIndex].DynamicsState[bodyIndices[0]].Motion.Pose;
            ref var poseB = ref bodies.Sets[setIndex].DynamicsState[bodyIndices[1]].Motion.Pose;
            Vector3Wide.ReadFirst(prestepBundle.LocalOffsetA, out var localOffsetA);
            Vector3Wide.ReadFirst(prestepBundle.LocalOffsetB, out var localOffsetB);
            Vector3Wide.ReadFirst(prestepBundle.LocalDirection, out var localDirection);
            QuaternionEx.Transform(localOffsetA, poseA.Orientation, out var worldOffsetA);
            QuaternionEx.Transform(localDirection, poseA.Orientation, out var worldDirection);
            QuaternionEx.Transform(localOffsetB, poseB.Orientation, out var worldOffsetB);

            var anchorA = poseA.Position + worldOffsetA;
            var anchorB = poseB.Position + worldOffsetB;
            var closestPointOnLine = Vector3.Dot(anchorB - anchorA, worldDirection) * worldDirection + anchorA;

            var color = new Vector3(0.2f, 0.2f, 1f) * tint;
            var packedColor = Helpers.PackColor(color);
            var backgroundColor = new Vector3(0f, 0f, 1f) * tint;
            lines.AllocateUnsafely() = new LineInstance(poseA.Position, anchorA, packedColor, 0);
            lines.AllocateUnsafely() = new LineInstance(anchorA, closestPointOnLine, packedColor, 0);
            lines.AllocateUnsafely() = new LineInstance(closestPointOnLine, anchorB, Helpers.PackColor(new Vector3(1, 0, 0) * tint), 0);
            lines.AllocateUnsafely() = new LineInstance(anchorB, poseB.Position, packedColor, 0);
        }
    }
}
