﻿using BepuUtilities.Collections;
using BepuPhysics;
using BepuPhysics.Constraints;
using System.Numerics;
using BepuUtilities;

namespace DemoRenderer.Constraints
{
    struct OneBodyLinearServoLineExtractor : IConstraintLineExtractor<OneBodyLinearServoPrestepData>
    {
        public static int LinesPerConstraint => 2;

        public static unsafe void ExtractLines(ref OneBodyLinearServoPrestepData prestepBundle, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance> lines)
        {
            //Could do bundles of constraints at a time, but eh.
            ref var pose = ref bodies.Sets[setIndex].DynamicsState[*bodyIndices].Motion.Pose;
            Vector3Wide.ReadFirst(prestepBundle.LocalOffset, out var localOffset);
            Vector3Wide.ReadFirst(prestepBundle.Target, out var target);
            QuaternionEx.Transform(localOffset, pose.Orientation, out var worldOffset);

            var anchor = pose.Position + worldOffset;
            
            var backgroundColor = new Vector3(0f, 0f, 1f) * tint;
            lines.AllocateUnsafely() = new LineInstance(pose.Position, anchor, Helpers.PackColor(new Vector3(0.2f, 0.2f, 1f) * tint), 0);
            lines.AllocateUnsafely() = new LineInstance(anchor, target, Helpers.PackColor(new Vector3(1, 0, 0) * tint), 0);
        }
    }
}
