using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics;
using BepuPhysics.Constraints;
using System.Numerics;
using Quaternion = BepuUtilities.Quaternion;
using BepuUtilities;
using System;

namespace DemoRenderer.Constraints
{
    struct OneBodyLinearServoLineExtractor : IConstraintLineExtractor<OneBodyLinearServoPrestepData>
    {
        public int LinesPerConstraint => 2;

        public unsafe void ExtractLines(ref OneBodyLinearServoPrestepData prestepBundle, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance> lines)
        {
            //Could do bundles of constraints at a time, but eh.
            var pose = bodies.Sets[setIndex].Poses[*bodyIndices];
            Vector3Wide.ReadFirst(prestepBundle.LocalOffset, out var localOffset);
            Vector3Wide.ReadFirst(prestepBundle.Target, out var target);
            Quaternion.Transform(localOffset, pose.Orientation, out var worldOffset);

            var anchor = pose.Position + worldOffset;
            
            var backgroundColor = new Vector3(0f, 0f, 1f) * tint;
            lines.AllocateUnsafely() = new LineInstance(pose.Position, anchor, Helpers.PackColor(new Vector3(0.2f, 0.2f, 1f) * tint), 0);
            lines.AllocateUnsafely() = new LineInstance(anchor, target, Helpers.PackColor(new Vector3(1, 0, 0) * tint), 0);
        }
    }
}
