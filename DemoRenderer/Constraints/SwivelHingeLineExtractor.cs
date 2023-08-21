﻿using BepuUtilities.Collections;
using BepuPhysics;
using BepuPhysics.Constraints;
using System.Numerics;
using BepuUtilities;

namespace DemoRenderer.Constraints
{
    struct SwivelHingeLineExtractor : IConstraintLineExtractor<SwivelHingePrestepData>
    {
        public static int LinesPerConstraint => 5;

        public static unsafe void ExtractLines(ref SwivelHingePrestepData prestepBundle, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance> lines)
        {
            //Could do bundles of constraints at a time, but eh.
            ref var poseA = ref bodies.Sets[setIndex].DynamicsState[bodyIndices[0]].Motion.Pose;
            ref var poseB = ref bodies.Sets[setIndex].DynamicsState[bodyIndices[1]].Motion.Pose;
            Vector3Wide.ReadFirst(prestepBundle.LocalSwivelAxisA, out var localSwivelAxisA);
            Vector3Wide.ReadFirst(prestepBundle.LocalOffsetA, out var localOffsetA);
            Vector3Wide.ReadFirst(prestepBundle.LocalHingeAxisB, out var localHingeAxisB);
            Vector3Wide.ReadFirst(prestepBundle.LocalOffsetB, out var localOffsetB);
            QuaternionEx.Transform(localOffsetA, poseA.Orientation, out var offsetA);
            QuaternionEx.Transform(localSwivelAxisA, poseA.Orientation, out var swivelAxis);
            QuaternionEx.Transform(localOffsetB, poseB.Orientation, out var offsetB);
            QuaternionEx.Transform(localHingeAxisB, poseB.Orientation, out var hingeAxis);
            var packedAxisColor = Helpers.PackColor(new Vector3(0.2f, 0.7f, 1f) * tint);
            var backgroundColor = new Vector3(0f, 0f, 1f) * tint;
            var jointAnchorA = poseA.Position + offsetA;
            var jointAnchorB = poseB.Position + offsetB;
            lines.AllocateUnsafely() = new LineInstance(jointAnchorA, jointAnchorA + 0.2f * swivelAxis, packedAxisColor, 0);
            lines.AllocateUnsafely() = new LineInstance(jointAnchorB, jointAnchorB + 0.2f * hingeAxis, packedAxisColor, 0);

            var packedOffsetColor = Helpers.PackColor(new Vector3(0.2f, 0.2f, 1f) * tint);
            lines.AllocateUnsafely() = new LineInstance(poseA.Position, jointAnchorA, packedOffsetColor, 0);
            lines.AllocateUnsafely() = new LineInstance(poseB.Position, jointAnchorB, packedOffsetColor, 0);

            var errorColor = new Vector3(1, 0, 0) * tint;
            var packedErrorColor = Helpers.PackColor(errorColor);
            lines.AllocateUnsafely() = new LineInstance(jointAnchorA, jointAnchorB, packedErrorColor, 0);
        }

    }
}
