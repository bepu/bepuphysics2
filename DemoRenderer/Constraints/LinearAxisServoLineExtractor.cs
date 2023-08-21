﻿using BepuUtilities.Collections;
using BepuPhysics;
using BepuPhysics.Constraints;
using System.Numerics;
using BepuUtilities;

namespace DemoRenderer.Constraints
{
    struct LinearAxisServoLineExtractor : IConstraintLineExtractor<LinearAxisServoPrestepData>
    {
        public static int LinesPerConstraint => 7;

        public static unsafe void ExtractLines(ref LinearAxisServoPrestepData prestepBundle, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance> lines)
        {
            //Could do bundles of constraints at a time, but eh.
            ref var poseA = ref bodies.Sets[setIndex].DynamicsState[bodyIndices[0]].Motion.Pose;
            ref var poseB = ref bodies.Sets[setIndex].DynamicsState[bodyIndices[1]].Motion.Pose;
            Vector3Wide.ReadFirst(prestepBundle.LocalOffsetA, out var localOffsetA);
            Vector3Wide.ReadFirst(prestepBundle.LocalOffsetB, out var localOffsetB);
            Vector3Wide.ReadFirst(prestepBundle.LocalPlaneNormal, out var localPlaneNormal);
            var targetOffset = GatherScatter.GetFirst(ref prestepBundle.TargetOffset);
            Matrix3x3.CreateFromQuaternion(poseA.Orientation, out var orientationA);
            Matrix3x3.Transform(localOffsetA, orientationA, out var worldOffsetA);
            Matrix3x3.Transform(localPlaneNormal, orientationA, out var worldPlaneNormal);
            QuaternionEx.Transform(localOffsetB, poseB.Orientation, out var worldOffsetB);

            var anchorA = poseA.Position + worldOffsetA;
            var anchorB = poseB.Position + worldOffsetB;
            var planeOffset = Vector3.Dot(anchorB - anchorA, worldPlaneNormal);
            var closestPointOnPlane = anchorB - planeOffset * worldPlaneNormal;

            var packedColor = Helpers.PackColor(new Vector3(0.2f, 0.2f, 1f) * tint);
            var packedBasisColor = Helpers.PackColor(new Vector3(0.2f, 0.6f, 1f) * tint);
            var backgroundColor = new Vector3(0f, 0f, 1f) * tint;
            lines.AllocateUnsafely() = new LineInstance(poseA.Position, anchorA, packedColor, 0);
            ContactLines.BuildOrthonormalBasis(localPlaneNormal, out var localTX, out var localTY);
            Matrix3x3.Transform(localTX, orientationA, out var tX);
            Matrix3x3.Transform(localTY, orientationA, out var tY);
            lines.AllocateUnsafely() = new LineInstance(anchorA - tX, anchorA + tX, packedBasisColor, 0);
            lines.AllocateUnsafely() = new LineInstance(anchorA - tY, anchorA + tY, packedBasisColor, 0);
            lines.AllocateUnsafely() = new LineInstance(anchorA, closestPointOnPlane, packedColor, 0);
            lines.AllocateUnsafely() = new LineInstance(anchorB, poseB.Position, packedColor, 0);
            if (targetOffset < 0)
            {
                targetOffset = -targetOffset;
                planeOffset = -planeOffset;
                worldPlaneNormal = -worldPlaneNormal;
            }
            var targetPoint = closestPointOnPlane + worldPlaneNormal * targetOffset;
            var packedErrorColor = Helpers.PackColor(new Vector3(1, 0, 0) * tint);
            if (planeOffset > targetOffset)
            {
                lines.AllocateUnsafely() = new LineInstance(closestPointOnPlane, targetPoint, packedColor, 0);
                lines.AllocateUnsafely() = new LineInstance(targetPoint, anchorB, packedErrorColor, 0);
            }
            else
            {
                lines.AllocateUnsafely() = new LineInstance(closestPointOnPlane, anchorB, packedColor, 0);
                lines.AllocateUnsafely() = new LineInstance(anchorB, targetPoint, packedErrorColor, 0);
            }
        }
    }
}
