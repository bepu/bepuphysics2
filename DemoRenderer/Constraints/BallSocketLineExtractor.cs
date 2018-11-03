using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics;
using BepuPhysics.Constraints;
using System.Numerics;
using Quaternion = BepuUtilities.Quaternion;
using BepuUtilities;

namespace DemoRenderer.Constraints
{
    struct BallSocketLineExtractor : IConstraintLineExtractor<BallSocketPrestepData>
    {
        public int LinesPerConstraint => 3;

        public unsafe void ExtractLines(ref BallSocketPrestepData prestepBundle, int innerIndex, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance, Buffer<LineInstance>> lines)
        {
            //Could do bundles of constraints at a time, but eh.
            var poseA = bodies.Sets[setIndex].Poses[bodyIndices[0]];
            var poseB = bodies.Sets[setIndex].Poses[bodyIndices[1]];
            Vector3Wide.ReadSlot(ref prestepBundle.LocalOffsetA, innerIndex, out var localOffsetA);
            Vector3Wide.ReadSlot(ref prestepBundle.LocalOffsetB, innerIndex, out var localOffsetB);
            Quaternion.Transform(localOffsetA, poseA.Orientation, out var worldOffsetA);
            Quaternion.Transform(localOffsetB, poseB.Orientation, out var worldOffsetB);
            var endA = poseA.Position + worldOffsetA;
            var endB = poseB.Position + worldOffsetB;
            var color = new Vector3(0.2f, 0.2f, 1f) * tint;
            var packedColor = Helpers.PackColor(color);
            var backgroundColor = new Vector3(0f, 0f, 1f) * tint;
            lines.AllocateUnsafely() = new LineInstance(poseA.Position, endA, packedColor, 0);
            lines.AllocateUnsafely() = new LineInstance(poseB.Position, endB, packedColor, 0);

            var errorColor = new Vector3(1, 0, 0) * tint;
            var packedErrorColor = Helpers.PackColor(errorColor);
            lines.AllocateUnsafely() = new LineInstance(endA, endB, packedErrorColor, 0);
        }
    }
}
