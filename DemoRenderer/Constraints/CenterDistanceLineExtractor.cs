using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics;
using BepuPhysics.Constraints;
using System.Numerics;
using Quaternion = BepuUtilities.Quaternion;
using BepuUtilities;

namespace DemoRenderer.Constraints
{
    struct CenterDistanceLineExtractor : IConstraintLineExtractor<CenterDistancePrestepData>
    {
        public int LinesPerConstraint => 2;

        public unsafe void ExtractLines(ref CenterDistancePrestepData prestepBundle, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance> lines)
        {
            //Could do bundles of constraints at a time, but eh.
            var poseA = bodies.Sets[setIndex].Poses[bodyIndices[0]];
            var poseB = bodies.Sets[setIndex].Poses[bodyIndices[1]];
            var targetDistance = GatherScatter.GetFirst(ref prestepBundle.TargetDistance);
            var color = new Vector3(0.2f, 0.2f, 1f) * tint;
            var packedColor = Helpers.PackColor(color);
            var backgroundColor = new Vector3(0f, 0f, 1f) * tint;
            //Draw a line from A to B. If the true distance is longer than the target distance, draw a red line to complete the gap.
            //If the true distance is shorter than the target distance, draw an overshooting red line.
            var offset = poseB.Position - poseA.Position;
            var length = offset.Length();
            var direction = length < 1e-9f ? new Vector3(1, 0, 0) : offset / length;
            var errorColor = new Vector3(1, 0, 0) * tint;
            var packedErrorColor = Helpers.PackColor(errorColor);
            var packedDistanceColor = Helpers.PackColor(color * 0.5f);
            var targetEnd = poseA.Position + direction * targetDistance;
            if (length < targetDistance)
            {
                lines.AllocateUnsafely() = new LineInstance(poseA.Position, poseB.Position, packedDistanceColor, 0);
                lines.AllocateUnsafely() = new LineInstance(poseB.Position, targetEnd, packedErrorColor, 0);
            }
            else
            {
                lines.AllocateUnsafely() = new LineInstance(poseA.Position, targetEnd, packedDistanceColor, 0);
                lines.AllocateUnsafely() = new LineInstance(targetEnd, poseB.Position, packedErrorColor, 0);
            }

        }
    }
}
