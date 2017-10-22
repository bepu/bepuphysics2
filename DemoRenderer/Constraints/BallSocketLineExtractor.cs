using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics;
using BepuPhysics.Constraints;
using System.Numerics;
using Quaternion = BepuUtilities.Quaternion;

namespace DemoRenderer.Constraints
{
    struct BallSocketLineExtractor : IConstraintLineExtractor<TwoBodyReferences, BallSocketPrestepData>
    {
        public int LinesPerConstraint => 3;

        public void ExtractLines(ref BallSocketPrestepData prestepBundle, ref TwoBodyReferences referencesBundle, int innerIndex,
            Bodies bodies, ref QuickList<LineInstance, Array<LineInstance>> lines)
        {
            //Could do bundles of constraints at a time, but eh.
            var indexA = GatherScatter.Get(ref referencesBundle.IndexA, innerIndex);
            var indexB = GatherScatter.Get(ref referencesBundle.IndexB, innerIndex);
            var poseA = bodies.Poses[indexA];
            var poseB = bodies.Poses[indexB];
            Vector3Wide.GetLane(ref prestepBundle.LocalOffsetA, innerIndex, out var localOffsetA);
            Vector3Wide.GetLane(ref prestepBundle.LocalOffsetB, innerIndex, out var localOffsetB);
            Quaternion.Transform(ref localOffsetA, ref poseA.Orientation, out var worldOffsetA);
            Quaternion.Transform(ref localOffsetB, ref poseB.Orientation, out var worldOffsetB);
            var endA = poseA.Position + worldOffsetA;
            var endB = poseB.Position + worldOffsetB;
            var color = new Vector3(0.2f, 0.2f, 1f);
            var lineA = new LineInstance(ref poseA.Position, ref endA, ref color);
            var lineB = new LineInstance(ref poseB.Position, ref endB, ref color);
            lines.AddUnsafely(ref lineA);
            lines.AddUnsafely(ref lineB);
            var errorColor = new Vector3(1, 0, 0);
            var errorLine = new LineInstance(ref endA, ref endB, ref errorColor);
            lines.AddUnsafely(ref errorLine);
        }
    }
}
