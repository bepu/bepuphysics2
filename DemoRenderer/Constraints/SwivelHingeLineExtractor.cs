using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics;
using BepuPhysics.Constraints;
using System.Numerics;
using Quaternion = BepuUtilities.Quaternion;
using BepuUtilities;

namespace DemoRenderer.Constraints
{
    struct SwivelHingeLineExtractor : IConstraintLineExtractor<SwivelHingePrestepData>
    {
        public int LinesPerConstraint => 5;

        public unsafe void ExtractLines(ref SwivelHingePrestepData prestepBundle, int innerIndex, int setIndex, int* bodyIndices,
            Bodies bodies, ref Vector3 tint, ref QuickList<LineInstance, Array<LineInstance>> lines)
        {
            //Could do bundles of constraints at a time, but eh.
            ref var poseA = ref bodies.Sets[setIndex].Poses[bodyIndices[0]];
            ref var poseB = ref bodies.Sets[setIndex].Poses[bodyIndices[1]];
            Vector3Wide.ReadSlot(ref prestepBundle.LocalSwivelAxisA, innerIndex, out var localSwivelAxisA);
            Vector3Wide.ReadSlot(ref prestepBundle.LocalOffsetA, innerIndex, out var localOffsetA);
            Vector3Wide.ReadSlot(ref prestepBundle.LocalHingeAxisB, innerIndex, out var localHingeAxisB);
            Vector3Wide.ReadSlot(ref prestepBundle.LocalOffsetB, innerIndex, out var localOffsetB);
            AngularSwivelHingeLineExtractor.CreateLines(localSwivelAxisA, localHingeAxisB, tint, poseA, poseB, ref lines);
            BallSocketLineExtractor.CreateLines(localOffsetA, localOffsetB, poseA, poseB, tint, ref lines);
        }

    }
}
