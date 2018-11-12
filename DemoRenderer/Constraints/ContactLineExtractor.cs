using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics;
using BepuPhysics.Constraints;
using System.Numerics;
using Quaternion = BepuUtilities.Quaternion;
using BepuUtilities;
using BepuPhysics.Constraints.Contact;
using System.Runtime.CompilerServices;
using System;

namespace DemoRenderer.Constraints
{
    public static class ContactLines
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void BuildOrthnormalBasis(in Vector3 normal, out Vector3 t1, out Vector3 t2)
        {
            //No frisvad or friends here- just want a simple and consistent basis with only one singularity.
            //Could be faster if needed.
            t1 = Vector3.Cross(normal, new Vector3(1, -1, 1));
            var lengthSquared = t1.LengthSquared();
            if (lengthSquared < 1e-8f)
            {
                t1 = Vector3.Cross(normal, new Vector3(-1, 1, 1));
                lengthSquared = t1.LengthSquared();
            }
            t1 /= MathF.Sqrt(lengthSquared);
            t2 = Vector3.Cross(normal, t1);
        }

        public static void Add(in RigidPose poseA, ref Vector3Wide offsetAWide, ref Vector3Wide normalWide, ref Vector<float> depthWide,
            in Vector3 tint, ref QuickList<LineInstance> lines)
        {
            Vector3Wide.ReadFirst(offsetAWide, out var offsetA);
            Vector3Wide.ReadFirst(normalWide, out var normal);
            var depth = depthWide[0];
            var contactPosition = offsetA + poseA.Position;
            BuildOrthnormalBasis(normal, out var t1, out var t2);
            var packedColor = Helpers.PackColor(tint * (depth >= 0 ? new Vector3(0,1,0) : new Vector3(0.15f, 0.25f, 0.15f)));
            t1 *= 0.5f;
            t2 *= 0.5f;
            var t1Line = new LineInstance(contactPosition - t1, contactPosition + t1, packedColor, 0);
            lines.AddUnsafely(t1Line);
            var t2Line = new LineInstance(contactPosition - t2, contactPosition + t2, packedColor, 0);
            lines.AddUnsafely(t2Line);

        }
    }
}
