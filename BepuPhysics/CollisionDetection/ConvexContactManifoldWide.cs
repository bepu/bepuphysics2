using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuUtilities;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection
{
    public struct Convex1ContactManifoldWide : IContactManifoldWide
    {
        public Vector3Wide OffsetA;
        public Vector3Wide Normal;
        public Vector<float> Depth;
        //Note that feature ids are a little bit questionable for single contact manifolds. We only keep them because:
        //1) it costs ~nothing, and
        //2) some pairs use feature ids to pass metadata (like triangles in meshes).
        public Vector<int> FeatureId;
        public Vector<int> ContactExists;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyFlipMask(ref Vector3Wide offsetB, in Vector<int> flipMask)
        {
            Vector3Wide.Negate(Normal, out var flippedNormal);
            Vector3Wide.Subtract(OffsetA, offsetB, out var flippedContactPosition);
            Vector3Wide.Negate(offsetB, out var flippedOffsetB);
            Vector3Wide.ConditionalSelect(flipMask, flippedNormal, Normal, out Normal);
            Vector3Wide.ConditionalSelect(flipMask, flippedContactPosition, OffsetA, out OffsetA);
            Vector3Wide.ConditionalSelect(flipMask, flippedOffsetB, offsetB, out offsetB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ReadFirst(in Vector3Wide offsetB, ref ConvexContactManifold target)
        {
            if (ContactExists[0] < 0)
            {
                target.Count = 1;
                target.OffsetB.X = offsetB.X[0];
                target.OffsetB.Y = offsetB.Y[0];
                target.OffsetB.Z = offsetB.Z[0];
                target.Contact0.Offset.X = OffsetA.X[0];
                target.Contact0.Offset.Y = OffsetA.Y[0];
                target.Contact0.Offset.Z = OffsetA.Z[0];
                target.Contact0.Depth = Depth[0];
                target.Contact0.FeatureId = FeatureId[0];

                target.Normal.X = Normal.X[0];
                target.Normal.Y = Normal.Y[0];
                target.Normal.Z = Normal.Z[0];
            }
            else
            {
                target.Count = 0;
            }
        }
    }

    public struct Convex2ContactManifoldWide : IContactManifoldWide
    {
        public Vector3Wide OffsetA0;
        public Vector3Wide OffsetA1;
        public Vector3Wide Normal;
        public Vector<float> Depth0;
        public Vector<float> Depth1;
        public Vector<int> FeatureId0;
        public Vector<int> FeatureId1;
        public Vector<int> Contact0Exists;
        public Vector<int> Contact1Exists;


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyFlipMask(ref Vector3Wide offsetB, in Vector<int> flipMask)
        {
            Vector3Wide.Negate(Normal, out var flippedNormal);
            Vector3Wide.Subtract(OffsetA0, offsetB, out var flippedA0);
            Vector3Wide.Subtract(OffsetA1, offsetB, out var flippedA1);
            Vector3Wide.Negate(offsetB, out var flippedOffsetB);
            Vector3Wide.ConditionalSelect(flipMask, flippedNormal, Normal, out Normal);
            Vector3Wide.ConditionalSelect(flipMask, flippedA0, OffsetA0, out OffsetA0);
            Vector3Wide.ConditionalSelect(flipMask, flippedA1, OffsetA1, out OffsetA1);
            Vector3Wide.ConditionalSelect(flipMask, flippedOffsetB, offsetB, out offsetB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ReadFirst(in Vector3Wide offsetB, ref ConvexContactManifold target)
        {
            target.Count = 0;
            if (Contact0Exists[0] < 0)
            {
                ++target.Count;
                target.Contact0.Offset.X = OffsetA0.X[0];
                target.Contact0.Offset.Y = OffsetA0.Y[0];
                target.Contact0.Offset.Z = OffsetA0.Z[0];
                target.Contact0.Depth = Depth0[0];
                target.Contact0.FeatureId = FeatureId0[0];
            }
            if (Contact1Exists[0] < 0)
            {
                ref var contact = ref Unsafe.Add(ref target.Contact0, target.Count++);
                contact.Offset.X = OffsetA1.X[0];
                contact.Offset.Y = OffsetA1.Y[0];
                contact.Offset.Z = OffsetA1.Z[0];
                contact.Depth = Depth1[0];
                contact.FeatureId = FeatureId1[0];
            }
            if (target.Count > 0)
            {
                target.OffsetB.X = offsetB.X[0];
                target.OffsetB.Y = offsetB.Y[0];
                target.OffsetB.Z = offsetB.Z[0];
                target.Normal.X = Normal.X[0];
                target.Normal.Y = Normal.Y[0];
                target.Normal.Z = Normal.Z[0];
            }
        }
    }

    public struct Convex4ContactManifoldWide : IContactManifoldWide
    {
        public Vector3Wide OffsetA0;
        public Vector3Wide OffsetA1;
        public Vector3Wide OffsetA2;
        public Vector3Wide OffsetA3;
        public Vector3Wide Normal;
        public Vector<float> Depth0;
        public Vector<float> Depth1;
        public Vector<float> Depth2;
        public Vector<float> Depth3;
        public Vector<int> FeatureId0;
        public Vector<int> FeatureId1;
        public Vector<int> FeatureId2;
        public Vector<int> FeatureId3;
        public Vector<int> Contact0Exists;
        public Vector<int> Contact1Exists;
        public Vector<int> Contact2Exists;
        public Vector<int> Contact3Exists;


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyFlipMask(ref Vector3Wide offsetB, in Vector<int> flipMask)
        {
            Vector3Wide.Negate(Normal, out var flippedNormal);
            Vector3Wide.Subtract(OffsetA0, offsetB, out var flippedA0);
            Vector3Wide.Subtract(OffsetA1, offsetB, out var flippedA1);
            Vector3Wide.Subtract(OffsetA2, offsetB, out var flippedA2);
            Vector3Wide.Subtract(OffsetA3, offsetB, out var flippedA3);
            Vector3Wide.Negate(offsetB, out var flippedOffsetB);
            Vector3Wide.ConditionalSelect(flipMask, flippedNormal, Normal, out Normal);
            Vector3Wide.ConditionalSelect(flipMask, flippedA0, OffsetA0, out OffsetA0);
            Vector3Wide.ConditionalSelect(flipMask, flippedA1, OffsetA1, out OffsetA1);
            Vector3Wide.ConditionalSelect(flipMask, flippedA2, OffsetA2, out OffsetA2);
            Vector3Wide.ConditionalSelect(flipMask, flippedA3, OffsetA3, out OffsetA3);
            Vector3Wide.ConditionalSelect(flipMask, flippedOffsetB, offsetB, out offsetB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ReadFirst(in Vector3Wide offsetB, ref ConvexContactManifold target)
        {
            target.Count = 0;
            if (Contact0Exists[0] < 0)
            {
                ++target.Count;
                target.Contact0.Offset.X = OffsetA0.X[0];
                target.Contact0.Offset.Y = OffsetA0.Y[0];
                target.Contact0.Offset.Z = OffsetA0.Z[0];
                target.Contact0.Depth = Depth0[0];
                target.Contact0.FeatureId = FeatureId0[0];
            }
            if (Contact1Exists[0] < 0)
            {
                ref var contact = ref Unsafe.Add(ref target.Contact0, target.Count++);
                contact.Offset.X = OffsetA1.X[0];
                contact.Offset.Y = OffsetA1.Y[0];
                contact.Offset.Z = OffsetA1.Z[0];
                contact.Depth = Depth1[0];
                contact.FeatureId = FeatureId1[0];
            }
            if (Contact2Exists[0] < 0)
            {
                ref var contact = ref Unsafe.Add(ref target.Contact0, target.Count++);
                contact.Offset.X = OffsetA2.X[0];
                contact.Offset.Y = OffsetA2.Y[0];
                contact.Offset.Z = OffsetA2.Z[0];
                contact.Depth = Depth2[0];
                contact.FeatureId = FeatureId2[0];
            }
            if (Contact3Exists[0] < 0)
            {
                ref var contact = ref Unsafe.Add(ref target.Contact0, target.Count++);
                contact.Offset.X = OffsetA3.X[0];
                contact.Offset.Y = OffsetA3.Y[0];
                contact.Offset.Z = OffsetA3.Z[0];
                contact.Depth = Depth3[0];
                contact.FeatureId = FeatureId3[0];
            }
            if (target.Count > 0)
            {
                target.OffsetB.X = offsetB.X[0];
                target.OffsetB.Y = offsetB.Y[0];
                target.OffsetB.Z = offsetB.Z[0];
                target.Normal.X = Normal.X[0];
                target.Normal.Y = Normal.Y[0];
                target.Normal.Z = Normal.Z[0];
            }
        }
    }

}
