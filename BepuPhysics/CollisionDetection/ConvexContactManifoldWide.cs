using BepuPhysics.CollisionDetection.CollisionTasks;
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
        public Vector<int> Count;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyFlipMask(ref Vector3Wide offsetB, ref Vector<int> flipMask)
        {
            Vector3Wide.Negate(ref Normal, out var flippedNormal);
            Vector3Wide.Subtract(ref OffsetA, ref offsetB, out var flippedContactPosition);
            Vector3Wide.Negate(ref offsetB, out var flippedOffsetB);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedNormal, ref Normal, out Normal);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedContactPosition, ref OffsetA, out OffsetA);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedOffsetB, ref offsetB, out offsetB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Scatter(ref Vector3Wide offsetB, ref ConvexContactManifold target)
        {
            target.Count = Count[0];
            if (target.Count > 0)
            {
                target.OffsetB.X = offsetB.X[0];
                target.OffsetB.Y = offsetB.Y[0];
                target.OffsetB.Z = offsetB.Z[0];
                target.Contact0.Offset.X = OffsetA.X[0];
                target.Contact0.Offset.Y = OffsetA.Y[0];
                target.Contact0.Offset.Z = OffsetA.Z[0];
                target.Contact0.Depth = Depth[0];
                target.Contact0.FeatureId = 0;
                Debug.Assert(target.Count == 1);

                target.Normal.X = Normal.X[0];
                target.Normal.Y = Normal.Y[0];
                target.Normal.Z = Normal.Z[0];
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
        public Vector<int> Count;


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyFlipMask(ref Vector3Wide offsetB, ref Vector<int> flipMask)
        {
            Vector3Wide.Negate(ref Normal, out var flippedNormal);
            Vector3Wide.Subtract(ref OffsetA0, ref offsetB, out var flippedA0);
            Vector3Wide.Subtract(ref OffsetA1, ref offsetB, out var flippedA1);
            Vector3Wide.Negate(ref offsetB, out var flippedOffsetB);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedNormal, ref Normal, out Normal);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedA0, ref OffsetA0, out OffsetA0);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedA1, ref OffsetA1, out OffsetA1);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedOffsetB, ref offsetB, out offsetB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Scatter(ref Vector3Wide offsetB, ref ConvexContactManifold target)
        {
            target.Count = Count[0];
            if (target.Count > 0)
            {
                target.OffsetB.X = offsetB.X[0];
                target.OffsetB.Y = offsetB.Y[0];
                target.OffsetB.Z = offsetB.Z[0];
                target.Contact0.Offset.X = OffsetA0.X[0];
                target.Contact0.Offset.Y = OffsetA0.Y[0];
                target.Contact0.Offset.Z = OffsetA0.Z[0];
                target.Contact0.Depth = Depth0[0];
                target.Contact0.FeatureId = FeatureId0[0];
                if (target.Count > 1)
                {
                    target.Contact1.Offset.X = OffsetA1.X[0];
                    target.Contact1.Offset.Y = OffsetA1.Y[0];
                    target.Contact1.Offset.Z = OffsetA1.Z[0];
                    target.Contact1.Depth = Depth1[0];
                    target.Contact1.FeatureId = FeatureId1[0];
                    Debug.Assert(target.Count == 2);
                }
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
        public Vector<int> Count;


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyFlipMask(ref Vector3Wide offsetB, ref Vector<int> flipMask)
        {
            Vector3Wide.Negate(ref Normal, out var flippedNormal);
            Vector3Wide.Subtract(ref OffsetA0, ref offsetB, out var flippedA0);
            Vector3Wide.Subtract(ref OffsetA1, ref offsetB, out var flippedA1);
            Vector3Wide.Subtract(ref OffsetA2, ref offsetB, out var flippedA2);
            Vector3Wide.Subtract(ref OffsetA3, ref offsetB, out var flippedA3);
            Vector3Wide.Negate(ref offsetB, out var flippedOffsetB);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedNormal, ref Normal, out Normal);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedA0, ref OffsetA0, out OffsetA0);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedA1, ref OffsetA1, out OffsetA1);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedA2, ref OffsetA2, out OffsetA2);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedA3, ref OffsetA3, out OffsetA3);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedOffsetB, ref offsetB, out offsetB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Scatter(ref Vector3Wide offsetB, ref ConvexContactManifold target)
        {
            target.Count = Count[0];
            if (target.Count > 0)
            {
                target.OffsetB.X = offsetB.X[0];
                target.OffsetB.Y = offsetB.Y[0];
                target.OffsetB.Z = offsetB.Z[0];
                target.Contact0.Offset.X = OffsetA0.X[0];
                target.Contact0.Offset.Y = OffsetA0.Y[0];
                target.Contact0.Offset.Z = OffsetA0.Z[0];
                target.Contact0.Depth = Depth0[0];
                target.Contact0.FeatureId = FeatureId0[0];
                if (target.Count > 1)
                {
                    target.Contact1.Offset.X = OffsetA1.X[0];
                    target.Contact1.Offset.Y = OffsetA1.Y[0];
                    target.Contact1.Offset.Z = OffsetA1.Z[0];
                    target.Contact1.Depth = Depth1[0];
                    target.Contact1.FeatureId = FeatureId1[0];
                    if (target.Count > 2)
                    {
                        target.Contact2.Offset.X = OffsetA2.X[0];
                        target.Contact2.Offset.Y = OffsetA2.Y[0];
                        target.Contact2.Offset.Z = OffsetA2.Z[0];
                        target.Contact2.Depth = Depth2[0];
                        target.Contact2.FeatureId = FeatureId2[0];
                        if (target.Count > 3)
                        {
                            target.Contact3.Offset.X = OffsetA3.X[0];
                            target.Contact3.Offset.Y = OffsetA3.Y[0];
                            target.Contact3.Offset.Z = OffsetA3.Z[0];
                            target.Contact3.Depth = Depth3[0];
                            target.Contact3.FeatureId = FeatureId3[0];
                            Debug.Assert(target.Count == 4);
                        }
                    }
                }
                target.Normal.X = Normal.X[0];
                target.Normal.Y = Normal.Y[0];
                target.Normal.Z = Normal.Z[0];
            }
        }
    }

}
