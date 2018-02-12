using BepuPhysics.CollisionDetection.CollisionTasks;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection
{
    public struct Convex1ContactManifoldWide : IContactManifoldWide
    {
        public Vector3Wide OffsetA0;
        public Vector3Wide Normal;
        public Vector<float> Depth;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyFlipMask(ref Vector3Wide offsetB, ref Vector<int> flipMask)
        {
            Vector3Wide.Negate(ref Normal, out var flippedNormal);
            Vector3Wide.Subtract(ref OffsetA0, ref offsetB, out var flippedContactPosition);
            Vector3Wide.Negate(ref offsetB, out var flippedOffsetB);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedNormal, ref Normal, out Normal);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedContactPosition, ref OffsetA0, out OffsetA0);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedOffsetB, ref offsetB, out offsetB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void PrepareManifoldForScatter(out ContactManifold manifold)
        {
            manifold = default;
            manifold.SetConvexityAndCount(1, true);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Scatter(ref Vector3Wide offsetB, ref ContactManifold target)
        {
            target.OffsetB.X = offsetB.X[0];
            target.OffsetB.Y = offsetB.Y[0];
            target.OffsetB.Z = offsetB.Z[0];
            target.Offset0.X = OffsetA0.X[0];
            target.Offset0.Y = OffsetA0.Y[0];
            target.Offset0.Z = OffsetA0.Z[0];
            target.Depth0 = Depth[0];
            target.ConvexNormal.X = Normal.X[0];
            target.ConvexNormal.Y = Normal.Y[0];
            target.ConvexNormal.Z = Normal.Z[0];
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
        public void PrepareManifoldForScatter(out ContactManifold manifold)
        {
            manifold = default;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Scatter(ref Vector3Wide offsetB, ref ContactManifold target)
        {
            var count = Count[0];
            target.SetConvexityAndCount(count, true);
            target.OffsetB.X = offsetB.X[0];
            target.OffsetB.Y = offsetB.Y[0];
            target.OffsetB.Z = offsetB.Z[0];
            target.Offset0.X = OffsetA0.X[0];
            target.Offset0.Y = OffsetA0.Y[0];
            target.Offset0.Z = OffsetA0.Z[0];
            target.Depth0 = Depth0[0];
            target.FeatureId0 = FeatureId0[0];
            if (count > 1)
            {
                target.Offset1.X = OffsetA1.X[0];
                target.Offset1.Y = OffsetA1.Y[0];
                target.Offset1.Z = OffsetA1.Z[0];
                target.Depth1 = Depth1[0];
                target.FeatureId1 = FeatureId1[0];
            }
            target.ConvexNormal.X = Normal.X[0];
            target.ConvexNormal.Y = Normal.Y[0];
            target.ConvexNormal.Z = Normal.Z[0];
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
        public void PrepareManifoldForScatter(out ContactManifold manifold)
        {
            manifold = default;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Scatter(ref Vector3Wide offsetB, ref ContactManifold target)
        {
            var count = Count[0];
            target.SetConvexityAndCount(count, true);
            target.OffsetB.X = offsetB.X[0];
            target.OffsetB.Y = offsetB.Y[0];
            target.OffsetB.Z = offsetB.Z[0];
            target.Offset0.X = OffsetA0.X[0];
            target.Offset0.Y = OffsetA0.Y[0];
            target.Offset0.Z = OffsetA0.Z[0];
            target.Depth0 = Depth0[0];
            target.FeatureId0 = FeatureId0[0];
            if (count > 1)
            {
                target.Offset1.X = OffsetA1.X[0];
                target.Offset1.Y = OffsetA1.Y[0];
                target.Offset1.Z = OffsetA1.Z[0];
                target.Depth1 = Depth1[0];
                target.FeatureId1 = FeatureId1[0];
                if (count > 2)
                {
                    target.Offset2.X = OffsetA2.X[0];
                    target.Offset2.Y = OffsetA2.Y[0];
                    target.Offset2.Z = OffsetA2.Z[0];
                    target.Depth2 = Depth2[0];
                    target.FeatureId2 = FeatureId2[0];
                    if (count > 3)
                    {
                        target.Offset2.X = OffsetA2.X[0];
                        target.Offset2.Y = OffsetA2.Y[0];
                        target.Offset2.Z = OffsetA2.Z[0];
                        target.Depth2 = Depth3[0];
                        target.FeatureId3 = FeatureId3[0];
                    }
                }
            }
            target.ConvexNormal.X = Normal.X[0];
            target.ConvexNormal.Y = Normal.Y[0];
            target.ConvexNormal.Z = Normal.Z[0];
        }
    }

}
