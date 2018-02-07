using BepuPhysics.CollisionDetection.CollisionTasks;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection
{
    public struct Convex1ContactManifoldWide : IContactManifoldWide<Convex1ContactManifoldWide>
    {
        public Vector3Wide OffsetA0;
        public Vector3Wide Normal;
        public Vector<float> Depth;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyFlipMask(ref Convex1ContactManifoldWide manifold, ref Vector3Wide offsetB, ref Vector<int> flipMask)
        {
            Vector3Wide.Negate(ref manifold.Normal, out var flippedNormal);
            Vector3Wide.Subtract(ref manifold.OffsetA0, ref offsetB, out var flippedContactPosition);
            Vector3Wide.Negate(ref offsetB, out var flippedOffsetB);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedNormal, ref manifold.Normal, out manifold.Normal);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedContactPosition, ref manifold.OffsetA0, out manifold.OffsetA0);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedOffsetB, ref offsetB, out offsetB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void PrepareManifoldForScatter(out ContactManifold manifold)
        {
            manifold = default;
            manifold.SetConvexityAndCount(1, true);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Scatter(ref Convex1ContactManifoldWide source, ref Vector3Wide offsetB, ref ContactManifold target)
        {
            target.OffsetB.X = offsetB.X[0];
            target.OffsetB.Y = offsetB.Y[0];
            target.OffsetB.Z = offsetB.Z[0];
            target.Offset0.X = source.OffsetA0.X[0];
            target.Offset0.Y = source.OffsetA0.Y[0];
            target.Offset0.Z = source.OffsetA0.Z[0];
            target.Depth0 = source.Depth[0];
            target.ConvexNormal.X = source.Normal.X[0];
            target.ConvexNormal.Y = source.Normal.Y[0];
            target.ConvexNormal.Z = source.Normal.Z[0];
        }        
    }
    public struct Convex2ContactManifoldWide : IContactManifoldWide<Convex2ContactManifoldWide>
    {
        public Vector3Wide OffsetA0;
        public Vector3Wide OffsetA1;
        public Vector3Wide Normal;
        public Vector<float> Depth0;
        public Vector<float> Depth1;
        public Vector<int> Count;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyFlipMask(ref Convex2ContactManifoldWide manifold, ref Vector3Wide offsetB, ref Vector<int> flipMask)
        {
            Vector3Wide.Negate(ref manifold.Normal, out var flippedNormal);
            Vector3Wide.Subtract(ref manifold.OffsetA0, ref offsetB, out var flippedA0);
            Vector3Wide.Subtract(ref manifold.OffsetA1, ref offsetB, out var flippedA1);
            Vector3Wide.Negate(ref offsetB, out var flippedOffsetB);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedNormal, ref manifold.Normal, out manifold.Normal);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedA0, ref manifold.OffsetA0, out manifold.OffsetA0);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedA1, ref manifold.OffsetA1, out manifold.OffsetA1);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedOffsetB, ref offsetB, out offsetB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void PrepareManifoldForScatter(out ContactManifold manifold)
        {
            manifold = default;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Scatter(ref Convex2ContactManifoldWide source, ref Vector3Wide offsetB, ref ContactManifold target)
        {
            var count = source.Count[0];
            target.SetConvexityAndCount(count, true);
            target.OffsetB.X = offsetB.X[0];
            target.OffsetB.Y = offsetB.Y[0];
            target.OffsetB.Z = offsetB.Z[0];
            target.Offset0.X = source.OffsetA0.X[0];
            target.Offset0.Y = source.OffsetA0.Y[0];
            target.Offset0.Z = source.OffsetA0.Z[0];
            target.Depth0 = source.Depth0[0];
            if (count > 1)
            {
                target.Offset1.X = source.OffsetA1.X[0];
                target.Offset1.Y = source.OffsetA1.Y[0];
                target.Offset1.Z = source.OffsetA1.Z[0];
                target.Depth1 = source.Depth1[0];
            }
            target.ConvexNormal.X = source.Normal.X[0];
            target.ConvexNormal.Y = source.Normal.Y[0];
            target.ConvexNormal.Z = source.Normal.Z[0];
        }
    }

    public struct Convex4ContactManifoldWide : IContactManifoldWide<Convex4ContactManifoldWide>
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
        public Vector<int> Count;


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyFlipMask(ref Convex4ContactManifoldWide manifold, ref Vector3Wide offsetB, ref Vector<int> flipMask)
        {
            Vector3Wide.Negate(ref manifold.Normal, out var flippedNormal);
            Vector3Wide.Subtract(ref manifold.OffsetA0, ref offsetB, out var flippedA0);
            Vector3Wide.Subtract(ref manifold.OffsetA1, ref offsetB, out var flippedA1);
            Vector3Wide.Subtract(ref manifold.OffsetA2, ref offsetB, out var flippedA2);
            Vector3Wide.Subtract(ref manifold.OffsetA3, ref offsetB, out var flippedA3);
            Vector3Wide.Negate(ref offsetB, out var flippedOffsetB);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedNormal, ref manifold.Normal, out manifold.Normal);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedA0, ref manifold.OffsetA0, out manifold.OffsetA0);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedA1, ref manifold.OffsetA1, out manifold.OffsetA1);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedA2, ref manifold.OffsetA2, out manifold.OffsetA2);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedA3, ref manifold.OffsetA3, out manifold.OffsetA3);
            Vector3Wide.ConditionalSelect(ref flipMask, ref flippedOffsetB, ref offsetB, out offsetB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void PrepareManifoldForScatter(out ContactManifold manifold)
        {
            manifold = default;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Scatter(ref Convex4ContactManifoldWide source, ref Vector3Wide offsetB, ref ContactManifold target)
        {
            var count = source.Count[0];
            target.SetConvexityAndCount(count, true);
            target.OffsetB.X = offsetB.X[0];
            target.OffsetB.Y = offsetB.Y[0];
            target.OffsetB.Z = offsetB.Z[0];
            target.Offset0.X = source.OffsetA0.X[0];
            target.Offset0.Y = source.OffsetA0.Y[0];
            target.Offset0.Z = source.OffsetA0.Z[0];
            target.Depth0 = source.Depth0[0];
            if (count > 1)
            {
                target.Offset1.X = source.OffsetA1.X[0];
                target.Offset1.Y = source.OffsetA1.Y[0];
                target.Offset1.Z = source.OffsetA1.Z[0];
                target.Depth1 = source.Depth1[0];
                if (count > 2)
                {
                    target.Offset2.X = source.OffsetA2.X[0];
                    target.Offset2.Y = source.OffsetA2.Y[0];
                    target.Offset2.Z = source.OffsetA2.Z[0];
                    target.Depth2 = source.Depth2[0];
                    if (count > 3)
                    {
                        target.Offset2.X = source.OffsetA2.X[0];
                        target.Offset2.Y = source.OffsetA2.Y[0];
                        target.Offset2.Z = source.OffsetA2.Z[0];
                        target.Depth2 = source.Depth3[0];
                    }
                }
            }
            target.ConvexNormal.X = source.Normal.X[0];
            target.ConvexNormal.Y = source.Normal.Y[0];
            target.ConvexNormal.Z = source.Normal.Z[0];
        }
    }

}
