using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.SweepTasks;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using DemoRenderer;
using DemoRenderer.Constraints;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using Quaternion = BepuUtilities.Quaternion;

namespace Demos.SpecializedTests
{
    /// <summary>
    /// Debug helper class for storing the simplexes associated with minkowski space method execution.
    /// </summary>
    public class MinkowskiSimplexes
    {
        public QuickList<int> SimplexIndices;
        public QuickList<Vector3> Points;

        BufferPool pool;

        public int SimplexCount { get { return SimplexIndices.Count; } }

        public MinkowskiSimplexes(BufferPool pool, int initialSimplexCapacity = 2048)
        {
            this.pool = pool;
            SimplexIndices = new QuickList<int>(initialSimplexCapacity, pool);
            Points = new QuickList<Vector3>(initialSimplexCapacity * 4, pool);
        }

        public Buffer<Vector3> AllocateSimplex(int count)
        {
            var newCount = Points.Count + count;
            Points.EnsureCapacity(newCount, pool);
            var simplexPoints = Points.Span.Slice(Points.Count, count);
            SimplexIndices.Add(Points.Count, pool);
            Points.Count += count;
            return simplexPoints;
        }

        public Buffer<Vector3> GetSimplex(int simplexIndex)
        {
            var simplexStart = SimplexIndices[simplexIndex];
            var simplexEnd = simplexIndex == SimplexIndices.Count ? Points.Count : SimplexIndices[simplexIndex + 1];
            return Points.Span.Slice(simplexStart, simplexEnd - simplexStart);
        }

        public void Clear()
        {
            SimplexIndices.Count = 0;
            Points.Count = 0;
        }

        public void Dispose()
        {
            SimplexIndices.Dispose(pool);
            Points.Dispose(pool);
        }
    }

    public static class SimplexVisualizer
    {
        public static void Draw(Renderer renderer, Buffer<Vector3> simplex, in Vector3 position, in Vector3 lineColor, in Vector3 backgroundColor)
        {
            var packedLineColor = Helpers.PackColor(lineColor);
            var packedBackgroundColor = Helpers.PackColor(backgroundColor);
            for (int i = 0; i < simplex.Length; ++i)
            {
                for (int j = i + 1; j < simplex.Length; ++j)
                {
                    renderer.Lines.Allocate() = new LineInstance(simplex[i] + position, simplex[j] + position, packedLineColor, packedBackgroundColor);
                }
            }
        }
    }

    public static class MinkowskiShapeVisualizer
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void FindSupport<TShapeA, TShapeWideA, TSupportFinderA, TShapeB, TShapeWideB, TSupportFinderB>
            (in TShapeWideA a, in TShapeWideB b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB, ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB, in Vector3Wide direction, out Vector3Wide support)
            where TShapeA : IConvexShape
            where TShapeWideA : IShapeWide<TShapeA>
            where TSupportFinderA : ISupportFinder<TShapeA, TShapeWideA>
            where TShapeB : IConvexShape
            where TShapeWideB : IShapeWide<TShapeB>
            where TSupportFinderB : ISupportFinder<TShapeB, TShapeWideB>
        {
            //support(N, A) - support(-N, B)
            supportFinderA.ComputeLocalSupport(a, direction, out var extremeA);
            Vector3Wide.Negate(direction, out var negatedDirection);
            supportFinderB.ComputeSupport(b, localOrientationB, negatedDirection, out var extremeB);
            Vector3Wide.Add(extremeB, localOffsetB, out extremeB);

            Vector3Wide.Subtract(extremeA, extremeB, out support);
        }

        public static Buffer<LineInstance> CreateLines<TShapeA, TShapeWideA, TSupportFinderA, TShapeB, TShapeWideB, TSupportFinderB>(
            in TShapeA a, in TShapeB b, in RigidPose poseA, in RigidPose poseB, int sampleCount,
            float lineLength, in Vector3 lineColor,
            float originLength, in Vector3 originColor, in Vector3 backgroundColor, in Vector3 basePosition, BufferPool pool)
            where TShapeA : IConvexShape
            where TShapeWideA : IShapeWide<TShapeA>
            where TSupportFinderA : ISupportFinder<TShapeA, TShapeWideA>
            where TShapeB : IConvexShape
            where TShapeWideB : IShapeWide<TShapeB>
            where TSupportFinderB : ISupportFinder<TShapeB, TShapeWideB>
        {
            var aWide = default(TShapeWideA);
            var bWide = default(TShapeWideB);
            aWide.Broadcast(a);
            bWide.Broadcast(b);
            var worldOffsetB = poseB.Position - poseA.Position;
            var localOrientationB = Matrix3x3.CreateFromQuaternion(Quaternion.Concatenate(poseB.Orientation, Quaternion.Conjugate(poseA.Orientation)));
            var localOffsetB = Quaternion.Transform(worldOffsetB, Quaternion.Conjugate(poseA.Orientation));
            Vector3Wide.Broadcast(localOffsetB, out var localOffsetBWide);
            Matrix3x3Wide.Broadcast(localOrientationB, out var localOrientationBWide);
            var supportFinderA = default(TSupportFinderA);
            var supportFinderB = default(TSupportFinderB);
            var inverseSampleCount = 1f / sampleCount;
            pool.Take<LineInstance>(sampleCount + 3, out var lines);
            var packedLineColor = Helpers.PackColor(lineColor);
            var packedBackgroundColor = Helpers.PackColor(backgroundColor);
            for (int i = 0; i < sampleCount; ++i)
            {
                var index = i + 0.5f;
                var phi = MathF.Acos(1f - 2f * index * inverseSampleCount);
                var theta = (MathF.PI * (1f + 2.2360679775f)) * index;
                var sinPhi = MathF.Sin(phi);
                var sampleDirection = new Vector3(MathF.Cos(theta) * sinPhi, MathF.Sin(theta) * sinPhi, MathF.Cos(phi));
                Vector3Wide.Broadcast(sampleDirection, out var sampleDirectionWide);
                //Could easily use the fact that this is vectorized, but it's marginally easier not to!
                FindSupport<TShapeA, TShapeWideA, TSupportFinderA, TShapeB, TShapeWideB, TSupportFinderB>(aWide, bWide, localOffsetBWide, localOrientationBWide, ref supportFinderA, ref supportFinderB, sampleDirectionWide, out var supportWide);
                Vector3Wide.ReadSlot(ref supportWide, 0, out var support);
                lines[i] = new LineInstance(basePosition + support, basePosition + support - sampleDirection * lineLength, packedLineColor, packedBackgroundColor);
            }
            var packedOriginColor = Helpers.PackColor(originColor);
            lines[sampleCount] = new LineInstance(basePosition - new Vector3(originLength, 0, 0), basePosition + new Vector3(originLength, 0, 0), packedOriginColor, packedBackgroundColor);
            lines[sampleCount + 1] = new LineInstance(basePosition - new Vector3(0, originLength, 0), basePosition + new Vector3(0, originLength, 0), packedOriginColor, packedBackgroundColor);
            lines[sampleCount + 2] = new LineInstance(basePosition - new Vector3(0, 0, originLength), basePosition + new Vector3(0, 0, originLength), packedOriginColor, packedBackgroundColor);
            return lines.Slice(0, sampleCount + 3);
        }

        public static void Draw(Buffer<LineInstance> lines, Renderer renderer)
        {
            for (int i = 0; i < lines.Length; ++i)
            {
                renderer.Lines.Allocate() = lines[i];
            }
        }
    }
}
