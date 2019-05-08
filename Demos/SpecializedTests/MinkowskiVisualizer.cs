using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
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
    public static class SimplexVisualizer
    {
        public static void Draw(Renderer renderer, Buffer<Vector3> simplex, in Vector3 position, in Vector3 lineColor, in Vector3 backgroundColor)
        {
            var packedLineColor = Helpers.PackColor(lineColor);
            var packedBackgroundColor = Helpers.PackColor(backgroundColor);
            if (simplex.Length == 1)
            {
                renderer.Lines.Allocate() = new LineInstance(simplex[0], simplex[0], packedLineColor, packedBackgroundColor);
            }
            else
            {
                for (int i = 0; i < simplex.Length; ++i)
                {
                    for (int j = i + 1; j < simplex.Length; ++j)
                    {
                        renderer.Lines.Allocate() = new LineInstance(simplex[i] + position, simplex[j] + position, packedLineColor, packedBackgroundColor);
                    }
                }

            }
        }
    }

    public static class MinkowskiShapeVisualizer
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void FindSupport<TShapeA, TShapeWideA, TSupportFinderA, TShapeB, TShapeWideB, TSupportFinderB>
            (in TShapeWideA a, in TShapeWideB b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB, ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB, in Vector3Wide direction,
            in Vector<int> terminatedLanes, out Vector3Wide support)
            where TShapeA : IConvexShape
            where TShapeWideA : IShapeWide<TShapeA>
            where TSupportFinderA : ISupportFinder<TShapeA, TShapeWideA>
            where TShapeB : IConvexShape
            where TShapeWideB : IShapeWide<TShapeB>
            where TSupportFinderB : ISupportFinder<TShapeB, TShapeWideB>
        {
            //support(N, A) - support(-N, B)
            supportFinderA.ComputeLocalSupport(a, direction, terminatedLanes, out var extremeA);
            Vector3Wide.Negate(direction, out var negatedDirection);
            supportFinderB.ComputeSupport(b, localOrientationB, negatedDirection, terminatedLanes, out var extremeB);
            Vector3Wide.Add(extremeB, localOffsetB, out extremeB);

            Vector3Wide.Subtract(extremeA, extremeB, out support);
        }

        public unsafe static Buffer<LineInstance> CreateLines<TShapeA, TShapeWideA, TSupportFinderA, TShapeB, TShapeWideB, TSupportFinderB>(
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
            if(aWide.InternalAllocationSize > 0)
            {
                var memory = stackalloc byte[aWide.InternalAllocationSize];
                aWide.Initialize(new RawBuffer(memory, aWide.InternalAllocationSize));
            }
            if (bWide.InternalAllocationSize > 0)
            {
                var memory = stackalloc byte[bWide.InternalAllocationSize];
                bWide.Initialize(new RawBuffer(memory, bWide.InternalAllocationSize));
            }
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
                FindSupport<TShapeA, TShapeWideA, TSupportFinderA, TShapeB, TShapeWideB, TSupportFinderB>(aWide, bWide, localOffsetBWide, localOrientationBWide, ref supportFinderA, ref supportFinderB, sampleDirectionWide, Vector<int>.Zero, out var supportWide);
                Vector3Wide.ReadSlot(ref supportWide, 0, out var support);
                lines[i] = new LineInstance(basePosition + support, basePosition + support - sampleDirection * lineLength, packedLineColor, packedBackgroundColor);
            }
            var packedOriginColor = Helpers.PackColor(originColor);
            lines[sampleCount] = new LineInstance(basePosition - new Vector3(originLength, 0, 0), basePosition + new Vector3(originLength, 0, 0), packedOriginColor, packedBackgroundColor);
            lines[sampleCount + 1] = new LineInstance(basePosition - new Vector3(0, originLength, 0), basePosition + new Vector3(0, originLength, 0), packedOriginColor, packedBackgroundColor);
            lines[sampleCount + 2] = new LineInstance(basePosition - new Vector3(0, 0, originLength), basePosition + new Vector3(0, 0, originLength), packedOriginColor, packedBackgroundColor);
            return lines;
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
