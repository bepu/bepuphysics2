using BepuPhysics.Collidables;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public interface ITestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB, TPairWide>
        where TShapeA : struct, IShape where TShapeB : struct, IShape
        where TShapeWideA : struct, IShapeWide<TShapeA, TShapeWideA> where TShapeWideB : struct, IShapeWide<TShapeB, TShapeWideB>
    {
        bool HasFlipMask { get; }
        int OrientationCount { get; }
        ref Vector<int> GetFlipMask(ref TPairWide pair);
        void GetPoseOffset(ref TPairWide pair, out Vector3Wide offsetB);
        ref TShapeWideA GetShapeA(ref TPairWide pair);
        ref TShapeWideB GetShapeB(ref TPairWide pair);
        ref QuaternionWide GetOrientationA(ref TPairWide pair);
        ref QuaternionWide GetOrientationB(ref TPairWide pair);
        void Gather(ref TestPair<TShapeA, TShapeB> source, ref TPairWide target);

    }

    public struct TestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> : ITestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB, TestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB>>
        where TShapeA : struct, IShape where TShapeB : struct, IShape
        where TShapeWideA : struct, IShapeWide<TShapeA, TShapeWideA> where TShapeWideB : struct, IShapeWide<TShapeB, TShapeWideB>
    {
        public TShapeWideA A;
        public TShapeWideB B;
        public Vector<int> FlipMask;
        //TODO: This will be affected by any alternate pose representation.
        public Vector3Wide PositionA;
        public QuaternionWide OrientationA;
        public Vector3Wide PositionB;
        public QuaternionWide OrientationB;

        public bool HasFlipMask
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return true; }
        }

        public int OrientationCount
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return 2; }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector<int> GetFlipMask(ref TestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair)
        {
            return ref pair.FlipMask;
        }
        //Little unfortunate that we can't return ref of struct instances.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref TShapeWideA GetShapeA(ref TestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair)
        {
            return ref pair.A;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref TShapeWideB GetShapeB(ref TestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair)
        {
            return ref pair.B;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref QuaternionWide GetOrientationA(ref TestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair)
        {
            return ref pair.OrientationA;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref QuaternionWide GetOrientationB(ref TestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair)
        {
            return ref pair.OrientationB;
        }
        //TODO: This looks a bit too simple to have a function dedicated to it, but it is intentionally separated out in case we end up changing the pose representation.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetPoseOffset(ref TestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair, out Vector3Wide offsetB)
        {
            Vector3Wide.Subtract(ref PositionB, ref PositionB, out offsetB);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Gather(ref TestPair<TShapeA, TShapeB> source, ref TestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> target)
        {
            default(TShapeWideA).Gather(ref source.A, ref target.A);
            default(TShapeWideB).Gather(ref source.B, ref target.B);
            ref var shared = ref source.Shared;
            Unsafe.As<Vector<int>, int>(ref target.FlipMask) = shared.FlipMask;
            Unsafe.As<Vector<float>, float>(ref target.PositionA.X) = shared.PoseA.Position.X;
            Unsafe.As<Vector<float>, float>(ref target.PositionA.Y) = shared.PoseA.Position.Y;
            Unsafe.As<Vector<float>, float>(ref target.PositionA.Z) = shared.PoseA.Position.Z;
            Unsafe.As<Vector<float>, float>(ref target.OrientationA.X) = shared.PoseA.Orientation.X;
            Unsafe.As<Vector<float>, float>(ref target.OrientationA.Y) = shared.PoseA.Orientation.Y;
            Unsafe.As<Vector<float>, float>(ref target.OrientationA.Z) = shared.PoseA.Orientation.Z;
            Unsafe.As<Vector<float>, float>(ref target.OrientationA.W) = shared.PoseA.Orientation.W;
            Unsafe.As<Vector<float>, float>(ref target.PositionB.X) = shared.PoseB.Position.X;
            Unsafe.As<Vector<float>, float>(ref target.PositionB.Y) = shared.PoseB.Position.Y;
            Unsafe.As<Vector<float>, float>(ref target.PositionB.Z) = shared.PoseB.Position.Z;
            Unsafe.As<Vector<float>, float>(ref target.OrientationB.X) = shared.PoseB.Orientation.X;
            Unsafe.As<Vector<float>, float>(ref target.OrientationB.Y) = shared.PoseB.Orientation.Y;
            Unsafe.As<Vector<float>, float>(ref target.OrientationB.Z) = shared.PoseB.Orientation.Z;
            Unsafe.As<Vector<float>, float>(ref target.OrientationB.W) = shared.PoseB.Orientation.W;
        }
    }
    public struct UnflippableTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> : ITestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB, UnflippableTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB>>
        where TShapeA : struct, IShape where TShapeB : struct, IShape
        where TShapeWideA : struct, IShapeWide<TShapeA, TShapeWideA> where TShapeWideB : struct, IShapeWide<TShapeB, TShapeWideB>
    {
        public TShapeWideA A;
        public TShapeWideB B;
        //TODO: This will be affected by any alternate pose representation.
        public Vector3Wide PositionA;
        public QuaternionWide OrientationA;
        public Vector3Wide PositionB;
        public QuaternionWide OrientationB;

        public bool HasFlipMask
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return false; }
        }

        public int OrientationCount
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return 2; }
        }
        
        public ref Vector<int> GetFlipMask(ref UnflippableTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair)
        {
            throw new NotImplementedException();
        }
        //Little unfortunate that we can't return ref of struct instances.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref TShapeWideA GetShapeA(ref UnflippableTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair)
        {
            return ref pair.A;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref TShapeWideB GetShapeB(ref UnflippableTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair)
        {
            return ref pair.B;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref QuaternionWide GetOrientationA(ref UnflippableTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair)
        {
            return ref pair.OrientationA;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref QuaternionWide GetOrientationB(ref UnflippableTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair)
        {
            return ref pair.OrientationB;
        }
        //TODO: This looks a bit too simple to have a function dedicated to it, but it is intentionally separated out in case we end up changing the pose representation.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetPoseOffset(ref UnflippableTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair, out Vector3Wide offsetB)
        {
            Vector3Wide.Subtract(ref PositionB, ref PositionB, out offsetB);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Gather(ref TestPair<TShapeA, TShapeB> source, ref UnflippableTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> target)
        {
            default(TShapeWideA).Gather(ref source.A, ref target.A);
            default(TShapeWideB).Gather(ref source.B, ref target.B);
            ref var shared = ref source.Shared;
            Unsafe.As<Vector<float>, float>(ref target.PositionA.X) = shared.PoseA.Position.X;
            Unsafe.As<Vector<float>, float>(ref target.PositionA.Y) = shared.PoseA.Position.Y;
            Unsafe.As<Vector<float>, float>(ref target.PositionA.Z) = shared.PoseA.Position.Z;
            Unsafe.As<Vector<float>, float>(ref target.OrientationA.X) = shared.PoseA.Orientation.X;
            Unsafe.As<Vector<float>, float>(ref target.OrientationA.Y) = shared.PoseA.Orientation.Y;
            Unsafe.As<Vector<float>, float>(ref target.OrientationA.Z) = shared.PoseA.Orientation.Z;
            Unsafe.As<Vector<float>, float>(ref target.OrientationA.W) = shared.PoseA.Orientation.W;
            Unsafe.As<Vector<float>, float>(ref target.PositionB.X) = shared.PoseB.Position.X;
            Unsafe.As<Vector<float>, float>(ref target.PositionB.Y) = shared.PoseB.Position.Y;
            Unsafe.As<Vector<float>, float>(ref target.PositionB.Z) = shared.PoseB.Position.Z;
            Unsafe.As<Vector<float>, float>(ref target.OrientationB.X) = shared.PoseB.Orientation.X;
            Unsafe.As<Vector<float>, float>(ref target.OrientationB.Y) = shared.PoseB.Orientation.Y;
            Unsafe.As<Vector<float>, float>(ref target.OrientationB.Z) = shared.PoseB.Orientation.Z;
            Unsafe.As<Vector<float>, float>(ref target.OrientationB.W) = shared.PoseB.Orientation.W;
        }
    }
    public struct OneOrientationTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> : ITestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB, OneOrientationTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB>>
        where TShapeA : struct, IShape where TShapeB : struct, IShape
        where TShapeWideA : struct, IShapeWide<TShapeA, TShapeWideA> where TShapeWideB : struct, IShapeWide<TShapeB, TShapeWideB>
    {
        public TShapeWideA A;
        public TShapeWideB B;
        public Vector<int> FlipMask;
        //TODO: This will be affected by any alternate pose representation.
        public Vector3Wide PositionA;
        public Vector3Wide PositionB;
        public QuaternionWide OrientationB;

        public bool HasFlipMask
        {
            //Because the shapes are guaranteed to be distinct (one is apparently a sphere and the other isn't), there will always be a flip mask.
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return true; }
        }

        public int OrientationCount
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return 1; }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector<int> GetFlipMask(ref OneOrientationTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair)
        {
            return ref pair.FlipMask;
        }
        //Little unfortunate that we can't return ref of struct instances.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref TShapeWideA GetShapeA(ref OneOrientationTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair)
        {
            return ref pair.A;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref TShapeWideB GetShapeB(ref OneOrientationTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair)
        {
            return ref pair.B;
        }
        public ref QuaternionWide GetOrientationA(ref OneOrientationTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair)
        {
            throw new NotImplementedException();
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref QuaternionWide GetOrientationB(ref OneOrientationTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair)
        {
            return ref pair.OrientationB;
        }
        //TODO: This looks a bit too simple to have a function dedicated to it, but it is intentionally separated out in case we end up changing the pose representation.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetPoseOffset(ref OneOrientationTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair, out Vector3Wide offsetB)
        {
            Vector3Wide.Subtract(ref PositionB, ref PositionB, out offsetB);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Gather(ref TestPair<TShapeA, TShapeB> source, ref OneOrientationTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> target)
        {
            default(TShapeWideA).Gather(ref source.A, ref target.A);
            default(TShapeWideB).Gather(ref source.B, ref target.B);
            ref var shared = ref source.Shared;
            Unsafe.As<Vector<int>, int>(ref target.FlipMask) = shared.FlipMask;
            Unsafe.As<Vector<float>, float>(ref target.PositionA.X) = shared.PoseA.Position.X;
            Unsafe.As<Vector<float>, float>(ref target.PositionA.Y) = shared.PoseA.Position.Y;
            Unsafe.As<Vector<float>, float>(ref target.PositionA.Z) = shared.PoseA.Position.Z;
            Unsafe.As<Vector<float>, float>(ref target.PositionB.X) = shared.PoseB.Position.X;
            Unsafe.As<Vector<float>, float>(ref target.PositionB.Y) = shared.PoseB.Position.Y;
            Unsafe.As<Vector<float>, float>(ref target.PositionB.Z) = shared.PoseB.Position.Z;
            Unsafe.As<Vector<float>, float>(ref target.OrientationB.X) = shared.PoseB.Orientation.X;
            Unsafe.As<Vector<float>, float>(ref target.OrientationB.Y) = shared.PoseB.Orientation.Y;
            Unsafe.As<Vector<float>, float>(ref target.OrientationB.Z) = shared.PoseB.Orientation.Z;
            Unsafe.As<Vector<float>, float>(ref target.OrientationB.W) = shared.PoseB.Orientation.W;
        }
    }
    public struct NoOrientationTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> :
        ITestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB, NoOrientationTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB>>
        where TShapeA : struct, IShape where TShapeB : struct, IShape
        where TShapeWideA : struct, IShapeWide<TShapeA, TShapeWideA> where TShapeWideB : struct, IShapeWide<TShapeB, TShapeWideB>
    {
        public TShapeWideA A;
        public TShapeWideB B;
        //TODO: This will be affected by any alternate pose representation.
        public Vector3Wide PositionA;
        public Vector3Wide PositionB;

        public bool HasFlipMask
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return false; }
        }
        public int OrientationCount
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return 0; }
        }

        public ref Vector<int> GetFlipMask(ref NoOrientationTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair)
        {
            throw new NotImplementedException();
        }
        //Little unfortunate that we can't return ref of struct instances.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref TShapeWideA GetShapeA(ref NoOrientationTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair)
        {
            return ref pair.A;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref TShapeWideB GetShapeB(ref NoOrientationTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair)
        {
            return ref pair.B;
        }
        public ref QuaternionWide GetOrientationA(ref NoOrientationTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair)
        {
            throw new NotImplementedException();
        }
        public ref QuaternionWide GetOrientationB(ref NoOrientationTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair)
        {
            throw new NotImplementedException();
        }
        //TODO: This looks a bit too simple to have a function dedicated to it, but it is intentionally separated out in case we end up changing the pose representation.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetPoseOffset(ref NoOrientationTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair, out Vector3Wide offsetB)
        {
            Vector3Wide.Subtract(ref PositionB, ref PositionB, out offsetB);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Gather(ref TestPair<TShapeA, TShapeB> source, ref NoOrientationTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> target)
        {
            default(TShapeWideA).Gather(ref source.A, ref target.A);
            default(TShapeWideB).Gather(ref source.B, ref target.B);
            ref var shared = ref source.Shared;
            Unsafe.As<Vector<float>, float>(ref target.PositionA.X) = shared.PoseA.Position.X;
            Unsafe.As<Vector<float>, float>(ref target.PositionA.Y) = shared.PoseA.Position.Y;
            Unsafe.As<Vector<float>, float>(ref target.PositionA.Z) = shared.PoseA.Position.Z;
            Unsafe.As<Vector<float>, float>(ref target.PositionB.X) = shared.PoseB.Position.X;
            Unsafe.As<Vector<float>, float>(ref target.PositionB.Y) = shared.PoseB.Position.Y;
            Unsafe.As<Vector<float>, float>(ref target.PositionB.Z) = shared.PoseB.Position.Z;
        }
    }

    public interface IPairTester<TShapeWideA, TShapeWideB, TManifoldWideType>
    {
        //Note that, while the interface requires all three of these implementations, concrete implementers will only ever have one defined or called.
        //Including the other unused functions is just here to simplify its use in the batch execution loop.
        void Test(ref TShapeWideA a, ref TShapeWideB b, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, out TManifoldWideType manifoldWide);
        void Test(ref TShapeWideA a, ref TShapeWideB b, ref Vector3Wide offsetB, ref QuaternionWide orientationB, out TManifoldWideType manifoldWide);
        void Test(ref TShapeWideA a, ref TShapeWideB b, ref Vector3Wide offsetB, out TManifoldWideType manifoldWide);
    }

    public interface IContactManifoldWide<TContactManifoldWide> where TContactManifoldWide : IContactManifoldWide<TContactManifoldWide>
    {
        void ApplyFlipMask(ref TContactManifoldWide manifold, ref Vector3Wide offsetB, ref Vector<int> flipMask);
        void PrepareManifoldForScatter(out ContactManifold manifold);
        void Scatter(ref TContactManifoldWide source, ref Vector3Wide offsetB, ref ContactManifold target);
    }

    class CollisionBatchExecutors
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void ExecuteBatch<TContinuations, TFilters, TShapeA, TShapeWideA, TShapeB, TShapeWideB, TPairWide, TManifoldWide, TPairTester>
            (ref UntypedList batch, ref StreamingBatcher batcher, ref TContinuations continuations, ref TFilters filters)
            where TShapeA : struct, IShape where TShapeB : struct, IShape
            where TShapeWideA : struct, IShapeWide<TShapeA, TShapeWideA> where TShapeWideB : struct, IShapeWide<TShapeB, TShapeWideB>
            where TPairWide : struct, ITestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB, TPairWide>
            where TPairTester : struct, IPairTester<TShapeWideA, TShapeWideB, TManifoldWide>
            where TManifoldWide : IContactManifoldWide<TManifoldWide>
            where TContinuations : IContinuations
        {
            ref var start = ref Unsafe.As<byte, TestPair<TShapeA, TShapeB>>(ref batch.Buffer[0]);
            var pairWide = default(TPairWide); //With any luck, the compiler will get rid of the unnecessary zero init.
            default(TManifoldWide).PrepareManifoldForScatter(out var manifold);
            TManifoldWide manifoldWide;

            for (int i = 0; i < batch.Count; i += Vector<float>.Count)
            {
                ref var bundleStart = ref Unsafe.Add(ref start, i);
                int countInBundle = batch.Count - i;
                if (countInBundle > Vector<float>.Count)
                    countInBundle = Vector<float>.Count;

                //TODO: If we have gather intrinsics, they might be a win.
                for (int j = 0; j < countInBundle; ++j)
                {
                    //Reposition the wide reference so that the first lane of the alias matches the target lane in the underlying memory.
                    ref var offsetFloatStart = ref Unsafe.Add(ref Unsafe.As<TPairWide, float>(ref pairWide), j);
                    ref var target = ref Unsafe.As<float, TPairWide>(ref offsetFloatStart);
                    default(TPairWide).Gather(ref Unsafe.Add(ref bundleStart, j), ref target);
                }

                default(TPairWide).GetPoseOffset(ref pairWide, out var offsetB);
                if (default(TPairWide).OrientationCount == 2)
                {
                    default(TPairTester).Test(
                        ref default(TPairWide).GetShapeA(ref pairWide),
                        ref default(TPairWide).GetShapeB(ref pairWide),
                        ref offsetB,
                        ref default(TPairWide).GetOrientationA(ref pairWide),
                        ref default(TPairWide).GetOrientationB(ref pairWide),
                        out manifoldWide);
                }
                else if (default(TPairWide).OrientationCount == 1)
                {
                    //Note that, in the event that there is only one orientation, it belongs to the second shape.
                    //The only shape that doesn't need orientation is a sphere, and it will be in slot A by convention.
                    Debug.Assert(typeof(TShapeWideA) == typeof(SphereWide));
                    default(TPairTester).Test(
                        ref default(TPairWide).GetShapeA(ref pairWide),
                        ref default(TPairWide).GetShapeB(ref pairWide),
                        ref offsetB,
                        ref default(TPairWide).GetOrientationB(ref pairWide),
                        out manifoldWide);
                }
                else
                {
                    Debug.Assert(default(TPairWide).OrientationCount == 0);
                    Debug.Assert(typeof(TShapeWideA) == typeof(SphereWide) && typeof(TShapeWideB) == typeof(SphereWide), "No orientation implies a special case involving two spheres.");
                    //Really, this could be made into a direct special case, but eh.
                    default(TPairTester).Test(
                        ref default(TPairWide).GetShapeA(ref pairWide),
                        ref default(TPairWide).GetShapeB(ref pairWide),
                        ref offsetB,
                        out manifoldWide);
                }

                //Flip back any contacts associated with pairs which had to be flipped for shape order.
                if (default(TPairWide).HasFlipMask)
                {
                    default(TManifoldWide).ApplyFlipMask(ref manifoldWide, ref offsetB, ref default(TPairWide).GetFlipMask(ref pairWide));
                }

                for (int j = 0; j < countInBundle; ++j)
                {
                    ref var offsetFloatStart = ref Unsafe.Add(ref Unsafe.As<TManifoldWide, float>(ref manifoldWide), j);
                    ref var source = ref Unsafe.As<float, TManifoldWide>(ref offsetFloatStart);
                    default(TManifoldWide).Scatter(ref source, ref offsetB, ref manifold);
                    continuations.Notify(Unsafe.Add(ref bundleStart, j).Shared.Continuation, &manifold);
                    if (typeof(TManifoldWide) == typeof(Convex1ContactManifoldWide))
                    {
                        Debug.Assert(manifold.ContactCount == 1 && manifold.Convex, "The notify function should not modify the provided manifold reference.");
                    }
                }
            }

        }
    }
}
