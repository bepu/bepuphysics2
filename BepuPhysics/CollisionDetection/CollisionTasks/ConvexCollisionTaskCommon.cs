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
        where TShapeWideA : struct, IShapeWide<TShapeA> where TShapeWideB : struct, IShapeWide<TShapeB>
    {
        bool HasFlipMask { get; }
        int OrientationCount { get; }
        void GetPoseOffset(out Vector3Wide offsetB);
        //Note the pair parameter. This is just to get around the fact that you cannot ref return struct fields like you can with classes, at least right now
        ref Vector<int> GetFlipMask(ref TPairWide pair);
        ref TShapeWideA GetShapeA(ref TPairWide pair);
        ref TShapeWideB GetShapeB(ref TPairWide pair);
        ref QuaternionWide GetOrientationA(ref TPairWide pair);
        ref QuaternionWide GetOrientationB(ref TPairWide pair);
        void Gather(ref TestPair<TShapeA, TShapeB> source);

    }

    public struct TestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> : ITestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB, TestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB>>
        where TShapeA : struct, IShape where TShapeB : struct, IShape
        where TShapeWideA : struct, IShapeWide<TShapeA> where TShapeWideB : struct, IShapeWide<TShapeB>
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
        public void GetPoseOffset(out Vector3Wide offsetB)
        {
            Vector3Wide.Subtract(ref PositionB, ref PositionA, out offsetB);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Gather(ref TestPair<TShapeA, TShapeB> source)
        {
            A.Gather(ref source.A);
            B.Gather(ref source.B);
            ref var shared = ref source.Shared;
            Unsafe.As<Vector<int>, int>(ref FlipMask) = shared.FlipMask;
            Unsafe.As<Vector<float>, float>(ref PositionA.X) = shared.PoseA.Position.X;
            Unsafe.As<Vector<float>, float>(ref PositionA.Y) = shared.PoseA.Position.Y;
            Unsafe.As<Vector<float>, float>(ref PositionA.Z) = shared.PoseA.Position.Z;
            Unsafe.As<Vector<float>, float>(ref OrientationA.X) = shared.PoseA.Orientation.X;
            Unsafe.As<Vector<float>, float>(ref OrientationA.Y) = shared.PoseA.Orientation.Y;
            Unsafe.As<Vector<float>, float>(ref OrientationA.Z) = shared.PoseA.Orientation.Z;
            Unsafe.As<Vector<float>, float>(ref OrientationA.W) = shared.PoseA.Orientation.W;
            Unsafe.As<Vector<float>, float>(ref PositionB.X) = shared.PoseB.Position.X;
            Unsafe.As<Vector<float>, float>(ref PositionB.Y) = shared.PoseB.Position.Y;
            Unsafe.As<Vector<float>, float>(ref PositionB.Z) = shared.PoseB.Position.Z;
            Unsafe.As<Vector<float>, float>(ref OrientationB.X) = shared.PoseB.Orientation.X;
            Unsafe.As<Vector<float>, float>(ref OrientationB.Y) = shared.PoseB.Orientation.Y;
            Unsafe.As<Vector<float>, float>(ref OrientationB.Z) = shared.PoseB.Orientation.Z;
            Unsafe.As<Vector<float>, float>(ref OrientationB.W) = shared.PoseB.Orientation.W;
        }
    }
    public struct UnflippableTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> : ITestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB, UnflippableTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB>>
        where TShapeA : struct, IShape where TShapeB : struct, IShape
        where TShapeWideA : struct, IShapeWide<TShapeA> where TShapeWideB : struct, IShapeWide<TShapeB>
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
        public void GetPoseOffset(out Vector3Wide offsetB)
        {
            Vector3Wide.Subtract(ref PositionB, ref PositionA, out offsetB);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Gather(ref TestPair<TShapeA, TShapeB> source)
        {
            A.Gather(ref source.A);
            B.Gather(ref source.B);
            ref var shared = ref source.Shared;
            Unsafe.As<Vector<float>, float>(ref PositionA.X) = shared.PoseA.Position.X;
            Unsafe.As<Vector<float>, float>(ref PositionA.Y) = shared.PoseA.Position.Y;
            Unsafe.As<Vector<float>, float>(ref PositionA.Z) = shared.PoseA.Position.Z;
            Unsafe.As<Vector<float>, float>(ref OrientationA.X) = shared.PoseA.Orientation.X;
            Unsafe.As<Vector<float>, float>(ref OrientationA.Y) = shared.PoseA.Orientation.Y;
            Unsafe.As<Vector<float>, float>(ref OrientationA.Z) = shared.PoseA.Orientation.Z;
            Unsafe.As<Vector<float>, float>(ref OrientationA.W) = shared.PoseA.Orientation.W;
            Unsafe.As<Vector<float>, float>(ref PositionB.X) = shared.PoseB.Position.X;
            Unsafe.As<Vector<float>, float>(ref PositionB.Y) = shared.PoseB.Position.Y;
            Unsafe.As<Vector<float>, float>(ref PositionB.Z) = shared.PoseB.Position.Z;
            Unsafe.As<Vector<float>, float>(ref OrientationB.X) = shared.PoseB.Orientation.X;
            Unsafe.As<Vector<float>, float>(ref OrientationB.Y) = shared.PoseB.Orientation.Y;
            Unsafe.As<Vector<float>, float>(ref OrientationB.Z) = shared.PoseB.Orientation.Z;
            Unsafe.As<Vector<float>, float>(ref OrientationB.W) = shared.PoseB.Orientation.W;
        }
    }
    public struct OneOrientationTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> : ITestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB, OneOrientationTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB>>
        where TShapeA : struct, IShape where TShapeB : struct, IShape
        where TShapeWideA : struct, IShapeWide<TShapeA> where TShapeWideB : struct, IShapeWide<TShapeB>
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
        public void GetPoseOffset(out Vector3Wide offsetB)
        {
            Vector3Wide.Subtract(ref PositionB, ref PositionA, out offsetB);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Gather(ref TestPair<TShapeA, TShapeB> source)
        {
            A.Gather(ref source.A);
            B.Gather(ref source.B);
            ref var shared = ref source.Shared;
            Unsafe.As<Vector<int>, int>(ref FlipMask) = shared.FlipMask;
            Unsafe.As<Vector<float>, float>(ref PositionA.X) = shared.PoseA.Position.X;
            Unsafe.As<Vector<float>, float>(ref PositionA.Y) = shared.PoseA.Position.Y;
            Unsafe.As<Vector<float>, float>(ref PositionA.Z) = shared.PoseA.Position.Z;
            Unsafe.As<Vector<float>, float>(ref PositionB.X) = shared.PoseB.Position.X;
            Unsafe.As<Vector<float>, float>(ref PositionB.Y) = shared.PoseB.Position.Y;
            Unsafe.As<Vector<float>, float>(ref PositionB.Z) = shared.PoseB.Position.Z;
            Unsafe.As<Vector<float>, float>(ref OrientationB.X) = shared.PoseB.Orientation.X;
            Unsafe.As<Vector<float>, float>(ref OrientationB.Y) = shared.PoseB.Orientation.Y;
            Unsafe.As<Vector<float>, float>(ref OrientationB.Z) = shared.PoseB.Orientation.Z;
            Unsafe.As<Vector<float>, float>(ref OrientationB.W) = shared.PoseB.Orientation.W;
        }
    }
    public struct NoOrientationTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> :
        ITestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB, NoOrientationTestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB>>
        where TShapeA : struct, IShape where TShapeB : struct, IShape
        where TShapeWideA : struct, IShapeWide<TShapeA> where TShapeWideB : struct, IShapeWide<TShapeB>
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
        public void GetPoseOffset(out Vector3Wide offsetB)
        {
            Vector3Wide.Subtract(ref PositionB, ref PositionA, out offsetB);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Gather(ref TestPair<TShapeA, TShapeB> source)
        {
            A.Gather(ref source.A);
            B.Gather(ref source.B);
            ref var shared = ref source.Shared;
            Unsafe.As<Vector<float>, float>(ref PositionA.X) = shared.PoseA.Position.X;
            Unsafe.As<Vector<float>, float>(ref PositionA.Y) = shared.PoseA.Position.Y;
            Unsafe.As<Vector<float>, float>(ref PositionA.Z) = shared.PoseA.Position.Z;
            Unsafe.As<Vector<float>, float>(ref PositionB.X) = shared.PoseB.Position.X;
            Unsafe.As<Vector<float>, float>(ref PositionB.Y) = shared.PoseB.Position.Y;
            Unsafe.As<Vector<float>, float>(ref PositionB.Z) = shared.PoseB.Position.Z;
        }
    }

    public interface IPairTester<TShapeWideA, TShapeWideB, TManifoldWideType>
    {
        //Note that, while the interface requires all three of these implementations, concrete implementers will only ever have one defined or called.
        //Including the other unused functions is just here to simplify its use in the batch execution loop.
        void Test(ref TShapeWideA a, ref TShapeWideB b, ref Vector3Wide offsetB, ref QuaternionWide orientationA, ref QuaternionWide orientationB, out TManifoldWideType manifold);
        void Test(ref TShapeWideA a, ref TShapeWideB b, ref Vector3Wide offsetB, ref QuaternionWide orientationB, out TManifoldWideType manifold);
        void Test(ref TShapeWideA a, ref TShapeWideB b, ref Vector3Wide offsetB, out TManifoldWideType manifold);
    }

    public interface IContactManifoldWide
    {
        void ApplyFlipMask(ref Vector3Wide offsetB, ref Vector<int> flipMask);
        void Scatter(ref Vector3Wide offsetB, ref ConvexContactManifold target);
    }

    class ConvexCollisionTaskCommon
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static unsafe void ExecuteBatch<TCallbacks, TShapeA, TShapeWideA, TShapeB, TShapeWideB, TPairWide, TManifoldWide, TPairTester>
            (ref UntypedList batch, ref CollisionBatcher<TCallbacks> batcher)
            where TShapeA : struct, IShape where TShapeB : struct, IShape
            where TShapeWideA : struct, IShapeWide<TShapeA> where TShapeWideB : struct, IShapeWide<TShapeB>
            where TPairWide : struct, ITestPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB, TPairWide>
            where TPairTester : struct, IPairTester<TShapeWideA, TShapeWideB, TManifoldWide>
            where TManifoldWide : IContactManifoldWide
            where TCallbacks : struct, ICollisionCallbacks
        {
            ref var start = ref Unsafe.As<byte, TestPair<TShapeA, TShapeB>>(ref batch.Buffer[0]);
            //With any luck, the compiler will eventually get rid of these unnecessary zero inits. 
            //Might be able to get rid of manifoldWide and defaultPairTester with some megahacks, but it comes with significant forward danger and questionable benefit.
            var pairWide = default(TPairWide);
            var manifoldWide = default(TManifoldWide);
            var defaultPairTester = default(TPairTester);
            var manifold = default(ConvexContactManifold);

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
                    target.Gather(ref Unsafe.Add(ref bundleStart, j));
                }

                pairWide.GetPoseOffset(out var offsetB);
                if (pairWide.OrientationCount == 2)
                {
                    defaultPairTester.Test(
                        ref pairWide.GetShapeA(ref pairWide),
                        ref pairWide.GetShapeB(ref pairWide),
                        ref offsetB,
                        ref pairWide.GetOrientationA(ref pairWide),
                        ref pairWide.GetOrientationB(ref pairWide),
                        out manifoldWide);
                }
                else if (pairWide.OrientationCount == 1)
                {
                    //Note that, in the event that there is only one orientation, it belongs to the second shape.
                    //The only shape that doesn't need orientation is a sphere, and it will be in slot A by convention.
                    Debug.Assert(typeof(TShapeWideA) == typeof(SphereWide));
                    defaultPairTester.Test(
                        ref pairWide.GetShapeA(ref pairWide),
                        ref pairWide.GetShapeB(ref pairWide),
                        ref offsetB,
                        ref pairWide.GetOrientationB(ref pairWide),
                        out manifoldWide);
                }
                else
                {
                    Debug.Assert(pairWide.OrientationCount == 0);
                    Debug.Assert(typeof(TShapeWideA) == typeof(SphereWide) && typeof(TShapeWideB) == typeof(SphereWide), "No orientation implies a special case involving two spheres.");
                    //Really, this could be made into a direct special case, but eh.
                    defaultPairTester.Test(
                        ref pairWide.GetShapeA(ref pairWide),
                        ref pairWide.GetShapeB(ref pairWide),
                        ref offsetB,
                        out manifoldWide);
                }

                //Flip back any contacts associated with pairs which had to be flipped for shape order.
                if (pairWide.HasFlipMask)
                {
                    manifoldWide.ApplyFlipMask(ref offsetB, ref pairWide.GetFlipMask(ref pairWide));
                }

                for (int j = 0; j < countInBundle; ++j)
                {
                    //TODO: You could just use the vector indexer here. Far more likely to work in the long run; use it if the performance is comparable.
                    ref var manifoldAsFloat = ref Unsafe.Add(ref Unsafe.As<TManifoldWide, float>(ref manifoldWide), j);
                    ref var manifoldSource = ref Unsafe.As<float, TManifoldWide>(ref manifoldAsFloat);
                    ref var offsetAsFloat = ref Unsafe.Add(ref Unsafe.As<Vector3Wide, float>(ref offsetB), j);
                    ref var offsetSource = ref Unsafe.As<float, Vector3Wide>(ref offsetAsFloat);
                    manifoldSource.Scatter(ref offsetSource, ref manifold);
                    batcher.ProcessConvexResult(&manifold, ref Unsafe.Add(ref bundleStart, j).Shared.Continuation);
                }
            }

        }
    }
}
