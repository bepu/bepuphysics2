using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using static BepuUtilities.GatherScatter;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    //We split out the pairs that the collision batcher writes because some pairs require different kinds of information. This was primarily motivated by the difference between
    //convex pairs and larger compound pairs. Something like a mesh requires calculating bounding boxes in its local space, and that requires velocities.
    //Rather than hand every single pair the worst case amount of properties, the collision batcher will check which kind of pair to write into a collision task's batch buffer.
    //Since we've already introduced a branch, we take advantage of it by also eliminating pointless data for same-type pairs and sphere pairs.

    /// <summary>
    /// Defines a type that holds scalar data for the collision batcher.
    /// </summary>
    public interface ICollisionPair<TPair> where TPair : ICollisionPair<TPair>
    {
        /// <summary>
        /// Gets the enumeration type associated with this pair type.
        /// </summary>
        static abstract CollisionTaskPairType PairType { get; }
        static abstract ref PairContinuation GetContinuation(ref TPair pair);
    }

    public unsafe struct CollisionPair : ICollisionPair<CollisionPair>
    {
        public void* A;
        public void* B;
        /// <summary>
        /// Stores whether the types involved in pair require that the resulting contact manifold be flipped to be consistent with the user-requested pair order.
        /// </summary>
        public int FlipMask;
        public Vector3 OffsetB;
        public Quaternion OrientationA;
        public Quaternion OrientationB;
        public float SpeculativeMargin;
        public PairContinuation Continuation;

        public static CollisionTaskPairType PairType => CollisionTaskPairType.StandardPair;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref PairContinuation GetContinuation(ref CollisionPair pair)
        {
            return ref pair.Continuation;
        }

    }
    public unsafe struct FliplessPair : ICollisionPair<FliplessPair>
    {
        public void* A;
        public void* B;
        public Vector3 OffsetB;
        public Quaternion OrientationA;
        public Quaternion OrientationB;
        public float SpeculativeMargin;
        public PairContinuation Continuation;

        public static CollisionTaskPairType PairType => CollisionTaskPairType.FliplessPair;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref PairContinuation GetContinuation(ref FliplessPair pair)
        {
            return ref pair.Continuation;
        }
    }
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct SpherePair : ICollisionPair<SpherePair>
    {
        public Sphere A;
        public Sphere B;
        public Vector3 OffsetB;
        public float SpeculativeMargin;
        public PairContinuation Continuation;

        public static CollisionTaskPairType PairType => CollisionTaskPairType.SpherePair;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref PairContinuation GetContinuation(ref SpherePair pair)
        {
            return ref pair.Continuation;
        }
    }
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public unsafe struct SphereIncludingPair : ICollisionPair<SphereIncludingPair>
    {
        public Sphere A;
        public void* B;
        /// <summary>
        /// Stores whether the types involved in pair require that the resulting contact manifold be flipped to be consistent with the user-requested pair order.
        /// </summary>
        public int FlipMask;
        public Vector3 OffsetB;
        public Quaternion OrientationB;
        public float SpeculativeMargin;
        public PairContinuation Continuation;

        public static CollisionTaskPairType PairType => CollisionTaskPairType.SphereIncludingPair;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref PairContinuation GetContinuation(ref SphereIncludingPair pair)
        {
            return ref pair.Continuation;
        }
    }
    /// <summary>
    /// Pair of objects awaiting collision processing that involves velocities for bounds calculation.
    /// </summary>
    public unsafe struct BoundsTestedPair : ICollisionPair<BoundsTestedPair>
    {
        public void* A;
        public void* B;
        public int FlipMask;
        public Vector3 OffsetB;
        public Quaternion OrientationB;
        public Quaternion OrientationA;
        public Vector3 RelativeLinearVelocityA;
        public Vector3 AngularVelocityA;
        public Vector3 AngularVelocityB;
        public float MaximumExpansion;
        public float SpeculativeMargin;
        public PairContinuation Continuation;

        public static CollisionTaskPairType PairType => CollisionTaskPairType.BoundsTestedPair;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref PairContinuation GetContinuation(ref BoundsTestedPair pair)
        {
            return ref pair.Continuation;
        }
    }

    public interface ICollisionPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB, TPair, TPairWide>
        where TShapeA : struct, IShape where TShapeB : struct, IShape
        where TShapeWideA : struct, IShapeWide<TShapeA> where TShapeWideB : struct, IShapeWide<TShapeB>
    {
        static abstract bool HasFlipMask { get; }
        static abstract int OrientationCount { get; }
        //Note the pair parameter. This is just to get around the fact that you cannot ref return struct fields like you can with classes, at least right now
        static abstract ref Vector<int> GetFlipMask(ref TPairWide pair);
        static abstract ref Vector<float> GetSpeculativeMargin(ref TPairWide pair);
        static abstract ref TShapeWideA GetShapeA(ref TPairWide pair);
        static abstract ref TShapeWideB GetShapeB(ref TPairWide pair);
        static abstract ref QuaternionWide GetOrientationA(ref TPairWide pair);
        static abstract ref QuaternionWide GetOrientationB(ref TPairWide pair);
        static abstract ref Vector3Wide GetOffsetB(ref TPairWide pair);
        void WriteSlot(int index, in TPair source);

    }


    public struct ConvexPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> :
        ICollisionPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB, CollisionPair, ConvexPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB>>
        where TShapeA : struct, IConvexShape where TShapeB : struct, IConvexShape
        where TShapeWideA : struct, IShapeWide<TShapeA> where TShapeWideB : struct, IShapeWide<TShapeB>
    {
        public TShapeWideA A;
        public TShapeWideB B;
        public Vector<int> FlipMask;
        public Vector3Wide OffsetB;
        public QuaternionWide OrientationA;
        public QuaternionWide OrientationB;
        public Vector<float> SpeculativeMargin;

        public static bool HasFlipMask
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return true; }
        }

        public static int OrientationCount
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return 2; }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref Vector<int> GetFlipMask(ref ConvexPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair)
        {
            return ref pair.FlipMask;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref Vector<float> GetSpeculativeMargin(ref ConvexPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair)
        {
            return ref pair.SpeculativeMargin;
        }
        //Little unfortunate that we can't return ref of struct instances.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref TShapeWideA GetShapeA(ref ConvexPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair)
        {
            return ref pair.A;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref TShapeWideB GetShapeB(ref ConvexPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair)
        {
            return ref pair.B;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref Vector3Wide GetOffsetB(ref ConvexPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair)
        {
            return ref pair.OffsetB;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref QuaternionWide GetOrientationA(ref ConvexPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair)
        {
            return ref pair.OrientationA;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref QuaternionWide GetOrientationB(ref ConvexPairWide<TShapeA, TShapeWideA, TShapeB, TShapeWideB> pair)
        {
            return ref pair.OrientationB;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void WriteSlot(int index, in CollisionPair source)
        {
            ref var offset = ref GetOffsetInstance(ref this, index);
            if (A.AllowOffsetMemoryAccess)
                offset.A.WriteFirst(Unsafe.AsRef<TShapeA>(source.A));
            else
                A.WriteSlot(index, Unsafe.AsRef<TShapeA>(source.A));
            if (B.AllowOffsetMemoryAccess)
                offset.B.WriteFirst(Unsafe.AsRef<TShapeB>(source.B));
            else
                B.WriteSlot(index, Unsafe.AsRef<TShapeB>(source.B));
            GetFirst(ref offset.FlipMask) = source.FlipMask;
            Vector3Wide.WriteFirst(source.OffsetB, ref offset.OffsetB);
            QuaternionWide.WriteFirst(source.OrientationA, ref offset.OrientationA);
            QuaternionWide.WriteFirst(source.OrientationB, ref offset.OrientationB);
            GetFirst(ref offset.SpeculativeMargin) = source.SpeculativeMargin;
        }
    }
    public struct FliplessPairWide<TShape, TShapeWide> : ICollisionPairWide<TShape, TShapeWide, TShape, TShapeWide, FliplessPair, FliplessPairWide<TShape, TShapeWide>>
        where TShape : struct, IConvexShape where TShapeWide : struct, IShapeWide<TShape>
    {
        public TShapeWide A;
        public TShapeWide B;
        public Vector3Wide OffsetB;
        public QuaternionWide OrientationA;
        public QuaternionWide OrientationB;
        public Vector<float> SpeculativeMargin;

        public static bool HasFlipMask
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return false; }
        }

        public static int OrientationCount
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return 2; }
        }

        public static ref Vector<int> GetFlipMask(ref FliplessPairWide<TShape, TShapeWide> pair)
        {
            throw new NotImplementedException();
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref Vector<float> GetSpeculativeMargin(ref FliplessPairWide<TShape, TShapeWide> pair)
        {
            return ref pair.SpeculativeMargin;
        }
        //Little unfortunate that we can't return ref of struct instances.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref TShapeWide GetShapeA(ref FliplessPairWide<TShape, TShapeWide> pair)
        {
            return ref pair.A;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref TShapeWide GetShapeB(ref FliplessPairWide<TShape, TShapeWide> pair)
        {
            return ref pair.B;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref Vector3Wide GetOffsetB(ref FliplessPairWide<TShape, TShapeWide> pair)
        {
            return ref pair.OffsetB;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref QuaternionWide GetOrientationA(ref FliplessPairWide<TShape, TShapeWide> pair)
        {
            return ref pair.OrientationA;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref QuaternionWide GetOrientationB(ref FliplessPairWide<TShape, TShapeWide> pair)
        {
            return ref pair.OrientationB;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void WriteSlot(int index, in FliplessPair source)
        {
            ref var offset = ref GetOffsetInstance(ref this, index);
            if (A.AllowOffsetMemoryAccess)
            {
                offset.A.WriteFirst(Unsafe.AsRef<TShape>(source.A));
                offset.B.WriteFirst(Unsafe.AsRef<TShape>(source.B));
            }
            else
            {
                A.WriteSlot(index, Unsafe.AsRef<TShape>(source.A));
                B.WriteSlot(index, Unsafe.AsRef<TShape>(source.B));
            }
            Vector3Wide.WriteFirst(source.OffsetB, ref offset.OffsetB);
            QuaternionWide.WriteFirst(source.OrientationA, ref offset.OrientationA);
            QuaternionWide.WriteFirst(source.OrientationB, ref offset.OrientationB);
            GetFirst(ref offset.SpeculativeMargin) = source.SpeculativeMargin;
        }
    }
    public struct SphereIncludingPairWide<TShape, TShapeWide> : ICollisionPairWide<Sphere, SphereWide, TShape, TShapeWide, SphereIncludingPair, SphereIncludingPairWide<TShape, TShapeWide>>
        where TShape : struct, IConvexShape where TShapeWide : struct, IShapeWide<TShape>
    {
        public SphereWide A;
        public TShapeWide B;
        public Vector<int> FlipMask;
        public Vector3Wide OffsetB;
        public QuaternionWide OrientationB;
        public Vector<float> SpeculativeMargin;

        public static bool HasFlipMask
        {
            //Because the shapes are guaranteed to be distinct (one is apparently a sphere and the other isn't), there will always be a flip mask.
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return true; }
        }

        public static int OrientationCount
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return 1; }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref Vector<int> GetFlipMask(ref SphereIncludingPairWide<TShape, TShapeWide> pair)
        {
            return ref pair.FlipMask;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref Vector<float> GetSpeculativeMargin(ref SphereIncludingPairWide<TShape, TShapeWide> pair)
        {
            return ref pair.SpeculativeMargin;
        }
        //Little unfortunate that we can't return ref of struct instances.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref SphereWide GetShapeA(ref SphereIncludingPairWide<TShape, TShapeWide> pair)
        {
            return ref pair.A;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref TShapeWide GetShapeB(ref SphereIncludingPairWide<TShape, TShapeWide> pair)
        {
            return ref pair.B;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref Vector3Wide GetOffsetB(ref SphereIncludingPairWide<TShape, TShapeWide> pair)
        {
            return ref pair.OffsetB;
        }
        public static ref QuaternionWide GetOrientationA(ref SphereIncludingPairWide<TShape, TShapeWide> pair)
        {
            throw new NotImplementedException();
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref QuaternionWide GetOrientationB(ref SphereIncludingPairWide<TShape, TShapeWide> pair)
        {
            return ref pair.OrientationB;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void WriteSlot(int index, in SphereIncludingPair source)
        {
            ref var offset = ref GetOffsetInstance(ref this, index);
            offset.A.WriteFirst(source.A);
            if (B.AllowOffsetMemoryAccess)
                offset.B.WriteFirst(Unsafe.AsRef<TShape>(source.B));
            else
                B.WriteSlot(index, Unsafe.AsRef<TShape>(source.B));
            GetFirst(ref offset.FlipMask) = source.FlipMask;
            Vector3Wide.WriteFirst(source.OffsetB, ref offset.OffsetB);
            QuaternionWide.WriteFirst(source.OrientationB, ref offset.OrientationB);
            GetFirst(ref offset.SpeculativeMargin) = source.SpeculativeMargin;
        }
    }
    public struct SpherePairWide : ICollisionPairWide<Sphere, SphereWide, Sphere, SphereWide, SpherePair, SpherePairWide>
    {
        public SphereWide A;
        public SphereWide B;
        public Vector3Wide OffsetB;
        public Vector<float> SpeculativeMargin;

        public static bool HasFlipMask
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return false; }
        }
        public static int OrientationCount
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return 0; }
        }

        public static ref Vector<int> GetFlipMask(ref SpherePairWide pair)
        {
            throw new NotImplementedException();
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref Vector<float> GetSpeculativeMargin(ref SpherePairWide pair)
        {
            return ref pair.SpeculativeMargin;
        }
        //Little unfortunate that we can't return ref of struct instances.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref SphereWide GetShapeA(ref SpherePairWide pair)
        {
            return ref pair.A;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref SphereWide GetShapeB(ref SpherePairWide pair)
        {
            return ref pair.B;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ref Vector3Wide GetOffsetB(ref SpherePairWide pair)
        {
            return ref pair.OffsetB;
        }
        public static ref QuaternionWide GetOrientationA(ref SpherePairWide pair)
        {
            throw new NotImplementedException();
        }
        public static ref QuaternionWide GetOrientationB(ref SpherePairWide pair)
        {
            throw new NotImplementedException();
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WriteSlot(int index, in SpherePair source)
        {
            ref var offset = ref GetOffsetInstance(ref this, index);
            offset.A.WriteFirst(source.A);
            offset.B.WriteFirst(source.B);
            Vector3Wide.WriteFirst(source.OffsetB, ref offset.OffsetB);
            GetFirst(ref offset.SpeculativeMargin) = source.SpeculativeMargin;
        }
    }
}
