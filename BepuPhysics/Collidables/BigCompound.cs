using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuUtilities.Memory;
using System.Diagnostics;
using Quaternion = BepuUtilities.Quaternion;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuPhysics.Trees;
using BepuPhysics.CollisionDetection.CollisionTasks;

namespace BepuPhysics.Collidables
{
    /// <summary>
    /// Compound shape containing a bunch of shapes accessible through a tree acceleration structure. Useful for compounds with lots of children.
    /// </summary>
    public struct BigCompound : ICompoundShape
    {
        public Tree Tree;
        /// <summary>
        /// Buffer of children within this compound.
        /// </summary>
        public Buffer<CompoundChild> Children;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public BigCompound(Buffer<CompoundChild> children, Shapes shapes, BufferPool pool)
        {
            Debug.Assert(children.Length > 0, "Compounds must have a nonzero number of children.");
            Children = children;
            Tree = new Tree(pool, children.Length);
            pool.Take(children.Length, out Buffer<BoundingBox> leafBounds);
            Compound.ComputeChildBounds(Children[0], Quaternion.Identity, shapes, out leafBounds[0].Min, out leafBounds[0].Max);
            for (int i = 1; i < Children.Length; ++i)
            {
                ref var bounds = ref leafBounds[i];
                Compound.ComputeChildBounds(Children[i], Quaternion.Identity, shapes, out bounds.Min, out bounds.Max);
            }
            Tree.SweepBuild(pool, leafBounds.Slice(0, children.Length));
            pool.Return(ref leafBounds);
        }

        public void ComputeBounds(in Quaternion orientation, Shapes shapeBatches, out Vector3 min, out Vector3 max)
        {
            Compound.ComputeChildBounds(Children[0], orientation, shapeBatches, out min, out max);
            for (int i = 1; i < Children.Length; ++i)
            {
                ref var child = ref Children[i];
                Compound.ComputeChildBounds(Children[i], orientation, shapeBatches, out var childMin, out var childMax);
                BoundingBox.CreateMerged(min, max, childMin, childMax, out min, out max);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddChildBoundsToBatcher(ref BoundingBoxBatcher batcher, in RigidPose pose, in BodyVelocity velocity, int bodyIndex)
        {
            Compound.AddChildBoundsToBatcher(ref Children, ref batcher, pose, velocity, bodyIndex);
        }

        unsafe struct LeafTester : IRayLeafTester
        {
            public CompoundChild* Children;
            public Shapes Shapes;
            public float MinimumT;
            public Vector3 MinimumNormal;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public LeafTester(in Buffer<CompoundChild> children, Shapes shapes)
            {
                Children = (CompoundChild*)children.Memory;
                Shapes = shapes;
                MinimumT = float.MaxValue;
                MinimumNormal = default;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe void TestLeaf(int leafIndex, RayData* rayData, float* maximumT)
            {
                ref var child = ref Children[leafIndex];
                if (Shapes[child.ShapeIndex.Type].RayTest(child.ShapeIndex.Index, child.LocalPose, rayData->Origin, rayData->Direction, *maximumT, out var t, out var normal) &&
                    t < MinimumT && t <= *maximumT)
                {
                    MinimumT = t;
                    MinimumNormal = normal;
                }
            }
        }

        public bool RayTest(in RigidPose pose, in Vector3 origin, in Vector3 direction, float maximumT, Shapes shapes, out float t, out Vector3 normal)
        {
            Matrix3x3.CreateFromQuaternion(pose.Orientation, out var orientation);
            Matrix3x3.TransformTranspose(origin - pose.Position, orientation, out var localOrigin);
            Matrix3x3.TransformTranspose(direction, orientation, out var localDirection);
            var leafTester = new LeafTester(Children, shapes);
            Tree.RayCast(localOrigin, localDirection, maximumT, ref leafTester);
            if (leafTester.MinimumT < float.MaxValue)
            {
                t = leafTester.MinimumT;
                Matrix3x3.Transform(leafTester.MinimumNormal, orientation, out normal);
                return true;
            }
            t = default;
            normal = default;
            return false;
        }

        public unsafe void RayTest<TRayHitHandler>(in RigidPose pose, Shapes shapes, ref RaySource rays, ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayBatchHitHandler
        {
            //TODO: Note that we dispatch a bunch of scalar tests here. You could be more clever than this- batched tests are possible. 
            //May be worth creating a different traversal designed for low ray counts- might be able to get some benefit out of a semidynamic packet or something.
            Matrix3x3.CreateFromQuaternion(pose.Orientation, out var orientation);
            Matrix3x3.Transpose(orientation, out var inverseOrientation);
            var leafTester = new LeafTester(Children, shapes);
            for (int i = 0; i < rays.RayCount; ++i)
            {
                rays.GetRay(i, out var ray, out var maximumT);
                Matrix3x3.Transform(ray->Origin - pose.Position, inverseOrientation, out var localOrigin);
                Matrix3x3.Transform(ray->Direction, inverseOrientation, out var localDirection);
                leafTester.MinimumT = float.MaxValue;
                Tree.RayCast(localOrigin, localDirection, *maximumT, ref leafTester);
                if (leafTester.MinimumT < float.MaxValue)
                {
                    Matrix3x3.Transform(leafTester.MinimumNormal, orientation, out var normal);
                    hitHandler.OnRayHit(i, leafTester.MinimumT, normal);
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ShapeBatch CreateShapeBatch(BufferPool pool, int initialCapacity, Shapes shapes)
        {
            return new CompoundShapeBatch<BigCompound>(pool, initialCapacity, shapes);
        }

        public int ChildCount
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return Children.Length; }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref CompoundChild GetChild(int compoundChildIndex)
        {
            return ref Children[compoundChildIndex];
        }

        unsafe struct Enumerator<TSubpairOverlaps> : IBreakableForEach<int> where TSubpairOverlaps : ICollisionTaskSubpairOverlaps
        {
            public BufferPool Pool;
            public void* Overlaps;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool LoopBody(int i)
            {
                Unsafe.AsRef<TSubpairOverlaps>(Overlaps).Allocate(Pool) = i;
                return true;
            }
        }

        public unsafe void FindLocalOverlaps<TOverlaps, TSubpairOverlaps>(ref Buffer<OverlapQueryForPair> pairs, BufferPool pool, Shapes shapes, ref TOverlaps overlaps)
            where TOverlaps : struct, ICollisionTaskOverlaps<TSubpairOverlaps>
            where TSubpairOverlaps : struct, ICollisionTaskSubpairOverlaps
        {
            //For now, we don't use anything tricky. Just traverse every child against the tree sequentially.
            //TODO: This sequentializes a whole lot of cache misses. You could probably get some benefit out of traversing all pairs 'simultaneously'- that is, 
            //using the fact that we have lots of independent queries to ensure the CPU always has something to do.
            Enumerator<TSubpairOverlaps> enumerator;
            enumerator.Pool = pool;
            for (int i = 0; i < pairs.Length; ++i)
            {
                ref var pair = ref pairs[i];
                enumerator.Overlaps = Unsafe.AsPointer(ref overlaps.GetOverlapsForPair(i));
                Unsafe.AsRef<BigCompound>(pair.Container).Tree.GetOverlaps(pair.Min, pair.Max, ref enumerator);
            }
        }

        unsafe struct SweepLeafTester<TOverlaps> : ISweepLeafTester where TOverlaps : ICollisionTaskSubpairOverlaps
        {
            public BufferPool Pool;
            public void* Overlaps;
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void TestLeaf(int leafIndex, ref float maximumT)
            {
                Unsafe.AsRef<TOverlaps>(Overlaps).Allocate(Pool) = leafIndex;
            }
        }
        public unsafe void FindLocalOverlaps<TOverlaps>(in Vector3 min, in Vector3 max, in Vector3 sweep, float maximumT, BufferPool pool, Shapes shapes, void* overlaps)
            where TOverlaps : ICollisionTaskSubpairOverlaps
        {
            SweepLeafTester<TOverlaps> enumerator;
            enumerator.Pool = pool;
            enumerator.Overlaps = overlaps;
            Tree.Sweep(min, max, sweep, maximumT, ref enumerator);
        }

        public void Dispose(BufferPool bufferPool)
        {
            bufferPool.Return(ref Children);
            Tree.Dispose(bufferPool);
        }

        /// <summary>
        /// Type id of compound shapes.
        /// </summary>
        public const int Id = 7;
        public int TypeId { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return Id; } }
    }


}
