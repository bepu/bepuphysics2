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
    public struct CompoundChild
    {
        public TypedIndex ShapeIndex;
        public RigidPose LocalPose;
    }

    /// <summary>
    /// Minimalist compound shape containing a list of child shapes. Does not make use of any internal acceleration structure; should be used only with small groups of shapes.
    /// </summary>
    public struct ListCompound : ICompoundShape
    {
        /// <summary>
        /// Buffer of children within this compound.
        /// </summary>
        public Buffer<CompoundChild> Children;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ListCompound(Buffer<CompoundChild> children)
        {
            Debug.Assert(children.Length > 0, "Compounds must have a nonzero number of children.");
            Children = children;
        }

        public void ComputeBounds(in Quaternion orientation, Shapes shapeBatches, out Vector3 min, out Vector3 max)
        {
            Compound.ComputeChildBounds(Children[0], orientation, shapeBatches, out min, out max);
            for (int i = 1; i < Children.Length; ++i)
            {
                ref var child = ref Children[i];
                Compound.ComputeChildBounds(Children[i], orientation, shapeBatches, out var childMin, out var childMax);
                BoundingBox.CreateMerged(ref min, ref max, ref childMin, ref childMax, out min, out max);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddChildBoundsToBatcher(ref BoundingBoxBatcher batcher, ref RigidPose pose, ref BodyVelocity velocity, int bodyIndex)
        {
            for (int i = 0; i < Children.Length; ++i)
            {
                ref var child = ref Children[i];
                Compound.GetWorldPose(child.LocalPose, pose, out var childPose);
                batcher.AddCompoundChild(bodyIndex, Children[i].ShapeIndex, ref childPose, ref velocity);
            }
        }

        public bool RayTest(in RigidPose pose, in Vector3 origin, in Vector3 direction, float maximumT, Shapes shapeBatches, out float t, out Vector3 normal)
        {
            t = float.MaxValue;
            normal = new Vector3();
            for (int i = 0; i < Children.Length; ++i)
            {
                ref var child = ref Children[i];
                Compound.GetRotatedChildPose(child.LocalPose, pose.Orientation, out var childPose);
                //TODO: This is an area that has to be updated for high precision poses.
                childPose.Position += pose.Position;
                if (shapeBatches[child.ShapeIndex.Type].RayTest(child.ShapeIndex.Index, childPose, origin, direction, maximumT, out var childT, out var childNormal) && childT < t)
                {
                    t = childT;
                    normal = childNormal;
                }
            }
            return t < float.MaxValue;
        }

        public void RayTest<TRayHitHandler>(in RigidPose pose, Shapes shapeBatches, ref RaySource rays, ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayHitHandler
        {
            for (int i = 0; i < Children.Length; ++i)
            {
                ref var child = ref Children[i];
                Compound.GetRotatedChildPose(child.LocalPose, pose.Orientation, out var childPose);
                //TODO: This is an area that has to be updated for high precision poses.
                childPose.Position += pose.Position;
                //Note that this will report an impact for every child, even if it's not the first impact.
                shapeBatches[child.ShapeIndex.Type].RayTest(child.ShapeIndex.Index, childPose, ref rays, ref hitHandler);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ShapeBatch CreateShapeBatch(BufferPool pool, int initialCapacity, Shapes shapes)
        {
            return new CompoundShapeBatch<ListCompound>(pool, initialCapacity, shapes);
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

        public unsafe void FindLocalOverlaps<TOverlaps, TSubpairOverlaps>(PairsToTestForOverlap* pairs, int count, BufferPool pool, Shapes shapes, ref TOverlaps overlaps)
            where TOverlaps : struct, ICollisionTaskOverlaps<TSubpairOverlaps>
            where TSubpairOverlaps : struct, ICollisionTaskSubpairOverlaps
        {
            for (int pairIndex = 0; pairIndex < count; ++pairIndex)
            {
                ref var pair = ref pairs[pairIndex];
                ref var compound = ref Unsafe.AsRef<ListCompound>(pair.Container);
                ref var overlapsForPair = ref overlaps.GetOverlapsForPair(pairIndex);
                for (int i = 0; i < compound.Children.Length; ++i)
                {
                    ref var child = ref compound.Children[i];
                    //TODO: This does quite a bit of work. May want to try a simple bounding sphere instead (based on a dedicated maximum radius request).
                    shapes[child.ShapeIndex.Type].ComputeBounds(child.ShapeIndex.Index, child.LocalPose.Orientation, out _, out _, out var min, out var max);
                    min += child.LocalPose.Position;
                    max += child.LocalPose.Position;
                    if (BoundingBox.Intersects(min, max, pair.Min, pair.Max))
                    {
                        overlapsForPair.Allocate(pool) = i;
                    }
                }
            }
        }

        public unsafe void FindLocalOverlaps<TOverlaps>(in Vector3 min, in Vector3 max, in Vector3 sweep, float maximumT, BufferPool pool, Shapes shapes, void* overlapsPointer)
            where TOverlaps : ICollisionTaskSubpairOverlaps
        {
            Tree.ConvertBoxToCentroidWithExtent(min, max, out var sweepOrigin, out var expansion);
            TreeRay.CreateFrom(sweepOrigin, sweep, maximumT, out var ray);
            ref var overlaps = ref Unsafe.AsRef<TOverlaps>(overlapsPointer);
            for (int i = 0; i < Children.Length; ++i)
            {
                ref var child = ref Children[i];
                shapes[child.ShapeIndex.Type].ComputeBounds(child.ShapeIndex.Index, child.LocalPose.Orientation, out _, out _, out var childMin, out var childMax);
                childMin = childMin + child.LocalPose.Position - expansion;
                childMax = childMax + child.LocalPose.Position + expansion;
                if (Tree.Intersects(childMin, childMax, &ray, out _))
                {
                    overlaps.Allocate(pool) = i;
                }
            }

        }
        
        public void Dispose(BufferPool bufferPool)
        {
            bufferPool.Return(ref Children);
        }

        /// <summary>
        /// Type id of list based compound shapes.
        /// </summary>
        public const int Id = 6;
        public int TypeId { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return Id; } }
    }


}
