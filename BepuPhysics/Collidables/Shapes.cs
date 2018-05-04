using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System.Numerics;
using System.Runtime.CompilerServices;
using System;
using System.Diagnostics;
using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuPhysics.Trees;

namespace BepuPhysics.Collidables
{
    public abstract class ShapeBatch
    {
        protected RawBuffer shapesData;
        protected int shapeDataSize;
        /// <summary>
        /// Gets the number of shapes that the batch can currently hold without resizing.
        /// </summary>
        public int Capacity { get { return shapesData.Length / shapeDataSize; } }
        protected BufferPool pool;
        protected IdPool<Buffer<int>> idPool;
        /// <summary>
        /// Gets the type id of the shape type in this batch.
        /// </summary>
        public int TypeId { get; protected set; }
        /// <summary>
        /// Gets whether this shape batch's contained type potentially contains children of different types.
        /// </summary>
        public bool Compound { get; protected set; }

        [Conditional("DEBUG")]
        protected abstract void ValidateRemoval(int index);

        public void RemoveAt(int index)
        {
            ValidateRemoval(index);
            idPool.Return(index, pool.SpecializeFor<int>());
        }

        public abstract void ComputeBounds(ref BoundingBoxBatcher batcher);
        public abstract void ComputeBounds(int shapeIndex, ref RigidPose pose, out Vector3 min, out Vector3 max);
        public abstract bool RayTest(int shapeIndex, in RigidPose pose, in Vector3 origin, in Vector3 direction, out float t, out Vector3 normal);
        public abstract void RayTest<TRayHitHandler>(int shapeIndex, in RigidPose rigidPose, ref RaySource rays, ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayHitHandler;

        /// <summary>
        /// Gets a raw untyped pointer to a shape's data.
        /// </summary>
        /// <param name="shapeIndex">Index of the shape to look up.</param>
        /// <param name="shapePointer">Pointer to the indexed shape data.</param>
        /// <param name="shapeSize">Size of the shape data in bytes.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void GetShapeData(int shapeIndex, out void* shapePointer, out int shapeSize)
        {
            shapePointer = shapesData.Memory + shapeDataSize * shapeIndex;
            shapeSize = shapeDataSize;
        }


        /// <summary>
        /// Frees all shape slots without returning any resources to the pool.
        /// </summary>
        public abstract void Clear();
        /// <summary>
        /// Increases the size of the type batch if necessary to hold the target capacity.
        /// </summary>
        /// <param name="shapeCapacity">Target capacity.</param>
        public abstract void EnsureCapacity(int shapeCapacity);
        /// <summary>
        /// Changes the size of the type batch if the target capacity is different than the current capacity. Note that shrinking allocations is conservative; resizing will
        /// never allow an existing shape to point to unallocated memory.
        /// </summary>
        /// <param name="shapeCapacity">Target capacity.</param>
        public abstract void Resize(int shapeCapacity);
        /// <summary>
        /// Returns all backing resources to the pool, leaving the batch in an unusable state.
        /// </summary>
        public abstract void Dispose();

        /// <summary>
        /// Shrinks or expands the allocation of the batch's id pool. Note that shrinking allocations is conservative; resizing will never allow any pending ids to be lost.
        /// </summary>
        /// <param name="targetIdCapacity">Number of slots to allocate space for in the id pool.</param>
        public void ResizeIdPool(int targetIdCapacity)
        {
            idPool.Resize(targetIdCapacity, pool.SpecializeFor<int>());
        }

    }

    public abstract class ShapeBatch<TShape> : ShapeBatch where TShape : struct, IShape//TODO: When blittable is supported, shapes should be made blittable. We store them in buffers.
    {
        internal Buffer<TShape> shapes;

        /// <summary>
        /// Gets a reference to the shape associated with an index.
        /// </summary>
        /// <param name="shapeIndex">Index of the shape reference to retrieve.</param>
        /// <returns>Reference to the shape at the given index.</returns>
        public ref TShape this[int shapeIndex] { get { return ref shapes[shapeIndex]; } }

        protected ShapeBatch(BufferPool pool, int initialShapeCount)
        {
            this.pool = pool;
            TypeId = default(TShape).TypeId;
            InternalResize(initialShapeCount, 0);
            IdPool<Buffer<int>>.Create(pool.SpecializeFor<int>(), initialShapeCount, out idPool);
        }

        protected override void ValidateRemoval(int index)
        {
            Debug.Assert(!SpanHelper.IsZeroed(ref shapes[index]),
                "Either a shape was default constructed (which is almost certainly invalid), or this is attempting to remove a shape that was already removed.");
            //Don't have to actually clear out the shape set since everything is blittable. For debug purposes, we do, just to catch invalid usages.
            shapes[index] = default(TShape);
        }


        //Note that shapes cannot be moved; there is no reference to the collidables using them, so we can't correct their indices.
        //But that's fine- we never directly iterate over the shapes set anyway.
        //(This doesn't mean that it's impossible to compact the shape set- it just requires doing so by iterating over collidables.)
        public int Add(ref TShape shape)
        {
            var shapeIndex = idPool.Take();
            if (shapes.Length <= shapeIndex)
            {
                InternalResize(shapeIndex + 1, shapes.Length);
            }
            Debug.Assert(SpanHelper.IsZeroed(ref shapes[shapeIndex]), "In debug mode, the slot a shape is stuck into should be cleared. If it's not, it is already in use.");
            shapes[shapeIndex] = shape;
            return shapeIndex;
        }


        void InternalResize(int shapeCount, int oldCopyLength)
        {
            shapeDataSize = Unsafe.SizeOf<TShape>();
            var requiredSizeInBytes = shapeCount * Unsafe.SizeOf<TShape>();
            pool.Take(requiredSizeInBytes, out var newShapesData);
            var newShapes = newShapesData.As<TShape>();
#if DEBUG
            //In debug mode, unused slots are kept at the default value. This helps catch misuse.
            if (newShapes.Length > shapes.Length)
                newShapes.Clear(shapes.Length, newShapes.Length - shapes.Length);
#endif
            if (shapesData.Allocated)
            {
                shapes.CopyTo(0, ref newShapes, 0, oldCopyLength);
                pool.Return(ref shapesData);
            }
            else
            {
                Debug.Assert(oldCopyLength == 0);
            }
            shapes = newShapes;
            shapesData = newShapesData;
        }

        public override void Clear()
        {
#if DEBUG
            shapes.Clear(0, idPool.HighestPossiblyClaimedId + 1);
#endif
            idPool.Clear();
        }
        public override void EnsureCapacity(int shapeCapacity)
        {
            if (shapes.Length < shapeCapacity)
            {
                InternalResize(shapeCapacity, idPool.HighestPossiblyClaimedId + 1);
            }
        }

        public override void Resize(int shapeCapacity)
        {
            shapeCapacity = BufferPool<TShape>.GetLowestContainingElementCount(Math.Max(idPool.HighestPossiblyClaimedId + 1, shapeCapacity));
            if (shapeCapacity != shapes.Length)
            {
                InternalResize(shapeCapacity, idPool.HighestPossiblyClaimedId + 1);
            }
        }
        public override void Dispose()
        {
            Debug.Assert(shapesData.Id == shapes.Id, "If the buffer ids don't match, there was some form of failed resize.");
            pool.Return(ref shapesData);
            idPool.Dispose(pool.SpecializeFor<int>());
        }
    }


    public class ConvexShapeBatch<TShape, TShapeWide> : ShapeBatch<TShape>
        where TShape : struct, IConvexShape
        where TShapeWide : struct, IShapeWide<TShape>
    {
        public ConvexShapeBatch(BufferPool pool, int initialShapeCount) : base(pool, initialShapeCount)
        {
        }

        public override void ComputeBounds(ref BoundingBoxBatcher batcher)
        {
            batcher.ExecuteConvexBatch(this);
        }

        public override void ComputeBounds(int shapeIndex, ref RigidPose pose, out Vector3 min, out Vector3 max)
        {
            shapes[shapeIndex].GetBounds(pose.Orientation, out min, out max);
            min += pose.Position;
            max += pose.Position;
        }

        public override bool RayTest(int shapeIndex, in RigidPose pose, in Vector3 origin, in Vector3 direction, out float t, out Vector3 normal)
        {
            return shapes[shapeIndex].RayTest(pose, origin, direction, out t, out normal);
        }        

        public override void RayTest<TRayHitHandler>(int index, in RigidPose pose, ref RaySource rays, ref TRayHitHandler hitHandler)
        {
            ref var shape = ref shapes[index];
            for (int i = 0; i < rays.RayCount; ++i)
            {
                ref readonly var ray = ref rays.GetRay(i);
                if (shape.RayTest(pose, ray.Origin, ray.Direction, out var t, out var normal))
                {
                    hitHandler.OnRayHit(i, t, normal);
                }
            }
        }
    }

    public class BroadcastableShapeBatch<TShape, TShapeWide> : ConvexShapeBatch<TShape, TShapeWide>
        where TShape : struct, IBroadcastableShape<TShape, TShapeWide>
        where TShapeWide : struct, IShapeWide<TShape>
    {
        public BroadcastableShapeBatch(BufferPool pool, int initialShapeCount) : base(pool, initialShapeCount)
        {
        }

        public override void RayTest<TRayHitHandler>(int index, in RigidPose pose, ref RaySource rays, ref TRayHitHandler hitHandler)
        {
            WideRayTester.Test<RaySource, TShape, TShapeWide, TRayHitHandler>(ref shapes[index], pose, ref rays, ref hitHandler);
        }
    }

    public class CompoundShapeBatch<TShape> : ShapeBatch<TShape> where TShape : struct, ICompoundShape
    {
        Shapes shapeBatches;

        public CompoundShapeBatch(BufferPool pool, int initialShapeCount, Shapes shapeBatches) : base(pool, initialShapeCount)
        {
            this.shapeBatches = shapeBatches;
            Compound = true;
        }

        public override void ComputeBounds(ref BoundingBoxBatcher batcher)
        {
            batcher.ExecuteCompoundBatch(this);
        }

        public override void ComputeBounds(int shapeIndex, ref RigidPose pose, out Vector3 min, out Vector3 max)
        {
            shapes[shapeIndex].GetBounds(pose.Orientation, shapeBatches, out min, out max);
            min += pose.Position;
            max += pose.Position;
        }
        public override bool RayTest(int shapeIndex, in RigidPose pose, in Vector3 origin, in Vector3 direction, out float t, out Vector3 normal)
        {
            return shapes[shapeIndex].RayTest(pose, origin, direction, shapeBatches, out t, out normal);
        }

        public override void RayTest<TRayHitHandler>(int shapeIndex, in RigidPose pose, ref RaySource rays, ref TRayHitHandler hitHandler)
        {
            shapes[shapeIndex].RayTest(pose, shapeBatches, ref rays, ref hitHandler);
        }
    }


    public class Shapes
    {
        QuickList<ShapeBatch, Array<ShapeBatch>> batches;

        //Note that not every index within the batches list is guaranteed to be filled. For example, if only a cylinder has been added, and a cylinder's type id is 7,
        //then the batches.Count and RegisteredTypeSpan will be 8- but indices 0 through 6 will be null.
        //We don't tend to do any performance sensitive iteration over shape type batches, so this lack of contiguity is fine.
        public int RegisteredTypeSpan => batches.Count;

        public int InitialCapacityPerTypeBatch { get; set; }
        public ShapeBatch this[int typeIndex] => batches[typeIndex];
        BufferPool pool;


        public Shapes(BufferPool pool, int initialCapacityPerTypeBatch)
        {
            InitialCapacityPerTypeBatch = initialCapacityPerTypeBatch;
            //This list pretty much will never resize unless something really strange happens, and since batches use virtual calls, we have to allow storage of reference types.
            QuickList<ShapeBatch, Array<ShapeBatch>>.Create(new PassthroughArrayPool<ShapeBatch>(), 16, out batches);
            this.pool = pool;
        }

        /// <summary>
        /// Computes a bounding box for a single shape.
        /// </summary>
        /// <param name="pose">Pose to calculate the bounding box of.</param>
        /// <param name="shapeIndex">Index of the shape.</param>
        /// <param name="bounds">Bounding box of the specified shape with the specified pose.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void UpdateBounds(ref RigidPose pose, ref TypedIndex shapeIndex, out BoundingBox bounds)
        {
            //Note: the min and max here are in absolute coordinates, which means this is a spot that has to be updated in the event that positions use a higher precision representation.
            batches[shapeIndex.Type].ComputeBounds(shapeIndex.Index, ref pose, out bounds.Min, out bounds.Max);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref TShape GetShape<TShape>(int shapeIndex) where TShape : struct, IShape
        {
            var typeId = default(TShape).TypeId;
            return ref Unsafe.As<ShapeBatch, ShapeBatch<TShape>>(ref batches[typeId])[shapeIndex];
        }


        public TypedIndex Add<TShape>(ref TShape shape) where TShape : struct, IShape
        {
            var typeId = default(TShape).TypeId;
            if (RegisteredTypeSpan <= typeId)
            {
                if (batches.Span.Length <= typeId)
                {
                    batches.Resize(typeId, new PassthroughArrayPool<ShapeBatch>());
                }
                batches.Count = typeId + 1;
            }
            if (batches[typeId] == null)
            {
                batches[typeId] = default(TShape).CreateShapeBatch(pool, InitialCapacityPerTypeBatch, this);
            }

            Debug.Assert(batches[typeId] is ShapeBatch<TShape>);
            var batch = Unsafe.As<ShapeBatch, ShapeBatch<TShape>>(ref batches[typeId]);
            var index = batch.Add(ref shape);
            return new TypedIndex(typeId, index);
        }



        public void Remove(TypedIndex shapeIndex)
        {
            Debug.Assert(RegisteredTypeSpan > shapeIndex.Type && batches[shapeIndex.Type] != null);
            batches[shapeIndex.Type].RemoveAt(shapeIndex.Index);
        }

        /// <summary>
        /// Clears all shapes from existing batches. Does not release any memory.
        /// </summary>
        public void Clear()
        {
            for (int i = 0; i < batches.Count; ++i)
            {
                if (batches[i] != null)
                    batches[i].Clear();
            }
        }

        //Technically we're missing some degrees of freedom here, but these are primarily convenience functions. The underlying batches have the remaining (much more rarely used) functionality.
        //You may also note that we don't have any form of per-type minimum capacities like we do in the solver. The solver benefits from tighter 'dynamic' control over allocations
        //because type batches are expected to be created and destroyed pretty frequently- sometimes multiple times a frame. Contact constraints come and go regardless of user input.
        //Shapes, on the other hand, only get added or removed by the user.
        /// <summary>
        /// Ensures a minimum capacity for all existing shape batches.
        /// </summary>
        /// <param name="shapeCapacity">Capacity to ensure for all existing shape batches.</param>
        public void EnsureBatchCapacities(int shapeCapacity)
        {
            for (int i = 0; i < batches.Count; ++i)
            {
                if (batches[i] != null)
                    batches[i].EnsureCapacity(shapeCapacity);
            }
        }

        /// <summary>
        /// Resizes all existing batches for a target capacity. Note that this is conservative; it will never orphan an existing shape.
        /// </summary>
        /// <param name="shapeCapacity">Capacity to target for all existing shape batches.</param>
        public void ResizeBatches(int shapeCapacity)
        {
            for (int i = 0; i < batches.Count; ++i)
            {
                if (batches[i] != null)
                    batches[i].Resize(shapeCapacity);
            }
        }

        /// <summary>
        /// Releases all memory from existing batches. Leaves shapes set in an unusable state.
        /// </summary>
        public void Dispose()
        {
            for (int i = 0; i < batches.Count; ++i)
            {
                if (batches[i] != null)
                    batches[i].Dispose();
            }
        }
    }
}
