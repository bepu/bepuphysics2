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
        protected Buffer<byte> shapesData;
        protected int shapeDataSize;
        /// <summary>
        /// Gets the number of shapes that the batch can currently hold without resizing.
        /// </summary>
        public int Capacity { get { return shapesData.Length / shapeDataSize; } }
        protected BufferPool pool;
        protected IdPool idPool;
        /// <summary>
        /// Gets the type id of the shape type in this batch.
        /// </summary>
        public int TypeId { get; protected set; }
        /// <summary>
        /// Gets whether this shape batch's contained type potentially contains children that require other shape batches.
        /// </summary>
        public bool Compound { get; protected set; }
        /// <summary>
        /// Gets the size of the shape type stored in this batch in bytes.
        /// </summary>
        public int ShapeDataSize { get { return shapeDataSize; } }

        protected abstract void Dispose(int index, BufferPool pool);
        protected abstract void RemoveAndDisposeChildren(int index, Shapes shapes, BufferPool pool);

        public void Remove(int index)
        {
            idPool.Return(index, pool);
        }

        public void RemoveAndDispose(int index, BufferPool pool)
        {
            Dispose(index, pool);
            Remove(index);
        }

        public void RecursivelyRemoveAndDispose(int index, Shapes shapes, BufferPool pool)
        {
            RemoveAndDisposeChildren(index, shapes, pool);
            RemoveAndDispose(index, pool);
        }

        public abstract void ComputeBounds(ref BoundingBoxBatcher batcher);
        public abstract void ComputeBounds(int shapeIndex, in RigidPose pose, out Vector3 min, out Vector3 max);
        internal virtual void ComputeBounds(int shapeIndex, in Quaternion orientation, out float maximumRadius, out float maximumAngularExpansion, out Vector3 min, out Vector3 max)
        {
            throw new InvalidOperationException("Nonconvex shapes are not required to have a maximum radius or angular expansion implementation. This should only ever be called on convexes.");
        }
        public abstract void RayTest<TRayHitHandler>(int shapeIndex, in RigidPose pose, in RayData ray, ref float maximumT, ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayHitHandler;
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
            Debug.Assert(shapeIndex >= 0 && shapeIndex < Capacity);
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
            idPool.Resize(targetIdCapacity, pool);
        }

    }

    public abstract class ShapeBatch<TShape> : ShapeBatch where TShape : unmanaged, IShape
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
            idPool = new IdPool(initialShapeCount, pool);
        }

        //Note that shapes cannot be moved; there is no reference to the collidables using them, so we can't correct their indices.
        //But that's fine- we never directly iterate over the shapes set anyway.
        //(This doesn't mean that it's impossible to compact the shape set- it just requires doing so by iterating over collidables.)
        public int Add(in TShape shape)
        {
            var shapeIndex = idPool.Take();
            if (shapes.Length <= shapeIndex)
            {
                InternalResize(shapeIndex + 1, shapes.Length);
            }
            shapes[shapeIndex] = shape;
            return shapeIndex;
        }


        void InternalResize(int shapeCount, int oldCopyLength)
        {
            shapeDataSize = Unsafe.SizeOf<TShape>();
            var requiredSizeInBytes = shapeCount * Unsafe.SizeOf<TShape>();
            pool.TakeAtLeast<byte>(requiredSizeInBytes, out var newShapesData);
            var newShapes = newShapesData.As<TShape>();
#if DEBUG
            //In debug mode, unused slots are kept at the default value. This helps catch misuse.
            if (newShapes.Length > shapes.Length)
                newShapes.Clear(shapes.Length, newShapes.Length - shapes.Length);
#endif
            if (shapesData.Allocated)
            {
                shapes.CopyTo(0, newShapes, 0, oldCopyLength);
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
            shapeCapacity = BufferPool.GetCapacityForCount<TShape>(Math.Max(idPool.HighestPossiblyClaimedId + 1, shapeCapacity));
            if (shapeCapacity != shapes.Length)
            {
                InternalResize(shapeCapacity, idPool.HighestPossiblyClaimedId + 1);
            }
        }
        public override void Dispose()
        {
            Debug.Assert(shapesData.Id == shapes.Id, "If the buffer ids don't match, there was some form of failed resize.");
            pool.Return(ref shapesData);
            idPool.Dispose(pool);
        }
    }


    public class ConvexShapeBatch<TShape, TShapeWide> : ShapeBatch<TShape>
        where TShape : unmanaged, IConvexShape
        where TShapeWide : unmanaged, IShapeWide<TShape>
    {
        public ConvexShapeBatch(BufferPool pool, int initialShapeCount) : base(pool, initialShapeCount)
        {
        }

        protected override void Dispose(int index, BufferPool pool)
        {
            //Most convex shapes with an associated Wide type doesn't have any internal resources to dispose.
        }

        protected override void RemoveAndDisposeChildren(int index, Shapes shapes, BufferPool pool)
        {
            //And they don't have any children.
        }

        public override void ComputeBounds(ref BoundingBoxBatcher batcher)
        {
            batcher.ExecuteConvexBatch(this);
        }

        public override void ComputeBounds(int shapeIndex, in RigidPose pose, out Vector3 min, out Vector3 max)
        {
            shapes[shapeIndex].ComputeBounds(pose.Orientation, out min, out max);
            min += pose.Position;
            max += pose.Position;
        }

        internal override void ComputeBounds(int shapeIndex, in Quaternion orientation, out float maximumRadius, out float angularExpansion, out Vector3 min, out Vector3 max)
        {
            ref var shape = ref shapes[shapeIndex];
            shape.ComputeBounds(orientation, out min, out max);
            shape.ComputeAngularExpansionData(out maximumRadius, out angularExpansion);
        }

        public override void RayTest<TRayHitHandler>(int shapeIndex, in RigidPose pose, in RayData ray, ref float maximumT, ref TRayHitHandler hitHandler)
        {
            if (shapes[shapeIndex].RayTest(pose, ray.Origin, ray.Direction, out var t, out var normal) && t <= maximumT)
            {
                hitHandler.OnRayHit(ray, ref maximumT, t, normal, 0);
            }
        }

        public unsafe override void RayTest<TRayHitHandler>(int index, in RigidPose pose, ref RaySource rays, ref TRayHitHandler hitHandler)
        {
            WideRayTester.Test<RaySource, TShape, TShapeWide, TRayHitHandler>(ref shapes[index], pose, ref rays, ref hitHandler);
        }
    }

    public class ConvexHullShapeBatch : ConvexShapeBatch<ConvexHull, ConvexHullWide>
    {
        public ConvexHullShapeBatch(BufferPool pool, int initialShapeCount) : base(pool, initialShapeCount)
        {
        }

        protected override void Dispose(int index, BufferPool pool)
        {
            shapes[index].Dispose(pool);
        }
    }


    public class HomogeneousCompoundShapeBatch<TShape, TChildShape, TChildShapeWide> : ShapeBatch<TShape> where TShape : unmanaged, IHomogeneousCompoundShape<TChildShape, TChildShapeWide>
        where TChildShape : IConvexShape
        where TChildShapeWide : IShapeWide<TChildShape>
    {
        public HomogeneousCompoundShapeBatch(BufferPool pool, int initialShapeCount) : base(pool, initialShapeCount)
        {
            Compound = true;
        }

        protected override void Dispose(int index, BufferPool pool)
        {
            shapes[index].Dispose(pool);
        }

        protected override void RemoveAndDisposeChildren(int index, Shapes shapes, BufferPool pool)
        {
            //Meshes and other single-type containers don't have any shape-registered children.
        }

        public override void ComputeBounds(ref BoundingBoxBatcher batcher)
        {
            batcher.ExecuteHomogeneousCompoundBatch(this);
        }

        public override void ComputeBounds(int shapeIndex, in RigidPose pose, out Vector3 min, out Vector3 max)
        {
            shapes[shapeIndex].ComputeBounds(pose.Orientation, out min, out max);
            min += pose.Position;
            max += pose.Position;
        }
        public override void RayTest<TRayHitHandler>(int shapeIndex, in RigidPose pose, in RayData ray, ref float maximumT, ref TRayHitHandler hitHandler)
        {
            shapes[shapeIndex].RayTest(pose, ray, ref maximumT, ref hitHandler);
        }

        public override void RayTest<TRayHitHandler>(int shapeIndex, in RigidPose pose, ref RaySource rays, ref TRayHitHandler hitHandler)
        {
            shapes[shapeIndex].RayTest(pose, ref rays, ref hitHandler);
        }
    }

    public class CompoundShapeBatch<TShape> : ShapeBatch<TShape> where TShape : unmanaged, ICompoundShape
    {
        Shapes shapeBatches;

        public CompoundShapeBatch(BufferPool pool, int initialShapeCount, Shapes shapeBatches) : base(pool, initialShapeCount)
        {
            this.shapeBatches = shapeBatches;
            Compound = true;
        }

        protected override void Dispose(int index, BufferPool pool)
        {
            shapes[index].Dispose(pool);
        }

        protected override void RemoveAndDisposeChildren(int index, Shapes shapes, BufferPool pool)
        {
            ref var shape = ref this.shapes[index];
            for (int i = 0; i < shape.ChildCount; ++i)
            {
                ref var child = ref shape.GetChild(i);
                shapes.RecursivelyRemoveAndDispose(child.ShapeIndex, pool);
            }
        }

        public override void ComputeBounds(ref BoundingBoxBatcher batcher)
        {
            batcher.ExecuteCompoundBatch(this);
        }

        public override void ComputeBounds(int shapeIndex, in RigidPose pose, out Vector3 min, out Vector3 max)
        {
            shapes[shapeIndex].ComputeBounds(pose.Orientation, shapeBatches, out min, out max);
            min += pose.Position;
            max += pose.Position;
        }

        public override void RayTest<TRayHitHandler>(int shapeIndex, in RigidPose pose, in RayData ray, ref float maximumT, ref TRayHitHandler hitHandler)
        {
            shapes[shapeIndex].RayTest(pose, ray, ref maximumT, shapeBatches, ref hitHandler);
        }

        public override void RayTest<TRayHitHandler>(int shapeIndex, in RigidPose pose, ref RaySource rays, ref TRayHitHandler hitHandler)
        {
            shapes[shapeIndex].RayTest(pose, ref rays, shapeBatches, ref hitHandler);
        }
    }



    public class Shapes
    {
        ShapeBatch[] batches;
        int registeredTypeSpan;

        //Note that not every index within the batches list is guaranteed to be filled. For example, if only a cylinder has been added, and a cylinder's type id is 7,
        //then the batches.Count and RegisteredTypeSpan will be 8- but indices 0 through 6 will be null.
        //We don't tend to do any performance sensitive iteration over shape type batches, so this lack of contiguity is fine.
        public int RegisteredTypeSpan => registeredTypeSpan;

        public int InitialCapacityPerTypeBatch { get; set; }
        public ShapeBatch this[int typeIndex] => batches[typeIndex];
        BufferPool pool;


        public Shapes(BufferPool pool, int initialCapacityPerTypeBatch)
        {
            InitialCapacityPerTypeBatch = initialCapacityPerTypeBatch;
            //This list pretty much will never resize unless something really strange happens, and since batches use virtual calls, we have to allow storage of reference types.
            batches = new ShapeBatch[16];
            this.pool = pool;
        }

        /// <summary>
        /// Computes a bounding box for a single shape.
        /// </summary>
        /// <param name="pose">Pose to calculate the bounding box of.</param>
        /// <param name="shapeIndex">Index of the shape.</param>
        /// <param name="bounds">Bounding box of the specified shape with the specified pose.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void UpdateBounds(in RigidPose pose, ref TypedIndex shapeIndex, out BoundingBox bounds)
        {
            //Note: the min and max here are in absolute coordinates, which means this is a spot that has to be updated in the event that positions use a higher precision representation.
            batches[shapeIndex.Type].ComputeBounds(shapeIndex.Index, pose, out bounds.Min, out bounds.Max);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref TShape GetShape<TShape>(int shapeIndex) where TShape : unmanaged, IShape
        {
            var typeId = default(TShape).TypeId;
            return ref Unsafe.As<ShapeBatch, ShapeBatch<TShape>>(ref batches[typeId])[shapeIndex];
        }


        public TypedIndex Add<TShape>(in TShape shape) where TShape : unmanaged, IShape
        {
            var typeId = default(TShape).TypeId;
            if (RegisteredTypeSpan <= typeId)
            {
                registeredTypeSpan = typeId + 1;
                if (batches.Length <= typeId)
                {
                    Array.Resize(ref batches, typeId + 1);
                }
            }
            if (batches[typeId] == null)
            {
                batches[typeId] = default(TShape).CreateShapeBatch(pool, InitialCapacityPerTypeBatch, this);
            }

            Debug.Assert(batches[typeId] is ShapeBatch<TShape>);
            var batch = Unsafe.As<ShapeBatch, ShapeBatch<TShape>>(ref batches[typeId]);
            var index = batch.Add(shape);
            return new TypedIndex(typeId, index);
        }



        /// <summary>
        /// Removes a shape and any existing children from the shapes collection and returns their resources to the given pool.
        /// </summary>
        /// <param name="shapeIndex">Index of the shape to remove.</param>
        /// <param name="pool">Pool to return all shape resources to.</param>
        public void RecursivelyRemoveAndDispose(TypedIndex shapeIndex, BufferPool pool)
        {
            if (shapeIndex.Exists)
            {
                Debug.Assert(RegisteredTypeSpan > shapeIndex.Type && batches[shapeIndex.Type] != null);
                batches[shapeIndex.Type].RecursivelyRemoveAndDispose(shapeIndex.Index, this, pool);
            }
        }

        /// <summary>
        /// Removes a shape from the shapes collection and returns its resources to the given pool. Does not remove or dispose any children.
        /// </summary>
        /// <param name="shapeIndex">Index of the shape to remove.</param>
        /// <param name="pool">Pool to return all shape resources to.</param>
        public void RemoveAndDispose(TypedIndex shapeIndex, BufferPool pool)
        {
            if (shapeIndex.Exists)
            {
                Debug.Assert(RegisteredTypeSpan > shapeIndex.Type && batches[shapeIndex.Type] != null);
                batches[shapeIndex.Type].RemoveAndDispose(shapeIndex.Index, pool);
            }
        }

        /// <summary>
        /// Removes a shape without removing its children or disposing any resources.
        /// </summary>
        /// <param name="shapeIndex">Index of the shape to remove.</param>
        public void Remove(TypedIndex shapeIndex)
        {
            if (shapeIndex.Exists)
            {
                Debug.Assert(RegisteredTypeSpan > shapeIndex.Type && batches[shapeIndex.Type] != null);
                batches[shapeIndex.Type].Remove(shapeIndex.Index);
            }
        }

        /// <summary>
        /// Clears all shapes from existing batches. Does not release any memory.
        /// </summary>
        public void Clear()
        {
            for (int i = 0; i < registeredTypeSpan; ++i)
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
            for (int i = 0; i < registeredTypeSpan; ++i)
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
            for (int i = 0; i < registeredTypeSpan; ++i)
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
            for (int i = 0; i < registeredTypeSpan; ++i)
            {
                if (batches[i] != null)
                    batches[i].Dispose();
            }
        }
    }
}
