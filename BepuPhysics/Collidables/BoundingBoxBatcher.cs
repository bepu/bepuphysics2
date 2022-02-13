using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics
{
    public struct BoundsContinuation
    {
        //Bits 0-30: body index
        //Bit 31: compound flag; if set, the continuation should merge into the target slot rather than merely setting it.
        uint packed;

        /// <summary>
        /// Gets the index of the body associated with this continuation.
        /// </summary>
        public int BodyIndex
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return (int)(packed & 0x7FFFFFFF);
            }
        }

        /// <summary>
        /// Gets whether this continuation is associated with a compound's child.
        /// </summary>
        public bool CompoundChild
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return (packed & (1u << 31)) > 0;
            }
        }

        /// <summary>
        /// Creates a bounding box calculation continuation for a given noncompound body.
        /// </summary>
        /// <param name="bodyIndex">Index of the body to set the bounding box of.</param>
        public static BoundsContinuation CreateContinuation(int bodyIndex)
        {
            Debug.Assert(bodyIndex >= 0);
            BoundsContinuation toReturn;
            toReturn.packed = (uint)bodyIndex;
            return toReturn;
        }
        /// <summary>
        /// Creates a bounding box calculation continuation for a given compound body.
        /// </summary>
        /// <param name="compoundBodyIndex">Index of the compound body to set the bounding box of.</param>
        public static BoundsContinuation CreateCompoundChildContinuation(int compoundBodyIndex)
        {
            Debug.Assert(compoundBodyIndex >= 0);
            BoundsContinuation toReturn;
            toReturn.packed = (1u << 31) | (uint)compoundBodyIndex;
            return toReturn;
        }
    }


    public struct BoundingBoxInstance
    {
        public int ShapeIndex;
        public BoundsContinuation Continuation;
    }

    public struct BoundingBoxBatch
    {
        public Buffer<int> ShapeIndices;
        public Buffer<BoundsContinuation> Continuations;
        public Buffer<MotionState> MotionStates;
        public int Count;
        public bool Allocated => ShapeIndices.Allocated;

        public BoundingBoxBatch(BufferPool pool, int initialCapacity)
        {
            pool.Take(initialCapacity, out ShapeIndices);
            pool.Take(initialCapacity, out Continuations);
            pool.Take(initialCapacity, out MotionStates);
            Count = 0;
        }
        internal void Add(int shapeIndex, RigidPose pose, BodyVelocity velocity, BoundsContinuation continuation)
        {
            ShapeIndices[Count] = shapeIndex;
            Continuations[Count] = continuation;
            ref var motionState = ref MotionStates[Count];
            motionState.Pose = pose;
            motionState.Velocity = velocity;
            Count++;
        }

        public void Dispose(BufferPool pool)
        {
            if (Allocated)
            {
                pool.Return(ref ShapeIndices);
                pool.Return(ref Continuations);
                pool.Return(ref MotionStates);
                Count = 0;
            }
        }

    }

    public struct BoundingBoxBatcher
    {
        internal BufferPool pool;
        internal Shapes shapes;
        internal Bodies bodies;
        internal BroadPhase broadPhase;
        internal float dt;

        int minimumBatchIndex, maximumBatchIndex;
        Buffer<BoundingBoxBatch> batches;

        /// <summary>
        /// The number of bodies to accumulate per type before executing an AABB update. The more bodies per batch, the less virtual overhead and execution divergence.
        /// However, this should be kept low enough such that the data that has to be gathered by the bounding box update is still usually in L1.
        /// </summary>
        public const int CollidablesPerFlush = 16;

        public unsafe BoundingBoxBatcher(Bodies bodies, Shapes shapes, BroadPhase broadPhase, BufferPool pool, float dt)
        {
            this.bodies = bodies;
            this.shapes = shapes;
            this.broadPhase = broadPhase;
            this.pool = pool;
            this.dt = dt;
            pool.TakeAtLeast(shapes.RegisteredTypeSpan, out batches);
            //Clearing is required ensure that we know when a batch needs to be created and when a batch needs to be disposed.
            batches.Clear(0, shapes.RegisteredTypeSpan);
            minimumBatchIndex = shapes.RegisteredTypeSpan;
            maximumBatchIndex = -1;
        }

        public unsafe void ExecuteConvexBatch<TShape, TShapeWide>(ConvexShapeBatch<TShape, TShapeWide> shapeBatch) where TShape : unmanaged, IConvexShape where TShapeWide : unmanaged, IShapeWide<TShape>
        {
            Unsafe.SkipInit(out TShapeWide shapeWide);
            if (shapeWide.InternalAllocationSize > 0) //TODO: Check to make sure the JIT omits the branch.
            {
                var memory = stackalloc byte[shapeWide.InternalAllocationSize];
                shapeWide.Initialize(new Buffer<byte>(memory, shapeWide.InternalAllocationSize));
            }
            ref var batch = ref batches[shapeBatch.TypeId];
            var shapeIndices = batch.ShapeIndices;
            var continuations = batch.Continuations;
            ref var activeSet = ref bodies.ActiveSet;
            var dtWide = new Vector<float>(dt);
            Span<float> minimumMarginSpan = stackalloc float[Vector<float>.Count];
            Span<float> maximumMarginSpan = stackalloc float[Vector<float>.Count];
            Span<int> allowExpansionBeyondSpeculativeMarginSpan = stackalloc int[Vector<int>.Count];
            for (int bundleStartIndex = 0; bundleStartIndex < batch.Count; bundleStartIndex += Vector<float>.Count)
            {
                int countInBundle = batch.Count - bundleStartIndex;
                if (countInBundle > Vector<float>.Count)
                    countInBundle = Vector<float>.Count;

                //Note that doing a gather-scatter to enable vectorized bundle execution isn't worth it for some shape types.
                //We're just ignoring that fact for simplicity. Bounding box updates aren't a huge concern overall. That said,
                //if you want to optimize this further, the shape batches could choose between vectorized and gatherless scalar implementations on a per-type basis.
                //TODO: This transposition could be significantly improved with intrinsics, as we did for the Bodies state gather operations.
                for (int innerIndex = 0; innerIndex < countInBundle; ++innerIndex)
                {
                    var indexInBatch = bundleStartIndex + innerIndex;
                    var shapeIndex = shapeIndices[indexInBatch];
                    shapeWide.WriteSlot(innerIndex, shapeBatch.shapes[shapeIndex]);
                    ref var collidable = ref activeSet.Collidables[continuations[indexInBatch].BodyIndex];
                    minimumMarginSpan[innerIndex] = collidable.MinimumSpeculativeMargin;
                    maximumMarginSpan[innerIndex] = collidable.MaximumSpeculativeMargin;
                    allowExpansionBeyondSpeculativeMarginSpan[innerIndex] = collidable.Continuity.AllowExpansionBeyondSpeculativeMargin ? -1 : 0;
                }

                Bodies.TransposeMotionStates(batch.MotionStates.Slice(bundleStartIndex, countInBundle), out var positions, out var orientations, out var velocities);
                shapeWide.GetBounds(ref orientations, countInBundle, out var maximumRadius, out var maximumAngularExpansion, out var bundleMin, out var bundleMax);
                //BoundingBoxBatcher is responsible for updating the bounding box AND speculative margin.
                //In order to know how much we're allowed to expand the bounding box, we need to know the speculative margin.
                //It's defined by the velocity of the body, and bounded by the body's minimum and maximum.
                var angularBoundsExpansion = BoundingBoxHelpers.GetAngularBoundsExpansion(Vector3Wide.Length(velocities.Angular), dtWide, maximumRadius, maximumAngularExpansion);
                var speculativeMargin = Vector3Wide.Length(velocities.Linear) * dtWide + angularBoundsExpansion;

                var minimumSpeculativeMargin = new Vector<float>(minimumMarginSpan);
                var maximumSpeculativeMargin = new Vector<float>(maximumMarginSpan);
                var allowExpansionBeyondSpeculativeMargin = new Vector<int>(allowExpansionBeyondSpeculativeMarginSpan);
                speculativeMargin = Vector.Max(minimumSpeculativeMargin, Vector.Min(maximumSpeculativeMargin, speculativeMargin));
                var maximumBoundsExpansion = Vector.ConditionalSelect(allowExpansionBeyondSpeculativeMargin, new Vector<float>(float.MaxValue), speculativeMargin);
                BoundingBoxHelpers.GetBoundsExpansion(velocities.Linear, dtWide, angularBoundsExpansion, out var minExpansion, out var maxExpansion);
                minExpansion = Vector3Wide.Max(-maximumBoundsExpansion, minExpansion);
                maxExpansion = Vector3Wide.Min(maximumBoundsExpansion, maxExpansion);
                //TODO: Note that this is an area that must be updated if you change the pose representation.
                bundleMin = positions + (bundleMin + minExpansion);
                bundleMax = positions + (bundleMax + maxExpansion);

                for (int innerIndex = 0; innerIndex < countInBundle; ++innerIndex)
                {
                    var continuation = continuations[bundleStartIndex + innerIndex];
                    ref var collidable = ref activeSet.Collidables[continuation.BodyIndex];
                    broadPhase.GetActiveBoundsPointers(collidable.BroadPhaseIndex, out var minPointer, out var maxPointer);

                    //Note that we merge with the existing bounding box if the body is compound. This requires compounds to be initialized to (maxvalue, -maxvalue).
                    //TODO: We bite the bullet on quite a bit of complexity to avoid merging on non-compounds. Could be better overall to simply merge on all bodies. Certainly simpler.
                    //Worth checking the performance; if it's undetectable, just swap to the simpler version.
                    if (continuation.CompoundChild)
                    {
                        collidable.SpeculativeMargin = MathF.Max(collidable.SpeculativeMargin, speculativeMargin[innerIndex]);
                        var min = new Vector3(bundleMin.X[innerIndex], bundleMin.Y[innerIndex], bundleMin.Z[innerIndex]);
                        var max = new Vector3(bundleMax.X[innerIndex], bundleMax.Y[innerIndex], bundleMax.Z[innerIndex]);
                        BoundingBox.CreateMerged(*minPointer, *maxPointer, min, max, out *minPointer, out *maxPointer);
                    }
                    else
                    {
                        collidable.SpeculativeMargin = speculativeMargin[innerIndex];
                        *minPointer = new Vector3(bundleMin.X[innerIndex], bundleMin.Y[innerIndex], bundleMin.Z[innerIndex]);
                        *maxPointer = new Vector3(bundleMax.X[innerIndex], bundleMax.Y[innerIndex], bundleMax.Z[innerIndex]);
                    }
                }
            }
        }

        public unsafe void ExecuteHomogeneousCompoundBatch<TShape, TChildShape, TChildShapeWide>(HomogeneousCompoundShapeBatch<TShape, TChildShape, TChildShapeWide> shapeBatch)
            where TShape : unmanaged, IHomogeneousCompoundShape<TChildShape, TChildShapeWide>
            where TChildShape : IConvexShape
            where TChildShapeWide : IShapeWide<TChildShape>
        {
            ref var batch = ref batches[shapeBatch.TypeId];
            ref var activeSet = ref bodies.ActiveSet;
            for (int i = 0; i < batch.Count; ++i)
            {
                var shapeIndex = batch.ShapeIndices[i];
                ref var motionState = ref batch.MotionStates[i];
                var bodyIndex = batch.Continuations[i].BodyIndex;
                ref var collidable = ref activeSet.Collidables[bodyIndex];
                shapeBatch[shapeIndex].ComputeBounds(motionState.Pose.Orientation, out var min, out var max);
                //Working on the assumption that dynamic meshes are extremely rare, and that dynamic meshes with extremely high angular velocity are even rarer,
                //we're just going to use a simplistic upper bound for angular expansion. This simplifies the mesh bounding box calculation quite a bit (no dot products).
                var absMin = Vector3.Abs(min);
                var absMax = Vector3.Abs(max);
                var maximumRadius = Vector3.Max(absMin, absMax).Length();

                var minimumComponents = Vector3.Min(absMin, absMax);
                var minimumRadius = MathHelper.Min(minimumComponents.X, MathHelper.Min(minimumComponents.Y, minimumComponents.Z));
                var maximumAngularExpansion = maximumRadius - minimumRadius;

                //BoundingBoxBatcher is responsible for updating the bounding box AND speculative margin.
                //In order to know how much we're allowed to expand the bounding box, we need to know the speculative margin.
                //It's defined by the velocity of the body, and bounded by the body's minimum and maximum.
                var angularBoundsExpansion = BoundingBoxHelpers.GetAngularBoundsExpansion(motionState.Velocity.Angular.Length(), dt, maximumRadius, maximumAngularExpansion);
                var speculativeMargin = motionState.Velocity.Linear.Length() * dt + angularBoundsExpansion;
                speculativeMargin = MathF.Max(collidable.MinimumSpeculativeMargin, MathF.Min(collidable.MaximumSpeculativeMargin, speculativeMargin));
                collidable.SpeculativeMargin = speculativeMargin;
                var maximumAllowedExpansion = collidable.Continuity.AllowExpansionBeyondSpeculativeMargin ? float.MaxValue : speculativeMargin;
                BoundingBoxHelpers.GetBoundsExpansion(motionState.Velocity.Linear, dt, angularBoundsExpansion, out var minExpansion, out var maxExpansion);
                var broadcastMaximumBoundsExpansion = new Vector3(maximumAllowedExpansion);
                minExpansion = Vector3.Max(-broadcastMaximumBoundsExpansion, minExpansion);
                maxExpansion = Vector3.Min(broadcastMaximumBoundsExpansion, maxExpansion);

                broadPhase.GetActiveBoundsPointers(collidable.BroadPhaseIndex, out var minPointer, out var maxPointer);
                *minPointer = motionState.Pose.Position + (min + minExpansion);
                *maxPointer = motionState.Pose.Position + (max + maxExpansion);
            }
        }

        public unsafe void ExecuteCompoundBatch<TShape>(CompoundShapeBatch<TShape> shapeBatch) where TShape : unmanaged, ICompoundShape
        {
            ref var batch = ref batches[shapeBatch.TypeId];
            ref var activeSet = ref bodies.ActiveSet;
            var minValue = new Vector3(float.MaxValue);
            var maxValue = new Vector3(-float.MaxValue);
            for (int i = 0; i < batch.Count; ++i)
            {
                var shapeIndex = batch.ShapeIndices[i];
                var bodyIndex = batch.Continuations[i].BodyIndex;
                ref var motionState = ref batch.MotionStates[i];
                //We have to clear out the bounds and speculative margin used by compounds, since each contributing body will merge their contribution into the whole.
                ref var collidable = ref activeSet.Collidables[bodyIndex];
                collidable.SpeculativeMargin = 0;
                broadPhase.GetActiveBoundsPointers(collidable.BroadPhaseIndex, out var min, out var max);
                *min = minValue;
                *max = maxValue;
                shapeBatch.shapes[batch.ShapeIndices[i]].AddChildBoundsToBatcher(ref this, motionState.Pose, motionState.Velocity, bodyIndex);
            }
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void Add(TypedIndex shapeIndex, RigidPose pose, BodyVelocity velocity, BoundsContinuation continuation)
        {
            var typeIndex = shapeIndex.Type;
            Debug.Assert(typeIndex >= 0 && typeIndex < batches.Length, "The preallocated type batch array should be able to hold every type index. Is the type index broken?");
            ref var batchSlot = ref batches[typeIndex];
            if (!batchSlot.Allocated)
            {
                //No list exists for this type yet.
                batchSlot = new BoundingBoxBatch(pool, CollidablesPerFlush);
                if (typeIndex < minimumBatchIndex)
                    minimumBatchIndex = typeIndex;
                if (typeIndex > maximumBatchIndex)
                    maximumBatchIndex = typeIndex;
            }
            //TODO: Arguably, batching up compounds is silly. 
            //It technically opens the door for vectorizing their child pose calculations, but those are almost certainly not worth vectorizing anyway.
            //May want to consider directly triggering a bounds dispatch for compounds here. 
            //(Doing so WOULD be more complicated, though.)
            batchSlot.Add(shapeIndex.Index, pose, velocity, continuation);
            if (batchSlot.Count == CollidablesPerFlush)
            {
                shapes[typeIndex].ComputeBounds(ref this);
                batchSlot.Count = 0;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Add(int bodyIndex, in RigidPose pose, in BodyVelocity velocity, in Collidable collidable)
        {
            //For convenience, this function handles the case where the collidable reference points to nothing.
            //Note that this touches the memory associated with the full collidable. That's okay- we'll be reading the rest of it shortly if it has a collidable.
            //Technically, you could make a second pass that only processes collidables, rather than iterating over all bodies and doing last second branches.
            //But then you'd be evicting everything from cache L1/L2. And, 99.99% of the time, bodies are going to have shapes, so this isn't going to be a difficult branch to predict.
            //Even if it was 50%, the cache benefit of executing alongside the just-touched data source would outweigh the misprediction.
            if (collidable.Shape.Exists)
            {
                Add(collidable.Shape, pose, velocity, BoundsContinuation.CreateContinuation(bodyIndex));
            }
        }
        public void AddCompoundChild(int bodyIndex, TypedIndex shapeIndex, in RigidPose pose, in BodyVelocity velocity)
        {
            Add(shapeIndex, pose, velocity, BoundsContinuation.CreateCompoundChildContinuation(bodyIndex));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Flush()
        {
#if DEBUG
            //We rely on shape type ids being assigned such that compounds and convexes are split into two contiguous groups with respect to their ids.
            //This avoids cycles in work generation, since compounds can only contain convexes. Since the task graph is acyclic, we can flush in one pass.
            bool compoundEncountered = false;
            for (int i = minimumBatchIndex; i <= maximumBatchIndex; ++i)
            {
                var batch = shapes[i];
                if (batch != null)
                {
                    if (compoundEncountered)
                    {
                        Debug.Assert(batch.Compound, "If a compound shape batch has been found, all subsequent batches must also be compound to avoid cycles.");
                    }
                    else if (batch.Compound)
                    {
                        compoundEncountered = true;
                    }
                }
            }
#endif
            //Note reverse iteration. Execute all compound batches first.
            for (int i = maximumBatchIndex; i >= minimumBatchIndex; --i)
            {
                ref var batch = ref batches[i];
                if (batch.Count > 0)
                {
                    shapes[i].ComputeBounds(ref this);
                }
                //Dispose of the batch and any associated buffers; since the flush is one pass, we won't be needing this again.
                batch.Dispose(pool);
            }
            pool.Return(ref batches);
        }
    }
}
