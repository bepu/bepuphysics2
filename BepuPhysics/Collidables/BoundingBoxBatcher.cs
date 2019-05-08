using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
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
        public RigidPose Pose;
        public BodyVelocity Velocities;
        public int ShapeIndex;
        public BoundsContinuation Continuation;
    }

    public struct BoundingBoxInstanceWide<TShape, TShapeWide> where TShape : struct, IShape where TShapeWide : struct, IShapeWide<TShape>
    {
        public TShapeWide Shape;
        public Vector<float> MaximumExpansion;
        public RigidPoses Pose;
        public BodyVelocities Velocities;
    }

    public struct BoundingBoxBatcher
    {
        internal BufferPool pool;
        internal Shapes shapes;
        internal Bodies bodies;
        internal BroadPhase broadPhase;
        internal float dt;

        int minimumBatchIndex, maximumBatchIndex;
        Buffer<QuickList<BoundingBoxInstance>> batches;

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

        public unsafe void ExecuteConvexBatch<TShape, TShapeWide>(ConvexShapeBatch<TShape, TShapeWide> shapeBatch) where TShape : struct, IConvexShape where TShapeWide : struct, IShapeWide<TShape>
        {
            var instanceBundle = default(BoundingBoxInstanceWide<TShape, TShapeWide>);
            if (instanceBundle.Shape.InternalAllocationSize > 0) //TODO: Check to make sure the JIT omits the branch.
            {
                var memory = stackalloc byte[instanceBundle.Shape.InternalAllocationSize];
                instanceBundle.Shape.Initialize(new RawBuffer(memory, instanceBundle.Shape.InternalAllocationSize));
            }
            ref var batch = ref batches[shapeBatch.TypeId];
            ref var instancesBase = ref batch[0];
            ref var activeSet = ref bodies.ActiveSet;

            for (int bundleStartIndex = 0; bundleStartIndex < batch.Count; bundleStartIndex += Vector<float>.Count)
            {
                int countInBundle = batch.Count - bundleStartIndex;
                if (countInBundle > Vector<float>.Count)
                    countInBundle = Vector<float>.Count;
                ref var bundleInstancesStart = ref Unsafe.Add(ref instancesBase, bundleStartIndex);
                //Note that doing a gather-scatter to enable vectorized bundle execution isn't worth it for some shape types.
                //We're just ignoring that fact for simplicity. Bounding box updates aren't a huge concern overall. That said,
                //if you want to optimize this further, the shape batches could choose between vectorized and gatherless scalar implementations on a per-type basis.
                for (int innerIndex = 0; innerIndex < countInBundle; ++innerIndex)
                {
                    ref var instance = ref Unsafe.Add(ref bundleInstancesStart, innerIndex);
                    ref var targetInstanceSlot = ref GatherScatter.GetOffsetInstance(ref instanceBundle, innerIndex);
                    //This property should be a constant value and the JIT has type knowledge, so this branch should optimize out.
                    if (instanceBundle.Shape.AllowOffsetMemoryAccess)
                        targetInstanceSlot.Shape.WriteFirst(shapeBatch.shapes[instance.ShapeIndex]);
                    else
                        instanceBundle.Shape.WriteSlot(innerIndex, shapeBatch.shapes[instance.ShapeIndex]);
                    Vector3Wide.WriteFirst(instance.Pose.Position, ref targetInstanceSlot.Pose.Position);
                    QuaternionWide.WriteFirst(instance.Pose.Orientation, ref targetInstanceSlot.Pose.Orientation);
                    Vector3Wide.WriteFirst(instance.Velocities.Linear, ref targetInstanceSlot.Velocities.Linear);
                    Vector3Wide.WriteFirst(instance.Velocities.Angular, ref targetInstanceSlot.Velocities.Angular);
                    ref var collidable = ref activeSet.Collidables[instance.Continuation.BodyIndex];
                    GatherScatter.GetFirst(ref targetInstanceSlot.MaximumExpansion) =
                        collidable.Continuity.AllowExpansionBeyondSpeculativeMargin ? float.MaxValue : collidable.SpeculativeMargin;
                }
                instanceBundle.Shape.GetBounds(ref instanceBundle.Pose.Orientation, countInBundle, out var maximumRadius, out var maximumAngularExpansion, out var bundleMin, out var bundleMax);
                BoundingBoxHelpers.ExpandBoundingBoxes(ref bundleMin, ref bundleMax, ref instanceBundle.Velocities, dt,
                    ref maximumRadius, ref maximumAngularExpansion, ref instanceBundle.MaximumExpansion);
                //TODO: Note that this is an area that must be updated if you change the pose representation.
                Vector3Wide.Add(instanceBundle.Pose.Position, bundleMin, out bundleMin);
                Vector3Wide.Add(instanceBundle.Pose.Position, bundleMax, out bundleMax);

                for (int innerIndex = 0; innerIndex < countInBundle; ++innerIndex)
                {
                    ref var instance = ref Unsafe.Add(ref bundleInstancesStart, innerIndex);
                    broadPhase.GetActiveBoundsPointers(activeSet.Collidables[instance.Continuation.BodyIndex].BroadPhaseIndex, out var minPointer, out var maxPointer);
                    ref var sourceBundleMin = ref GatherScatter.GetOffsetInstance(ref bundleMin, innerIndex);
                    ref var sourceBundleMax = ref GatherScatter.GetOffsetInstance(ref bundleMax, innerIndex);
                    //Note that we merge with the existing bounding box if the body is compound. This requires compounds to be initialized to (maxvalue, -maxvalue).
                    //TODO: We bite the bullet on quite a bit of complexity to avoid merging on non-compounds. Could be better overall to simply merge on all bodies. Certainly simpler.
                    //Worth checking the performance; if it's undetectable, just swap to the simpler version.
                    if (instance.Continuation.CompoundChild)
                    {
                        var min = new Vector3(sourceBundleMin.X[0], sourceBundleMin.Y[0], sourceBundleMin.Z[0]);
                        var max = new Vector3(sourceBundleMax.X[0], sourceBundleMax.Y[0], sourceBundleMax.Z[0]);
                        BoundingBox.CreateMerged(*minPointer, *maxPointer, min, max, out *minPointer, out *maxPointer);
                    }
                    else
                    {
                        *minPointer = new Vector3(sourceBundleMin.X[0], sourceBundleMin.Y[0], sourceBundleMin.Z[0]);
                        *maxPointer = new Vector3(sourceBundleMax.X[0], sourceBundleMax.Y[0], sourceBundleMax.Z[0]);
                    }
                }
            }
        }

        public unsafe void ExecuteHomogeneousCompoundBatch<TShape, TChildShape, TChildShapeWide>(HomogeneousCompoundShapeBatch<TShape, TChildShape, TChildShapeWide> shapeBatch)
            where TShape : struct, IHomogeneousCompoundShape<TChildShape, TChildShapeWide>
            where TChildShape : IConvexShape
            where TChildShapeWide : IShapeWide<TChildShape>
        {
            ref var batch = ref batches[shapeBatch.TypeId];
            ref var activeSet = ref bodies.ActiveSet;
            for (int i = 0; i < batch.Count; ++i)
            {
                ref var instance = ref batch[i];
                var bodyIndex = instance.Continuation.BodyIndex;
                ref var collidable = ref activeSet.Collidables[bodyIndex];
                broadPhase.GetActiveBoundsPointers(collidable.BroadPhaseIndex, out var min, out var max);
                var maximumAllowedExpansion = collidable.Continuity.AllowExpansionBeyondSpeculativeMargin ? float.MaxValue : collidable.SpeculativeMargin;
                shapeBatch[instance.ShapeIndex].ComputeBounds(instance.Pose.Orientation, out *min, out *max);
                //Working on the assumption that dynamic meshes are extremely rare, and that dynamic meshes with extremely high angular velocity are even rarer,
                //we're just going to use a simplistic upper bound for angular expansion. This simplifies the mesh bounding box calculation quite a bit (no dot products).
                var absMin = Vector3.Abs(*min);
                var absMax = Vector3.Abs(*max);
                var maximumRadius = Vector3.Max(absMin, absMax).Length();

                var minimumComponents = Vector3.Min(absMin, absMax);
                var minimumRadius = MathHelper.Min(minimumComponents.X, MathHelper.Min(minimumComponents.Y, minimumComponents.Z));
                BoundingBoxHelpers.ExpandBoundingBox(ref *min, ref *max, instance.Velocities.Linear, instance.Velocities.Angular, dt, maximumRadius, maximumRadius - minimumRadius, maximumAllowedExpansion);
                *min += instance.Pose.Position;
                *max += instance.Pose.Position;
            }
        }

        public unsafe void ExecuteCompoundBatch<TShape>(CompoundShapeBatch<TShape> shapeBatch) where TShape : struct, ICompoundShape
        {
            ref var batch = ref batches[shapeBatch.TypeId];
            ref var activeSet = ref bodies.ActiveSet;
            var minValue = new Vector3(float.MaxValue);
            var maxValue = new Vector3(-float.MaxValue);
            for (int i = 0; i < batch.Count; ++i)
            {
                ref var instance = ref batch[i];
                var bodyIndex = instance.Continuation.BodyIndex;
                //We have to clear out the bounds used by compounds, since each contributing body will merge their contribution into the whole.
                broadPhase.GetActiveBoundsPointers(activeSet.Collidables[bodyIndex].BroadPhaseIndex, out var min, out var max);
                *min = minValue;
                *max = maxValue;
                shapeBatch.shapes[instance.ShapeIndex].AddChildBoundsToBatcher(ref this, instance.Pose, instance.Velocities, bodyIndex);
            }
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void Add(TypedIndex shapeIndex, in RigidPose pose, in BodyVelocity velocity, BoundsContinuation continuation)
        {
            var typeIndex = shapeIndex.Type;
            Debug.Assert(typeIndex >= 0 && typeIndex < batches.Length, "The preallocated type batch array should be able to hold every type index. Is the type index broken?");
            ref var batchSlot = ref batches[typeIndex];
            if (!batchSlot.Span.Allocated)
            {
                //No list exists for this type yet.
                batchSlot = new QuickList<BoundingBoxInstance>(CollidablesPerFlush, pool);
                if (typeIndex < minimumBatchIndex)
                    minimumBatchIndex = typeIndex;
                if (typeIndex > maximumBatchIndex)
                    maximumBatchIndex = typeIndex;
            }
            //TODO: Arguably, batching up compounds is silly. 
            //It technically opens the door for vectorizing their child pose calculations, but those are almost certainly not worth vectorizing anyway.
            //May want to consider directly triggering a bounds dispatch for compounds here. 
            //(Doing so WOULD be more complicated, though.)
            ref var instance = ref batchSlot.AllocateUnsafely();
            var shapeBatch = shapes[typeIndex];
            instance.Pose = pose;
            instance.Velocities = velocity;
            instance.ShapeIndex = shapeIndex.Index;
            instance.Continuation = continuation;

            if (batchSlot.Count == CollidablesPerFlush)
            {
                shapeBatch.ComputeBounds(ref this);
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
                if (batch.Span.Allocated)
                {
                    batch.Dispose(pool);
                }
            }
            pool.Return(ref batches);
        }
    }
}
