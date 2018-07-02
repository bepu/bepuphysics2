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
        Buffer<QuickList<BoundingBoxInstance, Buffer<BoundingBoxInstance>>> batches;

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
            pool.SpecializeFor<QuickList<BoundingBoxInstance, Buffer<BoundingBoxInstance>>>().Take(shapes.RegisteredTypeSpan, out batches);
            //Clearing is required ensure that we know when a batch needs to be created and when a batch needs to be disposed.
            batches.Clear(0, shapes.RegisteredTypeSpan);
            minimumBatchIndex = shapes.RegisteredTypeSpan;
            maximumBatchIndex = -1;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetBoundsExpansion(ref Vector3Wide linearVelocity, ref Vector3Wide angularVelocity, float dt,
            ref Vector<float> maximumRadius, ref Vector<float> maximumAngularExpansion, out Vector3Wide minExpansion, out Vector3Wide maxExpansion)
        {
            /*
            If an object sitting on a plane had a raw (unexpanded) AABB that is just barely above the plane, no contacts would be generated. 
            If the velocity of the object would shove it down into the plane in the next frame, then it would generate contacts in the next frame- and, 
            potentially, this cycle would repeat and cause jitter.

            To solve this, there are a couple of options:
            1) Introduce an 'allowed penetration' so that objects can overlap a little bit. This tends to confuse people a little bit when they notice it, 
            and in some circumstances objects can be seen settling into the allowed penetration slowly. It looks a bit odd.
            2) Make contact constraints fight to maintain zero penetration depth, but expand the bounding box with velocity and allow contacts to be generated speculatively- 
            contacts with negative penetration depth.

            #2 is a form of continuous collision detection, but it's handy for general contact stability too. 
            In this version of the engine, all objects generate speculative contacts by default, though only within a per-collidable-tuned 'speculative margin'. 
            It's kind of like BEPUphysics v1's AllowedPenetration, except inverted. Speculative contacts that fall within the speculative margin- 
            that is, those with negative depth of a magnitude less than the margin- are kept.

            So, a user could choose to have a very large speculative margin, and the speculative contact generation would provide a form of continuous collision detection. 
            The main purpose, though, is just contact stability. With this in isolation, there's no strong reason to expand the bounding box more than the speculative margin. 
            This is the 'discrete' mode.

            However, consider what would happen if an object A with high velocity and this 'discrete' mode was headed towards an object B in a 'continuous' mode.
            Object B only expands its bounding box by its own velocity, and object A doesn't expand beyond its speculative margin. The collision between A and B could easily be missed. 
            To account for this, there is an intermediate mode- 'passive'- where the bounding box is allowed to expand beyond the margin, 
            but no further continuous collision detection is performed.

            The fully continuous modes fully expand the bounding boxes. Notably, the inner sphere continuous collision detection mode could get by with less, 
            but it would be pretty confusing to have the same kind of missed collision possibility if the other object in the pair was a substepping object.
            Two different inner sphere modes could be offered, but I'm unsure about the usefulness versus the complexity.

            (Note that there ARE situations where a bounding box which contains the full unconstrained motion will fail to capture constrained motion. 
            Consider object A flying at high speed to impact the stationary object B, which sits next to another stationary object C. 
            Object B's bounding box doesn't overlap with object C's bounding box- they're both stationary, so there's no velocity expansion. But during one frame, 
            object A slams into B, and object B's velocity during that frame now forces it to tunnel all the way through C unimpeded, because no contacts were generated between B and C. 
            There are ways to address this- all of which are a bit expensive- but CCD as implemented is not a hard guarantee. 
            It's a 'best effort' that compromises with performance. Later on, if it's really necessary, we could consider harder guarantees with higher costs, but... 
            given that no one seemed to have much of an issue with v1's rather limited CCD, it'll probably be fine.)

            So, how is the velocity expansion calculated?
            There's two parts, linear and angular.

            Linear is pretty simple- expand the bounding box in the direction of linear displacement (linearVelocity * dt).
            */

            Vector<float> vectorDt = new Vector<float>(dt);
            Vector3Wide.Scale(linearVelocity, vectorDt, out var linearDisplacement);

            var zero = Vector<float>.Zero;
            Vector3Wide.Min(zero, linearDisplacement, out minExpansion);
            Vector3Wide.Max(zero, linearDisplacement, out maxExpansion);

            /*
            Angular requires a bit more care. Since the goal is to create a tight bound, simply using a v = w * r approximation isn't ideal. A slightly tighter can be found:
            1) The maximum displacement along ANY axis during an intermediate time is equal to the distance from a starting position at MaximumRadius 
            to the position of that point at the intermediate time.
            2) The expansion cannot exceed the maximum radius, so angular deltas greater than pi/3 do not need to be considered. 
            (An expansion equal to the maximum radius would result in an equilateral triangle, which has an angle of 60 degrees in each corner.) 
            Larger values can simply be clamped.
            3) The largest displacement along any axis, at any time, is the distance from the starting position to the position at dt. Note that this only holds because of the clamp: 
            if the angle was allowed to wrap around, it the distance would start to go down again.
            4) position(time) = {radius * sin(angular speed * time), radius * cos(angular speed * time)}
            5) largest expansion required = ||position(dt) - position(0)|| = sqrt(2 * radius^2 * (1 - cos(dt * w)))
            6) Don't have any true SIMD sin function, but we can approximate it using a taylor series, like: cos(x) = 1 - x^2 / 2! + x^4 / 4! - x^6 / 6!
            7) Note that the cosine approximation should stop at a degree where it is smaller than the true value of cosine for the interval 0 to pi/3: this guarantees that the distance,
            which is larger when the cosine is smaller, is conservative and fully bounds the angular motion.

            Why do this extra work?
            1) The bounding box calculation phase, as a part of the pose integration phase, tends to be severely memory bound. 
            Spending a little of ALU time to get a smaller bounding box isn't a big concern, even though it includes a couple of sqrts.
            An extra few dozen ALU cycles is unlikely to meaningfully change the execution time.
            2) Shrinking the bounding box reduces the number of collision pairs. Collision pairs are expensive- many times more expensive than the cost of shrinking the bounding box.
            */
            Vector3Wide.Length(angularVelocity, out var angularVelocityMagnitude);
            var a = Vector.Min(angularVelocityMagnitude * vectorDt, new Vector<float>(MathHelper.Pi / 3f));
            var a2 = a * a;
            var a4 = a2 * a2;
            var a6 = a4 * a2;
            var cosAngleMinusOne = a2 * new Vector<float>(-1f / 2f) + a4 * new Vector<float>(1f / 24f) - a6 * new Vector<float>(1f / 720f);
            //Note that it's impossible for angular motion to cause an increase in bounding box size beyond (maximumRadius-minimumRadius) on any given axis.
            //That value, or a conservative approximation, is stored as the maximum angular expansion.
            var angularExpansion = Vector.Min(maximumAngularExpansion,
                Vector.SquareRoot(new Vector<float>(-2f) * maximumRadius * maximumRadius * cosAngleMinusOne));
            Vector3Wide.Subtract(minExpansion, angularExpansion, out minExpansion);
            Vector3Wide.Add(maxExpansion, angularExpansion, out maxExpansion);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ExpandBoundingBoxes(ref Vector3Wide min, ref Vector3Wide max, ref BodyVelocities velocities, float dt,
            ref Vector<float> maximumRadius, ref Vector<float> maximumAngularExpansion, ref Vector<float> maximumExpansion)
        {
            GetBoundsExpansion(ref velocities.Linear, ref velocities.Angular, dt, ref maximumRadius, ref maximumAngularExpansion, out var minDisplacement, out var maxDisplacement);
            //The maximum expansion passed into this function is the speculative margin for discrete mode collidables, and ~infinity for passive or continuous ones.
            Vector3Wide.Max(-maximumExpansion, minDisplacement, out minDisplacement);
            Vector3Wide.Min(maximumExpansion, maxDisplacement, out maxDisplacement);

            Vector3Wide.Add(min, minDisplacement, out min);
            Vector3Wide.Add(max, maxDisplacement, out max);
        }


        //This is simply a internally vectorized version of the above.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetAngularBoundsExpansion(in Vector3 angularVelocity, float dt,
            float maximumRadius, float maximumAngularExpansion, out Vector3 expansion)
        {
            var angularVelocityMagnitude = angularVelocity.Length();
            var a = MathHelper.Min(angularVelocityMagnitude * dt, MathHelper.Pi / 3f);
            var a2 = a * a;
            var a4 = a2 * a2;
            var a6 = a4 * a2;
            var cosAngleMinusOne = a2 * (-1f / 2f) + a4 * (1f / 24f) - a6 * (1f / 720f);
            //Note that it's impossible for angular motion to cause an increase in bounding box size beyond (maximumRadius-minimumRadius) on any given axis.
            //That value, or a conservative approximation, is stored as the maximum angular expansion.
            expansion = new Vector3(MathHelper.Min(maximumAngularExpansion,
                (float)Math.Sqrt(-2f * maximumRadius * maximumRadius * cosAngleMinusOne)));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ExpandBoundingBox(ref Vector3 min, ref Vector3 max, in Vector3 linearVelocity, in Vector3 angularVelocity, float dt,
        float maximumRadius, float maximumAngularExpansion, float maximumAllowedExpansion)
        {
            var linearDisplacement = linearVelocity * dt;
            Vector3 zero = default;
            var minExpansion = Vector3.Min(zero, linearDisplacement);
            var maxExpansion = Vector3.Max(zero, linearDisplacement);
            GetAngularBoundsExpansion(angularVelocity, dt, maximumRadius, maximumAngularExpansion, out var angularExpansion);

            var maximumAllowedExpansionBroadcasted = new Vector3(maximumAllowedExpansion);
            minExpansion = Vector3.Max(-maximumAllowedExpansionBroadcasted, minExpansion - angularExpansion);
            maxExpansion = Vector3.Min(maximumAllowedExpansionBroadcasted, maxExpansion + angularExpansion);
            min += minExpansion;
            max += maxExpansion;
        }

        public unsafe void ExecuteConvexBatch<TShape, TShapeWide>(ConvexShapeBatch<TShape, TShapeWide> shapeBatch) where TShape : struct, IConvexShape where TShapeWide : struct, IShapeWide<TShape>
        {
            var instanceBundle = default(BoundingBoxInstanceWide<TShape, TShapeWide>);
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
                    targetInstanceSlot.Shape.WriteFirst(ref shapeBatch.shapes[instance.ShapeIndex]);
                    Vector3Wide.WriteFirst(instance.Pose.Position, ref targetInstanceSlot.Pose.Position);
                    QuaternionWide.WriteFirst(instance.Pose.Orientation, ref targetInstanceSlot.Pose.Orientation);
                    Vector3Wide.WriteFirst(instance.Velocities.Linear, ref targetInstanceSlot.Velocities.Linear);
                    Vector3Wide.WriteFirst(instance.Velocities.Angular, ref targetInstanceSlot.Velocities.Angular);
                    ref var collidable = ref activeSet.Collidables[instance.Continuation.BodyIndex];
                    GatherScatter.GetFirst(ref targetInstanceSlot.MaximumExpansion) =
                        collidable.Continuity.AllowExpansionBeyondSpeculativeMargin ? float.MaxValue : collidable.SpeculativeMargin;
                }
                instanceBundle.Shape.GetBounds(ref instanceBundle.Pose.Orientation, out var maximumRadius, out var maximumAngularExpansion, out var bundleMin, out var bundleMax);
                ExpandBoundingBoxes(ref bundleMin, ref bundleMax, ref instanceBundle.Velocities, dt,
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
                        BoundingBox.CreateMerged(ref *minPointer, ref *maxPointer, ref min, ref max, out *minPointer, out *maxPointer);
                    }
                    else
                    {
                        *minPointer = new Vector3(sourceBundleMin.X[0], sourceBundleMin.Y[0], sourceBundleMin.Z[0]);
                        *maxPointer = new Vector3(sourceBundleMax.X[0], sourceBundleMax.Y[0], sourceBundleMax.Z[0]);
                    }
                }
            }
        }

        public unsafe void ExecuteMeshBatch<TShape>(MeshShapeBatch<TShape> shapeBatch) where TShape : struct, IMeshShape
        {
            ref var batch = ref batches[shapeBatch.TypeId];
            ref var activeSet = ref bodies.ActiveSet;
            var minValue = new Vector3(float.MaxValue);
            var maxValue = new Vector3(-float.MaxValue);
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
                ExpandBoundingBox(ref *min, ref *max, instance.Velocities.Linear, instance.Velocities.Angular, dt, maximumRadius, maximumRadius - minimumRadius, maximumAllowedExpansion);
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
                shapeBatch.shapes[instance.ShapeIndex].AddChildBoundsToBatcher(ref this, ref instance.Pose, ref instance.Velocities, bodyIndex);
            }
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void Add(TypedIndex shapeIndex, ref RigidPose pose, ref BodyVelocity velocity, BoundsContinuation continuation)
        {
            var typeIndex = shapeIndex.Type;
            Debug.Assert(typeIndex >= 0 && typeIndex < batches.Length, "The preallocated type batch array should be able to hold every type index. Is the type index broken?");
            ref var batchSlot = ref batches[typeIndex];
            if (!batchSlot.Span.Allocated)
            {
                //No list exists for this type yet.
                QuickList<BoundingBoxInstance, Buffer<BoundingBoxInstance>>.Create(pool.SpecializeFor<BoundingBoxInstance>(), CollidablesPerFlush, out batchSlot);
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

        public void Add(int bodyIndex)
        {
            //For convenience, this function handles the case where the collidable reference points to nothing.
            //Note that this touches the memory associated with the full collidable. That's okay- we'll be reading the rest of it shortly if it has a collidable.
            ref var activeSet = ref bodies.ActiveSet;
            ref var collidable = ref activeSet.Collidables[bodyIndex];
            //Technically, you could make a second pass that only processes collidables, rather than iterating over all bodies and doing last second branches.
            //But then you'd be evicting everything from cache L1/L2. And, 99.99% of the time, bodies are going to have shapes, so this isn't going to be a difficult branch to predict.
            //Even if it was 50%, the cache benefit of executing alongside the just-touched data source would outweigh the misprediction.
            if (collidable.Shape.Exists)
            {
                Add(collidable.Shape, ref activeSet.Poses[bodyIndex], ref activeSet.Velocities[bodyIndex], BoundsContinuation.CreateContinuation(bodyIndex));
            }
        }
        public void AddCompoundChild(int bodyIndex, TypedIndex shapeIndex, ref RigidPose pose, ref BodyVelocity velocity)
        {
            Add(shapeIndex, ref pose, ref velocity, BoundsContinuation.CreateCompoundChildContinuation(bodyIndex));
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
            var instancePool = pool.SpecializeFor<BoundingBoxInstance>();
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
                    batch.Dispose(instancePool);
                }
            }
            var listPool = pool.SpecializeFor<QuickList<BoundingBoxInstance, Buffer<BoundingBoxInstance>>>();
            listPool.Return(ref batches);
        }
    }
}
