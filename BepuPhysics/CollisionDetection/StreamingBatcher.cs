using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics.Collidables;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;

namespace BepuPhysics.CollisionDetection
{
    public interface IContinuations
    {
        //TODO: In the future, continuations will need to be able to take typed collision caches. The PairCache will store cached separating axes for hull-hull acceleration and similar things.
        unsafe void Notify(ContinuationIndex continuationId, ContactManifold* manifold);
    }

    /// <summary>
    /// Defines filters that some collision tasks may call when child tasks need to be spawned.
    /// </summary>
    public unsafe interface ICollisionSubtaskFilters
    {
        /// <summary>
        /// Checks whether further collision testing should be performed for a given subtask.
        /// </summary>
        /// <param name="parent">Parent of the child pair being checked.</param>
        /// <param name="childA">Index of the child belonging to collidable A in the subpair under consideration.</param>
        /// <param name="childB">Index of the child belonging to collidable B in the subpair under consideration.</param>
        /// <returns>True if testing should proceed, false otherwise.</returns>
        bool AllowCollisionTesting(CollidablePair parent, int childA, int childB);
        /// <summary>
        /// Provides control over subtask generated results before they are reported to the parent task.
        /// </summary>
        /// <param name="parent">Parent of the pair being configured.</param>
        /// <param name="childA">Index of the child belonging to collidable A in the subpair under consideration.</param>
        /// <param name="childB">Index of the child belonging to collidable B in the subpair under consideration.</param>
        /// <param name="manifold">Manifold of the child pair to configure.</param>
        void Configure(CollidablePair parent, int childA, int childB, ContactManifold* manifold);
    }

    public abstract class CollisionTask
    {
        /// <summary>
        /// Gets the number of tasks to batch together before executing this task.
        /// </summary>
        public int BatchSize { get; protected set; }
        /// <summary>
        /// Gets the first shape type index associated with the task. Shape pairs provided to the task for execution should be in the order defined by these type two indices.
        /// If a collision task isn't a top level shape pair task, this should be -1.
        /// </summary>
        public int ShapeTypeIndexA { get; protected set; }
        /// <summary>
        /// Gets the second shape type index associated with the task. Shape pairs provided to the task for execution should be in the order defined by these type two indices.
        /// If a collision task isn't a top level shape pair task, this should be -1.
        /// </summary>
        public int ShapeTypeIndexB { get; protected set; }
        /// <summary>
        /// Gets the type ids of the specialized subtasks registered by this task.
        /// </summary>
        public int[] SubtaskIndices { get; protected set; }
        /// <summary>
        /// Gets the set of collision tasks that this task may produce as a part of execution.
        /// </summary>
        public CollisionTask[] Subtasks { get; protected set; }

        //Note that we leave the details of input and output of a task's execution to be undefined.
        //A task can reach into the batcher and create new entries or trigger continuations as required.
        /// <summary>
        /// Executes the task on the given input.
        /// </summary>
        /// <typeparam name="TFilters">Type of the filters used to influence execution of collision tasks.</typeparam>
        /// <typeparam name="TContinuations">Type of the continuations that can be triggered by the this execution.</typeparam>
        /// <param name="batcher">Batcher responsible for the invocation.</param>
        /// <param name="batch">Batch of pairs to test.</param>
        /// <param name="continuations">Continuations to invoke upon completion of a top level pair.</param>
        /// <param name="filters">Filters to use to influence execution of the collision tasks.</param>
        public abstract void ExecuteBatch<TContinuations, TFilters>(ref UntypedList batch, ref StreamingBatcher batcher, ref TContinuations continuations, ref TFilters filters)
            where TContinuations : struct, IContinuations
            where TFilters : struct, ICollisionSubtaskFilters;

    }

    public struct CollisionTaskReference
    {
        public int TaskIndex;
        public int BatchSize;
        public int ExpectedFirstTypeId;
    }

    public class CollisionTaskRegistry
    {
        CollisionTaskReference[][] topLevelMatrix;
        internal CollisionTask[] tasks;
        int count;

        public CollisionTask this[int taskIndex]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return tasks[taskIndex];
            }
        }

        public CollisionTaskRegistry(int initialShapeCount = 9)
        {
            ResizeMatrix(initialShapeCount);
        }

        void ResizeMatrix(int newSize)
        {
            var oldSize = topLevelMatrix != null ? topLevelMatrix.Length : 0;
            Array.Resize(ref topLevelMatrix, newSize);
            for (int i = 0; i < newSize; ++i)
            {
                Array.Resize(ref topLevelMatrix[i], newSize);
                for (int j = oldSize; j < newSize; ++j)
                {
                    topLevelMatrix[i][j] = new CollisionTaskReference { TaskIndex = -1 };
                }
            }
        }

        void InsertTask(CollisionTask task, int index)
        {
            //This allocates a lot of garbage due to frequently resizing, but it does not matter- task registration a one time thing at program initialization.
            //Having tight bounds is more useful for performance in the end (by virtue of having a marginally simpler heap).
            int newCount = count + 1;
            if (tasks == null || newCount > tasks.Length)
                Array.Resize(ref tasks, newCount);
            if (index < count)
            {
                Array.Copy(tasks, index, tasks, index + 1, count - index);
                //Every task in the array that got moved needs to have its top level index bumped. 
                for (int i = 0; i < topLevelMatrix.Length; ++i)
                {
                    for (int j = 0; j < topLevelMatrix.Length; ++j)
                    {
                        if (topLevelMatrix[i][j].TaskIndex >= index)
                        {
                            ++topLevelMatrix[i][j].TaskIndex;
                        }
                    }
                }
                for (int i = index + 1; i < newCount; ++i)
                {
                    var t = tasks[i];
                    if (t.SubtaskIndices != null)
                    {
                        for (int j = 0; j < t.SubtaskIndices.Length; ++j)
                        {
                            ref var subtaskIndex = ref t.SubtaskIndices[j];
                            if (subtaskIndex >= index)
                            {
                                ++subtaskIndex;
                            }
                        }
                    }
                }
            }

            tasks[index] = task;
            count = newCount;
        }


        public int Register(CollisionTask task)
        {
            var index = count;
            //This task may have some dependencies that are already present. In order for the batcher's flush to work with a single pass,
            //the tasks must be stored in dependency order- any task that can create more subwork has to appear earlier in the list than the subwork's task.
            //Where is the earliest one?
            if (task.Subtasks != null)
            {
                for (int i = 0; i < task.Subtasks.Length; ++i)
                {
                    var subtaskIndex = Array.IndexOf(tasks, task.Subtasks[i], 0, count);
                    if (subtaskIndex >= 0 && subtaskIndex < index)
                        index = subtaskIndex;
                }
            }

            InsertTask(task, index);
            var a = task.ShapeTypeIndexA;
            var b = task.ShapeTypeIndexB;
            var highestShapeIndex = a > b ? a : b;
            if (highestShapeIndex >= 0)
            {
                //This only handles top level pairs (those associated with shape type pairs).
                //Some tasks are not directly associated with a top level entrypoint- instead, they're follow ups on top level tasks. Since they're not an entry point,
                //there is no need for them to appear in the top level matrix.
                if (highestShapeIndex >= topLevelMatrix.Length)
                    ResizeMatrix(highestShapeIndex + 1);
                var taskInfo = new CollisionTaskReference { TaskIndex = index, BatchSize = task.BatchSize, ExpectedFirstTypeId = task.ShapeTypeIndexA };
                topLevelMatrix[a][b] = taskInfo;
                topLevelMatrix[b][a] = taskInfo;
            }

#if DEBUG
            //Ensure that no task dependency cycles exist.
            if (task.Subtasks != null)
            {
                for (int i = 0; i < task.Subtasks.Length; ++i)
                {
                    for (int j = i + 1; j < task.Subtasks.Length; ++j)
                    {
                        Debug.Assert(Array.IndexOf(tasks[j].Subtasks, tasks[i]) == -1,
                            "Tasks must be stored in a strict order of work generation- if a task generates work for another task, the receiving task must appear later in the list. " +
                            "No cycles can exist.");
                    }
                }
            }
#endif
            //Register any unregistered subtasks.
            if (task.Subtasks != null)
            {
                for (int i = 0; i < task.Subtasks.Length; ++i)
                {
                    var subtaskIndex = Array.IndexOf(tasks, task.Subtasks[i], 0, count);
                    if (subtaskIndex < 0)
                    {
                        subtaskIndex = Register(task);
                    }
                    task.SubtaskIndices[i] = subtaskIndex;
                }
            }
            Debug.Assert(tasks[index] == task, "No subtask registrations should move the original task; that would imply a cycle in the dependency graph.");
            return index;

        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref CollisionTaskReference GetTaskReference(int topLevelTypeA, int topLevelTypeB)
        {
            return ref topLevelMatrix[topLevelTypeA][topLevelTypeB];
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref CollisionTaskReference GetTaskReference<TShapeA, TShapeB>()
            where TShapeA : struct, IShape
            where TShapeB : struct, IShape
        {
            return ref GetTaskReference(default(TShapeA).TypeId, default(TShapeB).TypeId);
        }
    }

    public struct TestPair
    {
        /// <summary>
        /// Stores whether the types involved in pair require that the resulting contact manifold be flipped to be consistent with the user-requested pair order.
        /// </summary>
        public int FlipMask;
        public ContinuationIndex Continuation;
        public RigidPose PoseA;
        public RigidPose PoseB;
    }

    //Writes by the narrowphase write shape data without type knowledge, so they can't easily operate on regular packing rules. Emulate this with a pack of 1.
    //This allows the reader to still have a quick way to interpret data rather than casting individual shapes.
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct TestPair<TShapeA, TShapeB>
            where TShapeA : struct, IShape where TShapeB : struct, IShape
    {
        public TShapeA A;
        public TShapeB B;
        public TestPair Shared;
    }


    public struct StreamingBatcher
    {
        //The streaming batcher contains batches for pending work submitted by the user.
        //This pending work can be top level pairs like sphere versus sphere, but it may also be subtasks of submitted work.
        //Consider two compound bodies colliding. The pair will decompose into a set of potentially many convex subpairs.
        //Similarly, a hull-hull collision test could spawn many subtasks, but those subtasks may not be of the same type as any top level pair.

        CollisionTaskRegistry typeMatrix;
        internal BufferPool pool;


        int minimumBatchIndex, maximumBatchIndex;
        Buffer<UntypedList> batches;
        //A subset of collision tasks require a place to return information.
        Buffer<UntypedList> localContinuations;


        public unsafe StreamingBatcher(BufferPool pool, CollisionTaskRegistry collisionTypeMatrix)
        {
            this.pool = pool;
            typeMatrix = collisionTypeMatrix;
            pool.SpecializeFor<UntypedList>().Take(collisionTypeMatrix.tasks.Length, out batches);
            pool.SpecializeFor<UntypedList>().Take(collisionTypeMatrix.tasks.Length, out localContinuations);
            //Clearing is required ensure that we know when a batch needs to be created and when a batch needs to be disposed.
            batches.Clear(0, collisionTypeMatrix.tasks.Length);
            localContinuations.Clear(0, collisionTypeMatrix.tasks.Length);
            minimumBatchIndex = collisionTypeMatrix.tasks.Length;
            maximumBatchIndex = -1;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void Add<TContinuations, TFilters>(ref CollisionTaskReference reference,
            int shapeSizeA, int shapeSizeB, void* shapeA, void* shapeB, ref RigidPose poseA, ref RigidPose poseB,
            int flipMask, ContinuationIndex continuationId, ref TContinuations continuations, ref TFilters filters)
            where TContinuations : struct, IContinuations
            where TFilters : struct, ICollisionSubtaskFilters
        {
            ref var batch = ref batches[reference.TaskIndex];
            var pairData = batch.AllocateUnsafely();
            Unsafe.CopyBlockUnaligned(pairData, shapeA, (uint)shapeSizeA);
            Unsafe.CopyBlockUnaligned(pairData += shapeSizeA, shapeB, (uint)shapeSizeB);
            var poses = (TestPair*)(pairData += shapeSizeB);
            Debug.Assert(continuationId.Exists);
            poses->FlipMask = flipMask;
            poses->Continuation = continuationId;
            poses->PoseA = poseA;
            poses->PoseB = poseB;
            if (batch.Count == reference.BatchSize)
            {
                typeMatrix[reference.TaskIndex].ExecuteBatch(ref batch, ref this, ref continuations, ref filters);
                batch.Count = 0;
                batch.ByteCount = 0;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Add<TContinuations, TFilters>(
            int shapeTypeIdA, int shapeTypeIdB, int shapeSizeA, int shapeSizeB, void* shapeA, void* shapeB, ref RigidPose poseA, ref RigidPose poseB,
            ContinuationIndex continuationId, ref TContinuations continuations, ref TFilters filters)
            where TContinuations : struct, IContinuations
            where TFilters : struct, ICollisionSubtaskFilters
        {
            ref var reference = ref typeMatrix.GetTaskReference(shapeTypeIdA, shapeTypeIdB);
            if (reference.TaskIndex < 0)
            {
                //There is no task for this shape type pair. Immediately respond with an empty manifold.
                var manifold = new ContactManifold();
                continuations.Notify(continuationId, &manifold);
                return;
            }
            ref var batch = ref batches[reference.TaskIndex];
            var pairSize = shapeSizeA + shapeSizeB + Unsafe.SizeOf<TestPair>();
            if (!batch.Buffer.Allocated)
            {
                batch = new UntypedList(pairSize, reference.BatchSize, pool);
                if (minimumBatchIndex > reference.TaskIndex)
                    minimumBatchIndex = reference.TaskIndex;
                if (maximumBatchIndex < reference.TaskIndex)
                    maximumBatchIndex = reference.TaskIndex;
            }
            Debug.Assert(batch.Buffer.Allocated && batch.ElementSizeInBytes > 0 && batch.ElementSizeInBytes < 131072, "How'd the batch get corrupted?");
            if (shapeTypeIdA != reference.ExpectedFirstTypeId)
            {
                //The inputs need to be reordered to guarantee that the collision tasks are handed data in the proper order.
                Add(ref reference, shapeSizeB, shapeSizeA, shapeB, shapeA, ref poseB, ref poseA, -1, continuationId, ref continuations, ref filters);
            }
            else
            {
                Add(ref reference, shapeSizeA, shapeSizeB, shapeA, shapeB, ref poseA, ref poseB, 0, continuationId, ref continuations, ref filters);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void Add<TShapeA, TShapeB, TContinuations, TFilters>(ref CollisionTaskReference reference,
            ref TShapeA shapeA, ref TShapeB shapeB, ref RigidPose poseA, ref RigidPose poseB,
            int flipMask, ContinuationIndex continuationId, ref TContinuations continuations, ref TFilters filters)
            where TShapeA : struct, IShape where TShapeB : struct, IShape
            where TContinuations : struct, IContinuations
            where TFilters : struct, ICollisionSubtaskFilters
        {
            ref var batch = ref batches[reference.TaskIndex];
            ref var pairData = ref batch.AllocateUnsafely<TestPair<TShapeA, TShapeB>>();
            pairData.A = shapeA;
            pairData.B = shapeB;
            Debug.Assert(continuationId.Exists);
            pairData.Shared.FlipMask = flipMask;
            pairData.Shared.Continuation = continuationId;
            pairData.Shared.PoseA = poseA;
            pairData.Shared.PoseB = poseB;
            if (batch.Count == reference.BatchSize)
            {
                typeMatrix[reference.TaskIndex].ExecuteBatch(ref batch, ref this, ref continuations, ref filters);
                batch.Count = 0;
                batch.ByteCount = 0;
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void Add<TShapeA, TShapeB, TContinuations, TFilters>(ref TShapeA shapeA, ref TShapeB shapeB, ref RigidPose poseA, ref RigidPose poseB,
            ContinuationIndex continuationId, ref TContinuations continuations, ref TFilters filters)
            where TShapeA : struct, IShape where TShapeB : struct, IShape
            where TContinuations : struct, IContinuations
            where TFilters : struct, ICollisionSubtaskFilters
        {
            ref var reference = ref typeMatrix.GetTaskReference<TShapeA, TShapeB>();
            if (reference.TaskIndex < 0)
            {
                //There is no task for this shape type pair. Immediately respond with an empty manifold.
                var manifold = new ContactManifold();
                continuations.Notify(continuationId, &manifold);
                return;
            }
            ref var batch = ref batches[reference.TaskIndex];
            if (!batch.Buffer.Allocated)
            {
                batch = new UntypedList(Unsafe.SizeOf<TestPair<TShapeA, TShapeB>>(), reference.BatchSize, pool);
                if (minimumBatchIndex > reference.TaskIndex)
                    minimumBatchIndex = reference.TaskIndex;
                if (maximumBatchIndex < reference.TaskIndex)
                    maximumBatchIndex = reference.TaskIndex;
            }
            //The type comparison should be a compilation constant.
            if (typeof(TShapeA) != typeof(TShapeB) && default(TShapeA).TypeId != reference.ExpectedFirstTypeId)
            {
                //The inputs need to be reordered to guarantee that the collision tasks are handed data in the proper order.
                Add(ref reference, ref shapeB, ref shapeA, ref poseB, ref poseA, -1, continuationId, ref continuations, ref filters);
            }
            else
            {
                Add(ref reference, ref shapeA, ref shapeB, ref poseA, ref poseB, 0, continuationId, ref continuations, ref filters);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Flush<TContinuations, TFilters>(ref TContinuations continuations, ref TFilters filters)
            where TContinuations : struct, IContinuations
            where TFilters : struct, ICollisionSubtaskFilters
        {
            //The collision task registry guarantees that tasks which create work for other tasks always appear sooner in the task array than their child tasks.
            //Since there are no cycles, only one flush pass is required.
            for (int i = minimumBatchIndex; i <= maximumBatchIndex; ++i)
            {
                ref var batch = ref batches[i];
                if (batch.Count > 0)
                {
                    typeMatrix.tasks[i].ExecuteBatch(ref batch, ref this, ref continuations, ref filters);
                }
                //Dispose of the batch and any associated buffers; since the flush is one pass, we won't be needing this again.
                if (batch.Buffer.Allocated)
                {
                    pool.Return(ref batch.Buffer);
                }
                //Note that the local continuations are not guaranteed to be allocated when the batch is; many tasks don't have any associated continuations.
                if (localContinuations[i].Buffer.Allocated)
                {
                    pool.Return(ref localContinuations[i].Buffer);
                }
            }
            var listPool = pool.SpecializeFor<UntypedList>();
            listPool.Return(ref batches);
            listPool.Return(ref localContinuations);
        }
    }
}
