using BepuPhysics.Collidables;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection
{
    public interface ICollisionCallbacks
    {
        //TODO: In the future, continuations will need to be able to take typed collision caches. The PairCache will store cached separating axes for hull-hull acceleration and similar things.
        unsafe void OnPairCompleted<TManifold>(int pairId, ref TManifold manifold) where TManifold : unmanaged, IContactManifold<TManifold>;

        /// <summary>
        /// Provides control over subtask generated results before they are reported to the parent task.
        /// </summary>
        /// <param name="pairId">Id of the parent pair that spawned this child pair.</param>
        /// <param name="childA">Index of the child belonging to collidable A in the subpair under consideration.</param>
        /// <param name="childB">Index of the child belonging to collidable B in the subpair under consideration.</param>
        /// <param name="manifold">Manifold of the child pair to configure.</param>
        unsafe void OnChildPairCompleted(int pairId, int childA, int childB, ref ConvexContactManifold manifold);

        /// <summary>
        /// Checks whether further collision testing should be performed for a given subtask.
        /// </summary>
        /// <param name="pairId">Id of the parent pair.</param>
        /// <param name="childA">Index of the child belonging to collidable A in the subpair under consideration.</param>
        /// <param name="childB">Index of the child belonging to collidable B in the subpair under consideration.</param>
        /// <returns>True if testing should proceed, false otherwise.</returns>
        bool AllowCollisionTesting(int pairId, int childA, int childB);
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
        /// Gets whether the task is capable of generating subtasks. Note that subtask generators cannot generate subtasks that are themselves subtask generators.
        /// </summary>
        public bool SubtaskGenerator { get; protected set; }
        /// <summary>
        /// Gets the pair type that the ExecuteBatch call requires.
        /// </summary>
        public CollisionTaskPairType PairType { get; protected set; }

        //Note that we leave the details of input and output of a task's execution to be undefined.
        //A task can reach into the batcher and create new entries or trigger continuations as required.
        /// <summary>
        /// Executes the task on the given input.
        /// </summary>
        /// <typeparam name="TCallbacks">Type of the callbacks used to handle results of collision tasks.</typeparam>
        /// <param name="batcher">Batcher responsible for the invocation.</param>
        /// <param name="batch">Batch of pairs to test.</param>
        /// <param name="continuations">Continuations to invoke upon completion of a top level pair.</param>
        /// <param name="filters">Filters to use to influence execution of the collision tasks.</param>
        public abstract void ExecuteBatch<TCallbacks>(ref UntypedList batch, ref CollisionBatcher<TCallbacks> batcher)
            where TCallbacks : struct, ICollisionCallbacks;

    }

    public enum CollisionTaskPairType
    {
        /// <summary>
        /// General pair for two shapes with full pose and flip mask, but no bounds related data.
        /// </summary>
        StandardPair,
        /// <summary>
        /// Pair specialized for convex pairs between two shapes of the same type.
        /// </summary>
        FliplessPair,
        /// <summary>
        /// Pair specialized for two spheres, requiring no flip mask or orientations.
        /// </summary>
        SpherePair,
        /// <summary>
        /// Pair specialized for convex pairs that involve one sphere which requires no orientation.
        /// </summary>
        SphereIncludingPair,
        /// <summary>
        /// Pair that requires computing local bounding boxes, and so requires extra information like velocity.
        /// </summary>
        BoundsTestedPair

    }

    public struct CollisionTaskReference
    {
        public int TaskIndex;
        public int BatchSize;
        public int ExpectedFirstTypeId;
        public CollisionTaskPairType PairType;
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

        public int Register(CollisionTask task)
        {
            //Some tasks can generate tasks. Note that this can only be one level deep; nesting compounds is not allowed.
            //All such generators will be placed at the beginning.
            var index = task.SubtaskGenerator ? 0 : count;

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
            }

            tasks[index] = task;
            count = newCount;

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
                var taskInfo = new CollisionTaskReference
                {
                    TaskIndex = index,
                    BatchSize = task.BatchSize,
                    ExpectedFirstTypeId = task.ShapeTypeIndexA,
                    PairType = task.PairType
                };
                topLevelMatrix[a][b] = taskInfo;
                topLevelMatrix[b][a] = taskInfo;
            }

#if DEBUG
            //Ensure that no task dependency cycles exist.
            bool encounteredNongenerator = false;
            for (int i = 0; i < count; ++i)
            {
                if (encounteredNongenerator)
                {
                    Debug.Assert(!tasks[i].SubtaskGenerator,
                        "To avoid cycles, the tasks list should be partitioned into two contiguous groups: subtask generators, followed by non-subtask generators.");
                }
                else
                {
                    if (!tasks[i].SubtaskGenerator)
                        encounteredNongenerator = true;
                }
            }
#endif           
            return index;

        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref CollisionTaskReference GetTaskReference(int topLevelTypeA, int topLevelTypeB)
        {
            return ref topLevelMatrix[topLevelTypeA][topLevelTypeB];
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref CollisionTaskReference GetTaskReference<TShapeA, TShapeB>()
            where TShapeA : unmanaged, IShape
            where TShapeB : unmanaged, IShape
        {
            return ref GetTaskReference(default(TShapeA).TypeId, default(TShapeB).TypeId);
        }
    }
}
