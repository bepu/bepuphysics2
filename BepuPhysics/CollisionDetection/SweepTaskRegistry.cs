using BepuPhysics.Collidables;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using Quaternion = BepuUtilities.Quaternion;

namespace BepuPhysics.CollisionDetection
{
    public interface ISweepFilter
    {
        /// <summary>
        /// Checks whether a swept test should be performed for children of swept shapes.
        /// </summary>
        /// <param name="childA">Index of the child belonging to collidable A.</param>
        /// <param name="childB">Index of the child belonging to collidable B.
        /// <returns>True if testing should proceed, false otherwise.</returns>
        bool AllowTest(int childA, int childB);
    }


    public abstract class SweepTask
    {
        /// <summary>
        /// Gets the first shape type index associated with the task.
        /// </summary>
        public int ShapeTypeIndexA { get; protected set; }
        /// <summary>
        /// Gets the second shape type index associated with the task.
        /// </summary>
        public int ShapeTypeIndexB { get; protected set; }

        public abstract unsafe bool Sweep<TSweepFilter>(
            void* shapeDataA, int shapeTypeA, in Quaternion orientationA, in BodyVelocity velocityA,
            void* shapeDataB, int shapeTypeB, in Vector3 offsetB, in Quaternion orientationB, in BodyVelocity velocityB, float maximumT,
            float minimumProgression, float convergenceThreshold, int maximumIterationCount,
            ref TSweepFilter filter, out float t0, out float t1, out Vector3 hitLocation, out Vector3 hitNormal) where TSweepFilter : ISweepFilter;

    }

    public class SweepTaskRegistry
    {
        int[][] topLevelMatrix;
        internal SweepTask[] tasks;
        int count;

        public SweepTask this[int taskIndex]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return tasks[taskIndex];
            }
        }

        public SweepTaskRegistry(int initialShapeCount = 9)
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
                    topLevelMatrix[i][j] = -1;
                }
            }
        }

        public int Register(SweepTask task)
        {
            //Some tasks can generate tasks. Note that this can only be one level deep; nesting compounds is not allowed.
            //All such generators will be placed at the beginning.
            var index = count;

            //This allocates a lot of garbage due to frequently resizing, but it does not matter- task registration a one time thing at program initialization.
            //Having tight bounds is more useful for performance in the end (by virtue of having a marginally simpler heap).
            int newCount = count + 1;
            if (tasks == null || newCount > tasks.Length)
                Array.Resize(ref tasks, newCount);
            tasks[index] = task;
            count = newCount;

            var a = task.ShapeTypeIndexA;
            var b = task.ShapeTypeIndexB;
            var highestShapeIndex = a > b ? a : b;
            if (highestShapeIndex >= topLevelMatrix.Length)
                ResizeMatrix(highestShapeIndex + 1);
            topLevelMatrix[a][b] = index;
            topLevelMatrix[b][a] = index;

            return index;

        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public SweepTask GetTask(int topLevelTypeA, int topLevelTypeB)
        {
            return tasks[topLevelMatrix[topLevelTypeA][topLevelTypeB]];
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public SweepTask GetTask<TShapeA, TShapeB>()
            where TShapeA : struct, IShape
            where TShapeB : struct, IShape
        {
            return GetTask(default(TShapeA).TypeId, default(TShapeB).TypeId);
        }
    }
}
