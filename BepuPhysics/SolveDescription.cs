using BepuUtilities.Memory;
using System;

namespace BepuPhysics
{
    /// <summary>
    /// Callback executed to determine how many velocity iterations should be used for a given substep.
    /// </summary>
    /// <param name="substepIndex">Index of the substep to schedule velocity iterations for.</param>
    /// <returns>Number of velocity iterations to use for the substep. If nonpositive, <see cref="SolveDescription.VelocityIterationCount"/> will be used for the substep instead.</returns>
    public delegate int SubstepVelocityIterationScheduler(int substepIndex);

    /// <summary>
    /// Describes how the solver should schedule substeps and velocity iterations.
    /// </summary>
    public struct SolveDescription
    {
        /// <summary>
        /// Number of velocity iterations to use in the solver if there is no <see cref="VelocityIterationScheduler"/> or if it returns a non-positive value for a substep.
        /// </summary>
        public int VelocityIterationCount;
        /// <summary>
        /// Number of substeps to execute each time the solver runs.
        /// </summary>
        public int SubstepCount;
        /// <summary>
        /// Number of synchronzed constraint batches to use before using a fallback approach.
        /// </summary>
        public int FallbackBatchThreshold;
        /// <summary>
        /// Callback executed to determine how many velocity iterations should be used for a given substep. If null, or if it returns a non-positive value, the <see cref="VelocityIterationCount"/> will be used instead.
        /// </summary>
        public SubstepVelocityIterationScheduler VelocityIterationScheduler;

        /// <summary>
        /// Default number of synchronized constraint batches to use before falling back to an alternative solving method.
        /// </summary>
        public const int DefaultFallbackBatchThreshold = 64;

        internal void ValidateDescription()
        {
            if (SubstepCount < 1)
                throw new ArgumentException("Substep count must be positive.");
            if (VelocityIterationCount < 1)
                throw new ArgumentException("Velocity iteration count must be positive.");
            if (FallbackBatchThreshold < 1)
                throw new ArgumentException("Fallback batch threshold must be positive.");
        }
        /// <summary>
        /// Creates a solve description.
        /// </summary>
        /// <param name="velocityIterationCount">Number of velocity iterations per substep.</param>
        /// <param name="substepCount">Number of substeps in the solve.</param>
        /// <param name="fallbackBatchThreshold">Number of synchronzed constraint batches to use before using a fallback approach.</param>
        public SolveDescription(int velocityIterationCount, int substepCount, int fallbackBatchThreshold = DefaultFallbackBatchThreshold)
        {
            SubstepCount = substepCount;
            VelocityIterationCount = velocityIterationCount;
            FallbackBatchThreshold = fallbackBatchThreshold;
            VelocityIterationScheduler = null;
            ValidateDescription();
        }

        /// <summary>
        /// Creates a solve description.
        /// </summary>
        /// <param name="substepCount">Number of substeps in the solve.</param>
        /// <param name="velocityIterationScheduler"></param>
        /// <param name="fallbackVelocityIterationCount">Number of velocity iterations per substep for any substep that is not given a positive number of velocity iterations by the scheduler.</param>
        /// <param name="fallbackBatchThreshold">Number of synchronzed constraint batches to use before using a fallback approach.</param>
        public SolveDescription(int substepCount, SubstepVelocityIterationScheduler velocityIterationScheduler, int fallbackVelocityIterationCount = 1, int fallbackBatchThreshold = DefaultFallbackBatchThreshold)
        {
            SubstepCount = substepCount;
            VelocityIterationCount = fallbackVelocityIterationCount;
            FallbackBatchThreshold = fallbackBatchThreshold;
            VelocityIterationScheduler = velocityIterationScheduler;
            ValidateDescription();
        }

        /// <summary>
        /// Creates a solve description.
        /// </summary>
        /// <param name="substepVelocityIterations">Number of velocity iterations to use in each substep. Number of substeps will be determined by the length of the span.</param>
        /// <param name="fallbackVelocityIterationCount">Number of velocity iterations per substep for any substep that is not given a positive number of velocity iterations by the scheduler.</param>
        /// <param name="fallbackBatchThreshold">Number of synchronzed constraint batches to use before using a fallback approach.</param>
        public SolveDescription(ReadOnlySpan<int> substepVelocityIterations, int fallbackVelocityIterationCount = 1, int fallbackBatchThreshold = DefaultFallbackBatchThreshold)
        {
            SubstepCount = substepVelocityIterations.Length;
            VelocityIterationCount = fallbackVelocityIterationCount;
            FallbackBatchThreshold = fallbackBatchThreshold;
            var copy = substepVelocityIterations.ToArray();
            VelocityIterationScheduler = substepIndex => copy[substepIndex];
            ValidateDescription();
        }

        /// <summary>
        /// Creates a solve description with the given number of velocity iterations and a single substep, with a fallback threshold of <see cref="DefaultFallbackBatchThreshold"/>.
        /// </summary>
        /// <param name="velocityIterationCount">Number of velocity iterations per substep.</param>
        public static implicit operator SolveDescription(int velocityIterationCount)
        {
            return new SolveDescription(velocityIterationCount, 1);
        }
        /// <summary>
        /// Creates a solve description with the given number of substeps and velocity iterations per substep and a fallback threshold of <see cref="DefaultFallbackBatchThreshold"/>.
        /// </summary>
        /// <param name="schedule">Number of substeps and iterations per solve.</param>
        public static implicit operator SolveDescription((int iterationsPerSubstep, int substepCount) schedule)
        {
            return new SolveDescription(schedule.substepCount, schedule.iterationsPerSubstep);
        }
        /// <summary>
        /// Creates a solve description with the given number of substeps and velocity iterations per substep and a fallback threshold of <see cref="DefaultFallbackBatchThreshold"/>.
        /// </summary>
        /// <param name="substepVelocityIterations">Number of velocity iterations to use in each substep. Number of substeps will be determined by the length of the span.</param>
        public static implicit operator SolveDescription(ReadOnlySpan<int> substepVelocityIterations)
        {
            return new SolveDescription(substepVelocityIterations);
        }
        /// <summary>
        /// Creates a solve description with the given number of substeps and velocity iterations per substep and a fallback threshold of <see cref="DefaultFallbackBatchThreshold"/>.
        /// </summary>
        /// <param name="substepVelocityIterations">Number of velocity iterations to use in each substep. Number of substeps will be determined by the length of the span.</param>
        public static implicit operator SolveDescription(int[] substepVelocityIterations)
        {
            return new SolveDescription(substepVelocityIterations);
        }
        /// <summary>
        /// Creates a solve description with the given number of substeps and velocity iterations per substep and a fallback threshold of <see cref="DefaultFallbackBatchThreshold"/>.
        /// </summary>
        /// <param name="substepVelocityIterations">Number of velocity iterations to use in each substep. Number of substeps will be determined by the length of the span.</param>
        public static implicit operator SolveDescription(Buffer<int> substepVelocityIterations)
        {
            return new SolveDescription(substepVelocityIterations);
        }
    }
}
