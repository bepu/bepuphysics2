using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace BepuPhysics.Collidables
{
    public enum ContinuousDetectionMode
    {
        /// <summary>
        /// <para>No dedicated continuous detection is performed. Default speculative contact generation will occur within the speculative margin.</para>
        /// <para>The collidable's bounding box will not be expanded by velocity beyond the speculative margin.</para>
        /// <para>This is the cheapest mode, but it may miss collisions. Note that if a Discrete mode collidable is moving quickly, the fact that its bounding box is not expanded
        /// may cause it to miss a collision even with a non-Discrete collidable.</para>
        /// </summary>
        Discrete = 0,
        /// <summary>
        /// <para>No dedicated continuous detection is performed. Default speculative contact generation will occur within the speculative margin.</para>
        /// <para>The collidable's bounding box will be expanded by velocity beyond the speculative margin if necessary.</para>
        /// <para>This is useful when a collidable may move quickly and does not itself require continuous detection, but there exist other collidables with continuous modes 
        /// that should avoid missing collisions.</para>
        /// </summary>
        Passive = 1,
        /// <summary>
        /// <para>Collision detection will start with a sweep test to identify a likely time of impact. Speculative contacts will be generated for the predicted collision.</para>
        /// <para>This mode can capture angular motion with very few ghost collisions. It can, however, miss secondary collisions that would have occurred due to the primary impact's velocity change.</para>
        /// </summary>
        Continuous = 2,
    }

    [StructLayout(LayoutKind.Explicit)]
    public struct ContinuousDetectionSettings
    {
        /// <summary>
        /// The continuous collision detection mode.
        /// </summary>
        [FieldOffset(0)]
        public ContinuousDetectionMode Mode;
        /// <summary>
        /// If using ContinuousDetectionMode.Continuous, MinimumSweepTimestep is the minimum progress that the sweep test will make when searching for the first time of impact.
        /// Collisions lasting less than MinimumProgress may be missed by the sweep test. Using larger values can significantly increase the performance of sweep tests.
        /// </summary>
        [FieldOffset(4)]
        public float MinimumSweepTimestep;
        /// <summary>
        /// If using ContinuousDetectionMode.Continuous, sweep tests will terminate if the time of impact region has been refined to be smaller than SweepConvergenceThreshold.
        /// Values closer to zero will converge more closely to the true time of impact, but for speculative contact generation larger values usually work fine.
        /// Larger values allow the sweep to terminate much earlier and can significantly improve sweep performance.
        /// </summary>
        [FieldOffset(8)]
        public float SweepConvergenceThreshold;

        internal bool AllowExpansionBeyondSpeculativeMargin { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return (uint)Mode > 0; } }

        /// <summary>
        /// <para>No dedicated continuous detection will be performed. Default speculative contact generation will occur within the speculative margin.</para>
        /// <para>The collidable's bounding box will not be expanded by velocity beyond the speculative margin.</para>
        /// <para>This is the cheapest mode, but it may miss collisions. Note that if a Discrete mode collidable is moving quickly, the fact that its bounding box is not expanded
        /// may cause it to miss a collision even with a non-Discrete collidable.</para>
        /// </summary>
        public static ContinuousDetectionSettings Discrete
        {
            get { return new ContinuousDetectionSettings(); }
        }

        /// <summary>
        /// <para>No dedicated continuous detection is performed. Default speculative contact generation will occur within the speculative margin.</para>
        /// <para>The collidable's bounding box will be expanded by velocity beyond the speculative margin if necessary.</para>
        /// <para>This is useful when a collidable may move quickly and does not itself require continuous detection, but there exist other collidables with continuous modes 
        /// that should avoid missing collisions.</para>
        /// </summary>
        public static ContinuousDetectionSettings Passive
        {
            get { return new ContinuousDetectionSettings() { Mode = ContinuousDetectionMode.Passive }; }
        }

        /// <summary>
        /// <para>Collision detection will start with a sweep test to identify a likely time of impact. Speculative contacts will be generated for the predicted collision.</para>
        /// <para>This mode can capture angular motion with very few ghost collisions. It can, however, miss secondary collisions that would have occurred due to the primary impact's velocity change.</para>
        /// </summary>
        /// <param name="minimumSweepTimestep">Minimum progress that the sweep test will make when searching for the first time of impact.
        /// Collisions lasting less than MinimumProgress may be missed by the sweep test. Using larger values can significantly increase the performance of sweep tests.</param>
        /// <param name="sweepConvergenceThreshold">Threshold against which the time of impact region is compared for sweep termination. 
        /// If the region has been refined to be smaller than SweepConvergenceThreshold, the sweep will terminate.
        /// Values closer to zero will converge more closely to the true time of impact, but for speculative contact generation larger values usually work fine.
        /// Larger values allow the sweep to terminate much earlier and can significantly improve sweep performance.</param>
        /// <returns>Settings reflecting a continuous detection mode.</returns>
        public static ContinuousDetectionSettings Continuous(float minimumSweepTimestep, float sweepConvergenceThreshold)
        {
            return new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Continuous, MinimumSweepTimestep = minimumSweepTimestep, SweepConvergenceThreshold = sweepConvergenceThreshold };
        }
    }

    /// <summary>
    /// Description of a collidable instance living in the broad phase and able to generate collision pairs.
    /// Collidables with a ShapeIndex that points to nothing (a default constructed TypedIndex) do not actually refer to any existing Collidable.
    /// This can be used for a body which needs no collidable representation.
    /// </summary>
    public struct Collidable
    {
        /// <summary>
        /// Continuous collision detection settings for this collidable. Includes the collision detection mode to use and tuning variables associated with those modes.
        /// </summary>
        public ContinuousDetectionSettings Continuity;

        /// <summary>
        /// Index of the shape used by the body. While this can be changed, any transition from shapeless->shapeful or shapeful->shapeless must be reported to the broad phase. 
        /// If you need to perform such a transition, consider using Bodies.ChangeShape or Bodies.ApplyDescription; those functions update the relevant state.
        /// </summary>
        public TypedIndex Shape;
        /// <summary>
        /// Size of the margin around the surface of the shape in which contacts can be generated. These contacts will have negative depth and only contribute if the frame's velocities
        /// would push the shapes of a pair into overlap. This should be positive to avoid jittering. It can also be used as a form of continuous collision detection, but excessively 
        /// high values combined with fast motion may result in visible 'ghost collision' artifacts. 
        /// <para>For continuous collision detection with less chance of ghost collisions, use the dedicated continuous collision detection modes.</para>
        /// </summary>
        public float SpeculativeMargin;
        /// <summary>
        /// Index of the collidable in the broad phase. Used to look up the target location for bounding box scatters. Under normal circumstances, this should not be set externally.
        /// </summary>
        public int BroadPhaseIndex;

    }
}
