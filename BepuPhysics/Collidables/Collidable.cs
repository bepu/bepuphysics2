using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace BepuPhysics.Collidables
{
    public enum ContinuousDetectionMode : byte
    {
        //These modes use bit flags to enable specific features.
        //Bit 0: If set, allows velocity expansion beyond speculative margin. If unset, expansion is clamped to the margin.
        //Bit 1: If set, collision tests use an extra inner sphere contact generation test.
        //Bit 2: If set, collision tests use substepping. 
        //Not all combinations are valid. If substepping is enabled, bounding box velocity expansion must be enabled, otherwise the substepping won't do much.
        //Note that it's possible to have another mode- the bounding box expansion clamped to speculative margin, but then potentially expanded by an inner sphere swept by linear velocity.
        //The extra complexity doesn't really seem worth it, considering that the user would need to be aware of the unique corner cases involved. (Some collisions would be missed.)

        /// <summary>
        /// <para>No dedicated continuous detection is performed. Default speculative contact generation will occur within the speculative margin.</para>
        /// <para>The collidable's bounding box will not be expanded by velocity beyond the speculative margin.</para>
        /// <para>This is the cheapest mode, but it may miss collisions. Note that if a Discrete mode collidable is moving quickly, the fact that its bounding box is not expanded
        /// may cause it to miss a collision even with a non-Discrete collidable.</para>
        /// </summary>
        Discrete = 0b000,
        /// <summary>
        /// <para>No dedicated continuous detection is performed. Default speculative contact generation will occur within the speculative margin.</para>
        /// <para>The collidable's bounding box will be expanded by velocity beyond the speculative margin if necessary.</para>
        /// <para>This is useful when a collidable may move quickly and does not itself require continuous detection, but there exist other collidables with continuous modes 
        /// that should avoid missing collisions.</para>
        /// </summary>
        Passive = 0b001,
        /// <summary>
        /// <para>In addition to the default speculative contact generation, a sphere embedded in the shape will be used as an additional speculative contact source when the 
        /// collidable is moving quickly enough relative to a collidable neighbor. The extra contact will only be used if room in the contact manifold is available.</para>
        /// <para>This is a very cheap form of continuous collision detection, and it tends to avoid ghost collisions better than simply increasing the speculative margin.
        /// On the other hand, the extra contact cannot capture angular motion, and it will tend to allow more penetration than substepping or a large speculative margin.</para>
        /// </summary>
        Linear = 0b011,
        /// <summary>
        /// <para>Collision detection will use multiple poses that span the time between frames. For any collidable neighbor, the earliest detected contact manifold is used.</para>
        /// <para>The number of substeps depends on the configured target step size and maximum step count. When moving slowly, substepping may be skipped entirely, and when moving quickly,
        /// many substeps may be used.</para>
        /// <para>This mode can capture angular motion with very few ghost collisions. Carefully choosing a target substep length and a speculative margin together can catch
        /// virtually all primary impacts in a very natural way.</para> 
        /// <para>Because it performs what amounts to multiple collision tests, this mode is more expensive for fast moving objects. Further, because part of its goal is to avoid ghost 
        /// collisions, it can miss secondary collisions that would have occurred due to the primary impact's velocity change.</para>
        /// </summary>
        Substep = 0b101,
        /// <summary>
        /// <para>Uses both Linear and Substep modes together. This is the most expensive collision detection mode, but it has the benefits of both of the continuous modes.</para>
        /// <para>The inner sphere contact generation helps avoid tunneling through secondary collisions that the substepping explicitly filtered out, while substepping captures 
        /// the difficult angular motion. However, the inner sphere contact may reintroduce ghost collisions at extremely high velocities.</para>
        /// </summary>
        LinearAndSubstep = 0b111

        //TODO: Not really happy with these names. "Linear" does an okay job at describing the goal of the mode, but it really doesn't tell you much about what it's doing
        //or what corner cases to expect intuitively. And while something like "InnerSphere" would describe the underlying mechanism well, it doesn't provide much insight at a glance.
        //And "LinearWithSubstep" is just unwieldy. I decided against something like "Full" there because it isn't some universal per-event conservative advancement- it still has
        //definite holes and compromises. Even if it is the most complete of the bunch.
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
        /// If using a substepping mode, this is the maximum number of substeps allowed. Fewer substeps than the maximum may be used for slower motion.
        /// </summary>
        [FieldOffset(1)]
        private byte MaximumSubstepCount;
        /// <summary>
        /// <para>The length of a motion path allowed before continuous collision detection will be used for this collidable.</para>
        /// <para>For modes using substepping, this is the maximum distance any part of the shape can move (due to linear or angular motion) before another substep is introduced.
        /// For modes using an inner sphere, this is the displacement in a single frame necessary to trigger the use of the extra inner sphere contact generator.</para>
        /// </summary>
        [FieldOffset(4)]
        public float MaximumStepLength;
        //Technically, there are situations where you would want to configure the step length for substepping and inner sphere separately. However, those are not terribly common,
        //and we really don't want to bloat the size of this structure- L1 cache isn't big.

        //Substepping CCD and AABB calculations also depend on the maximum radius, but that is handled by the Shape.
        //There isn't a strong reason to include per-collidable information for that.

        //Note that we disallow cases where expansion beyond the margin is off, yet continuous detection is on. So, there's no need for masking.
        internal bool AllowExpansionBeyondSpeculativeMargin { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return (uint)Mode > 0; } }
        internal bool UseInnerSphere { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return ((uint)Mode & 2) > 0; } }
        internal bool UseSubstepping { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return ((uint)Mode & 4) > 0; } }
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
        //These CCD settings are bundled together away from the rest of the collidable data for a few reasons:
        //1) They do a little packing to avoid pointless memory overhead,
        //2) It's possible that we'll want to split them out later if data access patterns suggest that it's a good idea,
        //3) Don't really want to pollute this structure's members with CCD-conditional tuning variables.
        
        //Note that the order of these members is intentional: the narrowphase tends to access Continuity and Shape together when dispatching narrow phase work.
        //Later on, the narrowphase will access the speculative margin when generating contacts. It never accesses the broad phase index.
        //The AABB updater, in contrast, uses all of the properties.
        //So, this order gives both systems contiguous access to their desired properties, increasing the chance that they'll all be in the same cache line.
        //(Splitting this structure would stop the narrow phase from loading the unnecessary broad phase index, but that's a pretty small benefit for redundant memory.)

        /// <summary>
        /// Index of the shape used by the body.
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
        /// Index of the collidable in the broad phase. Used to look up the target location for bounding box scatters.
        /// </summary>
        public int BroadPhaseIndex;

    }
}
