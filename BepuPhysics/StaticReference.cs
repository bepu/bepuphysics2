using BepuPhysics.Collidables;
using BepuUtilities;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics
{
    /// <summary>
    /// Convenience structure for directly referring to a static's properties.
    /// </summary>
    /// <remarks>Note that this type makes no attempt to protect against unsafe modification of static properties.</remarks>
    public struct StaticReference
    {
        /// <summary>
        /// Handle of the static that this reference refers to.
        /// </summary>
        public StaticHandle Handle;
        /// <summary>
        /// The collection containing the static.
        /// </summary>
        public Statics Statics;

        /// <summary>
        /// Constructs a new static reference.
        /// </summary>
        /// <param name="handle">Handle of the static to refer to.</param>
        /// <param name="statics">Collection containing the static.</param>
        public StaticReference(StaticHandle handle, Statics statics)
        {
            Handle = handle;
            Statics = statics;
        }

        /// <summary>
        /// Gets whether the static reference exists within the static set. True if the handle maps to a valid memory location that agrees that the handle points to it, false otherwise.
        /// </summary>
        public bool Exists
        {
            get
            {
                if (Statics == null || Handle.Value < 0 || Handle.Value >= Statics.HandleToIndex.Length)
                    return false;
                return Statics.IndexToHandle[Statics.HandleToIndex[Handle.Value]].Value == Handle.Value;
            }
        }


        /// <summary>
        /// Gets a the static's index in the statics collection.
        /// </summary>
        public int Index => Statics.HandleToIndex[Handle.Value];

        /// <summary>
        /// Gets a reference to the entirety of the static's memory.
        /// </summary>
        public ref Static Static => ref Statics.GetDirectReference(Handle);

        /// <summary>
        /// Gets a reference to the static's pose.
        /// </summary>
        public ref RigidPose Pose => ref Statics.GetDirectReference(Handle).Pose;

        /// <summary>
        /// Gets a reference to the static's collision continuity settings.
        /// </summary>
        public ref ContinuousDetection Continuity => ref Statics.GetDirectReference(Handle).Continuity;

        /// <summary>
        /// Gets the shape used by the static. To set the shape, use <see cref="SetShape(TypedIndex)"/> or <see cref="ApplyDescription(in StaticDescription)"/>.
        /// </summary>
        public TypedIndex Shape => Statics.GetDirectReference(Handle).Shape;

        /// <summary>
        /// <para>Gets a CollidableReference for this static. CollidableReferences uniquely identify a collidable object in a simulation by including both the dynamic/kinematic/static state of the object and its handle.</para>
        /// <para>Despite an unfortunate naming collision, CollidableReferences are distinct from a direct reference to a static's collidable data, which you can get from the Collidable property.</para>
        /// </summary>
        public CollidableReference CollidableReference
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return new CollidableReference(Handle);
            }
        }

        /// <summary>
        /// Gets a description of the static.
        /// </summary>
        /// <param name="description">Description of the static.</param>
        public void GetDescription(out StaticDescription description)
        {
            Statics.GetDescription(Handle, out description);
        }

        /// <summary>
        /// Sets a static's properties according to a description.
        /// </summary>
        /// <param name="description">Description of the static.</param>
        public void ApplyDescription(in StaticDescription description)
        {
            Statics.ApplyDescription(Handle, description);
        }

        /// <summary>
        /// Changes the shape of a static.
        /// </summary>
        /// <param name="newShape">Index of the new shape to use for the static.</param>
        public void SetShape(TypedIndex newShape)
        {
            Statics.SetShape(Handle, newShape);
        }

        /// <summary>
        /// Gets a copy of the static's bounding box.
        /// </summary>
        public unsafe BoundingBox BoundingBox
        {
            get
            {
                BoundingBox box;
                GetBoundsReferencesFromBroadPhase(out var min, out var max);
                box.Min = *min;
                box.Max = *max;
                return box;
            }
        }

        /// <summary>
        /// Gets direct pointers to the static's bounding box minimum and maximum in the broad phase. Outputs null if the static has no shape.
        /// </summary>
        /// <param name="min">Pointer to the bounding box minimum in the broad phase.</param>
        /// <param name="max">Pointer to the bounding box maximum in the broad phase.</param>
        public unsafe void GetBoundsReferencesFromBroadPhase(out Vector3* min, out Vector3* max)
        {
            var index = Index;
            ref var collidable = ref Statics[index];
            Debug.Assert(collidable.Shape.Exists, "Statics must have a shape. Something's not right here.");
            Statics.broadPhase.GetStaticBoundsPointers(collidable.BroadPhaseIndex, out min, out max);
        }

        /// <summary>
        /// Updates the static's bounds in the broad phase for its current state. Does not include velocity expansion. Does nothing if the static has no shape.
        /// </summary>
        /// <remarks>Can be useful if you made modifications to the static's state that you want reflected in the broad phase before the next timestep.
        /// For example, if you want to perform ray casts against the broad phase after moving objects around directly, their bounds must be updated or else the broad phase bounds will be out of date and the ray will likely miss.</remarks>
        public void UpdateBounds()
        {
            Statics.UpdateBounds(Handle);
        }
    }
}
