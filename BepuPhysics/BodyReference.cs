using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics
{
    /// <summary>
    /// Convenience structure for directly referring to a body's properties.
    /// </summary>
    /// <remarks>Note that this type makes no attempt to protect against unsafe modification of body properties, nor does modifying its properties try to wake up bodies if they are asleep.</remarks>
    public struct BodyReference
    {
        /// <summary>
        /// Handle of the body that this reference refers to.
        /// </summary>
        public BodyHandle Handle;
        /// <summary>
        /// The bodies collection containing the body.
        /// </summary>
        public Bodies Bodies;

        /// <summary>
        /// Constructs a new body reference.
        /// </summary>
        /// <param name="handle">Handle of the body to refer to.</param>
        /// <param name="bodies">Collection containing the body.</param>
        /// <remarks>This is equivalent to <see cref="Bodies.GetBodyReference"/> and <see cref="Bodies.this[BodyHandle]"/>.</remarks>
        public BodyReference(BodyHandle handle, Bodies bodies)
        {
            Handle = handle;
            Bodies = bodies;
        }

        /// <summary>
        /// Gets whether the body reference exists within the body set. True if the handle maps to a valid memory location that agrees that the handle points to it, false otherwise.
        /// </summary>
        public bool Exists
        {
            get
            {
                if (Bodies == null)
                    return false;
                return Bodies.BodyExists(Handle);
            }
        }


        /// <summary>
        /// Gets a reference to the body's memory location stored in the handle to location mapping.
        /// </summary>
        public ref BodyMemoryLocation MemoryLocation
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                Bodies.ValidateExistingHandle(Handle);
                return ref Bodies.HandleToLocation[Handle.Value];
            }
        }

        /// <summary>
        /// Gets or sets whether the body is in the active set. Setting this to true will attempt to wake the body; setting it to false will force the body and any constraint-connected bodies asleep.
        /// </summary>
        public bool Awake
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return MemoryLocation.SetIndex == 0; }
            set
            {
                if (Awake)
                {
                    if (!value)
                    {
                        Bodies.sleeper.Sleep(MemoryLocation.Index);
                    }
                }
                else
                {
                    if (value)
                    {
                        Bodies.awakener.AwakenBody(Handle);
                    }
                }
            }
        }

        /// <summary>
        /// Gets a reference to the body's velocity.
        /// </summary>
        public ref BodyVelocity Velocity
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                ref var location = ref MemoryLocation;
                return ref Bodies.Sets[location.SetIndex].SolverStates[location.Index].Motion.Velocity;
            }
        }

        /// <summary>
        /// Gets a reference to the body's pose.
        /// </summary>
        public ref RigidPose Pose
        {
            [MethodImpl(MethodImplOptions.NoInlining)]
            get
            {
                ref var location = ref MemoryLocation;
                return ref Bodies.Sets[location.SetIndex].SolverStates[location.Index].Motion.Pose;
            }
        }

        /// <summary>
        /// Gets a reference to the body's motion state, including both pose and velocity.
        /// </summary>
        public ref MotionState MotionState
        {
            [MethodImpl(MethodImplOptions.NoInlining)]
            get
            {
                ref var location = ref MemoryLocation;
                return ref Bodies.Sets[location.SetIndex].SolverStates[location.Index].Motion;
            }
        }

        /// <summary>
        /// Gets a reference to the body's solver-relevant state, including both pose, velocity, and inertia.
        /// </summary>
        public ref SolverState SolverState
        {
            [MethodImpl(MethodImplOptions.NoInlining)]
            get
            {
                ref var location = ref MemoryLocation;
                return ref Bodies.Sets[location.SetIndex].SolverStates[location.Index];
            }
        }

        /// <summary>
        /// Gets a reference to the body's collidable.
        /// </summary>
        public ref Collidable Collidable
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                ref var location = ref MemoryLocation;
                return ref Bodies.Sets[location.SetIndex].Collidables[location.Index];
            }
        }

        /// <summary>
        /// Gets a reference to the body's local inertia.
        /// </summary>
        public ref BodyInertia LocalInertia
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                ref var location = ref MemoryLocation;
                return ref Bodies.Sets[location.SetIndex].SolverStates[location.Index].Inertia.Local;
            }
        }

        /// <summary>
        /// Gets a reference to the body's activity state.
        /// </summary>
        public ref BodyActivity Activity
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                ref var location = ref MemoryLocation;
                return ref Bodies.Sets[location.SetIndex].Activity[location.Index];
            }
        }

        /// <summary>
        /// Gets a reference to the list of the body's connected constraints.
        /// </summary>
        public ref QuickList<BodyConstraintReference> Constraints
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                ref var location = ref MemoryLocation;
                return ref Bodies.Sets[location.SetIndex].Constraints[location.Index];
            }
        }

        /// <summary>
        /// <para>Gets a CollidableReference for this body. CollidableReferences uniquely identify a collidable object in a simulation by including both the dynamic/kinematic/static state of the object and its handle.</para>
        /// <para>Despite an unfortunate naming collision, CollidableReferences are distinct from a direct reference to a body's collidable data, which you can get from the Collidable property.</para>
        /// </summary>
        public CollidableReference CollidableReference
        {
            get
            {
                return new CollidableReference(Kinematic ? CollidableMobility.Kinematic : CollidableMobility.Dynamic, Handle);
            }
        }

        /// <summary>
        /// Gets whether the body is kinematic, meaning its inverse inertia and mass are all zero.
        /// </summary>
        public unsafe bool Kinematic { get { return Bodies.IsKinematicUnsafeGCHole(ref LocalInertia); } }

        /// <summary>
        /// Gets whether the body has locked inertia, meaning its inverse inertia tensor is zero.
        /// </summary>
        public unsafe bool HasLockedInertia { get { return Bodies.HasLockedInertia((Symmetric3x3*)Unsafe.AsPointer(ref LocalInertia.InverseInertiaTensor)); } }

        /// <summary>
        /// If the body is dynamic, turns the body kinematic by setting all inverse inertia and mass values to zero and activates it.
        /// Any constraints connected to the body that now only contain kinematic references are removed.
        /// If the body is kinematic, does nothing.
        /// </summary>
        public void BecomeKinematic()
        {
            if (!Kinematic)
            {
                Bodies.SetLocalInertia(Handle, default);
            }
        }

        /// <summary>
        /// Sets the body's local inertia to the provided inertia. Wakes up the body and correctly handles any transition between dynamic and kinematic states.
        /// If the body moves from dynamic to kinematic, any constraints connected to the body that now only contain kinematic references are removed.
        /// </summary>
        public void SetLocalInertia(in BodyInertia localInertia)
        {
            Bodies.SetLocalInertia(Handle, localInertia);
        }

        /// <summary>
        /// Computes the world space inverse inertia tensor for the body based on the LocalInertia and Pose.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeInverseInertia(out Symmetric3x3 inverseInertia)
        {
            ref var location = ref MemoryLocation;
            ref var set = ref Bodies.Sets[MemoryLocation.SetIndex];
            //Note that inertia.World is ephemeral data packed into the same cache line for the benefit of the solver.
            //It should not be assumed to contain up to date information outside of the velocity integration to pose integration interval, so this computes world inertia from scratch.
            ref var state = ref set.SolverStates[location.Index];
            PoseIntegration.RotateInverseInertia(state.Inertia.Local.InverseInertiaTensor, state.Motion.Pose.Orientation, out inverseInertia);
        }

        /// <summary>
        /// Gets a description of the body.
        /// </summary>
        /// <param name="description">Description of the body.</param>
        public void GetDescription(out BodyDescription description)
        {
            Bodies.GetDescription(Handle, out description);
        }

        /// <summary>
        /// Sets a body's properties according to a description. Properly handles any transitions between dynamic and kinematic and between shapeless and shapeful.
        /// If the body is becoming kinematic, any constraints which only contain kinematic bodies will be removed. Wakes up the body and updates its bounds in the broad phase.
        /// </summary>
        /// <param name="description">Description of the body.</param>
        public void ApplyDescription(in BodyDescription description)
        {
            Bodies.ApplyDescription(Handle, description);
        }

        /// <summary>
        /// Changes the shape of a body. Properly handles the transition between shapeless and shapeful. If the body is inactive, it will be forced awake. Updates the bounds of the body in the broad phase.
        /// </summary>
        /// <param name="newShape">Index of the new shape to use for the body.</param>
        public void SetShape(TypedIndex newShape)
        {
            Bodies.SetShape(Handle, newShape);
        }

        /// <summary>
        /// Gets a copy of the body's bounding box. If the body has no shape, the bounding box has a min at float.MaxValue and a max at float.MinValue.
        /// </summary>
        public unsafe BoundingBox BoundingBox
        {
            get
            {
                BoundingBox box;
                if (GetBoundsReferencesFromBroadPhase(out var min, out var max))
                {
                    box.Min = *min;
                    box.Max = *max;
                }
                else
                {
                    box.Min = new Vector3(float.MaxValue);
                    box.Max = new Vector3(float.MinValue);
                }
                return box;
            }
        }

        /// <summary>
        /// Gets direct pointers to the body's bounding box minimum and maximum in the broad phase. Outputs null if the body has no shape.
        /// </summary>
        /// <param name="min">Pointer to the bounding box minimum in the broad phase. Null if the body has no shape.</param>
        /// <param name="max">Pointer to the bounding box maximum in the broad phase. Null if the body has no shape.</param>
        /// <returns>True if the body has a shape and bounds, false otherwise.</returns>
        public unsafe bool GetBoundsReferencesFromBroadPhase(out Vector3* min, out Vector3* max)
        {
            ref var location = ref MemoryLocation;
            ref var collidable = ref Bodies.Sets[location.SetIndex].Collidables[location.Index];
            if (collidable.Shape.Exists)
            {
                if (location.SetIndex == 0)
                {
                    Bodies.broadPhase.GetActiveBoundsPointers(collidable.BroadPhaseIndex, out min, out max);
                }
                else
                {
                    Bodies.broadPhase.GetStaticBoundsPointers(collidable.BroadPhaseIndex, out min, out max);
                }
                return true;
            }
            else
            {
                //There is no shape, so there can be no bounds.
                min = null;
                max = null;
                return false;
            }
        }

        /// <summary>
        /// Updates the body's bounds in the broad phase for its current state. Does not include velocity expansion. Does nothing if the body has no shape.
        /// </summary>
        /// <remarks>Can be useful if you made modifications to the body's state that you want reflected in the broad phase before the next timestep.
        /// For example, if you want to perform ray casts against the broad phase after moving objects around directly, their bounds must be updated or else the broad phase bounds will be out of date and the ray will likely miss.</remarks>
        public void UpdateBounds()
        {
            Bodies.UpdateBounds(Handle);
        }

        /// <summary>
        /// Applies an impulse to a body by index. Does not wake the body up.
        /// </summary>
        /// <param name="pose">Pose of the body to apply impulse to.</param>
        /// <param name="velocity">Velocity of the body to apply impulse to.</param>
        /// <param name="localInertia">Local inertia of the body to apply impulse to.</param>
        /// <param name="impulse">Impulse to apply to the body.</param>
        /// <param name="impulseOffset">World space offset from the center of the body to apply the impulse at.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(in Vector3 impulse, in Vector3 impulseOffset, ref BodyInertia localInertia, ref RigidPose pose, ref BodyVelocity velocity)
        {
            PoseIntegration.RotateInverseInertia(localInertia.InverseInertiaTensor, pose.Orientation, out var inverseInertiaTensor);
            ApplyLinearImpulse(impulse, localInertia.InverseMass, ref velocity.Linear);
            ApplyAngularImpulse(Vector3.Cross(impulseOffset, impulse), inverseInertiaTensor, ref velocity.Angular);
        }

        /// <summary>
        /// Applies an impulse to a body by index. Does not wake the body up.
        /// </summary>
        /// <param name="set">Body set containing the body to apply an impulse to.</param>
        /// <param name="index">Index of the body in the body set.</param>
        /// <param name="impulse">Impulse to apply to the body.</param>
        /// <param name="impulseOffset">World space offset from the center of the body to apply the impulse at.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(in BodySet set, int index, in Vector3 impulse, in Vector3 impulseOffset)
        {
            ref var state = ref set.SolverStates[index];
            ApplyImpulse(impulse, impulseOffset, ref state.Inertia.Local, ref state.Motion.Pose, ref state.Motion.Velocity);
        }

        /// <summary>
        /// Applies an angular impulse to an angular velocity. Does not wake the body up.
        /// </summary>
        /// <param name="angularImpulse">Impulse to apply to the velocity.</param>
        /// <param name="inverseInertiaTensor">Inverse inertia tensor to transform the impulse with.</param>
        /// <param name="angularVelocity">Angular velocity to be modified.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyAngularImpulse(in Vector3 angularImpulse, in Symmetric3x3 inverseInertiaTensor, ref Vector3 angularVelocity)
        {
            Symmetric3x3.TransformWithoutOverlap(angularImpulse, inverseInertiaTensor, out var angularVelocityChange);
            angularVelocity += angularVelocityChange;
        }

        /// <summary>
        /// Applies an impulse to a linear velocity. Does not wake the body up.
        /// </summary>
        /// <param name="impulse">Impulse to apply to the velocity.</param>
        /// <param name="inverseMass">Inverse mass to transform the impulse with.</param>
        /// <param name="linearVelocity">Linear velocity to be modified.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyLinearImpulse(in Vector3 impulse, float inverseMass, ref Vector3 linearVelocity)
        {
            linearVelocity += impulse * inverseMass;
        }


        /// <summary>
        /// Applies an impulse to a body at the given world space position. Does not modify activity states.
        /// </summary>
        /// <param name="impulse">Impulse to apply to the body.</param>
        /// <param name="impulseOffset">World space offset to apply the impulse at.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyImpulse(in Vector3 impulse, in Vector3 impulseOffset)
        {
            ref var location = ref MemoryLocation;
            ApplyImpulse(Bodies.Sets[location.SetIndex], location.Index, impulse, impulseOffset);
        }

        /// <summary>
        /// Applies an impulse to a linear velocity. Does not wake the body up.
        /// </summary>
        /// <param name="impulse">Impulse to apply to the velocity.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyLinearImpulse(in Vector3 impulse)
        {
            ref var location = ref MemoryLocation;
            ref var set = ref Bodies.Sets[location.SetIndex];
            ref var state = ref set.SolverStates[location.Index];
            ApplyLinearImpulse(impulse, state.Inertia.Local.InverseMass, ref state.Motion.Velocity.Linear);
        }

        /// <summary>
        /// Computes the velocity of an offset point attached to the body.
        /// </summary>
        /// <param name="offset">Offset from the body's center to </param>
        /// <param name="velocity">Effective velocity of the point if it were attached to the body.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetVelocityForOffset(in Vector3 offset, out Vector3 velocity)
        {
            velocity = Velocity.Linear + Vector3.Cross(Velocity.Angular, offset);
        }

        /// <summary>
        /// Applies an angular impulse to an angular velocity. Does not wake the body up.
        /// </summary>
        /// <param name="angularImpulse">Impulse to apply to the velocity.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyAngularImpulse(in Vector3 angularImpulse)
        {
            ref var location = ref MemoryLocation;
            ref var set = ref Bodies.Sets[location.SetIndex];
            ref var state = ref set.SolverStates[location.Index];
            //Note that inertia.World is ephemeral data packed into the same cache line for the benefit of the solver.
            //It should not be assumed to contain up to date information outside of the velocity integration to pose integration interval, so this computes world inertia from scratch.
            PoseIntegration.RotateInverseInertia(state.Inertia.Local.InverseInertiaTensor, state.Motion.Pose.Orientation, out var inverseInertia);
            ApplyAngularImpulse(angularImpulse, inverseInertia, ref state.Motion.Velocity.Angular);
        }

        /// <summary>
        /// Implicitly converts a <see cref="BodyReference"/> to the <see cref="BodyHandle"/> that the body reference was created from.
        /// </summary>
        /// <param name="reference">Body reference to extract the handle from.</param>
        public static implicit operator BodyHandle(BodyReference reference)
        {
            return reference.Handle;
        }
    }
}
