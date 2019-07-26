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
        public int Handle;
        /// <summary>
        /// The bodies collection containing the body.
        /// </summary>
        public Bodies Bodies;

        /// <summary>
        /// Constructs a new body reference.
        /// </summary>
        /// <param name="handle">Handle of the body to refer to.</param>
        /// <param name="bodies">Collection containing the body.</param>
        public BodyReference(int handle, Bodies bodies)
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
        /// Gets a reference to the body's location stored in the handle to location mapping.
        /// </summary>
        public ref BodyLocation Location
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                Bodies.ValidateExistingHandle(Handle);
                return ref Bodies.HandleToLocation[Handle];
            }
        }

        /// <summary>
        /// Gets or sets whether the body is in the active set. Setting this to true will attempt to wake the body; setting it to false will force the body and any constraint-connected bodies asleep.
        /// </summary>
        public bool Awake
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return Location.SetIndex == 0; }
            set
            {
                if (Awake)
                {
                    if (!value)
                    {
                        Bodies.sleeper.Sleep(Location.Index);
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
                ref var location = ref Location;
                return ref Bodies.Sets[location.SetIndex].Velocities[location.Index];
            }
        }

        /// <summary>
        /// Gets a reference to the body's pose.
        /// </summary>
        public ref RigidPose Pose
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                ref var location = ref Location;
                return ref Bodies.Sets[location.SetIndex].Poses[location.Index];
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
                ref var location = ref Location;
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
                ref var location = ref Location;
                return ref Bodies.Sets[location.SetIndex].LocalInertias[location.Index];
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
                ref var location = ref Location;
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
                ref var location = ref Location;
                return ref Bodies.Sets[location.SetIndex].Constraints[location.Index];
            }
        }

        /// <summary>
        /// Gets whether the body is kinematic, meaning its inverse inertia and mass are all zero.
        /// </summary>
        public bool Kinematic { get { return Bodies.IsKinematic(LocalInertia); } }

        /// <summary>
        /// Gets whether the body has locked inertia, meaning its inverse inertia tensor is zero.
        /// </summary>
        public bool HasLockedInertia { get { return Bodies.HasLockedInertia(LocalInertia.InverseInertiaTensor); } }

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
            ref var location = ref Location;
            ref var set = ref Bodies.Sets[Location.SetIndex];
            ref var localInertia = ref set.LocalInertias[location.Index];
            ref var pose = ref set.Poses[location.Index];
            PoseIntegration.RotateInverseInertia(localInertia.InverseInertiaTensor, pose.Orientation, out inverseInertia);
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
        /// If the body is becoming kinematic, any constraints which only contain kinematic bodies will be removed. Wakes up the body.
        /// </summary>
        /// <param name="description">Description of the body.</param>
        public void ApplyDescription(in BodyDescription description)
        {
            Bodies.ApplyDescription(Handle, description);
        }

        /// <summary>
        /// Changes the shape of a body. Properly handles the transition between shapeless and shapeful. If the body is inactive, it will be forced awake.
        /// </summary>
        /// <param name="newShape">Index of the new shape to use for the body.</param>
        public void SetShape(TypedIndex newShape)
        {
            Bodies.SetShape(Handle, newShape);
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
            ref var localInertia = ref set.LocalInertias[index];
            ref var pose = ref set.Poses[index];
            ref var velocity = ref set.Velocities[index];
            ApplyImpulse(impulse, impulseOffset, ref localInertia, ref pose, ref velocity);
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
            ref var location = ref Location;
            ApplyImpulse(Bodies.Sets[location.SetIndex], location.Index, impulse, impulseOffset);
        }

        /// <summary>
        /// Applies an impulse to a linear velocity. Does not wake the body up.
        /// </summary>
        /// <param name="impulse">Impulse to apply to the velocity.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyLinearImpulse(in Vector3 impulse)
        {
            ref var location = ref Location;
            ref var set = ref Bodies.Sets[location.SetIndex];
            ApplyLinearImpulse(impulse, set.LocalInertias[location.Index].InverseMass, ref set.Velocities[location.Index].Linear);
        }

        /// <summary>
        /// Applies an angular impulse to an angular velocity. Does not wake the body up.
        /// </summary>
        /// <param name="angularImpulse">Impulse to apply to the velocity.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ApplyAngularImpulse(in Vector3 angularImpulse)
        {
            ref var location = ref Location;
            ref var set = ref Bodies.Sets[location.SetIndex];
            ref var localInertia = ref set.LocalInertias[location.Index];
            ref var pose = ref set.Poses[location.Index];
            PoseIntegration.RotateInverseInertia(localInertia.InverseInertiaTensor, pose.Orientation, out var inverseInertia);
            ApplyAngularImpulse(angularImpulse, inverseInertia, ref set.Velocities[location.Index].Angular);
        }
    }
}
