using BepuPhysics.Collidables;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics
{
    /// <summary>
    /// Convenience structure for directly referring to a body's properties.
    /// </summary>
    /// <remarks>Note that this type makes no attempt to protect against unsafe modification of body properties, nor does it try to wake up bodies if they are asleep.</remarks>
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
        /// Gets a reference to the body's location stored in the handle to location mapping.
        /// </summary>
        public ref BodyLocation Location
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return ref Bodies.HandleToLocation[Handle]; }
        }

        /// <summary>
        /// Gets whether the body is in the active set.
        /// </summary>
        public bool IsActive
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return Location.SetIndex == 0; }
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
        public ref QuickList<BodyConstraintReference, Buffer<BodyConstraintReference>> Constraints
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                ref var location = ref Location;
                return ref Bodies.Sets[location.SetIndex].Constraints[location.Index];
            }
        }
        

    }
}
