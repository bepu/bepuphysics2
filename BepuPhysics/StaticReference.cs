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
    /// Convenience structure for directly referring to a static's properties.
    /// </summary>
    /// <remarks>Note that this type makes no attempt to protect against unsafe modification of static properties.</remarks>
    public struct StaticReference
    {
        /// <summary>
        /// Handle of the body that this reference refers to.
        /// </summary>
        public int Handle;
        /// <summary>
        /// The bodies collection containing the body.
        /// </summary>
        public Statics Statics;

        /// <summary>
        /// Constructs a new static reference.
        /// </summary>
        /// <param name="handle">Handle of the static to refer to.</param>
        /// <param name="statics">Collection containing the static.</param>
        public StaticReference(int handle, Statics statics)
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
                if (Statics == null || Handle < 0 || Handle >= Statics.HandleToIndex.Length)
                    return false;
                return Statics.IndexToHandle[Statics.HandleToIndex[Handle]] == Handle;
            }
        }


        /// <summary>
        /// Gets a the static's index in the statics collection.
        /// </summary>
        public int Index
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return Statics.HandleToIndex[Handle]; }
        }
        
        /// <summary>
        /// Gets a reference to the static's pose.
        /// </summary>
        public ref RigidPose Pose
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return ref Statics.Poses[Statics.HandleToIndex[Handle]];
            }
        }

        /// <summary>
        /// Gets a reference to the static's collidable.
        /// </summary>
        public ref Collidable Collidable
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return ref Statics.Collidables[Statics.HandleToIndex[Handle]];
            }
        }
        
    }
}
