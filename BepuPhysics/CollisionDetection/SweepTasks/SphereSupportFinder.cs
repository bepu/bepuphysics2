using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.CollisionDetection.SweepTasks
{
    public struct SphereSupportFinder : ISupportFinder<Sphere, SphereWide>
    {
        public bool HasMargin
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return true; }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetMargin(in SphereWide shape, out Vector<float> margin)
        {
            margin = shape.Radius;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeSupport(in SphereWide shape, in Matrix3x3Wide orientation, in Vector3Wide direction, out Vector3Wide support)
        {
            support = default;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeLocalSupport(in SphereWide shape, in Vector3Wide direction, out Vector3Wide support)
        {
            support = default;
        }
    }
}
