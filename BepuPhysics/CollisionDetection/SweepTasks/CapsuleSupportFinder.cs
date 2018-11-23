using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.CollisionDetection.SweepTasks
{
    public struct CapsuleSupportFinder : ISupportFinder<Capsule, CapsuleWide>
    {
        public bool HasMargin
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return true; }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetMargin(ref CapsuleWide shape, out Vector<float> margin)
        {
            margin = shape.Radius;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeSupport(ref CapsuleWide shape, ref Matrix3x3Wide orientation, ref Vector3Wide direction, out Vector3Wide support)
        {
            Vector3Wide.Scale(orientation.Y, shape.HalfLength, out support);
            Vector3Wide.Negate(support, out var negated);
            Vector3Wide.Dot(orientation.Y, direction, out var dot);
            var shouldNegate = Vector.LessThan(dot, Vector<float>.Zero);
            Vector3Wide.ConditionalSelect(shouldNegate, negated, support, out support);
        }
    }
}
