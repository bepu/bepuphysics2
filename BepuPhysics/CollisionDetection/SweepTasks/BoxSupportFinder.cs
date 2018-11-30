using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.CollisionDetection.SweepTasks
{
    public struct BoxSupportFinder : ISupportFinder<Box, BoxWide>
    {
        public bool HasMargin
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return false; }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetMargin(in BoxWide shape, out Vector<float> margin)
        {
            margin = default;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeSupport(in BoxWide shape, in Matrix3x3Wide orientation, in Vector3Wide direction, out Vector3Wide support)
        {
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(direction, orientation, out var localDirection);
            ComputeLocalSupport(shape, localDirection, out var localSupport);
            Matrix3x3Wide.TransformWithoutOverlap(localSupport, orientation, out support);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeLocalSupport(in BoxWide shape, in Vector3Wide direction, out Vector3Wide support)
        {
            support.X = Vector.ConditionalSelect(Vector.LessThan(direction.X, Vector<float>.Zero), -shape.HalfWidth, shape.HalfWidth);
            support.Y = Vector.ConditionalSelect(Vector.LessThan(direction.Y, Vector<float>.Zero), -shape.HalfHeight, shape.HalfHeight);
            support.Z = Vector.ConditionalSelect(Vector.LessThan(direction.Z, Vector<float>.Zero), -shape.HalfLength, shape.HalfLength);
        }
    }
}
