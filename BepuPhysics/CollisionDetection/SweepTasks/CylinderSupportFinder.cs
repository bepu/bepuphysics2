using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.CollisionDetection.SweepTasks
{
    public struct CylinderSupportFinder : ISupportFinder<Cylinder, CylinderWide>
    {
        public bool HasMargin
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return false; }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetMargin(in CylinderWide shape, out Vector<float> margin)
        {
            margin = default;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeSupport(in CylinderWide shape, in Matrix3x3Wide orientation, in Vector3Wide direction, out Vector3Wide support)
        {
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(direction, orientation, out var localDirection);
            ComputeLocalSupport(shape, localDirection, out var localSupport);
            Matrix3x3Wide.TransformWithoutOverlap(localSupport, orientation, out support);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeLocalSupport(in CylinderWide shape, in Vector3Wide direction, out Vector3Wide support)
        {
            support.Y = Vector.ConditionalSelect(Vector.GreaterThan(direction.Y, Vector<float>.Zero), shape.HalfLength, -shape.HalfLength);
            //TODO: Using a hardware accelerated reciprocal sqrt approximation would be hugely beneficial here.
            //It would actually be meaningful to full frame time in simulations that rely on cylinders.
            var horizontalLength = Vector.SquareRoot(direction.X * direction.X + direction.Z * direction.Z);
            var normalizeScale = shape.Radius / horizontalLength;
            var useHorizontal = Vector.GreaterThan(horizontalLength, new Vector<float>(1e-8f));
            support.X = Vector.ConditionalSelect(useHorizontal, direction.X * normalizeScale, Vector<float>.Zero);
            support.Z = Vector.ConditionalSelect(useHorizontal, direction.Z * normalizeScale, Vector<float>.Zero);
        }
    }
}
