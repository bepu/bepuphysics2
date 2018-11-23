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
        public void GetMargin(ref CylinderWide shape, out Vector<float> margin)
        {
            margin = default;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeSupport(ref CylinderWide shape, ref Matrix3x3Wide orientation, ref Vector3Wide direction, out Vector3Wide support)
        {
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(direction, orientation, out var localDirection);

            Vector3Wide localSupport;
            localSupport.Y = Vector.ConditionalSelect(Vector.GreaterThan(localDirection.Y, Vector<float>.Zero), shape.HalfLength, -shape.HalfLength);
            var horizontalLength = Vector.SquareRoot(localDirection.X * localDirection.X + localDirection.Z * localDirection.Z);
            var normalizeScale = shape.Radius / horizontalLength;
            var useHorizontal = Vector.GreaterThan(horizontalLength, new Vector<float>(1e-8f));
            localSupport.X = Vector.ConditionalSelect(useHorizontal, localDirection.X * normalizeScale, Vector<float>.Zero);
            localSupport.Z = Vector.ConditionalSelect(useHorizontal, localDirection.Z * normalizeScale, Vector<float>.Zero);
            Matrix3x3Wide.TransformWithoutOverlap(localSupport, orientation, out support);
        }
    }
}
