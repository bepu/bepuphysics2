using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.CollisionDetection.SweepTasks
{
    public struct TriangleSupportFinder : ISupportFinder<Triangle, TriangleWide>
    {
        public bool HasMargin
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return false; }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetMargin(ref TriangleWide shape, out Vector<float> margin)
        {
            margin = default;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeSupport(ref TriangleWide shape, ref Matrix3x3Wide orientation, ref Vector3Wide direction, out Vector3Wide support)
        {
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(direction, orientation, out var localDirection);
            Vector3Wide.Dot(shape.A, localDirection, out var a);
            Vector3Wide.Dot(shape.B, localDirection, out var b);
            Vector3Wide.Dot(shape.C, localDirection, out var c);
            var max = Vector.Max(a, Vector.Max(b, c));
            Vector3Wide.ConditionalSelect(Vector.Equals(max, a), shape.A, shape.B, out var localSupport);
            Vector3Wide.ConditionalSelect(Vector.Equals(max, c), shape.C, localSupport, out localSupport);
            Matrix3x3Wide.TransformWithoutOverlap(localSupport, orientation, out support);
        }
    }
}
