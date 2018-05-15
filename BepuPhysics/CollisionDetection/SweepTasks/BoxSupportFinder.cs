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
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeSupport(ref BoxWide shape, ref Matrix3x3Wide orientation, ref Vector3Wide direction, out Vector3Wide support)
        {
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(ref direction, ref orientation, out var localDirection);
            Vector3Wide localSupport;
            localSupport.X = Vector.ConditionalSelect(Vector.LessThan(localDirection.X, Vector<float>.Zero), -shape.HalfWidth, shape.HalfWidth);
            localSupport.Y = Vector.ConditionalSelect(Vector.LessThan(localDirection.Y, Vector<float>.Zero), -shape.HalfHeight, shape.HalfHeight);
            localSupport.Z = Vector.ConditionalSelect(Vector.LessThan(localDirection.Z, Vector<float>.Zero), -shape.HalfLength, shape.HalfLength);
            Matrix3x3Wide.TransformWithoutOverlap(ref localSupport, ref orientation, out support);
        }
    }
}
