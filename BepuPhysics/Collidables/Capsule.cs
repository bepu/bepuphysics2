using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using BepuUtilities.Memory;
using System.Diagnostics;

namespace BepuPhysics.Collidables
{
    public struct CapsuleWide
    {
        public Vector<float> Radius;
        public Vector<float> HalfLength;
    }
    
    /// <summary>
    /// Collision shape representing a sphere-expanded line segment.
    /// </summary>
    public struct Capsule : IShape
    {
        /// <summary>
        /// Spherical expansion applied to the internal line segment.
        /// </summary>
        public float Radius;
        /// <summary>
        /// Half of the length of the internal line segment. Oriented along the local Y axis.
        /// </summary>
        public float HalfLength;

        /// <summary>
        /// Gets or sets the length of the capsule.
        /// </summary>
        public float Length { get { return HalfLength * 2; } set { HalfLength = value * 0.5f; } }

        public Capsule(float radius, float length)
        {
            Radius = radius;
            HalfLength = length * 0.5f;
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Gather(ref Buffer<Capsule> shapes, ref Vector<int> shapeIndices, int count, out CapsuleWide capsules)
        {
            ref var radiusBase = ref Unsafe.As<Vector<float>, float>(ref capsules.Radius);
            ref var halfLengthBase = ref Unsafe.As<Vector<float>, float>(ref capsules.HalfLength);
            ref var shapeIndicesBase = ref Unsafe.As<Vector<int>, int>(ref shapeIndices);
            Debug.Assert(count <= Vector<float>.Count);
            for (int i = 0; i < count; ++i)
            {
                ref var shape = ref shapes[Unsafe.Add(ref shapeIndicesBase, i)];
                Unsafe.Add(ref radiusBase, i) = shape.Radius;
                Unsafe.Add(ref halfLengthBase, i) = shape.HalfLength;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetBounds<TShape>(ref Buffer<TShape> shapes, ref Vector<int> shapeIndices, int count, ref QuaternionWide orientations,
            out Vector<float> maximumRadius, out Vector<float> maximumAngularExpansion, out Vector3Wide min, out Vector3Wide max)
            where TShape : struct, IShape
        {
            Gather(ref Unsafe.As<Buffer<TShape>, Buffer<Capsule>>(ref shapes), ref shapeIndices, count, out var capsules);
            QuaternionWide.TransformUnitY(ref orientations, out var segmentOffset);
            Vector3Wide.Scale(ref segmentOffset, ref capsules.HalfLength, out segmentOffset);
            Vector3Wide.Abs(ref segmentOffset, out segmentOffset);

            //The half length extends symmetrically along positive local Y and negative local Y.
            Vector3Wide.Add(ref segmentOffset, ref capsules.Radius, out max);
            Vector3Wide.Negate(ref max, out min);

            maximumRadius = capsules.HalfLength + capsules.Radius;
            //The minimum radius is capsules.Radius, so the maximum offset is simply the half length.
            maximumAngularExpansion = capsules.HalfLength;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetBounds(ref BepuUtilities.Quaternion orientation, out Vector3 min, out Vector3 max)
        {
            BepuUtilities.Quaternion.TransformUnitY(ref orientation, out var segmentOffset);
            max = Vector3.Abs(HalfLength * segmentOffset) + new Vector3(Radius);
            min = -max;
        }

        /// <summary>
        /// Type id of capsule shapes.
        /// </summary>
        public const int Id = 1;
        public int TypeId { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return Id; } }
    }
}
