using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using BepuUtilities.Memory;
using System.Diagnostics;
using Quaternion = BepuUtilities.Quaternion;
using BepuUtilities;
using BepuUtilities.Collections;

namespace BepuPhysics.Collidables
{
    public struct CompoundChild
    {
        public TypedIndex ShapeIndex;
        public RigidPose LocalPose;
    }

    /// <summary>
    /// Minimalist compound shape containing a list of child shapes. Does not make use of any internal acceleration structure; should be used only with small groups of shapes.
    /// </summary>
    public struct Compound : ICompoundShape
    {
        /// <summary>
        /// Buffer of children within this compound.
        /// </summary>
        public Buffer<CompoundChild> Children;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Compound(Buffer<CompoundChild> children)
        {
            Children = children;
        }

        public void GetBounds<TShape>(ref Buffer<TShape> shapes, ref Vector<int> shapeIndices, int count, ref QuaternionWide orientations, out Vector<float> maximumRadius, out Vector<float> maximumAngularExpansion, out Vector3Wide min, out Vector3Wide max) where TShape : struct, IShape
        {
            throw new NotImplementedException();
        }

        public void ComputeLocalInverseInertia(float inverseMass, Shapes shapesBatches, out Triangular3x3 localInverseInertia)
        {
            localInverseInertia = new Triangular3x3();
        }

        public void GetBounds(ref Quaternion orientation, Shapes shapeBatches, out Vector3 min, out Vector3 max)
        {
            throw new NotImplementedException();
        }

        public bool RayTest(ref RigidPose pose, ref Vector3 origin, ref Vector3 direction, Shapes shapeBatches, out float t, out Vector3 normal)
        {
            throw new NotImplementedException();
        }

        public ShapeBatch CreateShapeBatch(BufferPool pool, int initialCapacity, Shapes shapes)
        {
            return new CompoundShapeBatch<Compound>(pool, initialCapacity, shapes);
        }

        /// <summary>
        /// Type id of compound shapes.
        /// </summary>
        public const int Id = 3;
        public int TypeId { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return Id; } }
    }
}
