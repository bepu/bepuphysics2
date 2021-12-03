using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.Collidables
{
    /// <summary>
    /// Reusable convenience type for incrementally building compound shapes.
    /// </summary>
    public struct CompoundBuilder : IDisposable
    {
        public BufferPool Pool;
        public Shapes Shapes;

        public struct Child
        {
            public RigidPose LocalPose;
            public TypedIndex ShapeIndex;
            /// <summary>
            /// Weight associated with this child. Acts as the child's mass when interpreted as a dynamic compound.
            /// When interpreted as kinematic with recentering, it is used as a local pose weight to compute the center of rotation.
            /// </summary>
            public float Weight;
            /// <summary>
            /// Inertia tensor associated with the child. If inertia is all zeroes, it is interpreted as infinite.
            /// </summary>
            public Symmetric3x3 Inertia;
        }

        public QuickList<Child> Children;

        public CompoundBuilder(BufferPool pool, Shapes shapes, int builderCapacity)
        {
            Pool = pool;
            Shapes = shapes;
            Children = new QuickList<Child>(builderCapacity, Pool);
        }

        /// <summary>
        /// Adds a new shape to the accumulator, creating a new shape in the shapes set. The mass used to compute the inertia tensor will be based on the given weight.
        /// </summary>
        /// <typeparam name="TShape">Type of the shape to add to the accumulator and the shapes set.</typeparam>
        /// <param name="shape">Shape to add.</param>
        /// <param name="localPose">Pose of the shape in the compound's local space.</param>
        /// <param name="weight">Weight of the shape. If the compound is interpreted as a dynamic, this will be used as the mass and scales the inertia tensor. 
        /// Otherwise, it is used for recentering.</param>
        public void Add<TShape>(in TShape shape, in RigidPose localPose, float weight) where TShape : unmanaged, IConvexShape
        {
            ref var child = ref Children.Allocate(Pool);
            child.LocalPose = localPose;
            child.ShapeIndex = Shapes.Add(shape);
            child.Weight = weight;
            var inertia = shape.ComputeInertia(weight);
            Symmetric3x3.Invert(inertia.InverseInertiaTensor, out child.Inertia);
        }

        /// <summary>
        /// Adds a new shape to the accumulator, creating a new shape in the shapes set. Inertia is assumed to be infinite.
        /// </summary>
        /// <typeparam name="TShape">Type of the shape to add to the accumulator and the shapes set.</typeparam>
        /// <param name="shape">Shape to add.</param>
        /// <param name="localPose">Pose of the shape in the compound's local space.</param>
        /// <param name="weight">Weight of the shape. If the compound is interpreted as a dynamic, this will be used as the mass. Otherwise, it is used for recentering.</param>
        public void AddForKinematic<TShape>(in TShape shape, in RigidPose localPose, float weight) where TShape : unmanaged, IConvexShape
        {
            ref var child = ref Children.Allocate(Pool);
            child.LocalPose = localPose;
            child.ShapeIndex = Shapes.Add(shape);
            child.Weight = weight;
            child.Inertia = default;
        }

        /// <summary>
        /// Adds a new shape to the accumulator.
        /// </summary>
        /// <param name="shape">Index of the shape to add.</param>
        /// <param name="localPose">Pose of the shape in the compound's local space.</param>
        /// <param name="weight">Weight of the shape. If the compound is interpreted as a dynamic, this will be used as the mass. Otherwise, it is used for recentering.</param>
        /// <param name="inverseInertia">Inverse inertia tensor of the shape being added. This is assumed to already be scaled as desired by the weight.</param>
        public void Add(TypedIndex shape, in RigidPose localPose, in Symmetric3x3 inverseInertia, float weight)
        {
            Debug.Assert(Compound.ValidateChildIndex(shape, Shapes));
            ref var child = ref Children.Allocate(Pool);
            child.LocalPose = localPose;
            child.ShapeIndex = shape;
            child.Weight = weight;
            //This assumes the given inertia is nonsingular. That should be a valid assumption, unless the user is trying to supply an axis-locked tensor.
            //For such a use case, it's best to just lock the axis after computing a 'normal' inertia. 
            Debug.Assert(Symmetric3x3.Determinant(inverseInertia) > 0,
                "Shape inertia tensors should be invertible. If making an axis-locked compound, consider locking the axis on the completed inertia. " +
                "If making a kinematic, consider using the overload which takes no inverse inertia.");
            Symmetric3x3.Invert(inverseInertia, out child.Inertia);
        }

        /// <summary>
        /// Adds a new shape to the accumulator, assuming it has infinite inertia.
        /// </summary>
        /// <param name="shape">Index of the shape to add.</param>
        /// <param name="localPose">Pose of the shape in the compound's local space.</param>
        /// <param name="weight">Weight of the shape used for computing the center of rotation.</param>
        public void AddForKinematic(TypedIndex shape, in RigidPose localPose, float weight)
        {
            Debug.Assert(Compound.ValidateChildIndex(shape, Shapes));
            ref var child = ref Children.Allocate(Pool);
            child.LocalPose = localPose;
            child.ShapeIndex = shape;
            child.Weight = weight;
            child.Inertia = default;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetOffsetInertiaContribution(in Vector3 offset, float mass, out Symmetric3x3 contribution)
        {
            var innerProduct = Vector3.Dot(offset, offset);
            contribution.XX = mass * (innerProduct - offset.X * offset.X);
            contribution.YX = -mass * (offset.Y * offset.X);
            contribution.YY = mass * (innerProduct - offset.Y * offset.Y);
            contribution.ZX = -mass * (offset.Z * offset.X);
            contribution.ZY = -mass * (offset.Z * offset.Y);
            contribution.ZZ = mass * (innerProduct - offset.Z * offset.Z);
        }

        /// <summary>
        /// Builds a buffer of compound children from the accumulated set for a dynamic compound.
        /// Computes a center of mass and recenters child shapes relative to it. Does not reset the accumulator.
        /// </summary>
        /// <param name="children">List of children created from the accumulated set.</param>
        /// <param name="inertia">Combined inertia of the compound.</param>
        /// <param name="center">Computed center of rotation based on the poses and weights of accumulated children.</param>
        public void BuildDynamicCompound(out Buffer<CompoundChild> children, out BodyInertia inertia, out Vector3 center)
        {
            center = new Vector3();
            float totalWeight = 0;
            for (int i = 0; i < Children.Count; ++i)
            {
                center += Children[i].LocalPose.Position * Children[i].Weight;
                totalWeight += Children[i].Weight;
            }
            Debug.Assert(totalWeight > 0, "The compound as a whole must have nonzero weight when using a recentering build. The center is undefined.");

            inertia.InverseMass = 1f / totalWeight;
            center *= inertia.InverseMass;
            Pool.Take(Children.Count, out children);
            Symmetric3x3 summedInertia = default;
            for (int i = 0; i < Children.Count; ++i)
            {
                ref var sourceChild = ref Children[i];
                ref var targetChild = ref children[i];
                targetChild.LocalPose.Position = sourceChild.LocalPose.Position - center;
                GetOffsetInertiaContribution(targetChild.LocalPose.Position, sourceChild.Weight, out var contribution);
                Symmetric3x3.Add(contribution, summedInertia, out summedInertia);
                Symmetric3x3.Add(summedInertia, sourceChild.Inertia, out summedInertia);
                targetChild.LocalPose.Orientation = sourceChild.LocalPose.Orientation;
                targetChild.ShapeIndex = sourceChild.ShapeIndex;
            }
            Symmetric3x3.Invert(summedInertia, out inertia.InverseInertiaTensor);
        }

        /// <summary>
        /// Builds a buffer of compound children from the accumulated set for a dynamic compound. Does not recenter the children. Does not reset the accumulator.
        /// </summary>
        /// <param name="children">List of children created from the accumulated set.</param>
        /// <param name="inertia">Combined inertia of the compound.</param>
        public void BuildDynamicCompound(out Buffer<CompoundChild> children, out BodyInertia inertia)
        {
            float totalWeight = 0;
            for (int i = 0; i < Children.Count; ++i)
            {
                totalWeight += Children[i].Weight;
            }
            Debug.Assert(totalWeight > 0, "The compound as a whole must have nonzero weight when creating a dynamic compound.");

            inertia.InverseMass = 1f / totalWeight;
            Pool.Take(Children.Count, out children);
            Symmetric3x3 summedInertia = default;
            for (int i = 0; i < Children.Count; ++i)
            {
                ref var sourceChild = ref Children[i];
                ref var targetChild = ref children[i];
                targetChild.LocalPose.Position = sourceChild.LocalPose.Position;
                GetOffsetInertiaContribution(targetChild.LocalPose.Position, sourceChild.Weight, out var contribution);
                Symmetric3x3.Add(contribution, summedInertia, out summedInertia);
                Symmetric3x3.Add(summedInertia, sourceChild.Inertia, out summedInertia);
                targetChild.LocalPose.Orientation = sourceChild.LocalPose.Orientation;
                targetChild.ShapeIndex = sourceChild.ShapeIndex;
            }
            Symmetric3x3.Invert(summedInertia, out inertia.InverseInertiaTensor);
        }

        /// <summary>
        /// Builds a buffer of compound children from the accumulated set for a kinematic compound.
        /// Computes a center of mass and recenters child shapes relative to it. Does not reset the accumulator.
        /// </summary>
        /// <param name="children">List of children created from the accumulated set.</param>
        /// <param name="inertia">Combined inertia of the compound.</param>
        /// <param name="center">Computed center of rotation based on the poses and weights of accumulated children.</param>
        public void BuildKinematicCompound(out Buffer<CompoundChild> children, out Vector3 center)
        {
            center = new Vector3();
            float totalWeight = 0;
            for (int i = 0; i < Children.Count; ++i)
            {
                center += Children[i].LocalPose.Position * Children[i].Weight;
                totalWeight += Children[i].Weight;
            }
            Debug.Assert(totalWeight > 0, "The compound as a whole must have nonzero weight when using a recentering build. The center is undefined.");

            var inverseWeight = 1f / totalWeight;
            center *= inverseWeight;
            Pool.Take(Children.Count, out children);
            for (int i = 0; i < Children.Count; ++i)
            {
                ref var sourceChild = ref Children[i];
                ref var targetChild = ref children[i];
                targetChild.LocalPose.Position = sourceChild.LocalPose.Position - center;
                targetChild.LocalPose.Orientation = sourceChild.LocalPose.Orientation;
                targetChild.ShapeIndex = sourceChild.ShapeIndex;
            }
        }

        /// <summary>
        /// Builds a buffer of compound children from the accumulated set for a kinematic compound. Does not recenter children. Does not reset the accumulator.
        /// </summary>
        /// <param name="children">List of children created from the accumulated set.</param>
        /// <param name="inertia">Combined inertia of the compound.</param>
        /// <param name="center">Computed center of rotation based on the poses and weights of accumulated children.</param>
        public void BuildKinematicCompound(out Buffer<CompoundChild> children)
        {
            Pool.Take(Children.Count, out children);
            for (int i = 0; i < Children.Count; ++i)
            {
                ref var sourceChild = ref Children[i];
                ref var targetChild = ref children[i];
                targetChild.LocalPose.Position = sourceChild.LocalPose.Position;
                targetChild.LocalPose.Orientation = sourceChild.LocalPose.Orientation;
                targetChild.ShapeIndex = sourceChild.ShapeIndex;
            }
        }

        /// <summary>
        /// Empties out the accumulated children.
        /// </summary>
        public void Reset()
        {
            Children.Count = 0;
        }

        /// <summary>
        /// Returns internal resources to the pool, rendering the builder unusable.
        /// </summary>
        public void Dispose()
        {
            Children.Dispose(Pool);
        }
    }
}
