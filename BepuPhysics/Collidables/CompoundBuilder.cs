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
            /// Inverse inertia tensor of the child in its local space.
            /// </summary>
            public Symmetric3x3 LocalInverseInertia;
        }

        public QuickList<Child> Children;

        /// <summary>
        /// Creates a compound builder.
        /// </summary>
        /// <param name="pool">Buffer pool to allocate memory from when necessary.</param>
        /// <param name="shapes">Shapes collection to access when constructing the compound children.</param>
        /// <param name="initialBuilderCapacity">Number of children the compound builder can hold without resizing.</param>
        public CompoundBuilder(BufferPool pool, Shapes shapes, int initialBuilderCapacity)
        {
            Pool = pool;
            Shapes = shapes;
            Children = new QuickList<Child>(initialBuilderCapacity, Pool);
        }

        /// <summary>
        /// Adds a new shape to the accumulator.
        /// </summary>
        /// <param name="shape">Index of the shape to add.</param>
        /// <param name="localPose">Pose of the shape in the compound's local space.</param>
        /// <param name="weight">Weight of the shape. If the compound is interpreted as a dynamic, this will be used as the mass. Otherwise, it is used for recentering.</param>
        /// <param name="localInverseInertia">Inverse inertia tensor of the shape being added in its local space. This is assumed to already be scaled as desired by the weight.</param>
        public void Add(TypedIndex shape, in RigidPose localPose, in Symmetric3x3 localInverseInertia, float weight)
        {
            Debug.Assert(Compound.ValidateChildIndex(shape, Shapes));
            ref var child = ref Children.Allocate(Pool);
            child.LocalPose = localPose;
            child.ShapeIndex = shape;
            child.Weight = weight;
            child.LocalInverseInertia = localInverseInertia;
            //This assumes the given inertia is nonsingular. That should be a valid assumption, unless the user is trying to supply an axis-locked tensor.
            //For such a use case, it's best to just lock the axis after computing a 'normal' inertia. 
            Debug.Assert(Symmetric3x3.Determinant(localInverseInertia) > 0,
                "Child inertia tensors should be invertible. If making an axis-locked compound, consider locking the axis on the completed inertia. " +
                "If making a kinematic, consider using the overload which takes no inverse inertia.");
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
            child.LocalInverseInertia = default;
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
            Add(Shapes.Add(shape), localPose, shape.ComputeInertia(weight).InverseInertiaTensor, weight);
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
            AddForKinematic(Shapes.Add(shape), localPose, weight);
        }


        /// <summary>
        /// Gets the contribution to an inertia tensor of a point mass at the given offset from the center of mass.
        /// </summary>
        /// <param name="offset">Offset from the center of mass.</param>
        /// <param name="mass">Mass of the point.</param>
        /// <param name="contribution">Contribution to the inertia tensor.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetOffsetInertiaContribution(Vector3 offset, float mass, out Symmetric3x3 contribution)
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
                targetChild.LocalPosition = sourceChild.LocalPose.Position - center;
                targetChild.LocalOrientation = sourceChild.LocalPose.Orientation;
                targetChild.ShapeIndex = sourceChild.ShapeIndex;
                Symmetric3x3.Add(ComputeInertiaForChild(targetChild.LocalPosition, targetChild.LocalOrientation, sourceChild.LocalInverseInertia, sourceChild.Weight), summedInertia, out summedInertia);
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
                targetChild.LocalPosition = sourceChild.LocalPose.Position;
                targetChild.LocalOrientation = sourceChild.LocalPose.Orientation;
                targetChild.ShapeIndex = sourceChild.ShapeIndex;
                Symmetric3x3.Add(ComputeInertiaForChild(sourceChild.LocalPose.Position, sourceChild.LocalPose.Orientation, sourceChild.LocalInverseInertia, sourceChild.Weight), summedInertia, out summedInertia);
            }
            Symmetric3x3.Invert(summedInertia, out inertia.InverseInertiaTensor);
        }

        /// <summary>
        /// Computes the uninverted inertia contribution of a child.
        /// </summary>
        /// <param name="pose">Pose of the child.</param>
        /// <param name="inverseLocalInertia">Inverse inertia tensor of the child in its local space.</param>
        /// <param name="mass">Mass of the child.</param>
        /// <returns>Inertia contribution of the child to a compound given its relative pose.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Symmetric3x3 ComputeInertiaForChild(in RigidPose pose, Symmetric3x3 inverseLocalInertia, float mass)
        {
            return ComputeInertiaForChild(pose.Position, pose.Orientation, inverseLocalInertia, mass);
        }
        /// <summary>
        /// Computes the uninverted inertia contribution of a child.
        /// </summary>
        /// <param name="position">Position of the child.</param>
        /// <param name="orientation">Orientation of the child.</param>
        /// <param name="inverseLocalInertia">Inverse inertia tensor of the child in its local space.</param>
        /// <param name="mass">Mass of the child.</param>
        /// <returns>Inertia contribution of the child to a compound given its relative pose.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Symmetric3x3 ComputeInertiaForChild(Vector3 position, Quaternion orientation, Symmetric3x3 inverseLocalInertia, float mass)
        {
            GetOffsetInertiaContribution(position, mass, out var offsetContribution);
            //This assumes the given inertia is nonsingular. That should be a valid assumption, unless the user is trying to supply an axis-locked tensor.
            //For such a use case, it's best to just lock the axis after computing a 'normal' inertia. 
            Debug.Assert(Symmetric3x3.Determinant(inverseLocalInertia) > 0,
                "Child inertia tensors should be invertible. If making an axis-locked compound, consider locking the axis on the completed inertia. " +
                "If making a kinematic, consider using the overload which takes no inverse inertia.");
            PoseIntegration.RotateInverseInertia(inverseLocalInertia, orientation, out var rotatedInverseInertia);
            Symmetric3x3.Invert(rotatedInverseInertia, out var inertia);
            Symmetric3x3.Add(offsetContribution, inertia, out inertia);
            return inertia;
        }

        /// <summary>
        /// Computes the inertia for a set of compound children based on their poses and the provided inverse inertias. Does not recenter the children.
        /// </summary>
        /// <param name="children">Children and their associated poses.</param>
        /// <param name="inverseLocalInertias">Inverse inertias of the children, each in the child's local space. Assumed to have already been premultiplied by the mass of the child.</param>
        /// <param name="childMasses">Masses of each child in the compound.</param>
        /// <returns><see cref="BodyInertia"/> of the compound.</returns>
        public static BodyInertia ComputeInverseInertia(Span<CompoundChild> children, Span<Symmetric3x3> inverseLocalInertias, Span<float> childMasses)
        {
            Symmetric3x3 summedInertia = default;
            float massSum = 0;
            for (int i = 0; i < children.Length; ++i)
            {
                ref var child = ref children[i];
                summedInertia += ComputeInertiaForChild(child.LocalPosition, child.LocalOrientation, inverseLocalInertias[i], childMasses[i]);
                massSum += childMasses[i];
            }
            BodyInertia inertia;
            Symmetric3x3.Invert(summedInertia, out inertia.InverseInertiaTensor);
            inertia.InverseMass = 1f / massSum;
            return inertia;
        }

        /// <summary>
        /// Computes the inverse inertia for a set of compound children based on their poses and the provided inverse inertias. Does not recenter the children.
        /// </summary>
        /// <param name="childPoses">Poses of the compound's children.</param>
        /// <param name="inverseLocalInertias">Inverse inertias of the children, each in the child's local space. Assumed to have already been premultiplied by the mass of the child.</param>
        /// <param name="childMasses">Masses of each child in the compound.</param>
        /// <returns><see cref="BodyInertia"/> of the compound.</returns>
        public static BodyInertia ComputeInverseInertia(Span<RigidPose> childPoses, Span<Symmetric3x3> inverseLocalInertias, Span<float> childMasses)
        {
            Symmetric3x3 summedInertia = default;
            float massSum = 0;
            for (int i = 0; i < childPoses.Length; ++i)
            {
                summedInertia += ComputeInertiaForChild(childPoses[i], inverseLocalInertias[i], childMasses[i]);
                massSum += childMasses[i];
            }
            BodyInertia inertia;
            Symmetric3x3.Invert(summedInertia, out inertia.InverseInertiaTensor);
            inertia.InverseMass = 1f / massSum;
            return inertia;
        }
        /// <summary>
        /// Computes the center of mass of a compound.
        /// </summary>
        /// <param name="children">Children of the compound.</param>
        /// <param name="childMasses">Masses of the children in the compound.</param>
        /// <param name="inverseMass">Inverse of the sum of all child masses.</param>
        /// <returns>The compound's center of mass.</returns>
        public static Vector3 ComputeCenterOfMass(Span<CompoundChild> children, Span<float> childMasses, out float inverseMass)
        {
            Vector3 sum = default;
            float massSum = 0;
            for (int i = 0; i < children.Length; ++i)
            {
                sum += childMasses[i] * children[i].LocalPosition;
                massSum += childMasses[i];
            }
            inverseMass = 1f / massSum;
            return sum * inverseMass;
        }
        /// <summary>
        /// Computes the center of mass of a compound.
        /// </summary>
        /// <param name="childPoses">Poses of the children in the compound.</param>
        /// <param name="childMasses">Masses of the children in the compound.</param>
        /// <param name="inverseMass">Inverse of the sum of all child masses.</param>
        /// <returns>The compound's center of mass.</returns>
        public static Vector3 ComputeCenterOfMass(Span<RigidPose> childPoses, Span<float> childMasses, out float inverseMass)
        {
            Vector3 sum = default;
            float massSum = 0;
            for (int i = 0; i < childPoses.Length; ++i)
            {
                sum += childMasses[i] * childPoses[i].Position;
                massSum += childMasses[i];
            }
            inverseMass = 1f / massSum;
            return sum * inverseMass;
        }
        /// <summary>
        /// Computes the center of mass of a compound.
        /// </summary>
        /// <param name="children">Children of the compound.</param>
        /// <param name="childMasses">Masses of the children in the compound.</param>
        /// <returns>The compound's center of mass.</returns>
        public static Vector3 ComputeCenterOfMass(Span<CompoundChild> children, Span<float> childMasses) => ComputeCenterOfMass(children, childMasses, out _);

        /// <summary>
        /// Computes the center of mass of a compound.
        /// </summary>
        /// <param name="childPoses">Poses of the children in the compound.</param>
        /// <param name="childMasses">Masses of the children in the compound.</param>
        /// <returns>The compound's center of mass.</returns>
        public static Vector3 ComputeCenterOfMass(Span<RigidPose> childPoses, Span<float> childMasses) => ComputeCenterOfMass(childPoses, childMasses, out _);

        /// <summary>
        /// Computes the inertia for a set of compound children based on their poses and the provided inverse inertias. Recenters the children onto the computed center of mass.
        /// </summary>
        /// <param name="children">Children and their associated poses. Center of mass will be subtracted from the child position.</param>
        /// <param name="inverseLocalInertias">Inverse inertias of the children, each in the child's local space. Assumed to have already been premultiplied by the mass of the child.</param>
        /// <param name="childMasses">Masses of each child in the compound.</param>
        /// <param name="centerOfMass">Computed center of mass that was subtracted from the child positions.</param>
        /// <returns><see cref="BodyInertia"/> of the compound.</returns>
        public static BodyInertia ComputeInverseInertia(Span<CompoundChild> children, Span<Symmetric3x3> inverseLocalInertias, Span<float> childMasses, out Vector3 centerOfMass)
        {
            Symmetric3x3 summedInertia = default;
            BodyInertia inertia;
            centerOfMass = ComputeCenterOfMass(children, childMasses, out inertia.InverseMass);
            for (int i = 0; i < children.Length; ++i)
            {
                ref var child = ref children[i];
                child.LocalPosition -= centerOfMass;
                summedInertia += ComputeInertiaForChild(child.LocalPosition, child.LocalOrientation, inverseLocalInertias[i], childMasses[i]);
            }
            Symmetric3x3.Invert(summedInertia, out inertia.InverseInertiaTensor);
            return inertia;
        }
        /// <summary>
        /// Computes the inertia for a set of compound children based on their poses and the provided inverse inertias. Recenters the children onto the computed center of mass.
        /// </summary>
        /// <param name="childPoses">Poses of the compound's children. Center of mass will be subtracted from the child position.</param>
        /// <param name="inverseLocalInertias">Inverse inertias of the children, each in the child's local space. Assumed to have already been premultiplied by the mass of the child.</param>
        /// <param name="childMasses">Masses of each child in the compound.</param>
        /// <param name="centerOfMass">Computed center of mass that was subtracted from the child positions.</param>
        /// <returns><see cref="BodyInertia"/> of the compound.</returns>
        public static BodyInertia ComputeInverseInertia(Span<RigidPose> childPoses, Span<Symmetric3x3> inverseLocalInertias, Span<float> childMasses, out Vector3 centerOfMass)
        {
            Symmetric3x3 summedInertia = default;
            BodyInertia inertia;
            centerOfMass = ComputeCenterOfMass(childPoses, childMasses, out inertia.InverseMass);
            for (int i = 0; i < childPoses.Length; ++i)
            {
                childPoses[i].Position -= centerOfMass;
                summedInertia += ComputeInertiaForChild(childPoses[i], inverseLocalInertias[i], childMasses[i]);
            }
            Symmetric3x3.Invert(summedInertia, out inertia.InverseInertiaTensor);
            return inertia;
        }

        /// <summary>
        /// Computes the inertia of a compound. Does not recenter the child poses.
        /// </summary>
        /// <param name="children">Children of the compound.</param>
        /// <param name="shapes">Shapes collection containing the data for the compound child shapes.</param>
        /// <param name="childMasses">Masses of the children.</param>
        /// <returns>Inertia of the compound.</returns>
        public static BodyInertia ComputeInertia(Span<CompoundChild> children, Span<float> childMasses, Shapes shapes)
        {
            Span<Symmetric3x3> localInverseInertias = stackalloc Symmetric3x3[children.Length];
            for (int i = 0; i < children.Length; ++i)
            {
                ref var child = ref children[i];
                if (shapes[child.ShapeIndex.Type] is IConvexShapeBatch batch)
                {
                    localInverseInertias[i] = batch.ComputeInertia(child.ShapeIndex.Index, childMasses[i]).InverseInertiaTensor;
                }
            }
            return ComputeInverseInertia(children, localInverseInertias, childMasses);
        }

        /// <summary>
        /// Computes the inertia of a compound. Recenters the child poses around the calculated center of mass.
        /// </summary>
        /// <param name="children">Children of the compound. Child local positions will have the calculated center of mass subtracted from them.</param>
        /// <param name="shapes">Shapes collection containing the data for the compound child shapes.</param>
        /// <param name="childMasses">Masses of the children.</param>
        /// <param name="centerOfMass">Calculated center of mass of the compound. Subtracted from all the compound child poses.</param>
        /// <returns>Inertia of the compound.</returns>
        public static BodyInertia ComputeInertia(Span<CompoundChild> children, Span<float> childMasses, Shapes shapes, out Vector3 centerOfMass)
        {
            Span<Symmetric3x3> localInverseInertias = stackalloc Symmetric3x3[children.Length];
            for (int i = 0; i < children.Length; ++i)
            {
                ref var child = ref children[i];
                if (shapes[child.ShapeIndex.Type] is IConvexShapeBatch batch)
                {
                    localInverseInertias[i] = batch.ComputeInertia(child.ShapeIndex.Index, childMasses[i]).InverseInertiaTensor;
                }
            }
            return ComputeInverseInertia(children, localInverseInertias, childMasses, out centerOfMass);
        }

        /// <summary>
        /// Builds a buffer of compound children from the accumulated set for a kinematic compound.
        /// Computes a center of mass and recenters child shapes relative to it. Does not reset the accumulator.
        /// </summary>
        /// <param name="children">List of children created from the accumulated set.</param>
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
                targetChild.LocalPosition = sourceChild.LocalPose.Position - center;
                targetChild.LocalOrientation = sourceChild.LocalPose.Orientation;
                targetChild.ShapeIndex = sourceChild.ShapeIndex;
            }
        }

        /// <summary>
        /// Builds a buffer of compound children from the accumulated set for a kinematic compound. Does not recenter children. Does not reset the accumulator.
        /// </summary>
        /// <param name="children">List of children created from the accumulated set.</param>
        public void BuildKinematicCompound(out Buffer<CompoundChild> children)
        {
            Pool.Take(Children.Count, out children);
            for (int i = 0; i < Children.Count; ++i)
            {
                ref var sourceChild = ref Children[i];
                ref var targetChild = ref children[i];
                targetChild.LocalPosition = sourceChild.LocalPose.Position;
                targetChild.LocalOrientation = sourceChild.LocalPose.Orientation;
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
