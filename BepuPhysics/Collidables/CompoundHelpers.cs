using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.Collidables
{
    public struct CompoundInertiaBuilder
    {
        /// <summary>
        /// Combined mass added to the builder so far.
        /// </summary>
        public float AccumulatedMass;
        /// <summary>
        /// Combined inertia tensor from all posed contributions.
        /// </summary>
        public Triangular3x3 AccumulatedInertiaTensor;
        public Triangular3x3 InverseAccumulatedInertiaTensor
        {
            get
            {
                Triangular3x3.SymmetricInvert(ref AccumulatedInertiaTensor, out var toReturn);
                return toReturn;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetOffsetContribution(ref Vector3 offset, float mass, out Triangular3x3 contribution)
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
        /// Accumulates an inertia contribution with a given offset.
        /// </summary>
        /// <param name="offset">Offset at which to place the inertia contribution.</param>
        /// <param name="mass">Mass of the contribution.</param>
        /// <param name="inertiaTensor">Inertia tensor to accumulate at the given offset.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Add(ref Vector3 offset, float mass, ref Triangular3x3 inertiaTensor)
        {
            GetOffsetContribution(ref offset, mass, out var contribution);
            Triangular3x3.Add(ref inertiaTensor, ref contribution, out contribution);
            Triangular3x3.Add(ref AccumulatedInertiaTensor, ref contribution, out AccumulatedInertiaTensor);
            AccumulatedMass += mass;
        }

        /// <summary>
        /// Accumulates the inertia associated with a shape's default inertia tensor with a given pose and mass.
        /// </summary>
        /// <typeparam name="TShape">Type of the convex shape to get the default inertia tensor from.</typeparam>
        /// <param name="pose"></param>
        /// <param name="mass">Mass associated with the shape entry.</param>
        /// <param name="shape">Convex shape to compute a default inertia tensor for.</param>
        public void Add<TShape>(ref RigidPose pose, float mass, ref TShape shape) where TShape : struct, IConvexShape
        {
            shape.ComputeLocalInverseInertia(1f / mass, out var localInverseInertia);
            PoseIntegrator.RotateInverseInertia(ref localInverseInertia, ref pose.Orientation, out var rotatedInverseInertia);
            Triangular3x3.SymmetricInvert(ref rotatedInverseInertia, out var rotatedInertia);
            Add(ref pose.Position, mass, ref rotatedInertia);
        }

        /// <summary>
        /// Accumulates a contribution from an explicitly provided mass and inertia tensor at the specified pose.
        /// </summary>
        /// <param name="pose">Pose of the contribution being accumulated.</param>
        /// <param name="mass">Mass of the contribution.</param>
        /// <param name="localInertiaTensor">Local inertia tensor of the contribution being accumulated.</param>
        public void Add(ref RigidPose pose, float mass, ref Triangular3x3 localInertiaTensor)
        {
            PoseIntegrator.RotateInverseInertia(ref localInertiaTensor, ref pose.Orientation, out var rotatedInertia);
            Add(ref pose.Position, mass, ref rotatedInertia);
        }
    }
    /// <summary>
    /// Reusable convenience type for incrementally building compound shapes.
    /// </summary>
    public struct CompoundChildBuilder : IDisposable
    {
        public BufferPool Pool;
        public Shapes Shapes;

        QuickList<CompoundChild, Buffer<CompoundChild>> accumulatedChildren;

        public CompoundChildBuilder(BufferPool pool, Shapes shapes, int builderCapacity)
        {
            Pool = pool;
            Shapes = shapes;
            QuickList<CompoundChild, Buffer<CompoundChild>>.Create(Pool.SpecializeFor<CompoundChild>(), builderCapacity, out accumulatedChildren);
        }

        /// <summary>
        /// Adds a new shape to the accumulator, creating a new shape in the shapes set.
        /// </summary>
        /// <typeparam name="TShape">Type of the shape to add to the accumulator and the shapes set.</typeparam>
        /// <param name="shape">Shape to add.</param>
        /// <param name="localPose">Pose of the shape in the compound's local space.</param>
        public void Add<TShape>(ref TShape shape, ref RigidPose localPose) where TShape : struct, IConvexShape
        {
            var shapeIndex = Shapes.Add(ref shape);
            var child = new CompoundChild { LocalPose = localPose, ShapeIndex = Shapes.Add(ref shape) };
            accumulatedChildren.Add(ref child, Pool.SpecializeFor<CompoundChild>());
        }

        /// <summary>
        /// Adds a new shape to the accumulator.
        /// </summary>
        /// <param name="shapeIndex">Index of the shape to add.</param>
        /// <param name="localPose">Pose of the shape in the compound's local space.</param>
        public void Add(TypedIndex shapeIndex, ref RigidPose localPose)
        {
            var child = new CompoundChild { LocalPose = localPose, ShapeIndex = shapeIndex };
            accumulatedChildren.Add(ref child, Pool.SpecializeFor<CompoundChild>());
        }

        /// <summary>
        /// Builds a buffer of compound children from the accumulated set. Does not reset the accumulator.
        /// </summary>
        /// <param name="children">List of children created from the accumulated set.</param>
        public void BuildCompound(out Buffer<CompoundChild> children)
        {
            Pool.SpecializeFor<CompoundChild>().Take(accumulatedChildren.Count, out children);
            //Note that the buffer returned by the pool is only guaranteed to be at least as large as the requested size.
            //The compound expects the buffer's length to exactly match the number of children, so we explicitly slice to avoid relying on the size of the returned buffer.
            children = children.Slice(0, accumulatedChildren.Count);
            accumulatedChildren.Span.CopyTo(0, ref children, 0, accumulatedChildren.Count);
        }

        /// <summary>
        /// Builds a compound from the accumulated set. Does not reset the accumulator.
        /// </summary>
        /// <param name="compound">Compound created from the accumulated set.</param>
        public void BuildCompound(out Compound compound)
        {
            BuildCompound(out compound.Children);
        }


        /// <summary>
        /// Empties out the accumulated children.
        /// </summary>
        public void Reset()
        {
            accumulatedChildren.Count = 0;
        }

        /// <summary>
        /// Returns internal resources to the pool, rendering the builder unusable.
        /// </summary>
        public void Dispose()
        {
            accumulatedChildren.Dispose(Pool.SpecializeFor<CompoundChild>());
        }
    }
    /// <summary>
    /// Reusable convenience type for incrementally building dynamic compound shapes and calculating their default properties.
    /// </summary>
    public struct CompoundBuilder : IDisposable
    {
        public CompoundChildBuilder ChildBuilder;
        public CompoundInertiaBuilder InertiaBuilder;

        public CompoundBuilder(BufferPool pool, Shapes shapes, int builderCapacity = 32)
        {
            ChildBuilder = new CompoundChildBuilder(pool, shapes, builderCapacity);
            InertiaBuilder = new CompoundInertiaBuilder();
        }

        /// <summary>
        /// Adds a new shape to the accumulator, creating a new shape in the shapes set.
        /// </summary>
        /// <typeparam name="TShape">Type of the shape to add to the accumulator and the shapes set.</typeparam>
        /// <param name="shape">Shape to add.</param>
        /// <param name="localPose">Pose of the shape in the compound's local space.</param>
        /// <param name="mass">Mass of the shape being added.</param>
        public void Add<TShape>(ref TShape shape, ref RigidPose localPose, float mass) where TShape : struct, IConvexShape
        {
            ChildBuilder.Add(ref shape, ref localPose);
            InertiaBuilder.Add(ref localPose, mass, ref shape);
        }

        /// <summary>
        /// Adds a new shape to the accumulator.
        /// </summary>
        /// <param name="shapeIndex">Index of the shape to add.</param>
        /// <param name="localPose">Pose of the shape in the compound's local space.</param>
        /// <param name="mass">Mass of the shape being added.</param>
        /// <param name="localInertiaTensor">Local inertia tensor of the shape being added.</param>
        public void Add(TypedIndex shapeIndex, ref RigidPose localPose, float mass, ref Triangular3x3 localInertiaTensor)
        {
            ChildBuilder.Add(shapeIndex, ref localPose);
            InertiaBuilder.Add(ref localPose, mass, ref localInertiaTensor);
        }

        /// <summary>
        /// Builds a compound from the accumulated children. Does not reset the accumulator.
        /// </summary>
        /// <param name="compound">Compound built from the accumulated children.</param>
        /// <param name="inertia">Inertia of the created compound.</param>
        public void BuildCompound(out Compound compound, out BodyInertia inertia)
        {
            ChildBuilder.BuildCompound(out compound);
            inertia.InverseMass = 1f / InertiaBuilder.AccumulatedMass;
            Triangular3x3.SymmetricInvert(ref InertiaBuilder.AccumulatedInertiaTensor, out inertia.InverseInertiaTensor);
        }

        /// <summary>
        /// Empties out the accumulated children.
        /// </summary>
        public void Reset()
        {
            ChildBuilder.Reset();
            InertiaBuilder = new CompoundInertiaBuilder();
        }

        /// <summary>
        /// Returns internal resources to the pool, rendering the builder unusable.
        /// </summary>
        public void Dispose()
        {
            ChildBuilder.Dispose();
        }
    }
}
