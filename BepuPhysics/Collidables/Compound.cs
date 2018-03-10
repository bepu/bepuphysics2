using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuUtilities.Memory;
using System.Diagnostics;
using Quaternion = BepuUtilities.Quaternion;
using BepuUtilities;

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
            Debug.Assert(children.Length > 0, "Compounds must have a nonzero number of children.");
            Children = children;
        }

        [Conditional("DEBUG")]
        void ValidateChildIndices(Shapes shapeBatches)
        {
            for (int i = 0; i < Children.Length; ++i)
            {
                Debug.Assert(shapeBatches[Children[i].ShapeIndex.Type].Compound, "All children of a compound must be convex.");
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void GetRotatedChildPose(ref RigidPose localPose, ref Quaternion orientation, out RigidPose rotatedChildPose)
        {
            Quaternion.ConcatenateWithoutOverlap(ref orientation, ref localPose.Orientation, out rotatedChildPose.Orientation);
            Quaternion.Transform(ref localPose.Position, ref orientation, out rotatedChildPose.Position);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void ComputeChildBounds(ref CompoundChild child, ref Quaternion orientation, Shapes shapeBatches, out Vector3 childMin, out Vector3 childMax)
        {
            GetRotatedChildPose(ref child.LocalPose, ref orientation, out var childPose);
            shapeBatches[child.ShapeIndex.Type].ComputeBounds(child.ShapeIndex.Index, ref childPose, out childMin, out childMax);
        }

        public void GetBounds(ref Quaternion orientation, Shapes shapeBatches, out Vector3 min, out Vector3 max)
        {
            ComputeChildBounds(ref Children[0], ref orientation, shapeBatches, out min, out max);
            for (int i = 1; i < Children.Length; ++i)
            {
                ref var child = ref Children[i];
                ComputeChildBounds(ref Children[i], ref orientation, shapeBatches, out var childMin, out var childMax);
                BoundingBox.CreateMerged(ref min, ref max, ref childMin, ref childMax, out min, out max);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddChildBoundsToBatcher(ref BoundingBoxBatcher batcher, ref RigidPose pose, ref BodyVelocity velocity, int bodyIndex)
        {
            for (int i = 0; i < Children.Length; ++i)
            {
                ref var child = ref Children[i];
                GetRotatedChildPose(ref child.LocalPose, ref pose.Orientation, out var childPose);
                //TODO: This is an area that has to be updated for high precision poses. May be able to centralize positional work
                //by deferring it until the final bounds scatter step. Would require looking up the position then, but could be worth simplicity.
                childPose.Position += pose.Position;
                batcher.AddCompoundChild(bodyIndex, Children[i].ShapeIndex, ref childPose, ref velocity);
            }
        }

        public bool RayTest(ref RigidPose pose, ref Vector3 origin, ref Vector3 direction, Shapes shapeBatches, out float t, out Vector3 normal)
        {
            t = float.MaxValue;
            normal = new Vector3();
            for (int i = 0; i < Children.Length; ++i)
            {
                ref var child = ref Children[i];
                GetRotatedChildPose(ref child.LocalPose, ref pose.Orientation, out var childPose);
                //TODO: This is an area that has to be updated for high precision poses.
                childPose.Position += pose.Position;
                if (shapeBatches[child.ShapeIndex.Type].RayTest(child.ShapeIndex.Index, ref childPose, ref origin, ref direction, out var childT, out var childNormal) && childT < t)
                {
                    t = childT;
                    normal = childNormal;
                }
            }
            return t < float.MaxValue;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
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

}
