using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuUtilities.Memory;
using System.Diagnostics;
using Quaternion = BepuUtilities.Quaternion;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuPhysics.Trees;
using BepuPhysics.CollisionDetection.CollisionTasks;

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
        public static void ValidateChildIndices(ref Buffer<CompoundChild> children, Shapes shapeBatches)
        {
            for (int i = 0; i < children.Length; ++i)
            {
                Debug.Assert(shapeBatches[children[i].ShapeIndex.Type].Compound, "All children of a compound must be convex.");
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetRotatedChildPose(in RigidPose localPose, in Quaternion orientation, out RigidPose rotatedChildPose)
        {
            Quaternion.ConcatenateWithoutOverlap(localPose.Orientation, orientation, out rotatedChildPose.Orientation);
            Quaternion.Transform(localPose.Position, orientation, out rotatedChildPose.Position);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetRotatedChildPose(in RigidPoses localPose, in QuaternionWide orientation, out Vector3Wide childPosition, out QuaternionWide childOrientation)
        {
            QuaternionWide.ConcatenateWithoutOverlap(localPose.Orientation, orientation, out childOrientation);
            QuaternionWide.TransformWithoutOverlap(localPose.Position, orientation, out childPosition);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetRotatedChildPose(in RigidPoses localPose, in QuaternionWide orientation, out RigidPoses rotatedChildPose)
        {
            GetRotatedChildPose(localPose, orientation, out rotatedChildPose.Position, out rotatedChildPose.Orientation);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetWorldPose(in RigidPose localPose, in RigidPose transform, out RigidPose worldPose)
        {
            GetRotatedChildPose(localPose, transform.Orientation, out worldPose);
            //TODO: This is an area that has to be updated for high precision poses. May be able to centralize positional work
            //by deferring it until the final bounds scatter step. Would require looking up the position then, but could be worth simplicity.
            worldPose.Position += transform.Position;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeChildBounds(in CompoundChild child, in Quaternion orientation, Shapes shapeBatches, out Vector3 childMin, out Vector3 childMax)
        {
            GetRotatedChildPose(child.LocalPose, orientation, out var childPose);
            shapeBatches[child.ShapeIndex.Type].ComputeBounds(child.ShapeIndex.Index, childPose, out childMin, out childMax);
        }

        public void ComputeBounds(in Quaternion orientation, Shapes shapeBatches, out Vector3 min, out Vector3 max)
        {
            ComputeChildBounds(Children[0], orientation, shapeBatches, out min, out max);
            for (int i = 1; i < Children.Length; ++i)
            {
                ref var child = ref Children[i];
                ComputeChildBounds(Children[i], orientation, shapeBatches, out var childMin, out var childMax);
                BoundingBox.CreateMerged(min, max, childMin, childMax, out min, out max);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void AddChildBoundsToBatcher(ref Buffer<CompoundChild> children, ref BoundingBoxBatcher batcher, in RigidPose pose, in BodyVelocity velocity, int bodyIndex)
        {
            //Note that this approximates the velocity of the child using a piecewise extrapolation using the parent's angular velocity.
            //For significant angular velocities, this is actually wrong, but this is how v1 worked forever and it's cheap.
            //May want to revisit this later- it would likely require that the BoundingBoxBatcher have a continuation, or to include more information
            //for the convex path to condition on.
            BodyVelocity childVelocity;
            childVelocity.Angular = velocity.Angular;
            for (int i = 0; i < children.Length; ++i)
            {
                ref var child = ref children[i];
                GetRotatedChildPose(child.LocalPose, pose.Orientation, out var childPose);
                var angularContributionToChildLinear = Vector3.Cross(velocity.Angular, childPose.Position);
                var contributionLengthSquared = angularContributionToChildLinear.LengthSquared();
                var localPoseRadiusSquared = childPose.Position.LengthSquared();
                if (contributionLengthSquared > localPoseRadiusSquared)
                {
                    angularContributionToChildLinear *= (float)(Math.Sqrt(localPoseRadiusSquared) / Math.Sqrt(contributionLengthSquared));
                }
                childVelocity.Linear = velocity.Linear + angularContributionToChildLinear;
                childPose.Position += pose.Position;
                batcher.AddCompoundChild(bodyIndex, children[i].ShapeIndex, childPose, childVelocity);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddChildBoundsToBatcher(ref BoundingBoxBatcher batcher, in RigidPose pose, in BodyVelocity velocity, int bodyIndex)
        {
            AddChildBoundsToBatcher(ref Children, ref batcher, pose, velocity, bodyIndex);
        }

        public bool RayTest(in RigidPose pose, in Vector3 origin, in Vector3 direction, float maximumT, Shapes shapeBatches, out float t, out Vector3 normal)
        {
            t = float.MaxValue;
            normal = new Vector3();
            for (int i = 0; i < Children.Length; ++i)
            {
                ref var child = ref Children[i];
                GetRotatedChildPose(child.LocalPose, pose.Orientation, out var childPose);
                //TODO: This is an area that has to be updated for high precision poses.
                childPose.Position += pose.Position;
                if (shapeBatches[child.ShapeIndex.Type].RayTest(child.ShapeIndex.Index, childPose, origin, direction, maximumT, out var childT, out var childNormal) && childT < t)
                {
                    t = childT;
                    normal = childNormal;
                }
            }
            return t < float.MaxValue;
        }

        public void RayTest<TRayHitHandler>(in RigidPose pose, Shapes shapeBatches, ref RaySource rays, ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayBatchHitHandler
        {
            for (int i = 0; i < Children.Length; ++i)
            {
                ref var child = ref Children[i];
                GetRotatedChildPose(child.LocalPose, pose.Orientation, out var childPose);
                //TODO: This is an area that has to be updated for high precision poses.
                childPose.Position += pose.Position;
                //Note that this will report an impact for every child, even if it's not the first impact.
                shapeBatches[child.ShapeIndex.Type].RayTest(child.ShapeIndex.Index, childPose, ref rays, ref hitHandler);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ShapeBatch CreateShapeBatch(BufferPool pool, int initialCapacity, Shapes shapes)
        {
            return new CompoundShapeBatch<Compound>(pool, initialCapacity, shapes);
        }

        public int ChildCount
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return Children.Length; }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref CompoundChild GetChild(int compoundChildIndex)
        {
            return ref Children[compoundChildIndex];
        }

        public unsafe void FindLocalOverlaps<TOverlaps, TSubpairOverlaps>(ref Buffer<OverlapQueryForPair> pairs, BufferPool pool, Shapes shapes, ref TOverlaps overlaps)
            where TOverlaps : struct, ICollisionTaskOverlaps<TSubpairOverlaps>
            where TSubpairOverlaps : struct, ICollisionTaskSubpairOverlaps
        {
            for (int pairIndex = 0; pairIndex < pairs.Length; ++pairIndex)
            {
                ref var pair = ref pairs[pairIndex];
                ref var compound = ref Unsafe.AsRef<Compound>(pair.Container);
                ref var overlapsForPair = ref overlaps.GetOverlapsForPair(pairIndex);
                for (int i = 0; i < compound.Children.Length; ++i)
                {
                    ref var child = ref compound.Children[i];
                    //TODO: This does quite a bit of work. May want to try a simple bounding sphere instead (based on a dedicated maximum radius request).
                    shapes[child.ShapeIndex.Type].ComputeBounds(child.ShapeIndex.Index, child.LocalPose.Orientation, out _, out _, out var min, out var max);
                    min += child.LocalPose.Position;
                    max += child.LocalPose.Position;
                    if (BoundingBox.Intersects(min, max, pair.Min, pair.Max))
                    {
                        overlapsForPair.Allocate(pool) = i;
                    }
                }
            }
        }

        public unsafe void FindLocalOverlaps<TOverlaps>(in Vector3 min, in Vector3 max, in Vector3 sweep, float maximumT, BufferPool pool, Shapes shapes, void* overlapsPointer)
            where TOverlaps : ICollisionTaskSubpairOverlaps
        {
            Tree.ConvertBoxToCentroidWithExtent(min, max, out var sweepOrigin, out var expansion);
            TreeRay.CreateFrom(sweepOrigin, sweep, maximumT, out var ray);
            ref var overlaps = ref Unsafe.AsRef<TOverlaps>(overlapsPointer);
            for (int i = 0; i < Children.Length; ++i)
            {
                ref var child = ref Children[i];
                shapes[child.ShapeIndex.Type].ComputeBounds(child.ShapeIndex.Index, child.LocalPose.Orientation, out _, out _, out var childMin, out var childMax);
                childMin = childMin + child.LocalPose.Position - expansion;
                childMax = childMax + child.LocalPose.Position + expansion;
                if (Tree.Intersects(childMin, childMax, &ray, out _))
                {
                    overlaps.Allocate(pool) = i;
                }
            }

        }
        
        public void Dispose(BufferPool bufferPool)
        {
            bufferPool.Return(ref Children);
        }

        /// <summary>
        /// Type id of list based compound shapes.
        /// </summary>
        public const int Id = 6;
        public int TypeId { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return Id; } }
    }


}
