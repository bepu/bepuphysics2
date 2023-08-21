﻿using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuUtilities.Memory;
using System.Diagnostics;
using BepuUtilities;
using BepuPhysics.Trees;
using BepuPhysics.CollisionDetection.CollisionTasks;
using System.Runtime.InteropServices;

namespace BepuPhysics.Collidables
{
    /// <summary>
    /// Shape and pose of a child within a compound shape.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct CompoundChild
    {
        /// <summary>
        /// Local orientation of the child in the compound.
        /// </summary>
        public Quaternion LocalOrientation;
        /// <summary>
        /// Local position of the child in the compound.
        /// </summary>
        public Vector3 LocalPosition;
        /// <summary>
        /// Index of the shape within whatever shape collection holds the compound's child shape data.
        /// </summary>
        public TypedIndex ShapeIndex;

        /// <summary>
        /// Reintreprets the 32 bytes of a compound child as a pose.
        /// </summary>
        /// <param name="child">Child to reinterpret.</param>
        /// <returns>Reference to the child as a pose.</returns>
        public static ref RigidPose AsPose(ref CompoundChild child) => ref Unsafe.As<CompoundChild, RigidPose>(ref child); //TODO: This could be made a little easier with UnscopedRef.
    }

    struct CompoundChildShapeTester : IShapeRayHitHandler
    {
        //We use a non-generic hit handler to capture the final result of a leaf test.
        //This requires caching out the T and Normal for reading by whatever ended up calling this, but it's worth it to avoid AOT pipelines barfing on infinite recursion.
        public float T;
        public Vector3 Normal;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowTest(int childIndex)
        {
            Debug.Assert(childIndex == 0, "Compounds can contain only convexes, so the child index is always zero.");
            //The actual test filtering took place in the TestLeaf function, where we call Handler.AllowTest.
            return true;
        }

        public void OnRayHit(in RayData ray, ref float maximumT, float t, Vector3 normal, int childIndex)
        {
            Debug.Assert(childIndex == 0, "Compounds can contain only convexes, so the child index is always zero.");
            T = t;
            Normal = normal;
        }
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

        /// <summary>
        /// Creates a compound shape with no acceleration structure.
        /// </summary>
        /// <param name="children">Set of children in the compound.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Compound(Buffer<CompoundChild> children)
        {
            Debug.Assert(children.Length > 0, "Compounds must have a nonzero number of children.");
            Children = children;
        }

        /// <summary>
        /// Checks if a shape index.
        /// </summary>
        /// <param name="shapeIndex">Shape index to analyze.</param>
        /// <param name="shapeBatches">Shape collection into which the index indexes.</param>
        /// <returns>True if the index is valid, false otherwise.</returns>
        public static bool ValidateChildIndex(TypedIndex shapeIndex, Shapes shapeBatches)
        {
            if (shapeIndex.Type < 0 || shapeIndex.Type >= shapeBatches.RegisteredTypeSpan)
            {
                Debug.Fail("Child shape type needs to fit within the shape batch registered types.");
                return false;
            }
            var batch = shapeBatches[shapeIndex.Type];
            if (shapeIndex.Index < 0 || shapeIndex.Index >= batch.Capacity)
            {
                Debug.Fail("Child shape index should point to a valid buffer location in the sahpe batch.");
                return false;
            }
            if (shapeBatches[shapeIndex.Type].Compound)
            {
                Debug.Fail("Child shape type should be convex.");
                return false;
            }
            //TODO: We don't have a cheap way to verify that a specific index actually contains a shape right now.
            return true;
        }

        /// <summary>
        /// Checks if a set of children shape indices are all valid.
        /// </summary>
        /// <param name="children">Children to examine.</param>
        /// <param name="shapeBatches">Shape collection into which the children index.</param>
        /// <returns>True if all child indices are valid, false otherwise.</returns>
        public static bool ValidateChildIndices(Span<CompoundChild> children, Shapes shapeBatches)
        {
            for (int i = 0; i < children.Length; ++i)
            {
                ValidateChildIndex(children[i].ShapeIndex, shapeBatches);
            }
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetRotatedChildPose(in RigidPose localPose, Quaternion orientation, out RigidPose rotatedChildPose)
        {
            GetRotatedChildPose(localPose.Position, localPose.Orientation, orientation, out rotatedChildPose.Position, out rotatedChildPose.Orientation);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetRotatedChildPose(Vector3 localPosition, Quaternion localOrientation, Quaternion orientation, out RigidPose rotatedChildPose)
        {
            GetRotatedChildPose(localPosition, localOrientation, orientation, out rotatedChildPose.Position, out rotatedChildPose.Orientation);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetRotatedChildPose(Vector3 localPosition, Quaternion localOrientation, Quaternion parentOrientation, out Vector3 rotatedPosition, out Quaternion rotatedOrientation)
        {
            QuaternionEx.ConcatenateWithoutOverlap(localOrientation, parentOrientation, out rotatedOrientation);
            QuaternionEx.Transform(localPosition, parentOrientation, out rotatedPosition);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetRotatedChildPose(in RigidPoseWide localPose, in QuaternionWide orientation, out Vector3Wide childPosition, out QuaternionWide childOrientation)
        {
            QuaternionWide.ConcatenateWithoutOverlap(localPose.Orientation, orientation, out childOrientation);
            QuaternionWide.TransformWithoutOverlap(localPose.Position, orientation, out childPosition);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetRotatedChildPose(in RigidPoseWide localPose, in QuaternionWide orientation, out RigidPoseWide rotatedChildPose)
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
        public static void ComputeChildBounds(in CompoundChild child, Quaternion orientation, Shapes shapeBatches, out Vector3 childMin, out Vector3 childMax)
        {
            GetRotatedChildPose(child.LocalPosition, child.LocalOrientation, orientation, out var childPose);
            Debug.Assert(!shapeBatches[child.ShapeIndex.Type].Compound, "All children of a compound must be convex.");
            shapeBatches[child.ShapeIndex.Type].ComputeBounds(child.ShapeIndex.Index, childPose, out childMin, out childMax);
        }

        public void ComputeBounds(Quaternion orientation, Shapes shapeBatches, out Vector3 min, out Vector3 max)
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
                GetRotatedChildPose(child.LocalPosition, child.LocalOrientation, pose.Orientation, out var childPose);
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

        public void RayTest<TRayHitHandler>(in RigidPose pose, in RayData ray, ref float maximumT, Shapes shapeBatches, ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayHitHandler
        {
            Matrix3x3.CreateFromQuaternion(pose.Orientation, out var orientation);
            RayData localRay;
            Matrix3x3.TransformTranspose(ray.Origin - pose.Position, orientation, out localRay.Origin);
            Matrix3x3.TransformTranspose(ray.Direction, orientation, out localRay.Direction);
            localRay.Id = 0;

            for (int i = 0; i < Children.Length; ++i)
            {
                if (hitHandler.AllowTest(i))
                {
                    ref var child = ref Children[i];
                    CompoundChildShapeTester tester;
                    tester.T = -1;
                    tester.Normal = default;
                    shapeBatches[child.ShapeIndex.Type].RayTest(child.ShapeIndex.Index, CompoundChild.AsPose(ref child), localRay, ref maximumT, ref tester);
                    if (tester.T >= 0)
                    {
                        Debug.Assert(maximumT >= tester.T, "Whatever generated this ray hit should have obeyed the current maximumT value.");
                        Matrix3x3.Transform(tester.Normal, orientation, out var rotatedNormal);
                        hitHandler.OnRayHit(ray, ref maximumT, tester.T, rotatedNormal, i);
                    }
                }
            }
        }

        public unsafe void RayTest<TRayHitHandler>(in RigidPose pose, ref RaySource rays, Shapes shapeBatches, ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayHitHandler
        {
            //TODO: Note that we dispatch a bunch of scalar tests here. You could be more clever than this- batched tests are possible.
            //It's relatively easy to do batching for this compound type since there is no hierarchy traversal, but we refactored things to avoid an infinite generic expansion issue in AOT compilation.
            //There are plenty of ways to work around that, but right now our batched raytracing implementation is bad enough that spending extra work here is questionable. We'll avoid breaking it for now, but that's all.
            for (int i = 0; i < rays.RayCount; ++i)
            {
                rays.GetRay(i, out var ray, out var maximumT);
                RayTest(pose, *ray, ref *maximumT, shapeBatches, ref hitHandler);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static ShapeBatch CreateShapeBatch(BufferPool pool, int initialCapacity, Shapes shapes)
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

        /// <summary>
        /// Adds a child to the compound.
        /// </summary>
        /// <param name="child">Child to add to the compound.</param>
        /// <param name="pool">Pool to use to resize the compound's children buffer if necessary.</param>
        public void Add(CompoundChild child, BufferPool pool)
        {
            pool.Resize(ref Children, Children.Length + 1, Children.Length);
            Children[^1] = child;
        }

        /// <summary>
        /// Removes a child from the compound by index. The last child is pulled to fill the gap left by the removed child.
        /// </summary>
        /// <param name="childIndex">Index of the child to remove from the compound.</param>
        /// <param name="pool">Pool to use to resize the compound's children buffer if necessary.</param>
        public void RemoveAt(int childIndex, BufferPool pool)
        {
            var lastIndex = Children.Length - 1;
            if (childIndex < lastIndex)
            {
                Children[childIndex] = Children[lastIndex];
            }
            //Shrinking the buffer takes care of 'removing' the now-empty last slot.
            pool.Resize(ref Children, Children.Length - 1, Children.Length - 1);
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
                    //Could also benefit from using the BoundingBox layout test, which is a little faster than 4 independent values.
                    shapes[child.ShapeIndex.Type].ComputeBounds(child.ShapeIndex.Index, child.LocalOrientation, out _, out _, out var min, out var max);
                    min += child.LocalPosition;
                    max += child.LocalPosition;
                    if (BoundingBox.Intersects(min, max, pair.Min, pair.Max))
                    {
                        overlapsForPair.Allocate(pool) = i;
                    }
                }
            }
        }

        public unsafe void FindLocalOverlaps<TOverlaps>(Vector3 min, Vector3 max, Vector3 sweep, float maximumT, BufferPool pool, Shapes shapes, void* overlapsPointer)
            where TOverlaps : ICollisionTaskSubpairOverlaps
        {
            Tree.ConvertBoxToCentroidWithExtent(min, max, out var sweepOrigin, out var expansion);
            TreeRay.CreateFrom(sweepOrigin, sweep, maximumT, out var ray);
            ref var overlaps = ref Unsafe.AsRef<TOverlaps>(overlapsPointer);
            for (int i = 0; i < Children.Length; ++i)
            {
                ref var child = ref Children[i];
                shapes[child.ShapeIndex.Type].ComputeBounds(child.ShapeIndex.Index, child.LocalOrientation, out _, out _, out var childMin, out var childMax);
                childMin = childMin + child.LocalPosition - expansion;
                childMax = childMax + child.LocalPosition + expansion;
                if (Tree.Intersects(childMin, childMax, &ray, out _))
                {
                    overlaps.Allocate(pool) = i;
                }
            }
        }

        /// <summary>
        /// Computes the inertia of a compound. Does not recenter the child poses.
        /// </summary>
        /// <param name="childMasses">Masses of the children.</param>
        /// <param name="shapes">Shapes collection containing the data for the compound child shapes.</param>
        /// <returns>Inertia of the compound.</returns>
        public BodyInertia ComputeInertia(Span<float> childMasses, Shapes shapes)
        {
            return CompoundBuilder.ComputeInertia(Children, childMasses, shapes);
        }

        /// <summary>
        /// Computes the inertia of a compound. Recenters the child poses around the calculated center of mass.
        /// </summary>
        /// <param name="shapes">Shapes collection containing the data for the compound child shapes.</param>
        /// <param name="childMasses">Masses of the children.</param>
        /// <param name="centerOfMass">Calculated center of mass of the compound. Subtracted from all the compound child poses.</param>
        /// <returns>Inertia of the compound.</returns>
        public BodyInertia ComputeInertia(Span<float> childMasses, Shapes shapes, out Vector3 centerOfMass)
        {
            return CompoundBuilder.ComputeInertia(Children, childMasses, shapes, out centerOfMass);
        }

        public void Dispose(BufferPool bufferPool)
        {
            bufferPool.Return(ref Children);
        }

        /// <summary>
        /// Type id of list based compound shapes.
        /// </summary>
        public const int Id = 6;
        public static int TypeId { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return Id; } }
    }


}
