using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Memory;

namespace BepuPhysics.Collidables
{

    public struct ConvexHull : IConvexShape
    {
        public Buffer<Vector3Wide> Points;

        //TODO: With platform intrinsics, we could improve the 'horizontal' parts of these functions.
        public void ComputeAngularExpansionData(out float maximumRadius, out float maximumAngularExpansion)
        {
            Vector<float> maximumRadiusSquaredWide = default;
            Vector<float> minimumRadiusSquaredWide = new Vector<float>(float.MaxValue);
            for (int i = 0; i < Points.Length; ++i)
            {
                Vector3Wide.LengthSquared(Points[i], out var candidate);
                maximumRadiusSquaredWide = Vector.Max(candidate, maximumRadiusSquaredWide);
                minimumRadiusSquaredWide = Vector.Min(candidate, minimumRadiusSquaredWide);
            }
            var minimumRadiusWide = Vector.SquareRoot(minimumRadiusSquaredWide);
            var maximumRadiusWide = Vector.SquareRoot(maximumRadiusSquaredWide);
            maximumRadius = maximumRadiusWide[0];
            float minimumRadius = minimumRadiusWide[0];
            for (int i = 1; i < Vector<float>.Count; ++i)
            {
                var maxCandidate = maximumRadiusWide[i];
                var minCandidate = minimumRadiusWide[i];
                if (maxCandidate > maximumRadius)
                    maximumRadius = maxCandidate;
                if (minCandidate < minimumRadius)
                    minimumRadius = minCandidate;
            }
            maximumAngularExpansion = maximumRadius - minimumRadius;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void ComputeBounds(in QuaternionWide orientationWide, out Vector3 min, out Vector3 max)
        {
            Matrix3x3Wide.CreateFromQuaternion(orientationWide, out var orientationMatrix);
            Vector3Wide minWide = default, maxWide = default;
            for (int i = 0; i < Points.Length; ++i)
            {
                Matrix3x3Wide.TransformWithoutOverlap(Points[i], orientationMatrix, out var p);
                Vector3Wide.Min(minWide, p, out minWide);
                Vector3Wide.Max(maxWide, p, out maxWide);
            }
            Vector3Wide.ReadFirst(minWide, out min);
            Vector3Wide.ReadFirst(maxWide, out max);
            for (int i = 1; i < Vector<float>.Count; ++i)
            {
                //TODO: Check codegen. Bounds checks elided?
                var minCandidate = new Vector3(minWide.X[i], minWide.Y[i], minWide.Z[i]);
                var maxCandidate = new Vector3(maxWide.X[i], maxWide.Y[i], maxWide.Z[i]);
                min = Vector3.Min(minCandidate, min);
                max = Vector3.Min(maxCandidate, max);
            }
        }

        public void ComputeBounds(in BepuUtilities.Quaternion orientation, out Vector3 min, out Vector3 max)
        {
            QuaternionWide.Broadcast(orientation, out var orientationWide);
            ComputeBounds(orientationWide, out min, out max);
        }

        public void ComputeInertia(float mass, out BodyInertia inertia)
        {
            //Consider each face as a set of triangles. Integrate over their volume to get the inertia. (The winding of the edges is consistent.)
            inertia = default;
        }

        public ShapeBatch CreateShapeBatch(BufferPool pool, int initialCapacity, Shapes shapeBatches)
        {
            return new ConvexShapeBatch<ConvexHull, ConvexHullWide>(pool, initialCapacity);
        }

        public bool RayTest(in RigidPose pose, in Vector3 origin, in Vector3 direction, out float t, out Vector3 normal)
        {
            normal = default;
            t = default;
            return false;
        }

        /// <summary>
        /// Type id of convex hull shapes.
        /// </summary>
        public const int Id = 5;
        public int TypeId { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return Id; } }
    }

    public struct ConvexHullWide : IShapeWide<ConvexHull>
    {
        //Unlike the other shapes, single convex hulls are internally vectorized.
        //The "wide" variant is simply a collection of convex hull instances.
        public Buffer<ConvexHull> Hulls;

        public int MinimumWideRayCount => int.MaxValue; //'Wide' ray tests just fall through to scalar tests anyway.

        public bool AllowOffsetMemoryAccess => false;

        public void Broadcast(in ConvexHull shape)
        {
            for (int i = 0; i < Hulls.Length; ++i)
                Hulls[i] = shape;
        }

        public void GetBounds(ref QuaternionWide orientations, out Vector<float> maximumRadius, out Vector<float> maximumAngularExpansion, out Vector3Wide min, out Vector3Wide max)
        {
            Vector<float> minimumRadius;
            for (int i = 0; i < Vector<float>.Count; ++i)
            {
                QuaternionWide.Rebroadcast(orientations, i, out var orientationWide);
                Matrix3x3Wide.CreateFromQuaternion(orientationWide, out var orientationMatrix);
                Vector3Wide minWide = default, maxWide = default;
                Vector<float> maximumRadiusSquaredWide = default;
                Vector<float> minimumRadiusSquaredWide = new Vector<float>(float.MaxValue);
                ref var hull = ref Hulls[i];
                for (int j = 0; j < hull.Points.Length; ++j)
                {
                    ref var localPoint = ref hull.Points[j];
                    Matrix3x3Wide.TransformWithoutOverlap(localPoint, orientationMatrix, out var p);
                    Vector3Wide.LengthSquared(localPoint, out var lengthSquared);
                    maximumRadiusSquaredWide = Vector.Max(lengthSquared, maximumRadiusSquaredWide);
                    minimumRadiusSquaredWide = Vector.Min(lengthSquared, minimumRadiusSquaredWide);
                    Vector3Wide.Min(minWide, p, out minWide);
                    Vector3Wide.Max(maxWide, p, out maxWide);
                }
                Vector3Wide.ReadFirst(minWide, out var minNarrow);
                Vector3Wide.ReadFirst(maxWide, out var maxNarrow);
                float maximumRadiusSquared = maximumRadiusSquaredWide[0];
                float minimumRadiusSquared = minimumRadiusSquaredWide[0];
                for (int j = 1; j < Vector<float>.Count; ++j)
                {
                    //TODO: Check codegen. Bounds checks elided?
                    var minCandidate = new Vector3(minWide.X[j], minWide.Y[j], minWide.Z[j]);
                    var maxCandidate = new Vector3(maxWide.X[j], maxWide.Y[j], maxWide.Z[j]);
                    minNarrow = Vector3.Min(minCandidate, minNarrow);
                    maxNarrow = Vector3.Min(maxCandidate, maxNarrow);

                    var maxRadiusCandidate = maximumRadiusSquaredWide[i];
                    var minRadiusCandidate = minimumRadiusSquaredWide[i];
                    if (maxRadiusCandidate > maximumRadiusSquared)
                        maximumRadiusSquared = maxRadiusCandidate;
                    if (minRadiusCandidate < minimumRadiusSquared)
                        minimumRadiusSquared = minRadiusCandidate;
                }
                GatherScatter.Get(ref maximumRadius, i) = maximumRadiusSquared;
                GatherScatter.Get(ref minimumRadius, i) = minimumRadiusSquared;
                Vector3Wide.WriteSlot(minNarrow, i, ref min);
                Vector3Wide.WriteSlot(maxNarrow, i, ref max);
            }
            minimumRadius = Vector.SquareRoot(minimumRadius);
            maximumRadius = Vector.SquareRoot(maximumRadius);
            maximumAngularExpansion = maximumRadius - minimumRadius;
        }

        public void RayTest(ref RigidPoses poses, ref RayWide rayWide, out Vector<int> intersected, out Vector<float> t, out Vector3Wide normal)
        {
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WriteFirst(in ConvexHull source)
        {
            Hulls[0] = source;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WriteSlot(int index, in ConvexHull source)
        {
            Hulls[index] = source;
        }
    }

    public struct ConvexHullSupportFinder : ISupportFinder<ConvexHull, ConvexHullWide>
    {
        public bool HasMargin => false;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeLocalSupport(in ConvexHullWide shape, in Vector3Wide direction, out Vector3Wide support)
        {
            for (int i = 0; i < Vector<float>.Count; ++i)
            {
                ref var hull = ref shape.Hulls[i];
                Vector3Wide.Rebroadcast(direction, i, out var slotDirection);
                var slotSupport = hull.Points[0];
                Vector3Wide.Dot(slotDirection, slotSupport, out var dot);
                for (int j = 1; j < hull.Points.Length; ++j)
                {
                    ref var candidate = ref hull.Points[j];
                    Vector3Wide.Dot(slotDirection, candidate, out var dotCandidate);
                    dot = Vector.Max(dot, dotCandidate);
                    Vector3Wide.ConditionalSelect(Vector.GreaterThan(dotCandidate, dot), candidate, slotSupport, out slotSupport);
                }

                var bestSlotIndex = 0;
                var bestSlotDot = dot[0];
                for (int j = 1; i < Vector<float>.Count; ++j)
                {
                    var candidate = dot[j];
                    if (candidate > bestSlotDot)
                    {
                        bestSlotDot = candidate;
                        bestSlotIndex = j;
                    }
                }
                Vector3Wide.CopySlot(ref slotSupport, bestSlotIndex, ref support, i);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeSupport(in ConvexHullWide shape, in Matrix3x3Wide orientation, in Vector3Wide direction, out Vector3Wide support)
        {
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(direction, orientation, out var localDirection);
            ComputeLocalSupport(shape, localDirection, out var localSupport);
            Matrix3x3Wide.Transform(localSupport, orientation, out support);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetMargin(in ConvexHullWide shape, out Vector<float> margin)
        {
            margin = default;
        }
    }


}
