using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
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

        public void ComputeBounds(in BepuUtilities.Quaternion orientation, out Vector3 min, out Vector3 max)
        {
            QuaternionWide.Broadcast(orientation, out var orientationWide);
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

        public int MinimumWideRayCount => 4;

        public bool AllowOffsetMemoryAccess => false;

        public void Broadcast(in ConvexHull shape)
        {
            throw new NotImplementedException();
        }

        public void GetBounds(ref QuaternionWide orientations, out Vector<float> maximumRadius, out Vector<float> maximumAngularExpansion, out Vector3Wide min, out Vector3Wide max)
        {
            throw new NotImplementedException();
        }

        public void RayTest(ref RigidPoses poses, ref RayWide rayWide, out Vector<int> intersected, out Vector<float> t, out Vector3Wide normal)
        {
            throw new NotImplementedException();
        }

        public void WriteFirst(in ConvexHull source)
        {
            Hulls[0] = source;
        }

        public void WriteSlot(int index, in ConvexHull source)
        {
            throw new NotImplementedException();
        }
    }

}
