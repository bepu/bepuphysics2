using BepuPhysics.Trees;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;

namespace BepuPhysics.Collidables
{
    public struct Mesh : IMeshShape
    {
        public Tree Tree;
        public Buffer<Triangle> Triangles;
        internal Vector3 scale;
        internal Vector3 inverseScale;
        public Vector3 Scale
        {
            get
            {
                return scale;
            }
            set
            {
                Debug.Assert(value.X != 0 && value.Y != 0 && value.Z != 0, "All components of scale must be nonzero.");
                scale = value;
                inverseScale = Vector3.One / value;
            }
        }

        public Mesh(Buffer<Triangle> triangles, Vector3 scale, BufferPool pool) : this()
        {
            Triangles = triangles;
            Tree = new Tree(pool, triangles.Length);
            pool.Take<BoundingBox>(triangles.Length, out var boundingBoxes);
            for (int i = 0; i < triangles.Length; ++i)
            {
                ref var t = ref triangles[i];
                ref var bounds = ref boundingBoxes[i];
                bounds.Min = Vector3.Min(t.A, Vector3.Min(t.B, t.C));
                bounds.Max = Vector3.Max(t.A, Vector3.Max(t.B, t.C));
            }
            Tree.SweepBuild(pool, boundingBoxes);
            Scale = scale;
        }

        public void ComputeBounds(in BepuUtilities.Quaternion orientation, out Vector3 min, out Vector3 max)
        {
            Matrix3x3.CreateFromQuaternion(orientation, out var r);
            min = new Vector3(float.MaxValue);
            max = new Vector3(-float.MaxValue);
            for (int i = 0; i < Triangles.Length; ++i)
            {
                //This isn't an ideal bounding box calculation for a mesh. 
                //-You might be able to get a win out of widely vectorizing.
                //-Indexed smooth meshes would tend to have a third as many max/min operations.
                //-Even better would be a set of extreme points that are known to fully enclose the mesh, eliminating the need to test the vast majority.
                //But optimizing this only makes sense if dynamic meshes are common, and they really, really, really should not be.
                ref var triangle = ref Triangles[i];
                Matrix3x3.Transform(triangle.A, r, out var a);
                Matrix3x3.Transform(triangle.B, r, out var b);
                Matrix3x3.Transform(triangle.C, r, out var c);
                var min0 = Vector3.Min(a, b);
                var min1 = Vector3.Min(c, min);
                var max0 = Vector3.Max(a, b);
                var max1 = Vector3.Max(c, max);
                min = Vector3.Min(min0, min1);
                max = Vector3.Max(max0, max1);
            }
        }

        public ShapeBatch CreateShapeBatch(BufferPool pool, int initialCapacity, Shapes shapeBatches)
        {
            return new MeshShapeBatch<Mesh>(pool, initialCapacity);
        }

        public bool RayTest(in RigidPose pose, in Vector3 origin, in Vector3 direction, out float t, out Vector3 normal)
        {
            t = 0;
            normal = default;
            return false;
        }

        public void RayTest<TRayHitHandler>(RigidPose pose, ref RaySource rays, ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayHitHandler
        {
        }

        public void FindOverlaps(in Vector3 min, in Vector3 max, BufferPool pool, out QuickList<Triangle, Buffer<Triangle>> overlaps)
        {
            overlaps = default;
        }

        public void FindOverlaps(ref Buffer<IntPtr> meshes, in Vector3Wide min, in Vector3Wide max, int count, BufferPool pool, ref Buffer<QuickList<Triangle, Buffer<Triangle>>> overlaps, ref Buffer<QuickList<int, Buffer<int>>> childIndices)
        {
            for (int i = 0; i < count; ++i)
            {
                overlaps[i] = default;
                childIndices[i] = default;
            }
        }
        public int TypeId => 6;
    }
}
