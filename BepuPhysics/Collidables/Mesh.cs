using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuPhysics.Trees;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using Quaternion = BepuUtilities.Quaternion;
namespace BepuPhysics.Collidables
{
    public unsafe struct ShapeTreeOverlapEnumerator<TSubpairOverlaps> : IBreakableForEach<int> where TSubpairOverlaps : ICollisionTaskSubpairOverlaps
    {
        public BufferPool Pool;
        public void* Overlaps;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool LoopBody(int i)
        {
            Unsafe.AsRef<TSubpairOverlaps>(Overlaps).Allocate(Pool) = i;
            return true;
        }
    }
    public unsafe struct ShapeTreeSweepLeafTester<TOverlaps> : ISweepLeafTester where TOverlaps : ICollisionTaskSubpairOverlaps
    {
        public BufferPool Pool;
        public void* Overlaps;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void TestLeaf(int leafIndex, ref float maximumT)
        {
            Unsafe.AsRef<TOverlaps>(Overlaps).Allocate(Pool) = leafIndex;
        }
    }

    /// <summary>
    /// Shape designed to contain a whole bunch of triangles. Triangle collisions and ray tests are one-sided; only tests which see the triangle as wound clockwise will generate contacts.
    /// </summary>
    public struct Mesh : IHomogeneousCompoundShape<Triangle, TriangleWide>
    {
        /// <summary>
        /// Acceleration structure of the mesh.
        /// </summary>
        public Tree Tree;
        /// <summary>
        /// Buffer of triangles composing the mesh. Triangles will only collide with tests which see the triangle as wound clockwise.
        /// </summary>
        public Buffer<Triangle> Triangles;
        internal Vector3 scale;
        internal Vector3 inverseScale;
        /// <summary>
        /// Gets or sets the scale of the mesh.
        /// </summary>
        public Vector3 Scale
        {
            get
            {
                return scale;
            }
            set
            {
                scale = value;
                inverseScale = new Vector3(
                    value.X != 0 ? 1f / value.X : float.MaxValue,
                    value.Y != 0 ? 1f / value.Y : float.MaxValue,
                    value.Z != 0 ? 1f / value.Z : float.MaxValue);
            }
        }

        /// <summary>
        /// Creates a mesh shape.
        /// </summary>
        /// <param name="triangles">Triangles to use in the mesh.</param>
        /// <param name="scale">Scale to apply to all vertices at runtime.
        /// Note that the scale is not baked into the triangles or acceleration structure; the same set of triangles and acceleration structure can be used across multiple Mesh instances with different scales.</param>
        /// <param name="pool">Pool used to allocate acceleration structures.</param>
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
            pool.Return(ref boundingBoxes);
            Scale = scale;
        }

        public int ChildCount => Triangles.Length;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void GetLocalChild(int triangleIndex, out Triangle target)
        {
            ref var source = ref Triangles[triangleIndex];
            target.A = scale * source.A;
            target.B = scale * source.B;
            target.C = scale * source.C;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void GetPosedLocalChild(int triangleIndex, out Triangle target, out RigidPose childPose)
        {
            GetLocalChild(triangleIndex, out target);
            childPose = new RigidPose((target.A + target.B + target.C) * (1f / 3f));
            target.A -= childPose.Position;
            target.B -= childPose.Position;
            target.C -= childPose.Position;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void GetLocalChild(int triangleIndex, ref TriangleWide target)
        {
            //This inserts a triangle into the first slot of the given wide instance.
            ref var source = ref Triangles[triangleIndex];
            Vector3Wide.WriteFirst(source.A * scale, ref target.A);
            Vector3Wide.WriteFirst(source.B * scale, ref target.B);
            Vector3Wide.WriteFirst(source.C * scale, ref target.C);
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
                Matrix3x3.Transform(scale * triangle.A, r, out var a);
                Matrix3x3.Transform(scale * triangle.B, r, out var b);
                Matrix3x3.Transform(scale * triangle.C, r, out var c);
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
            return new HomogeneousCompoundShapeBatch<Mesh, Triangle, TriangleWide>(pool, initialCapacity);
        }

        unsafe struct HitLeafTester<T> : IRayLeafTester where T : IShapeRayHitHandler
        {
            public Triangle* Triangles;
            public T HitHandler;
            public Matrix3x3 Orientation;
            public Vector3 InverseScale;
            public RayData OriginalRay;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe void TestLeaf(int leafIndex, RayData* rayData, float* maximumT)
            {
                ref var triangle = ref Triangles[leafIndex];
                if (Triangle.RayTest(triangle.A, triangle.B, triangle.C, rayData->Origin, rayData->Direction, out var t, out var normal) && t <= *maximumT)
                {
                    //Pull the hit back into world space before handing it off to the user. This does cost a bit more, but not much, and you can always add a specialized no-transform path later.
                    Matrix3x3.Transform(normal * InverseScale, Orientation, out normal);
                    normal = Vector3.Normalize(normal);
                    HitHandler.OnRayHit(OriginalRay, ref *maximumT, t, normal, leafIndex);
                }
            }
        }

        /// <summary>
        /// Casts a ray against the mesh. Executes a callback for every test candidate and every hit.
        /// </summary>
        /// <typeparam name="TRayHitHandler">Type of the callback to execute for every test candidate and hit.</typeparam>
        /// <param name="pose">Pose of the mesh during the ray test.</param>
        /// <param name="ray">Ray to test against the mesh.</param>
        /// <param name="maximumT">Maximum length of the ray in units of the ray direction length.</param>
        /// <param name="hitHandler">Callback to execute for every hit.</param>
        public unsafe void RayTest<TRayHitHandler>(in RigidPose pose, in RayData ray, ref float maximumT, ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayHitHandler
        {
            HitLeafTester<TRayHitHandler> leafTester;
            leafTester.Triangles = (Triangle*)Triangles.Memory;
            leafTester.HitHandler = hitHandler;
            Matrix3x3.CreateFromQuaternion(pose.Orientation, out leafTester.Orientation);
            leafTester.InverseScale = inverseScale;
            leafTester.OriginalRay = ray;
            Matrix3x3.TransformTranspose(ray.Origin - pose.Position, leafTester.Orientation, out var localOrigin);
            Matrix3x3.TransformTranspose(ray.Direction, leafTester.Orientation, out var localDirection);
            localOrigin *= inverseScale;
            localDirection *= inverseScale;
            Tree.RayCast(localOrigin, localDirection, ref maximumT, ref leafTester);
            //The leaf tester could have mutated the hit handler; copy it back over.
            hitHandler = leafTester.HitHandler;
        }

        /// <summary>
        /// Casts a bunch of rays against the mesh at the same time, executing a callback for every test candidate and every hit.
        /// </summary>
        /// <typeparam name="TRayHitHandler">Type of the callback to execute for every ray test candidate and every hit.</typeparam>
        /// <param name="pose">Pose of the mesh during the ray test.</param>
        /// <param name="rays">Set of rays to cast against the mesh.</param>
        /// <param name="hitHandler">Callbacks to execute.</param>
        public unsafe void RayTest<TRayHitHandler>(in RigidPose pose, ref RaySource rays, ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayHitHandler
        {
            HitLeafTester<TRayHitHandler> leafTester;
            leafTester.Triangles = (Triangle*)Triangles.Memory;
            leafTester.HitHandler = hitHandler;
            Matrix3x3.CreateFromQuaternion(pose.Orientation, out leafTester.Orientation);
            Matrix3x3.Transpose(leafTester.Orientation, out var inverseOrientation);
            leafTester.InverseScale = inverseScale;
            for (int i = 0; i < rays.RayCount; ++i)
            {
                rays.GetRay(i, out var ray, out var maximumT);
                leafTester.OriginalRay = *ray;
                Matrix3x3.Transform(ray->Origin - pose.Position, inverseOrientation, out var localOrigin);
                Matrix3x3.Transform(ray->Direction, inverseOrientation, out var localDirection);
                localOrigin *= inverseScale;
                localDirection *= inverseScale;
                Tree.RayCast(localOrigin, localDirection, ref *maximumT, ref leafTester);
            }
            //The leaf tester could have mutated the hit handler; copy it back over.
            hitHandler = leafTester.HitHandler;
        }

        public unsafe void FindLocalOverlaps<TOverlaps, TSubpairOverlaps>(ref Buffer<OverlapQueryForPair> pairs, BufferPool pool, Shapes shapes, ref TOverlaps overlaps)
            where TOverlaps : struct, ICollisionTaskOverlaps<TSubpairOverlaps>
            where TSubpairOverlaps : struct, ICollisionTaskSubpairOverlaps
        {
            //For now, we don't use anything tricky. Just traverse every child against the tree sequentially.
            //TODO: This sequentializes a whole lot of cache misses. You could probably get some benefit out of traversing all pairs 'simultaneously'- that is, 
            //using the fact that we have lots of independent queries to ensure the CPU always has something to do.
            ShapeTreeOverlapEnumerator<TSubpairOverlaps> enumerator;
            enumerator.Pool = pool;
            for (int i = 0; i < pairs.Length; ++i)
            {
                ref var pair = ref pairs[i];
                ref var mesh = ref Unsafe.AsRef<Mesh>(pair.Container);
                var scaledMin = mesh.inverseScale * pair.Min;
                var scaledMax = mesh.inverseScale * pair.Max;
                enumerator.Overlaps = Unsafe.AsPointer(ref overlaps.GetOverlapsForPair(i));
                mesh.Tree.GetOverlaps(scaledMin, scaledMax, ref enumerator);
            }
        }

        public unsafe void FindLocalOverlaps<TOverlaps>(in Vector3 min, in Vector3 max, in Vector3 sweep, float maximumT, BufferPool pool, Shapes shapes, void* overlaps)
            where TOverlaps : ICollisionTaskSubpairOverlaps
        {
            var scaledMin = min * inverseScale;
            var scaledMax = max * inverseScale;
            var scaledSweep = sweep * inverseScale;
            ShapeTreeSweepLeafTester<TOverlaps> enumerator;
            enumerator.Pool = pool;
            enumerator.Overlaps = overlaps;
            Tree.Sweep(scaledMin, scaledMax, scaledSweep, maximumT, ref enumerator);
        }

        public struct MeshTriangleSource : ITriangleSource
        {
            Mesh mesh;
            int triangleIndex;

            public MeshTriangleSource(in Mesh mesh)
            {
                this.mesh = mesh;
                triangleIndex = 0;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool GetNextTriangle(out Vector3 a, out Vector3 b, out Vector3 c)
            {
                if (triangleIndex < mesh.Triangles.Length)
                {
                    ref var triangle = ref mesh.Triangles[triangleIndex++];
                    a = triangle.A * mesh.scale;
                    b = triangle.B * mesh.scale;
                    c = triangle.C * mesh.scale;
                    return true;
                }
                a = default;
                b = default;
                c = default;
                return false;
            }
        }

        /// <summary>
        /// Subtracts the newCenter from all points in the mesh hull.
        /// </summary>
        /// <param name="newCenter">New center that all points will be made relative to.</param>
        public unsafe void Recenter(in Vector3 newCenter)
        {
            var scaledOffset = newCenter * inverseScale;
            for (int i = 0; i < Triangles.Length; ++i)
            {
                ref var triangle = ref Triangles[i];
                triangle.A -= scaledOffset;
                triangle.B -= scaledOffset;
                triangle.C -= scaledOffset;
            }
            for (int i = 0; i < Tree.NodeCount; ++i)
            {
                ref var node = ref Tree.nodes[i];
                node.A.Min -= scaledOffset;
                node.A.Max -= scaledOffset;
                node.B.Min -= scaledOffset;
                node.B.Max -= scaledOffset;
            }
        }

        /// <summary>
        /// Computes the inertia of the mesh around its volumetric center and recenters the points of the mesh around it.
        /// Assumes the mesh is closed and should be treated as solid.
        /// </summary>
        /// <param name="mass">Mass to scale the inertia tensor with.</param>
        /// <param name="inertia">Inertia tensor of the closed mesh.</param>
        /// <param name="center">Center of the closed mesh.</param>
        public void ComputeClosedInertia(float mass, out BodyInertia inertia, out Vector3 center)
        {
            var triangleSource = new MeshTriangleSource(this);
            MeshInertiaHelper.ComputeClosedInertia(ref triangleSource, mass, out _, out var inertiaTensor, out center);
            MeshInertiaHelper.GetInertiaOffset(mass, center, out var inertiaOffset);
            Symmetric3x3.Add(inertiaTensor, inertiaOffset, out var recenteredInertia);
            Recenter(center);
            Symmetric3x3.Invert(recenteredInertia, out inertia.InverseInertiaTensor);
            inertia.InverseMass = 1f / mass;
        }

        /// <summary>
        /// Computes the inertia of the mesh.
        /// Assumes the mesh is closed and should be treated as solid.
        /// </summary>
        /// <param name="mass">Mass to scale the inertia tensor with.</param>
        /// <param name="inertia">Inertia of the closed mesh.</param>
        public void ComputeClosedInertia(float mass, out BodyInertia inertia)
        {
            var triangleSource = new MeshTriangleSource(this);
            MeshInertiaHelper.ComputeClosedInertia(ref triangleSource, mass, out _, out var inertiaTensor);
            inertia.InverseMass = 1f / mass;
            Symmetric3x3.Invert(inertiaTensor, out inertia.InverseInertiaTensor);
        }

        /// <summary>
        /// Computes the volume and center of mass of the mesh. Assumes the mesh is closed and should be treated as solid.
        /// </summary>
        /// <param name="volume">Volume of the closed mesh.</param>
        /// <param name="center">Center of mass of the closed mesh.</param>
        public void ComputeClosedCenterOfMass(out float volume, out Vector3 center)
        {
            var triangleSource = new MeshTriangleSource(this);
            MeshInertiaHelper.ComputeClosedCenterOfMass(ref triangleSource, out volume, out center);
        }

        /// <summary>
        /// Computes the center of mass of the mesh.
        /// Assumes the mesh is closed and should be treated as solid.
        /// </summary>
        /// <returns>Center of mass of the closed mesh.</returns>
        public Vector3 ComputeClosedCenterOfMass()
        {
            var triangleSource = new MeshTriangleSource(this);
            MeshInertiaHelper.ComputeClosedCenterOfMass(ref triangleSource, out _, out var center);
            return center;
        }

        /// <summary>
        /// Computes the inertia of the mesh around its volumetric center and recenters the points of the mesh around it.
        /// Assumes the mesh is open and should be treated as a triangle soup.
        /// </summary>
        /// <param name="mass">Mass to scale the inertia tensor with.</param>
        /// <param name="inertia">Inertia tensor of the closed mesh.</param>
        /// <param name="center">Center of the open mesh.</param>
        public void ComputeOpenInertia(float mass, out BodyInertia inertia, out Vector3 center)
        {
            var triangleSource = new MeshTriangleSource(this);
            MeshInertiaHelper.ComputeOpenInertia(ref triangleSource, mass, out var inertiaTensor, out center);
            MeshInertiaHelper.GetInertiaOffset(mass, center, out var inertiaOffset);
            Symmetric3x3.Add(inertiaTensor, inertiaOffset, out var recenteredInertia);
            Recenter(center);
            Symmetric3x3.Invert(recenteredInertia, out inertia.InverseInertiaTensor);
            inertia.InverseMass = 1f / mass;
        }

        /// <summary>
        /// Computes the inertia of the mesh.
        /// Assumes the mesh is open and should be treated as a triangle soup.
        /// </summary>
        /// <param name="mass">Mass to scale the inertia tensor with.</param>
        /// <param name="inertia">Inertia of the open mesh.</param>
        public void ComputeOpenInertia(float mass, out BodyInertia inertia)
        {
            var triangleSource = new MeshTriangleSource(this);
            MeshInertiaHelper.ComputeOpenInertia(ref triangleSource, mass, out var inertiaTensor);
            inertia.InverseMass = 1f / mass;
            Symmetric3x3.Invert(inertiaTensor, out inertia.InverseInertiaTensor);
        }

        /// <summary>
        /// Computes the center of mass of the mesh.
        /// Assumes the mesh is open and should be treated as a triangle soup.
        /// </summary>
        /// <returns>Center of mass of the open mesh.</returns>
        public Vector3 ComputeOpenCenterOfMass()
        {
            var triangleSource = new MeshTriangleSource(this);
            return MeshInertiaHelper.ComputeOpenCenterOfMass(ref triangleSource);
        }

        /// <summary>
        /// Returns the mesh's resources to a buffer pool.
        /// </summary>
        /// <param name="bufferPool">Pool to return the mesh's resources to.</param>
        public void Dispose(BufferPool bufferPool)
        {
            bufferPool.Return(ref Triangles);
            Tree.Dispose(bufferPool);
        }


        /// <summary>
        /// Type id of mesh shapes.
        /// </summary>
        public const int Id = 8;
        public int TypeId => Id;

    }
}
