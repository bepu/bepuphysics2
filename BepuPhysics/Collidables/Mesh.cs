﻿using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuPhysics.Trees;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
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
    /// Shape designed to contain a whole bunch of triangles. Triangle collisions and ray tests are one-sided; only tests which see the triangle as wound clockwise in right handed coordinates or counterclockwise in left handed coordinates will generate contacts.
    /// </summary>
    public struct Mesh : IHomogeneousCompoundShape<Triangle, TriangleWide>
    {
        /// <summary>
        /// Acceleration structure of the mesh.
        /// </summary>
        public Tree Tree;
        /// <summary>
        /// Buffer of triangles composing the mesh. Triangles will only collide with tests which see the triangle as wound clockwise in right handed coordinates or counterclockwise in left handed coordinates.
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
        /// Fills a buffer of subtrees according to a buffer of triangles.
        /// </summary>
        /// <remarks>The term "subtree" is used because the binned builder does not care whether the input came from leaf nodes or a refinement process's internal nodes.</remarks>
        /// <param name="triangles">Triangles to build subtrees from.</param>
        /// <param name="subtrees">Subtrees created for the triangles.</param>
        public static void FillSubtreesForTriangles(Span<Triangle> triangles, Span<NodeChild> subtrees)
        {
            if (subtrees.Length != triangles.Length)
                throw new ArgumentException("Triangles and subtrees span lengths should match.");
            for (int i = 0; i < triangles.Length; ++i)
            {
                ref var t = ref triangles[i];
                ref var subtree = ref subtrees[i];
                subtree.Min = Vector3.Min(t.A, Vector3.Min(t.B, t.C));
                subtree.Max = Vector3.Max(t.A, Vector3.Max(t.B, t.C));
                subtree.LeafCount = 1;
                subtree.Index = Tree.Encode(i);
            }
        }

        /// <summary>
        /// Creates a mesh shape instance, but leaves the Tree in an unbuilt state. The tree must be built before the mesh can be used.
        /// </summary>
        /// <param name="triangles">Triangles to use in the mesh.</param>
        /// <param name="scale">Scale to apply to all vertices at runtime.
        /// Note that the scale is not baked into the triangles or acceleration structure; the same set of triangles and acceleration structure can be used across multiple Mesh instances with different scales.</param>
        /// <param name="pool">Pool used to allocate acceleration structures.</param>
        /// <returns>Created mesh shape.</returns>
        /// <remarks>In some cases, the default binned build may not be the ideal builder. This function does everything needed to set up a tree without the expense of figuring out the details of the acceleration structure.
        /// The user can then run whatever build/refinement process is appropriate.</remarks>
        public static Mesh CreateWithoutTreeBuild(Buffer<Triangle> triangles, Vector3 scale, BufferPool pool)
        {
            Mesh mesh = default;
            mesh.Triangles = triangles;
            mesh.Tree = new Tree(pool, triangles.Length)
            {
                //If this codepath is being used, we're assuming that the triangles are going to be the actual children
                //so we can go ahead and set the node/leaf counts.
                //(This is in contrast to creating a tree with a certain capacity, but then relying on incremental adds/removes later.)
                //Note that the tree still has a root node even if there's one leaf; it's a partial node and requires special handling.
                NodeCount = int.Max(1, triangles.Length - 1),
                LeafCount = triangles.Length
            };
            mesh.Scale = scale;
            return mesh;
        }

        /// <summary>
        /// Creates a mesh shape instance and builds an acceleration structure using a sweep builder.
        /// </summary>
        /// <param name="triangles">Triangles to use in the mesh.</param>
        /// <param name="scale">Scale to apply to all vertices at runtime.
        /// Note that the scale is not baked into the triangles or acceleration structure; the same set of triangles and acceleration structure can be used across multiple Mesh instances with different scales.</param>
        /// <param name="pool">Pool used to allocate acceleration structures.</param>
        /// <returns>Created mesh shape.</returns>
        /// <remarks>The sweep builder is significantly slower than the binned builder, but can sometimes create higher quality trees.
        /// <para>Note that the binned builder can be tuned to create higher quality trees. That is usually a better choice than trying to use the sweep builder; this is here primarily for legacy reasons.</para></remarks>
        public unsafe static Mesh CreateWithSweepBuild(Buffer<Triangle> triangles, Vector3 scale, BufferPool pool)
        {
            var mesh = CreateWithoutTreeBuild(triangles, scale, pool);
            pool.Take<NodeChild>(triangles.Length, out var subtrees);
            FillSubtreesForTriangles(triangles, subtrees);
            Debug.Assert(sizeof(BoundingBox) == sizeof(NodeChild),
                "This assumption *should* hold, because the binned builder relies on it. If it doesn't, something weird as happened." +
                "Did you forget about this requirement when revamping for 64 bit or something?");
            //NodeChild intentionally shares the same memory layout as BoundingBox. NodeChild just includes some extra data in the fields unused by bounds.
            mesh.Tree.SweepBuild(pool, subtrees.As<BoundingBox>());
            pool.Return(ref subtrees);
            return mesh;
        }

        /// <summary>
        /// Creates a mesh shape.
        /// </summary>
        /// <param name="triangles">Triangles to use in the mesh.</param>
        /// <param name="scale">Scale to apply to all vertices at runtime.
        /// Note that the scale is not baked into the triangles or acceleration structure; the same set of triangles and acceleration structure can be used across multiple Mesh instances with different scales.</param>
        /// <param name="pool">Pool used to allocate acceleration structures.</param>
        /// <param name="dispatcher">Dispatcher to use to multithread the execution of the mesh build process. If null, the build will be single threaded.</param>
        public unsafe Mesh(Buffer<Triangle> triangles, Vector3 scale, BufferPool pool, IThreadDispatcher dispatcher = null)
        {
            this = CreateWithoutTreeBuild(triangles, scale, pool);
            pool.Take<NodeChild>(triangles.Length, out var subtrees);
            FillSubtreesForTriangles(triangles, subtrees);
            Tree.BinnedBuild(subtrees, pool, dispatcher);
            pool.Return(ref subtrees);
        }

        /// <summary>
        /// Loads a mesh from data stored in a byte buffer previously stored by the Serialize function.
        /// </summary>
        /// <param name="data">Data to load the mesh from.</param>
        /// <param name="pool">Pool to create the mesh with.</param>
        public unsafe Mesh(Span<byte> data, BufferPool pool)
        {
            if (data.Length < 16)
                throw new ArgumentException("Data is not large enough to contain a header.");
            this = default;
            Scale = Unsafe.As<byte, Vector3>(ref data[0]);
            var triangleCount = Unsafe.As<byte, int>(ref data[12]);
            var triangleByteCount = triangleCount * sizeof(Triangle);
            if (data.Length < 4 + triangleCount * sizeof(Triangle))
                throw new ArgumentException($"Data is not large enough to contain the number of triangles specified in the header, {triangleCount}.");
            Tree = new Tree(data.Slice(16 + triangleByteCount, data.Length - 16 - triangleByteCount), pool);
            pool.Take(triangleCount, out Triangles);
            Unsafe.CopyBlockUnaligned(ref *(byte*)Triangles.Memory, ref data[16], (uint)triangleByteCount);
        }

        /// <summary>
        /// Gets the number of bytes it would take to store the given mesh in a byte buffer.
        /// </summary>
        /// <returns>Number of bytes it would take to store the mesh.</returns>
        public readonly unsafe int GetSerializedByteCount()
        {
            return 16 + Triangles.Length * sizeof(Triangle) + Tree.GetSerializedByteCount();
        }

        /// <summary>
        /// Writes a mesh's data to a byte buffer.
        /// </summary>
        /// <param name="data">Byte buffer to store the mesh in.</param>
        public readonly unsafe void Serialize(Span<byte> data)
        {
            var requiredSizeInBytes = GetSerializedByteCount();
            if (data.Length < requiredSizeInBytes)
                throw new ArgumentException($"Target span size {data.Length} is less than the required size of {requiredSizeInBytes}.");
            Unsafe.As<byte, Vector3>(ref data[0]) = scale;
            Unsafe.As<byte, int>(ref data[12]) = Triangles.Length;
            var triangleByteCount = Triangles.Length * sizeof(Triangle);
            Unsafe.CopyBlockUnaligned(ref data[16], ref *(byte*)Triangles.Memory, (uint)triangleByteCount);
            Tree.Serialize(data.Slice(16 + triangleByteCount, data.Length - 16 - triangleByteCount));
        }

        public readonly int ChildCount => Triangles.Length;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public readonly void GetLocalChild(int triangleIndex, out Triangle target)
        {
            ref var source = ref Triangles[triangleIndex];
            target.A = scale * source.A;
            target.B = scale * source.B;
            target.C = scale * source.C;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public readonly void GetPosedLocalChild(int triangleIndex, out Triangle target, out RigidPose childPose)
        {
            GetLocalChild(triangleIndex, out target);
            childPose = (target.A + target.B + target.C) * (1f / 3f);
            target.A -= childPose.Position;
            target.B -= childPose.Position;
            target.C -= childPose.Position;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public readonly void GetLocalChild(int triangleIndex, ref TriangleWide target)
        {
            //This inserts a triangle into the first slot of the given wide instance.
            ref var source = ref Triangles[triangleIndex];
            Vector3Wide.WriteFirst(source.A * scale, ref target.A);
            Vector3Wide.WriteFirst(source.B * scale, ref target.B);
            Vector3Wide.WriteFirst(source.C * scale, ref target.C);
        }

        public readonly void ComputeBounds(Quaternion orientation, out Vector3 min, out Vector3 max)
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

        public static ShapeBatch CreateShapeBatch(BufferPool pool, int initialCapacity, Shapes shapeBatches)
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
            public void TestLeaf(int leafIndex, RayData* rayData, float* maximumT)
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
        public readonly unsafe void RayTest<TRayHitHandler>(in RigidPose pose, in RayData ray, ref float maximumT, ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayHitHandler
        {
            HitLeafTester<TRayHitHandler> leafTester;
            leafTester.Triangles = Triangles.Memory;
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
        public readonly unsafe void RayTest<TRayHitHandler>(in RigidPose pose, ref RaySource rays, ref TRayHitHandler hitHandler) where TRayHitHandler : struct, IShapeRayHitHandler
        {
            HitLeafTester<TRayHitHandler> leafTester;
            leafTester.Triangles = Triangles.Memory;
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

        public readonly unsafe void FindLocalOverlaps<TOverlaps, TSubpairOverlaps>(ref Buffer<OverlapQueryForPair> pairs, BufferPool pool, Shapes shapes, ref TOverlaps overlaps)
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
                //Take a min/max to compensate for negative scales.
                mesh.Tree.GetOverlaps(Vector3.Min(scaledMin, scaledMax), Vector3.Max(scaledMin, scaledMax), ref enumerator);
            }
        }

        public readonly unsafe void FindLocalOverlaps<TOverlaps>(Vector3 min, Vector3 max, Vector3 sweep, float maximumT, BufferPool pool, Shapes shapes, void* overlaps)
            where TOverlaps : ICollisionTaskSubpairOverlaps
        {
            var scaledMin = min * inverseScale;
            var scaledMax = max * inverseScale;
            var scaledSweep = sweep * inverseScale;
            ShapeTreeSweepLeafTester<TOverlaps> enumerator;
            enumerator.Pool = pool;
            enumerator.Overlaps = overlaps;
            //Take a min/max to compensate for negative scales.
            Tree.Sweep(Vector3.Min(scaledMin, scaledMax), Vector3.Max(scaledMin, scaledMax), scaledSweep, maximumT, ref enumerator);
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
        public void Recenter(Vector3 newCenter)
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
                ref var node = ref Tree.Nodes[i];
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
        /// <param name="center">Center of the closed mesh.</param>
        /// <returns>Inertia tensor of the closed mesh.</returns>
        public BodyInertia ComputeClosedInertia(float mass, out Vector3 center)
        {
            var triangleSource = new MeshTriangleSource(this);
            MeshInertiaHelper.ComputeClosedInertia(ref triangleSource, mass, out _, out var inertiaTensor, out center);
            MeshInertiaHelper.GetInertiaOffset(mass, center, out var inertiaOffset);
            Symmetric3x3.Add(inertiaTensor, inertiaOffset, out var recenteredInertia);
            Recenter(center);
            BodyInertia inertia;
            Symmetric3x3.Invert(recenteredInertia, out inertia.InverseInertiaTensor);
            inertia.InverseMass = 1f / mass;
            return inertia;
        }

        /// <summary>
        /// Computes the inertia of the mesh.
        /// Assumes the mesh is closed and should be treated as solid.
        /// </summary>
        /// <param name="mass">Mass to scale the inertia tensor with.</param>
        /// <returns>Inertia tensor of the closed mesh.</returns>
        public readonly BodyInertia ComputeClosedInertia(float mass)
        {
            var triangleSource = new MeshTriangleSource(this);
            MeshInertiaHelper.ComputeClosedInertia(ref triangleSource, mass, out _, out var inertiaTensor);
            BodyInertia inertia;
            inertia.InverseMass = 1f / mass;
            Symmetric3x3.Invert(inertiaTensor, out inertia.InverseInertiaTensor);
            return inertia;
        }

        /// <summary>
        /// Computes the volume and center of mass of the mesh. Assumes the mesh is closed and should be treated as solid.
        /// </summary>
        /// <param name="volume">Volume of the closed mesh.</param>
        /// <param name="center">Center of mass of the closed mesh.</param>
        public readonly void ComputeClosedCenterOfMass(out float volume, out Vector3 center)
        {
            var triangleSource = new MeshTriangleSource(this);
            MeshInertiaHelper.ComputeClosedCenterOfMass(ref triangleSource, out volume, out center);
        }

        /// <summary>
        /// Computes the center of mass of the mesh.
        /// Assumes the mesh is closed and should be treated as solid.
        /// </summary>
        /// <returns>Center of mass of the closed mesh.</returns>
        public readonly Vector3 ComputeClosedCenterOfMass()
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
        /// <param name="center">Center of the open mesh.</param>
        /// <returns>Inertia tensor of the closed mesh.</returns>
        public BodyInertia ComputeOpenInertia(float mass, out Vector3 center)
        {
            var triangleSource = new MeshTriangleSource(this);
            MeshInertiaHelper.ComputeOpenInertia(ref triangleSource, mass, out var inertiaTensor, out center);
            MeshInertiaHelper.GetInertiaOffset(mass, center, out var inertiaOffset);
            Symmetric3x3.Add(inertiaTensor, inertiaOffset, out var recenteredInertia);
            Recenter(center);
            BodyInertia inertia;
            Symmetric3x3.Invert(recenteredInertia, out inertia.InverseInertiaTensor);
            inertia.InverseMass = 1f / mass;
            return inertia;
        }

        /// <summary>
        /// Computes the inertia of the mesh.
        /// Assumes the mesh is open and should be treated as a triangle soup.
        /// </summary>
        /// <param name="mass">Mass to scale the inertia tensor with.</param>
        /// <returns>Inertia of the open mesh.</returns>
        public readonly BodyInertia ComputeOpenInertia(float mass)
        {
            var triangleSource = new MeshTriangleSource(this);
            MeshInertiaHelper.ComputeOpenInertia(ref triangleSource, mass, out var inertiaTensor);
            BodyInertia inertia;
            inertia.InverseMass = 1f / mass;
            Symmetric3x3.Invert(inertiaTensor, out inertia.InverseInertiaTensor);
            return inertia;
        }

        /// <summary>
        /// Computes the center of mass of the mesh.
        /// Assumes the mesh is open and should be treated as a triangle soup.
        /// </summary>
        /// <returns>Center of mass of the open mesh.</returns>
        public readonly Vector3 ComputeOpenCenterOfMass()
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
        public static int TypeId => Id;

    }
}
