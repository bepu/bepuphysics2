using BepuPhysics.Collidables;
using System;
using System.Numerics;
using BepuUtilities.Memory;
using DemoContentLoader;
using BepuUtilities;
using BepuPhysics.Trees;

namespace Demos
{
    public static class DemoMeshHelper
    {
        public static Mesh LoadModel(ContentArchive content, BufferPool pool, string contentName, Vector3 scaling)
        {
            var meshContent = content.Load<MeshContent>(contentName);
            pool.Take<Triangle>(meshContent.Triangles.Length, out var triangles);
            for (int i = 0; i < meshContent.Triangles.Length; ++i)
            {
                triangles[i] = new Triangle(meshContent.Triangles[i].A, meshContent.Triangles[i].B, meshContent.Triangles[i].C);
            }
            return new Mesh(triangles, scaling, pool);
        }

        public static Mesh CreateFan(int triangleCount, float radius, Vector3 scaling, BufferPool pool)
        {
            var anglePerTriangle = 2 * MathF.PI / triangleCount;
            pool.Take<Triangle>(triangleCount, out var triangles);

            for (int i = 0; i < triangleCount; ++i)
            {
                var firstAngle = i * anglePerTriangle;
                var secondAngle = ((i + 1) % triangleCount) * anglePerTriangle;

                ref var triangle = ref triangles[i];
                triangle.A = new Vector3(radius * MathF.Cos(firstAngle), 0, radius * MathF.Sin(firstAngle));
                triangle.B = new Vector3(radius * MathF.Cos(secondAngle), 0, radius * MathF.Sin(secondAngle));
                triangle.C = new Vector3();
            }
            return new Mesh(triangles, scaling, pool);
        }

        public static Mesh CreateDeformedPlane(int width, int height, Func<int, int, Vector3> deformer, Vector3 scaling, BufferPool pool, IThreadDispatcher dispatcher = null)
        {
            pool.Take<Vector3>(width * height, out var vertices);
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    vertices[width * j + i] = deformer(i, j);
                }
            }

            var quadWidth = width - 1;
            var quadHeight = height - 1;
            var triangleCount = quadWidth * quadHeight * 2;
            pool.Take<Triangle>(triangleCount, out var triangles);

            for (int i = 0; i < quadWidth; ++i)
            {
                for (int j = 0; j < quadHeight; ++j)
                {
                    var triangleIndex = (j * quadWidth + i) * 2;
                    ref var triangle0 = ref triangles[triangleIndex];
                    ref var v00 = ref vertices[width * j + i];
                    ref var v01 = ref vertices[width * j + i + 1];
                    ref var v10 = ref vertices[width * (j + 1) + i];
                    ref var v11 = ref vertices[width * (j + 1) + i + 1];
                    triangle0.A = v00;
                    triangle0.B = v01;
                    triangle0.C = v10;
                    ref var triangle1 = ref triangles[triangleIndex + 1];
                    triangle1.A = v01;
                    triangle1.B = v11;
                    triangle1.C = v10;
                }
            }
            pool.Return(ref vertices);
            return new Mesh(triangles, scaling, pool);
        }

        /// <summary>
        /// Creates a bunch of nodes and associates them with leaves with absolutely no regard for where the leaves are.
        /// </summary>
        static void CreateDummyNodes(ref Tree tree, int nodeIndex, int nodeLeafCount, ref int leafCounter)
        {
            ref var node = ref tree.Nodes[nodeIndex];
            node.A.LeafCount = nodeLeafCount / 2;
            if (node.A.LeafCount > 1)
            {
                node.A.Index = nodeIndex + 1;
                tree.Metanodes[node.A.Index] = new Metanode { IndexInParent = 0, Parent = nodeIndex };
                CreateDummyNodes(ref tree, node.A.Index, node.A.LeafCount, ref leafCounter);
            }
            else
            {
                tree.Leaves[leafCounter] = new Leaf(nodeIndex, 0);
                node.A.Index = Tree.Encode(leafCounter++);
            }
            node.B.LeafCount = nodeLeafCount - node.A.LeafCount;
            if (node.B.LeafCount > 1)
            {
                node.B.Index = nodeIndex + node.A.LeafCount;
                tree.Metanodes[node.B.Index] = new Metanode { IndexInParent = 1, Parent = nodeIndex };
                CreateDummyNodes(ref tree, node.B.Index, node.B.LeafCount, ref leafCounter);
            }
            else
            {
                tree.Leaves[leafCounter] = new Leaf(nodeIndex, 1);
                node.B.Index = Tree.Encode(leafCounter++);
            }
        }

        /// <summary>
        /// Takes a large number of triangles and creates a Mesh from them, but does not attempt to compute any bounds. 
        /// The topology of the mesh's acceleration structure is based entirely on the order of the triangles.
        /// This is intended to be used with <see cref="Tree.Refit"/>, <see cref="Tree.RefitAndRefine(BufferPool, int, float)"/>,
        /// or <see cref="Tree.RefitAndRefineMultithreadedContext.RefitAndRefine(ref Tree, BufferPool, IThreadDispatcher, int, float)"/> to provide bounds and higher quality.
        /// </summary>
        /// <param name="triangles">Large number of triangles to build a mesh from.</param>
        /// <param name="scaling">Scale to use for the mesh shape.</param>
        /// <param name="pool">Buffer pool to allocate resources for the mesh.</param>
        /// <returns>Created mesh with no bounds.</returns>
        /// <remarks>This exists primarily as an easy example of how to work around the slow sequential default mesh building options for very large meshes, like heightmaps.
        /// It is not optimized anywhere close to as much as it could be.
        /// In the future, I'd like to give the Tree and Mesh much faster (and multithreaded) constructors that achieve quality and speed in one shot.</remarks>
        public unsafe static Mesh CreateGiantMeshFastWithoutBounds(Buffer<Triangle> triangles, Vector3 scaling, BufferPool pool)
        {
            if (triangles.Length < 128)
            {
                //The special logic isn't necessary for tiny meshes, and we also don't handle the corner case of leaf counts <= 2. Just use the regular constructor.
                return new Mesh(triangles, scaling, pool);
            }
            var mesh = Mesh.CreateMeshWithoutTreeBuild(triangles, scaling, pool);
            int leafCounter = 0;
            CreateDummyNodes(ref mesh.Tree, 0, triangles.Length, ref leafCounter);
            for (int i = 0; i < triangles.Length; ++i)
            {
                ref var t = ref triangles[i];
                mesh.Tree.GetBoundsPointers(i, out var min, out var max);
                *min = Vector3.Min(t.A, Vector3.Min(t.B, t.C));
                *max = Vector3.Max(t.A, Vector3.Max(t.B, t.C));
            }
            return mesh;
        }

        /// <summary>
        /// Takes a very large number of triangles and turns them into a mesh by simply assuming that the input triangles are in an order that'll happen to produce an okay-ish acceleration structure.
        /// If you have a large height map, you might want to use this instead of the Mesh constructor's default sweep build or insertion builder.
        /// The quality is much lower than a sweep build (or even insertion build for that matter), but it can be orders of magnitude faster.
        /// Consider using refinement to get the tree quality closer to the sweep builder's quality afterwards.
        /// </summary>
        /// <param name="triangles">Large number of triangles to build a mesh from.</param>
        /// <param name="scaling">Scale to use for the mesh shape.</param>
        /// <param name="pool">Buffer pool to allocate resources for the mesh.</param>
        /// <returns>Created mesh.</returns>
        /// <remarks>This exists primarily as an easy example of how to work around the slow sequential default mesh building options for very large meshes, like heightmaps.
        /// It is not optimized anywhere close to as much as it could be.
        /// In the future, I'd like to give the Tree and Mesh much faster (and multithreaded) constructors that achieve quality and speed in one shot.</remarks>
        public static Mesh CreateGiantMeshFast(Buffer<Triangle> triangles, Vector3 scaling, BufferPool pool)
        {
            var mesh = CreateGiantMeshFastWithoutBounds(triangles, scaling, pool);
            //None of the nodes actually have bounds. Give them some now.
            mesh.Tree.Refit();
            return mesh;
        }

        /// <summary>
        /// Takes a very large number of triangles and turns them into a mesh by first creating a dummy topology and then incrementally refining it.
        /// If you have a large height map, you might want to use this instead of the Mesh constructor's default sweep build or insertion builder.
        /// The quality can approach <see cref="Tree.SweepBuild(BufferPool, Buffer{BoundingBox})"/> at a much lower cost thanks to a more efficient algorithm and multithreading.
        /// </summary>
        /// <param name="triangles">Large number of triangles to build a mesh from.</param>
        /// <param name="scaling">Scale to use for the mesh shape.</param>
        /// <param name="pool">Buffer pool to allocate resources for the mesh.</param>
        /// <returns>Created mesh.</returns>
        /// <remarks>This exists primarily as an easy example of how to work around the slow sequential default mesh building options for very large meshes, like heightmaps.
        /// It is not optimized anywhere close to as much as it could be.
        /// In the future, I'd like to give the Tree and Mesh much faster (and multithreaded) constructors that achieve quality and speed in one shot.</remarks>
        public static Mesh CreateGiantMeshWithRefinements(Buffer<Triangle> triangles, Vector3 scaling, BufferPool pool, Tree.RefitAndRefineMultithreadedContext context, IThreadDispatcher threadDispatcher, int refinementIterationCount = 8)
        {
            var mesh = CreateGiantMeshFastWithoutBounds(triangles, scaling, pool);
            //None of the nodes actually have bounds. Give them some now.
            for (int i = 0; i < refinementIterationCount; ++i)
                context.RefitAndRefine(ref mesh.Tree, pool, threadDispatcher, i, 20);
            return mesh;
        }

    }
}


