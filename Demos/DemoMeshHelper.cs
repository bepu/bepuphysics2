using BepuPhysics.Collidables;
using System;
using System.Numerics;
using BepuUtilities.Memory;
using DemoContentLoader;

namespace Demos
{
    public static class DemoMeshHelper
    {
        public static void LoadModel(ContentArchive content, BufferPool pool, string contentName, in Vector3 scaling, out Mesh mesh)
        {
            var meshContent = content.Load<MeshContent>(contentName);
            pool.Take<Triangle>(meshContent.Triangles.Length, out var triangles);
            for (int i = 0; i < meshContent.Triangles.Length; ++i)
            {
                triangles[i] = new Triangle(meshContent.Triangles[i].A, meshContent.Triangles[i].B, meshContent.Triangles[i].C);
            }
            mesh = new Mesh(triangles, scaling, pool);
        }

        public static void CreateFan(int triangleCount, float radius, in Vector3 scaling, BufferPool pool, out Mesh mesh)
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
            mesh = new Mesh(triangles, scaling, pool);
        }

        public static void CreateDeformedPlane(int width, int height, Func<int, int, Vector3> deformer, Vector3 scaling, BufferPool pool, out Mesh mesh)
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
            mesh = new Mesh(triangles, scaling, pool);
        }
    }
}


