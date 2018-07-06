using DemoContentLoader;
using ObjLoader.Loader.Loaders;
using System.Collections.Generic;
using System.IO;
using System.Numerics;

namespace DemoContentBuilder
{
    public static class MeshBuilder
    {
        public unsafe static MeshContent Build(Stream dataStream)
        {
            var result = new ObjLoaderFactory().Create().Load(dataStream);
            var triangles = new List<TriangleContent>();
            for (int i = 0; i < result.Groups.Count; ++i)
            {
                var group = result.Groups[i];
                for (int j = 0; j < group.Faces.Count; ++j)
                {
                    var face = group.Faces[j];
                    var a = result.Vertices[face[0].VertexIndex];
                    for (int k = 1; k < face.Count - 1; ++k)
                    {
                        var b = result.Vertices[face[k].VertexIndex];
                        var c = result.Vertices[face[k + 1].VertexIndex];
                        triangles.Add(new TriangleContent
                        {
                            A = new Vector3(a.X, a.Y, a.Z),
                            B = new Vector3(b.X, b.Y, b.Z),
                            C = new Vector3(c.X, c.Y, c.Z)
                        });
                    }
                }
            }
            return new MeshContent(triangles.ToArray());
        }
    }
}
