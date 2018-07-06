using System;
using System.Collections.Generic;
using System.IO;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace DemoContentLoader
{
    public class MeshIO
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ReadVector3(BinaryReader reader, out Vector3 v)
        {
            v = new Vector3(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());
        }

        public static MeshContent Load(BinaryReader reader)
        {
            var triangleCount = reader.ReadInt32();
            var triangles = new TriangleContent[triangleCount];
            for (int i = 0; i < triangleCount; ++i)
            {
                ref var triangle = ref triangles[i];
                ReadVector3(reader, out triangle.A);
                ReadVector3(reader, out triangle.B);
                ReadVector3(reader, out triangle.C);
            }
            return new MeshContent(triangles);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Write(BinaryWriter writer, in Vector3 v)
        {
            writer.Write(v.X);
            writer.Write(v.Y);
            writer.Write(v.Z);
        }

        public static void Save(MeshContent content, BinaryWriter writer)
        {
            writer.Write(content.Triangles.Length);
            for (int i = 0; i < content.Triangles.Length; ++i)
            {
                ref var triangle = ref content.Triangles[i];
                Write(writer, triangle.A);
                Write(writer, triangle.B);
                Write(writer, triangle.C);
            }
        }
    }
}