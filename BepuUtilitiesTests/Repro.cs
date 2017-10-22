using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Repro
{
    class Repro
    {
        public struct Matrix4
        {
            public Vector4 X;
            public Vector4 Y;
            public Vector4 Z;
            public Vector4 W;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Transform(ref Vector4 v, ref Matrix4 m, out Vector4 result)
            {
                var x = new Vector4(v.X);
                var y = new Vector4(v.Y);
                var z = new Vector4(v.Z);
                var w = new Vector4(v.W);
                result = m.X * x + m.Y * y + m.Z * z + m.W * w;
            }
        }

        [StructLayout(LayoutKind.Explicit, Size = 48)]
        public struct Matrix3
        {
            [FieldOffset(0)]
            public Vector3 X;
            [FieldOffset(16)]
            public Vector3 Y;
            [FieldOffset(32)]
            public Vector3 Z;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public static void Transform(ref Vector3 v, ref Matrix3 m, out Vector3 result)
            {
                var x = new Vector3(v.X);
                var y = new Vector3(v.Y);
                var z = new Vector3(v.Z);
                result = m.X * x + m.Y * y + m.Z * z;
            }
        }

        static void Main2(string[] args)
        {
            const int iterationCount = 10000000;

            //MATRIX4x4
            {
                var v = new Vector4(1, 2, 3, 4);
                var m = new Matrix4
                {
                    X = new Vector4(1, 0, 0, 0),
                    Y = new Vector4(0, 1, 0, 0),
                    Z = new Vector4(0, 0, 1, 0),
                    W = new Vector4(0, 0, 0, 1)
                };

                //Warmup
                {
                    Vector4 result;
                    Matrix4.Transform(ref v, ref m, out result);
                }
                Vector4 accumulator = new Vector4();
                var startTime = Stopwatch.GetTimestamp();
                for (int i = 0; i < iterationCount; ++i)
                {
                    Vector4 result;
                    Matrix4.Transform(ref v, ref m, out result);
                    Matrix4.Transform(ref result, ref m, out result);
                    Matrix4.Transform(ref result, ref m, out result);
                    Matrix4.Transform(ref result, ref m, out result);
                    Matrix4.Transform(ref result, ref m, out result);
                    Matrix4.Transform(ref result, ref m, out result);
                    Matrix4.Transform(ref result, ref m, out result);
                    Matrix4.Transform(ref result, ref m, out result);
                    Matrix4.Transform(ref result, ref m, out result);
                    Matrix4.Transform(ref result, ref m, out result);
                    accumulator += result; //Avoid optimizing out the loop.
                }
                var endTime = Stopwatch.GetTimestamp();
                Console.WriteLine($"4x4 Time: {(endTime - startTime) / (double)Stopwatch.Frequency}");
            }

            //MATRIX3x3
            {
                var v = new Vector3(1, 2, 3);
                var m = new Matrix3 
                {
                    X = new Vector3(1, 0, 0),
                    Y = new Vector3(0, 1, 0),
                    Z = new Vector3(0, 0, 1),
                };
                //Warmup
                {
                    Vector3 result;
                    Matrix3.Transform(ref v, ref m, out result);
                }
                Vector3 accumulator = new Vector3();
                var startTime = Stopwatch.GetTimestamp();
                for (int i = 0; i < iterationCount; ++i)
                {
                    Vector3 result;
                    Matrix3.Transform(ref v, ref m, out result);
                    Matrix3.Transform(ref result, ref m, out result);
                    Matrix3.Transform(ref result, ref m, out result);
                    Matrix3.Transform(ref result, ref m, out result);
                    Matrix3.Transform(ref result, ref m, out result);
                    Matrix3.Transform(ref result, ref m, out result);
                    Matrix3.Transform(ref result, ref m, out result);
                    Matrix3.Transform(ref result, ref m, out result);
                    Matrix3.Transform(ref result, ref m, out result);
                    Matrix3.Transform(ref result, ref m, out result);
                    accumulator += result; //Avoid optimizing out the loop.
                }
                var endTime = Stopwatch.GetTimestamp();
                Console.WriteLine($"3x3 Time: {(endTime - startTime) / (double)Stopwatch.Frequency}");
            }
        }
    }
}
