using System;
using System.Diagnostics;
using SharpDX;
using SharpDX.Direct3D11;
using Buffer = SharpDX.Direct3D11.Buffer;
using System.Runtime.CompilerServices;
using System.Numerics;
using System.Runtime.InteropServices;
using BepuUtilities;

namespace DemoRenderer
{
    public static class Helpers
    {
        /// <summary>
        /// Sets an index buffer using a binding.
        /// </summary>
        /// <param name="inputAssembler">Assembler to set.</param>
        /// <param name="binding">Binding to set to the device for indices.</param>
        /// <param name="offset">Offset from the beginning of the index buffer to start reading.</param>
        public static void SetIndexBuffer(this InputAssemblerStage inputAssembler, IndexBuffer binding, int offset = 0)
        {
            inputAssembler.SetIndexBuffer(binding.Buffer, binding.Format, offset);
        }

        /// <summary>
        /// Updates a buffer using UpdateSubresource.
        /// </summary>
        /// <typeparam name="T">Type of the elements in the buffer.</typeparam>
        /// <param name="context">Context used to update the buffer.</param>
        /// <param name="buffer">Buffer to update.</param>
        /// <param name="newValues">Values to upload into the buffer.</param>
        /// <param name="sourceOffset">Starting index in the new values array to read from.</param>
        /// <param name="count">Number of elements in the values to upload into the buffer.</param>
        /// <param name="destinationOffset">Offset from the beginning of the buffer to store the new values.</param>
        public static unsafe void UpdateBuffer<T>(this DeviceContext context, Buffer buffer, Span<T> newValues, int count, int sourceOffset = 0, int destinationOffset = 0) where T : struct
        {
            ref var raw = ref MemoryMarshal.GetReference(newValues);
            fixed (byte* bytes = &Unsafe.As<T, byte>(ref raw))
            {
                var strideInBytes = Unsafe.SizeOf<T>();
                context.UpdateSubresource(
                    new DataBox(new IntPtr(bytes + sourceOffset * strideInBytes)), buffer, 0,
                    new ResourceRegion(
                        destinationOffset * strideInBytes, 0, 0,
                        (destinationOffset + count) * strideInBytes, 1, 1));
            }

        }

        /// <summary>
        /// Creates an index buffer of the specified size for screenspace quads.
        /// </summary>
        /// <param name="quadCount">Number of quads to create indices for.</param>
        /// <returns>Index buffer for screenspace quads.</returns> 
        /// <remarks>Using redundant indices for batches avoids a slow path for low triangle count instancing. This is hardware/driver specific; it may change on newer cards.</remarks>
        public static uint[] GetQuadIndices(int quadCount)
        {
            var indexData = new uint[quadCount * 6];
            uint baseVertex = 0;
            for (int glyphIndexStart = 0; glyphIndexStart < indexData.Length; glyphIndexStart += 6)
            {
                //Front facing triangles are counterclockwise.
                //Quad layout: 
                // 0____1
                // |  / |
                // | /  |
                // 2____3
                //0 2 1
                //1 2 3 
                indexData[glyphIndexStart + 0] = baseVertex + 0;
                indexData[glyphIndexStart + 1] = baseVertex + 2;
                indexData[glyphIndexStart + 2] = baseVertex + 1;
                indexData[glyphIndexStart + 3] = baseVertex + 1;
                indexData[glyphIndexStart + 4] = baseVertex + 2;
                indexData[glyphIndexStart + 5] = baseVertex + 3;
                baseVertex += 4;
            }
            return indexData;
        }
        /// <summary>
        /// Creates an index buffer of the specified size for boxes.
        /// </summary>
        /// <param name="quadCount">Number of boxes to create indices for.</param>
        /// <returns>Index buffer for boxes.</returns> 
        /// <remarks>Using redundant indices for batches avoids a slow path for low triangle count instancing. This is hardware/driver specific; it may change on newer cards.</remarks>
        public static uint[] GetBoxIndices(int boxCount)
        {
            //Build a AABB mesh's index buffer, repeated. A redundant index buffer tends to be faster than instancing tiny models. Impact varies between hardware.
            var indexData = new uint[boxCount * 36];
            uint baseVertex = 0;
            for (int glyphIndexStart = 0; glyphIndexStart < indexData.Length; glyphIndexStart += 36)
            {
                //Note that the order and winding is important and must be consistent with the RenderSpheres.hlsl VS usage.
                //It assumes that an unset bit in the 3-bit string of the vertex id corresponds to the minimum position: 
                //vertex id 0 becomes (-1, -1, -1), while vertex id 7 becomes (1, 1, 1).
                //-X
                indexData[glyphIndexStart + 0] = baseVertex + 0;
                indexData[glyphIndexStart + 1] = baseVertex + 4;
                indexData[glyphIndexStart + 2] = baseVertex + 6;
                indexData[glyphIndexStart + 3] = baseVertex + 6;
                indexData[glyphIndexStart + 4] = baseVertex + 2;
                indexData[glyphIndexStart + 5] = baseVertex + 0;
                //+X
                indexData[glyphIndexStart + 6] = baseVertex + 5;
                indexData[glyphIndexStart + 7] = baseVertex + 1;
                indexData[glyphIndexStart + 8] = baseVertex + 3;
                indexData[glyphIndexStart + 9] = baseVertex + 3;
                indexData[glyphIndexStart + 10] = baseVertex + 7;
                indexData[glyphIndexStart + 11] = baseVertex + 5;
                //-Y
                indexData[glyphIndexStart + 12] = baseVertex + 0;
                indexData[glyphIndexStart + 13] = baseVertex + 1;
                indexData[glyphIndexStart + 14] = baseVertex + 5;
                indexData[glyphIndexStart + 15] = baseVertex + 5;
                indexData[glyphIndexStart + 16] = baseVertex + 4;
                indexData[glyphIndexStart + 17] = baseVertex + 0;
                //+Y
                indexData[glyphIndexStart + 18] = baseVertex + 2;
                indexData[glyphIndexStart + 19] = baseVertex + 6;
                indexData[glyphIndexStart + 20] = baseVertex + 7;
                indexData[glyphIndexStart + 21] = baseVertex + 7;
                indexData[glyphIndexStart + 22] = baseVertex + 3;
                indexData[glyphIndexStart + 23] = baseVertex + 2;
                //-Z
                indexData[glyphIndexStart + 24] = baseVertex + 1;
                indexData[glyphIndexStart + 25] = baseVertex + 0;
                indexData[glyphIndexStart + 26] = baseVertex + 2;
                indexData[glyphIndexStart + 27] = baseVertex + 2;
                indexData[glyphIndexStart + 28] = baseVertex + 3;
                indexData[glyphIndexStart + 29] = baseVertex + 1;
                //+Z
                indexData[glyphIndexStart + 30] = baseVertex + 4;
                indexData[glyphIndexStart + 31] = baseVertex + 5;
                indexData[glyphIndexStart + 32] = baseVertex + 7;
                indexData[glyphIndexStart + 33] = baseVertex + 7;
                indexData[glyphIndexStart + 34] = baseVertex + 6;
                indexData[glyphIndexStart + 35] = baseVertex + 4;
                baseVertex += 8;
            }
            return indexData;
        }
        /// <summary>
        /// Packs an RGB color in a UNORM manner such that bits 0 through 10 are R, bits 11 through 21 are G, and bits 22 through 31 are B. 
        /// </summary>
        /// <param name="color">RGB color to pack.</param>
        /// <returns>Color packed into 32 bits.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint PackColor(in Vector3 color)
        {
            const uint RScale = (1 << 11) - 1;
            const uint GScale = (1 << 11) - 1;
            const uint BScale = (1 << 10) - 1;
            var r = (uint)(color.X * RScale);
            var g = (uint)(color.Y * GScale);
            var b = (uint)(color.Z * BScale);
            if (r > RScale)
                r = RScale;
            if (g > GScale)
                g = GScale;
            if (b > BScale)
                b = BScale;
            return r | (g << 11) | (b << 22);
        }

        /// <summary>
        /// Unpacks a 3 component color packed by the Helpers.PackColor function.
        /// </summary>
        /// <param name="packedColor">Packed representation of the color to unpack.</param>
        /// <param name="color">Unpacked color.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void UnpackColor(uint packedColor, out Vector3 color)
        {
            //We don't support any form of alpha, so we dedicate 11 bits to R, 11 bits to G, and 10 bits to B.
            //B is stored in most significant, R in least significant.
            color.X = (packedColor & 0x7FF) / (float)((1 << 11) - 1);
            color.Y = ((packedColor >> 11) & 0x7FF) / (float)((1 << 11) - 1);
            color.Z = ((packedColor >> 22) & 0x3FF) / (float)((1 << 10) - 1);
        }

        /// <summary>
        /// Packs an RGBA color in a UNORM manner such that bits 0 through 7 are R, bits 8 through 15 are G, and bits 16 through 23 are B, and 24 through 31 are A. 
        /// </summary>
        /// <param name="color">RGBA color to pack.</param>
        /// <returns>Color packed into 32 bits.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint PackColor(in Vector4 color)
        {
            var scaledColor = Vector4.Max(Vector4.Zero, Vector4.Min(Vector4.One, color)) * 255;
            return (uint)scaledColor.X | ((uint)scaledColor.Y << 8) | ((uint)scaledColor.Z << 16) | ((uint)scaledColor.W << 24);
        }

        /// <summary>
        /// Unpacks a 4 component color packed by the Helpers.PackColor function.
        /// </summary>
        /// <param name="packedColor">Packed representation of the color to unpack.</param>
        /// <param name="color">Unpacked color.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void UnpackColor(uint packedColor, out Vector4 color)
        {
            color = new Vector4(
                (packedColor & 255) / 255f,
                ((packedColor >> 8) & 255) / 255f,
                ((packedColor >> 16) & 255) / 255f,
                ((packedColor >> 24) & 255) / 255f);
        }

        /// <summary>
        /// Weakly packs an orientation into a 3 component vector by ensuring the W component is positive and then dropping it. Remaining components are kept in full precision.
        /// </summary>
        /// <param name="source">Orientation to pack.</param>
        /// <param name="packed">W-less packed orientation, with remaining components negated to guarantee that the reconstructed positive W component is valid.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void PackOrientation(in Quaternion source, out Vector3 packed)
        {
            packed = new Vector3(source.X, source.Y, source.Z);
            if (source.W < 0)
            {
                packed = -packed;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void PackDuplicateZeroSNORM(float source, out ushort packed)
        {
            Debug.Assert(source >= -1 && source <= 1);
            var magnitude = source * ((1 << 15) - 1);
            ref var reinterpreted = ref Unsafe.As<float, uint>(ref magnitude);
            //Cache the sign bit and move it into position.
            var sign = (int)((reinterpreted & 0x8000_0000u) >> 16);
            //Clear the sign bit.
            reinterpreted &= 0x7FFF_FFFF;
            packed = (ushort)((int)magnitude | sign);
        }
        /// <summary>
        /// Packs an orientation into 8 bytes by storing each component in 16 bits. The most significant bit of each component is used as a sign bit. The remaining bits are used for magnitude.
        /// </summary>
        /// <param name="source">Orientation to pack.</param>
        /// <param name="packed">Packed orientation.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe static ulong PackOrientationU64(ref Quaternion source)
        {
            //This isn't exactly a clever packing, but with 64 bits, cleverness isn't required.
            ref var vectorSource = ref Unsafe.As<float, Vector4>(ref source.X);
            var clamped = Vector4.Max(new Vector4(-1), Vector4.Min(new Vector4(1), vectorSource));
            ulong packed;
            ref var packedShorts = ref Unsafe.As<ulong, ushort>(ref *&packed);
            PackDuplicateZeroSNORM(clamped.X, out packedShorts);
            PackDuplicateZeroSNORM(clamped.Y, out Unsafe.Add(ref packedShorts, 1));
            PackDuplicateZeroSNORM(clamped.Z, out Unsafe.Add(ref packedShorts, 2));
            PackDuplicateZeroSNORM(clamped.W, out Unsafe.Add(ref packedShorts, 3));
            return packed;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static unsafe float UnpackDuplicateZeroSNORM(ushort packed)
        {
            var unpacked = (packed & ((1 << 15) - 1)) * (1f / ((1 << 15) - 1));
            ref var reinterpreted = ref Unsafe.As<float, uint>(ref *&unpacked);
            //Set the sign bit.
            reinterpreted |= (packed & (1u << 15)) << 16;
            return unpacked;
        }
        public static void UnpackOrientation(ulong packed, out Quaternion orientation)
        {
            ref var packedShorts = ref Unsafe.As<ulong, ushort>(ref packed);
            orientation.X = UnpackDuplicateZeroSNORM(packedShorts);
            orientation.Y = UnpackDuplicateZeroSNORM(Unsafe.Add(ref packedShorts, 1));
            orientation.Z = UnpackDuplicateZeroSNORM(Unsafe.Add(ref packedShorts, 2));
            orientation.W = UnpackDuplicateZeroSNORM(Unsafe.Add(ref packedShorts, 3));
            QuaternionEx.Normalize(ref orientation);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool GetScreenLocation(in Vector3 position, in Matrix viewProjection, in Vector2 resolution, out Vector2 screenLocation)
        {
            Matrix.Transform(new Vector4(position, 1), viewProjection, out var projected);
            projected /= projected.W;
            if (projected.Z <= 0 || MathF.Abs(projected.X) > 1 || MathF.Abs(projected.Y) > 1)
            {
                screenLocation = default;
                return false;
            }
            var ndc = new Vector2(projected.X, projected.Y);
            screenLocation = (ndc * new Vector2(0.5f, -0.5f) + new Vector2(0.5f)) * resolution;
            return true;
        }

        /// <summary>
        /// Applies the sRGB gamma curve to a scalar input.
        /// </summary>
        /// <param name="x">Value to apply the curve to.</param>
        /// <returns>Transformed value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float ToSRGB(float x)
        {
            return MathF.Max(0, MathF.Min(1, x <= 0.0031308f ?
                12.92f * x :
                1.055f * MathF.Pow(x, 1f / 2.4f) - 0.055f));
        }

        /// <summary>
        /// Applies the sRGB gamma curve to a vector.
        /// </summary>
        /// <param name="linear">Linear input to apply the curve to.</param>
        /// <param name="srgb">Transformed value.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ToSRGB(in Vector3 linear, out Vector3 srgb)
        {
            srgb = new Vector3(ToSRGB(linear.X), ToSRGB(linear.Y), ToSRGB(linear.Z));
        }

        [Conditional("DEBUG")]
        public static void CheckForUndisposed(bool disposed, object o)
        {
            Debug.Assert(disposed, "An object of type " + o.GetType() + " was not disposed prior to finalization.");
        }

        public static void Dispose<T>(ref T disposable) where T : IDisposable
        {
            if (disposable != null)
                disposable.Dispose();
            disposable = default(T);
        }

    }


}
