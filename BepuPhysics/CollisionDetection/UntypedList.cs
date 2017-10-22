using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection
{
    public struct UntypedList
    {
        public RawBuffer Buffer;
        /// <summary>
        /// The number of bytes in the list.
        /// </summary>
        public int Count;
        public int ByteCount;



        public UntypedList(int initialSizeInBytes, BufferPool pool)
        {
            pool.Take(initialSizeInBytes, out Buffer);
            Count = 0;
            ByteCount = 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe ref T GetFromBytes<T>(int byteIndex)
        {
            return ref Unsafe.As<byte, T>(ref Buffer.Memory[byteIndex]);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe ref T Get<T>(int index)
        {
            return ref Unsafe.Add(ref Unsafe.As<byte, T>(ref *Buffer.Memory), index);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe byte* AllocateUnsafely(int sizeInBytes)
        {
            var newSize = ByteCount + sizeInBytes;
            Count++;
            var byteIndex = ByteCount;
            ByteCount = newSize;
            return Buffer.Memory + byteIndex;
        }
    
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe ref T AllocateUnsafely<T>()
        {
            var newSize = ByteCount + Unsafe.SizeOf<T>();
            //If we store only byte count, we'd have to divide to get the element index.
            //If we store only count, we would have to store per-type size somewhere since the PairCache constructor doesn't have an id->type mapping.
            //So we just store both. It's pretty cheap and simple.
            Count++;
            var byteIndex = ByteCount;
            ByteCount = newSize;
            return ref Unsafe.As<byte, T>(ref Buffer[byteIndex]);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe int Allocate<T>(int minimumElementCount, BufferPool pool)
        {
            var newSize = ByteCount + Unsafe.SizeOf<T>();
            if (!Buffer.Allocated)
            {
                //This didn't exist at all before; create a new entry for this type.
                pool.Take(Math.Max(newSize, minimumElementCount * Unsafe.SizeOf<T>()), out Buffer);
            }
            else
            {
                if (newSize > Buffer.Length)
                {
                    //This will bump up to the next allocated block size, so we don't have to worry about constant micro-resizes.
                    pool.Take(newSize, out var newBuffer);
                    Unsafe.CopyBlockUnaligned(newBuffer.Memory, Buffer.Memory, (uint)Buffer.Length);
                    pool.ReturnUnsafely(ref Buffer);
                    Buffer = newBuffer;
                }
            }
            Debug.Assert(Buffer.Length >= newSize);
            //If we store only byte count, we'd have to divide to get the element index.
            //If we store only count, we would have to store per-type size somewhere since the PairCache constructor doesn't have an id->type mapping.
            //So we just store both. It's pretty cheap and simple.
            Count++;
            var byteIndex = ByteCount;
            ByteCount = newSize;
            return byteIndex;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe int Add<T>(ref T data, int minimumCount, BufferPool pool)
        {
            var byteIndex = Allocate<T>(minimumCount, pool);
            GetFromBytes<T>(byteIndex) = data;
            return byteIndex;
        }
    }
}
