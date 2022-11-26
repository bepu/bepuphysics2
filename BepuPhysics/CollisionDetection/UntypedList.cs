using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection
{
    public struct UntypedList
    {
        public Buffer<byte> Buffer;
        public int Count;
        public int ByteCount;
        public int ElementSizeInBytes;


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public UntypedList(int elementSizeInBytes, int initialCapacityInElements, BufferPool pool)
        {
            pool.TakeAtLeast(initialCapacityInElements * elementSizeInBytes, out Buffer);
            Count = 0;
            ByteCount = 0;
            ElementSizeInBytes = elementSizeInBytes;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal void EnsureCapacityInBytes(int elementSizeInBytes, int targetCapacityInBytes, BufferPool pool)
        {
            //EnsureCapacity is basically a secondary constructor, but it can be used on already-existing caches. It has the same required output.
            Debug.Assert(ElementSizeInBytes == 0 || ElementSizeInBytes == elementSizeInBytes,
                "Ensuring capacity should not change an already existing list's element size in bytes.");
            ElementSizeInBytes = elementSizeInBytes;
            if (Buffer.Length < targetCapacityInBytes)
            {
                pool.ResizeToAtLeast(ref Buffer, targetCapacityInBytes, ByteCount);
            }
        }

        [Conditional("DEBUG")]
        void Validate()
        {
            Debug.Assert(ElementSizeInBytes > 0);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe ref T GetFromBytes<T>(int byteIndex)
        {
            Validate();
            return ref Unsafe.As<byte, T>(ref Buffer.Memory[byteIndex]);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe ref T Get<T>(int index)
        {
            Validate();
            return ref Unsafe.Add(ref Unsafe.As<byte, T>(ref *Buffer.Memory), index);
        }



        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe byte* AllocateUnsafely()
        {
            Validate();
            var newSize = ByteCount + ElementSizeInBytes;
            Count++;
            var byteIndex = ByteCount;
            ByteCount = newSize;
            return Buffer.Memory + byteIndex;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe ref T AllocateUnsafely<T>()
        {
            Validate();
            Debug.Assert(Unsafe.SizeOf<T>() == ElementSizeInBytes);
            var newSize = ByteCount + Unsafe.SizeOf<T>();
            //If we store only byte count, we'd have to divide to get the element index.
            //If we store only count, we would have to store per-type size somewhere since the PairCache constructor doesn't have an id->type mapping.
            //So we just store both. It's pretty cheap and simple.
            Count++;
            var byteIndex = ByteCount;
            ByteCount = newSize;
            return ref Unsafe.As<byte, T>(ref Buffer[byteIndex]);
        }

        /// <summary>
        /// Allocates an element in the list, initializing the backing buffer if needed.
        /// </summary>
        /// <param name="elementSizeInBytes">Number of bytes per element.</param>
        /// <param name="minimumElementCount">Minimum size of the backing buffer to create if this is a new allocation.</param>
        /// <param name="pool">Pool to pull allocations from.</param>
        /// <returns>Index of the element in bytes within the list's buffer.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe int Allocate(int elementSizeInBytes, int minimumElementCount, BufferPool pool)
        {
            var newSize = ByteCount + elementSizeInBytes;
            if (!Buffer.Allocated)
            {
                //This didn't exist at all before; create a new entry for this type.
                ElementSizeInBytes = elementSizeInBytes;
                pool.TakeAtLeast(Math.Max(newSize, minimumElementCount * elementSizeInBytes), out Buffer);
            }
            else
            {
                Debug.Assert(elementSizeInBytes == ElementSizeInBytes);
                if (newSize > Buffer.Length)
                {
                    //This will bump up to the next allocated block size, so we don't have to worry about constant micro-resizes.
                    pool.TakeAtLeast<byte>(newSize, out var newBuffer);
                    Unsafe.CopyBlockUnaligned(newBuffer.Memory, Buffer.Memory, (uint)Buffer.Length);
                    pool.ReturnUnsafely(Buffer.Id);
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
        public unsafe int Allocate<T>(int minimumElementCount, BufferPool pool)
        {
            var elementSizeInBytes = Unsafe.SizeOf<T>();
            return Allocate(elementSizeInBytes, minimumElementCount, pool);
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
