using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using BepuUtilities.Memory;
using System.Diagnostics;

namespace BepuPhysics.Collidables
{
    public struct Sphere : IShape
    {
        public float Radius;

        public Sphere(float radius)
        {
            Radius = radius;
        }

        //Note that spheres are sufficiently simple that no explicit bundle is required. A single vector<float> suffices.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Gather(ref Buffer<Sphere> shapes, ref Vector<int> shapeIndices, int count, out Vector<float> radii)
        {
            ref var radiiBase = ref Unsafe.As<Vector<float>, float>(ref radii);
            ref var shapeIndicesBase = ref Unsafe.As<Vector<int>, int>(ref shapeIndices);
            Debug.Assert(count <= Vector<float>.Count);
            for (int i = 0; i < count; ++i)
            {
                Unsafe.Add(ref radiiBase, i) = shapes[Unsafe.Add(ref shapeIndicesBase, i)].Radius;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetBounds<TShape>(ref Buffer<TShape> shapes, ref Vector<int> shapeIndices, int count, ref QuaternionWide orientations,
            out Vector<float> maximumRadius, out Vector<float> maximumAngularExpansion, out Vector3Wide min, out Vector3Wide max)
            where TShape : struct, IShape
        {
            //TODO: You could directly create the min and max during a scalar gather-like operation. Spheres don't really take advantage of bundled math.
            //Might be worth a quick try, but don't expect it to move the time more than a few microseconds.
            Gather(ref Unsafe.As<Buffer<TShape>, Buffer<Sphere>>(ref shapes), ref shapeIndices, count, out var radii);

            //Spheres have perfect symmetry, so there is no need for angular expansion.
            maximumRadius = new Vector<float>();
            maximumAngularExpansion = new Vector<float>();

            //It's technically true that spheres (and only spheres) do not require orientation to be loaded and could be special cased to reduce memory traffic, but just heck no.
            //It's very likely that the orientation loaded for the sphere was already in L1 anyway due to the online batching performed during the pose integrator.
            var negatedRadii = -radii;
            max = new Vector3Wide(ref radii);
            min = new Vector3Wide(ref negatedRadii);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetBounds(ref BepuUtilities.Quaternion orientation, out Vector3 min, out Vector3 max)
        {
            min = new Vector3(-Radius);
            max = new Vector3(Radius);
        }

        /// <summary>
        /// Type id of sphere shapes.
        /// </summary>
        public const int Id = 0;
        public int TypeId { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return Id; } }
    }
}
