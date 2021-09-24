using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.CollisionDetection
{
    public unsafe struct CompoundMeshReduction : ICollisionTestContinuation
    {
        public int RegionCount;
        public Buffer<(int Start, int Count)> ChildManifoldRegions;
        public Buffer<BoundingBox> QueryBounds;
        public Buffer<Triangle> Triangles;
        //MeshReduction relies on all of a mesh's triangles being in slot B, as they appear in the mesh collision tasks.
        //However, the original user may have provided this pair in unknown order and triggered a flip. We'll compensate for that when examining contact positions.
        public bool RequiresFlip;
        //The triangles array is in the mesh's local space. In order to test any contacts against them, we need to be able to transform contacts.
        public Quaternion MeshOrientation;
        //This uses all of the nonconvex reduction's logic, so we just nest it.
        public NonconvexReduction Inner;

        public Mesh* Mesh; //TODO: This is not flexible with respect to different mesh types. Not a problem right now, but it will be in the future.

        public void Create(int childManifoldCount, BufferPool pool)
        {
            Inner.Create(childManifoldCount, pool);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void OnChildCompleted<TCallbacks>(ref PairContinuation report, ref ConvexContactManifold manifold, ref CollisionBatcher<TCallbacks> batcher)
            where TCallbacks : struct, ICollisionCallbacks
        {
            Inner.OnChildCompleted(ref report, ref manifold, ref batcher);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void OnChildCompletedEmpty<TCallbacks>(ref PairContinuation report, ref CollisionBatcher<TCallbacks> batcher) where TCallbacks : struct, ICollisionCallbacks
        {
            Inner.OnChildCompletedEmpty(ref report, ref batcher);
        }
        
        public unsafe bool TryFlush<TCallbacks>(int pairId, ref CollisionBatcher<TCallbacks> batcher) where TCallbacks : struct, ICollisionCallbacks
        {
            Debug.Assert(Inner.ChildCount > 0);
            if (Inner.CompletedChildCount == Inner.ChildCount)
            {
                Matrix3x3.CreateFromQuaternion(MeshOrientation, out var meshOrientation);
                Matrix3x3.Transpose(meshOrientation, out var meshInverseOrientation);

                for (int i = 0; i < RegionCount; ++i)
                {
                    ref var region = ref ChildManifoldRegions[i];
                    if (region.Count > 0)
                    {
                        MeshReduction.ReduceManifolds(ref Triangles, ref Inner.Children, region.Start, region.Count, RequiresFlip, QueryBounds[i], meshOrientation, meshInverseOrientation, Mesh, batcher.Pool);
                    }
                }

                //Now that boundary smoothing analysis is done, we can safely clean up the continuation resources.
                batcher.Pool.Return(ref Triangles);
                batcher.Pool.Return(ref ChildManifoldRegions);
                batcher.Pool.Return(ref QueryBounds);
                Inner.Flush(pairId, ref batcher);
                return true;
            }
            return false;
        }

    }
}
