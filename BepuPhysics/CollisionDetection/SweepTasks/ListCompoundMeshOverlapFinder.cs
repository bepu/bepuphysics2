using BepuPhysics.Collidables;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System.Numerics;
using System.Runtime.CompilerServices;
using Quaternion = BepuUtilities.Quaternion;

namespace BepuPhysics.CollisionDetection.SweepTasks
{
    public struct ListCompoundMeshOverlapFinder : ICompoundMeshSweepOverlapFinder<Compound, Mesh>
    {
        public unsafe void FindOverlaps(ref Compound compound, in Quaternion compoundOrientation, in BodyVelocity compoundVelocity,
            ref Mesh mesh, in Vector3 meshOffset, in Quaternion meshOrientation, in BodyVelocity meshVelocity, float maximumT,
            Shapes shapes, BufferPool pool, ref QuickList<(int, int), Buffer<(int, int)>> childOverlaps)
        {
            //For list based compounds, there is no compound side acceleration structure to consider. We'll test every single child.
            //The only difference between this test and the usual convex test is that we need to take into account the compound child's local pose 
            //and velocity when computing the mesh local bounding box.
            QuickList<int, Buffer<int>>.Create(pool.SpecializeFor<int>(), 128, out var triangleIndices);
            for (int i = 0; i < compound.Children.Length; ++i)
            {
                ref var child = ref compound.Children[i];
                BoundingBoxHelpers.GetLocalBoundingBoxForSweep(
                    child.ShapeIndex, shapes, child.LocalPose, compoundOrientation, compoundVelocity,
                    meshOffset, meshOrientation, meshVelocity, maximumT, out var sweep, out var min, out var max);
                mesh.FindLocalOverlaps(min, max, sweep, maximumT, pool, ref triangleIndices);
                ref var start = ref childOverlaps.Allocate(triangleIndices.Count, pool.SpecializeFor<(int, int)>());
                for (int j = 0; j < triangleIndices.Count; ++j)
                {
                    Unsafe.Add(ref start, j) = (i, triangleIndices[j]);
                }
                triangleIndices.Count = 0;
            }
            triangleIndices.Dispose(pool.SpecializeFor<int>());
        }
    }
}
