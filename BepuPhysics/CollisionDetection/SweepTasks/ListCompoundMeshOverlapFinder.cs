using BepuPhysics.Collidables;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using Quaternion = BepuUtilities.Quaternion;

namespace BepuPhysics.CollisionDetection.SweepTasks
{
    public struct ListCompoundMeshOverlapFinder : ICompoundMeshSweepOverlapFinder<Compound, Mesh>
    {
        public void FindOverlaps(ref Compound compound, in Quaternion compoundOrientation, in BodyVelocity compoundVelocity, 
            ref Mesh mesh, in Vector3 meshOffset, in Quaternion meshOrientation, in BodyVelocity meshVelocity, 
            BufferPool pool, ref QuickList<(int, int), Buffer<(int, int)>> childOverlaps)
        {
            //For list based compounds, there is no compound side acceleration structure to consider. We'll test every single child.
            //The only difference between this test and the usual convex test is that we need to take into account the compound child's local pose 
            //and velocity when computing the mesh local bounding box.
            
        }
    }
}
