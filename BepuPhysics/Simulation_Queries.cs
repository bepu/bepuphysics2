using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Trees;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics
{
    partial class Simulation
    {
        struct RayHitDispatcher<TRayHitHandler> : IBroadPhaseRayTester where TRayHitHandler : IRayHitHandler
        {
            public Simulation Simulation;
            public TRayHitHandler HitHandler;

            unsafe void Test(CollidableReference reference, TypedIndex shape, ref RigidPose pose, RayData* ray, float* maximumT)
            {
                //TODO: consider adding a filter that only considers the leaf, not the ray-leaf combination. In many cases, users won't care about the ray-leaf combo, 
                //and leaf-only filtering would be quite a bit simpler.
                //TODO: Arguably, moving filtering into the core traversal would be the simplest option, and it would have some performance benefits. The raytest stack wouldn't 
                //have all the unnecessary rays added to it in the first place.
                if (HitHandler.AllowTest(ref *ray, ref *maximumT, reference))
                {
                    if (Simulation.Shapes[shape.Type].RayTest(shape.Index, ref pose, ref ray->Origin, ref ray->Direction, out var t, out var normal) && t < *maximumT)
                    {
                        HitHandler.OnRayHit(ref *ray, ref *maximumT, t, ref normal, reference);
                    }
                }
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe void RayTest(CollidableReference reference, RayData* rayData, float* maximumT)
            {
                if (reference.Mobility == CollidableMobility.Static)
                {
                    var index = Simulation.Statics.HandleToIndex[reference.Handle];
                    Test(reference, Simulation.Statics.Collidables[index].Shape, ref Simulation.Statics.Poses[index], rayData, maximumT);
                }
                else
                {
                    ref var location = ref Simulation.Bodies.HandleToLocation[reference.Handle];
                    ref var set = ref Simulation.Bodies.Sets[location.SetIndex];
                    Test(reference, set.Collidables[location.Index].Shape, ref set.Poses[location.Index], rayData, maximumT);
                }
            }
        }


        public unsafe void RayCast<THitHandler>(ref Vector3 origin, ref Vector3 direction, float maximumT, ref THitHandler hitHandler, int id = 0) where THitHandler : IRayHitHandler
        {
            TreeRay.CreateFrom(ref origin, ref direction, maximumT, id, out var rayData, out var treeRay);
            RayHitDispatcher<THitHandler> dispatcher;
            dispatcher.HitHandler = hitHandler;
            dispatcher.Simulation = this;
            BroadPhase.RayCast(ref origin, ref direction, maximumT, ref dispatcher, id);
        }
    }
}
