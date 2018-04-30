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
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal unsafe void GetPoseAndShape(CollidableReference reference, out RigidPose* pose, out TypedIndex shape)
        {
            if (reference.Mobility == CollidableMobility.Static)
            {
                var index = Statics.HandleToIndex[reference.Handle];
                pose = (RigidPose*)Statics.Poses.Memory + index;
                shape = Statics.Collidables[index].Shape;
            }
            else
            {
                ref var location = ref Bodies.HandleToLocation[reference.Handle];
                ref var set = ref Bodies.Sets[location.SetIndex];
                pose = (RigidPose*)set.Poses.Memory + location.Index;
                shape = set.Collidables[location.Index].Shape;
            }
        }
        struct RayHitDispatcher<TRayHitHandler> : IBroadPhaseRayTester where TRayHitHandler : IRayHitHandler
        {
            public Simulation Simulation;
            public TRayHitHandler HitHandler;


            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe void RayTest(CollidableReference reference, RayData* rayData, float* maximumT)
            {
                if (HitHandler.AllowTest(reference))
                {
                    Simulation.GetPoseAndShape(reference, out var pose, out var shape);
                    if (Simulation.Shapes[shape.Type].RayTest(shape.Index, *pose, rayData->Origin, rayData->Direction, out var t, out var normal) && t < *maximumT)
                    {
                        HitHandler.OnRayHit(*rayData, ref *maximumT, t, normal, reference);
                    }
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


        public unsafe void RayCast2<THitHandler>(ref Vector3 origin, ref Vector3 direction, float maximumT, ref THitHandler hitHandler, int id = 0) where THitHandler : IRayHitHandler
        {
            TreeRay.CreateFrom(ref origin, ref direction, maximumT, id, out var rayData, out var treeRay);
            RayHitDispatcher<THitHandler> dispatcher;
            dispatcher.HitHandler = hitHandler;
            dispatcher.Simulation = this;
            BroadPhase.RayCast2(ref origin, ref direction, maximumT, ref dispatcher, id);
        }
    }
}
