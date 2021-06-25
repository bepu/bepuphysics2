﻿using BepuPhysics.Collidables;
using BepuPhysics.Trees;
using BepuUtilities;
using BepuUtilities.Memory;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.CollisionDetection
{
    /// <summary>
    /// Helper class for creating runtime specialized vectorized ray intersection tests with shapes that support broadcasting.
    /// </summary>
    public static class WideRayTester
    {
        public unsafe static void Test<TRaySource, TShape, TShapeWide, TRayHitHandler>(ref TShape shape, in RigidPose pose, ref TRaySource raySource, ref TRayHitHandler rayHitHandler)
            where TShape : unmanaged, IConvexShape
            where TShapeWide : unmanaged, IShapeWide<TShape>
            where TRaySource : IRaySource
            where TRayHitHandler : struct, IShapeRayHitHandler
        {
            RayWide rayWide = default;
            TShapeWide wide;
            if ((*&wide).InternalAllocationSize > 0)
            {
                var memory = stackalloc byte[wide.InternalAllocationSize];
                wide.Initialize(new RawBuffer(memory, wide.InternalAllocationSize));
            }
            wide.Broadcast(shape);
            RigidPoses poses;
            Vector3Wide.Broadcast(pose.Position, out poses.Position);
            QuaternionWide.Broadcast(pose.Orientation, out poses.Orientation);
            for (int i = 0; i < raySource.RayCount; i += Vector<float>.Count)
            {
                var count = raySource.RayCount - i;
                if (count < wide.MinimumWideRayCount)
                {
                    for (int j = 0; j < count; ++j)
                    {
                        raySource.GetRay(i + j, out var ray, out var maximumT);
                        if (shape.RayTest(pose, ray->Origin, ray->Direction, out var t, out var normal) && t <= *maximumT)
                        {
                            rayHitHandler.OnRayHit(*ray, ref *maximumT, t, normal, 0);
                        }
                    }
                }
                else
                {
                    if (count > Vector<float>.Count)
                        count = Vector<float>.Count;
                    for (int j = 0; j < count; ++j)
                    {
                        //TODO: Pointer initialization skip hack. Replace with Unsafe.SkipInit?
                        GatherScatter.GetOffsetInstance(ref *&rayWide, j).Gather(raySource.GetRay(i + j));
                    }

                    wide.RayTest(ref poses, ref rayWide, out var intersected, out var t, out var normal);

                    for (int j = 0; j < count; ++j)
                    {
                        Vector3 scalarNormal;
                        scalarNormal.X = normal.X[j];
                        scalarNormal.Y = normal.Y[j];
                        scalarNormal.Z = normal.Z[j];
                        if (intersected[j] < 0)
                        {
                            raySource.GetRay(i + j, out var ray, out var maximumT);
                            var tj = t[j];
                            if (tj <= *maximumT)
                                rayHitHandler.OnRayHit(*ray, ref *maximumT, tj, scalarNormal, 0);
                        }
                    }
                }

            }
        }
    }

}
