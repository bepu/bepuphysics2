using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Trees;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace Demos.Demos
{
    /// <summary>
    /// Some blobs composed of springy welds and volume preservation constraints.
    /// </summary>
    public class BlobDemo : Demo
    {
        struct RayHitHandler : ICompoundRayHitHandler
        {
            public BufferPool Pool;
            public QuickList<float, Buffer<float>> Impacts;

            public RayHitHandler(BufferPool pool)
            {
                Pool = pool;
                QuickList<float, Buffer<float>>.Create(Pool.SpecializeFor<float>(), 8, out Impacts);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe void OnRayHit(int childIndex, float* maximumT, float t, in Vector3 normal)
            {

            }

            public void Dispose()
            {
                Impacts.Dispose(Pool.SpecializeFor<float>());
                this = new RayHitHandler();
            }
        }


        static void Voxelize(ContentArchive content, BufferPool pool, string modelName, in Vector3 scaling, int xCellCount, int yCellCount, int zCellCount)
        {
            MeshDemo.LoadModel(content, pool, modelName, scaling, out var mesh);
            //Grab the full tree bounds to anchor our voxelization grid.
            ref var root = ref mesh.Tree.Nodes[0];
            BoundingBox.CreateMerged(root.A.Min, root.A.Max, root.B.Min, root.B.Max, out var min, out var max);
            var span = max - min;
            var cellSize = new Vector3(span.X, span.Y, 0) / new Vector3(xCellCount, yCellCount, 1);
            var minimumRayOrigin = min + cellSize * 0.5f;
            var rayHitHandler = new RayHitHandler(pool);
            for (int x = 0; x < xCellCount; ++x)
            {
                for (int y = 0; y < yCellCount; ++y)
                {
                    var rayOrigin = minimumRayOrigin + cellSize * new Vector3(x, y, 0);
                    mesh.RayTest(RigidPose.Identity, rayOrigin, Vector3.UnitZ, span.Z, ref rayHitHandler);
                }
            }
            mesh.Dispose(pool);
            rayHitHandler.Dispose();
        }
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-30, 8, -110);
            camera.Yaw = MathHelper.Pi * 3f / 4;

            Simulation = Simulation.Create(BufferPool, new TestCallbacks());
            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);

            MeshDemo.LoadModel(content, BufferPool, "Content\\newt.obj", new Vector3(10), out var mesh);
            var box = new Box(25, 15, 50);
            box.ComputeInertia(1, out var inertia);
            Simulation.Bodies.Add(new BodyDescription
            {
                Pose = new RigidPose(new Vector3(0, 10, 0)),
                LocalInertia = inertia,
                Collidable = new CollidableDescription(Simulation.Shapes.Add(mesh), 0.1f),
                Activity = new BodyActivityDescription(0.001f)
            });


            var staticShape = new Box(1500, 1, 1500);
            var staticShapeIndex = Simulation.Shapes.Add(staticShape);

            var staticDescription = new StaticDescription
            {
                Collidable = new CollidableDescription
                {
                    Continuity = new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Discrete },
                    Shape = staticShapeIndex,
                    SpeculativeMargin = 0.1f
                },
                Pose = new RigidPose
                {
                    Position = new Vector3(1, -0.5f, 1),
                    Orientation = BepuUtilities.Quaternion.Identity
                }
            };
            Simulation.Statics.Add(staticDescription);

        }


    }
}
