using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using Quaternion = BepuUtilities.Quaternion;

namespace Demos.Demos
{
    struct ClothCollisionFilter
    {
        ushort x;
        ushort y;
        int instanceId;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ClothCollisionFilter(int x, int y, int instanceId)
        {
            const int max = 1 << 16;
            Debug.Assert(x >= 0 && x < max && y >= 0 && y < max, "This filter packs local indices, so their range is limited.");
            this.x = (ushort)x;
            this.y = (ushort)y;
            this.instanceId = instanceId;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool Test(in ClothCollisionFilter a, in ClothCollisionFilter b)
        {
            if (a.instanceId != b.instanceId)
                return true;
            //Disallow collisions between vertices which are near each other. We measure distance as max(abs(ax - bx), abs(ay - by), abs(az - bz)).
            const int minimumDistance = 1;
            var differenceX = a.x - b.x;
            if (differenceX < -minimumDistance || differenceX > minimumDistance)
                return true;
            var differenceY = a.y - b.y;
            if (differenceY < -minimumDistance || differenceY > minimumDistance)
                return true;
            return false;
        }
    }


    struct ClothCallbacks : INarrowPhaseCallbacks
    {
        public BodyProperty<ClothCollisionFilter> Filters;
        public void Initialize(Simulation simulation)
        {
            Filters.Initialize(simulation.Bodies);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b)
        {
            if (a.Mobility != CollidableMobility.Static && b.Mobility != CollidableMobility.Static)
            {
                return ClothCollisionFilter.Test(Filters[a.Handle], Filters[b.Handle]);
            }
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
        {
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void ConfigureMaterial(out PairMaterialProperties pairMaterial)
        {
            pairMaterial.FrictionCoefficient = 0.25f;
            pairMaterial.MaximumRecoveryVelocity = 2f;
            pairMaterial.SpringSettings = new SpringSettings(30, 1);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, NonconvexContactManifold* manifold, out PairMaterialProperties pairMaterial)
        {
            ConfigureMaterial(out pairMaterial);
            return true;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, ConvexContactManifold* manifold, out PairMaterialProperties pairMaterial)
        {
            ConfigureMaterial(out pairMaterial);
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ConvexContactManifold* manifold)
        {
            return true;
        }

        public void Dispose()
        {
            Filters.Dispose();
        }
    }

    /// <summary>
    /// Shows a few different examples of cloth-ish constraint lattices.
    /// </summary>
    public class ClothDemo : Demo
    {
        delegate bool KinematicDecider(int rowIndex, int columnIndex, int width, int height);

        int[,] CreateBodyGrid(in Vector3 position, in Quaternion orientation, int width, int height, float spacing, float bodyRadius, float massPerBody,
            int instanceId, BodyProperty<ClothCollisionFilter> filters, KinematicDecider isKinematic)
        {
            var description = new BodyDescription
            {
                Activity = new BodyActivityDescription(0.01f),
                Collidable = new CollidableDescription(Simulation.Shapes.Add(new Sphere(bodyRadius)), 0.1f),
                LocalInertia = default,
                Pose = new RigidPose(default, orientation)
            };
            var inverseMass = 1f / massPerBody;
            int[,] handles = new int[height, width];
            for (int rowIndex = 0; rowIndex < height; ++rowIndex)
            {
                for (int columnIndex = 0; columnIndex < width; ++columnIndex)
                {
                    description.LocalInertia.InverseMass = isKinematic(rowIndex, columnIndex, width, height) ? 0 : inverseMass;
                    var localPosition = new Vector3(columnIndex * spacing, rowIndex * -spacing, 0);
                    Quaternion.TransformWithoutOverlap(localPosition, orientation, out var rotatedPosition);
                    description.Pose.Position = rotatedPosition + position;
                    var handle = Simulation.Bodies.Add(description);
                    handles[rowIndex, columnIndex] = handle;
                    filters.Allocate(handle) = new ClothCollisionFilter(rowIndex, columnIndex, instanceId);
                }
            }
            return handles;
        }

        void CreateAreaConstraints(int[,] bodyHandles, SpringSettings springSettings)
        {
            for (int rowIndex = 0; rowIndex < bodyHandles.GetLength(0) - 1; ++rowIndex)
            {
                for (int columnIndex = 0; columnIndex < bodyHandles.GetLength(1) - 1; ++columnIndex)
                {
                    var aHandle = bodyHandles[rowIndex, columnIndex];
                    var bHandle = bodyHandles[rowIndex + 1, columnIndex];
                    var cHandle = bodyHandles[rowIndex, columnIndex + 1];
                    var dHandle = bodyHandles[rowIndex + 1, columnIndex + 1];
                    var a = new BodyReference(aHandle, Simulation.Bodies);
                    var b = new BodyReference(bHandle, Simulation.Bodies);
                    var c = new BodyReference(cHandle, Simulation.Bodies);
                    var d = new BodyReference(dHandle, Simulation.Bodies);
                    //Not worried about kinematics here- we create at most one row of kinematics in this demo. These are three body constraints that operate in a local quad, so 
                    //there's no way for them to all be kinematic.
                    Simulation.Solver.Add(aHandle, bHandle, cHandle, new AreaConstraint(a.Pose.Position, b.Pose.Position, c.Pose.Position, springSettings));
                    Simulation.Solver.Add(bHandle, cHandle, dHandle, new AreaConstraint(b.Pose.Position, c.Pose.Position, d.Pose.Position, springSettings));
                }
            }
        }
        void CreateDistanceConstraints(int[,] bodyHandles, SpringSettings springSettings)
        {
            void CreateConstraintBetweenBodies(int aHandle, int bHandle)
            {
                var a = new BodyReference(aHandle, Simulation.Bodies);
                var b = new BodyReference(bHandle, Simulation.Bodies);
                //Don't create constraints between two kinematic bodies.
                if (a.LocalInertia.InverseMass > 0 || b.LocalInertia.InverseMass > 0)
                {
                    Simulation.Solver.Add(aHandle, bHandle, new CenterDistanceConstraint(Vector3.Distance(a.Pose.Position, b.Pose.Position), springSettings));
                }
            }
            for (int rowIndex = 0; rowIndex < bodyHandles.GetLength(0); ++rowIndex)
            {
                for (int columnIndex = 0; columnIndex < bodyHandles.GetLength(1) - 1; ++columnIndex)
                {
                    CreateConstraintBetweenBodies(bodyHandles[rowIndex, columnIndex], bodyHandles[rowIndex, columnIndex + 1]);
                }
            }
            for (int rowIndex = 0; rowIndex < bodyHandles.GetLength(0) - 1; ++rowIndex)
            {
                for (int columnIndex = 0; columnIndex < bodyHandles.GetLength(1); ++columnIndex)
                {
                    CreateConstraintBetweenBodies(bodyHandles[rowIndex, columnIndex], bodyHandles[rowIndex + 1, columnIndex]);
                }
            }
            for (int rowIndex = 0; rowIndex < bodyHandles.GetLength(0) - 1; ++rowIndex)
            {
                for (int columnIndex = 0; columnIndex < bodyHandles.GetLength(1) - 1; ++columnIndex)
                {
                    CreateConstraintBetweenBodies(bodyHandles[rowIndex, columnIndex], bodyHandles[rowIndex + 1, columnIndex + 1]);
                    CreateConstraintBetweenBodies(bodyHandles[rowIndex, columnIndex + 1], bodyHandles[rowIndex + 1, columnIndex]);
                }
            }
        }

        RolloverInfo rolloverInfo;

        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 25, 100);
            camera.Yaw = 0;
            camera.Pitch = 0;

            var filters = new BodyProperty<ClothCollisionFilter>();
            Simulation = Simulation.Create(BufferPool, new ClothCallbacks() { Filters = filters }, new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));
            rolloverInfo = new RolloverInfo();

            bool KinematicTopCorners(int rowIndex, int columnIndex, int width, int height)
            {
                return rowIndex == 0 && (columnIndex == width - 1 || columnIndex == 0);
            }
            bool FullyDynamic(int rowIndex, int columnIndex, int width, int height)
            {
                return false;
            }

            int clothInstanceId = 0;
            var initialRotation = Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), MathF.PI * -0.5f);
            {
                var position = new Vector3(-90, 40, 0);
                var handles = CreateBodyGrid(position, initialRotation, 10, 30, 1f, 0.65f, 1, clothInstanceId++, filters, KinematicTopCorners);
                CreateDistanceConstraints(handles, new SpringSettings(20, 1));
                rolloverInfo.Add(position + new Vector3(5, 2, 0), "Stiff distance constraints only, no area constraints");
            }
            {
                var position = new Vector3(-70, 40, 0);
                var handles = CreateBodyGrid(position, initialRotation, 10, 30, 1f, 0.65f, 1, clothInstanceId++, filters, KinematicTopCorners);
                CreateDistanceConstraints(handles, new SpringSettings(20, 1));
                CreateAreaConstraints(handles, new SpringSettings(30, 1));
                rolloverInfo.Add(position + new Vector3(5, 2, 0), "Stiff distance constraints with area constraints");
            }
            {
                var position = new Vector3(-50, 40, 0);
                var handles = CreateBodyGrid(position, initialRotation, 10, 30, 1f, 0.65f, 1, clothInstanceId++, filters, KinematicTopCorners);
                CreateDistanceConstraints(handles, new SpringSettings(3, 1));
                rolloverInfo.Add(position + new Vector3(5, 2, 0), "Soft distance constraints only, no area constraints");
            }
            {
                var position = new Vector3(-30, 40, 0);
                var handles = CreateBodyGrid(position, initialRotation, 10, 30, 1f, 0.65f, 1, clothInstanceId++, filters, KinematicTopCorners);
                CreateDistanceConstraints(handles, new SpringSettings(3, 1));
                CreateAreaConstraints(handles, new SpringSettings(30, 1));
                rolloverInfo.Add(position + new Vector3(5, 2, 0), "Soft distance constraints with area constraints (note narrowing)");
            }


            Simulation.Statics.Add(new StaticDescription(
                new Vector3(60, 20, 0), Quaternion.CreateFromAxisAngle(new Vector3(0, 0, 1), MathF.PI * 0.5f),
                new CollidableDescription(Simulation.Shapes.Add(new Capsule(8, 120)), 0.1f)));
            Simulation.Statics.Add(new StaticDescription(
                new Vector3(30, 5, 0), Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), MathF.PI * 0.5f),
                new CollidableDescription(Simulation.Shapes.Add(new Capsule(8, 60)), 0.1f)));

            {
                var position = new Vector3(10, 40, -32);
                var handles = CreateBodyGrid(position, initialRotation, 96, 96, 0.666f, 0.5f, 1, clothInstanceId++, filters, FullyDynamic);
                CreateDistanceConstraints(handles, new SpringSettings(10, 1));
                CreateAreaConstraints(handles, new SpringSettings(30, 1));
                rolloverInfo.Add(position + new Vector3(32, 2, 0), "Medium stiffness with area constraints");
            }

            Simulation.Statics.Add(new StaticDescription(new Vector3(-40, 0, 0), new CollidableDescription(Simulation.Shapes.Add(new Box(200, 1, 200)), 0.1f)));

        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            rolloverInfo.Render(renderer, camera, input, text, font);
            base.Render(renderer, camera, input, text, font);
        }

    }
}
