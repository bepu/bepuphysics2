using BepuUtilities;
using DemoRenderer;
using DemoUtilities;
using BepuPhysics;
using BepuPhysics.Collidables;
using System;
using System.Numerics;
using System.Diagnostics;
using BepuUtilities.Memory;
using BepuUtilities.Collections;
using DemoContentLoader;

namespace Demos
{
    public class ReactivateTests : Demo
    {
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-3f, 3, -3f);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = MathHelper.Pi * 0.1f;
            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)),
            initialAllocationSizes: new SimulationAllocationSizes
            {
                Bodies = 1,
                ConstraintCountPerBodyEstimate = 1,
                Constraints = 1,
                ConstraintsPerTypeBatch = 1,
                Islands = 1,
                ShapesPerType = 1,
                Statics = 1
            });

            var shape = new Sphere(0.5f);
            shape.ComputeInertia(1, out var sphereInertia);
            var shapeIndex = Simulation.Shapes.Add(shape);
            const int width = 12;
            const int height = 4;
            const int length = 12;
            var latticeSpacing = 5.1f;
            var latticeOffset = -0.5f * width * latticeSpacing;
            SimulationSetup.BuildLattice(
                new RegularGridBuilder(new Vector3(latticeSpacing, 1.5f, latticeSpacing), new Vector3(latticeOffset, 10, latticeOffset), sphereInertia, shapeIndex),
                new ConstraintlessLatticeBuilder(),
                width, height, length, Simulation, out var bodyHandles, out var constraintHandles);
            Simulation.Deterministic = false;

            var staticShape = new Sphere(4);
            var staticShapeIndex = Simulation.Shapes.Add(staticShape);
            const int staticGridWidthInSpheres = 100;
            const float staticSpacing = 6;
            for (int i = 0; i < staticGridWidthInSpheres; ++i)
            {
                for (int j = 0; j < staticGridWidthInSpheres; ++j)
                {
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
                            Position = new Vector3(
                            -staticGridWidthInSpheres * staticSpacing * 0.5f + i * staticSpacing,
                            -4,
                            -staticGridWidthInSpheres * staticSpacing * 0.5f + j * staticSpacing),
                            Orientation = BepuUtilities.Quaternion.Identity
                        }
                    };
                    Simulation.Statics.Add(staticDescription);
                }
            }


        }

        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            for (int iterationIndex = 0; iterationIndex < 100; ++iterationIndex)
            {
                var bodyIndicesToDeactivate = new QuickList<int>(Simulation.Bodies.ActiveSet.Count, BufferPool);
                for (int i = 0; i < Simulation.Bodies.ActiveSet.Count; ++i)
                {
                    bodyIndicesToDeactivate.AllocateUnsafely() = i;
                }
                Simulation.Sleeper.Sleep(ref bodyIndicesToDeactivate);

                bodyIndicesToDeactivate.Dispose(BufferPool);

                var setsToActivate = new QuickList<int>(Simulation.Bodies.Sets.Length, BufferPool);
                for (int i = 1; i < Simulation.Bodies.Sets.Length; ++i)
                {
                    if (Simulation.Bodies.Sets[i].Allocated)
                    {
                        setsToActivate.AllocateUnsafely() = i;
                    }
                }

                Simulation.Awakener.AwakenSets(ref setsToActivate);
                setsToActivate.Dispose(BufferPool);

            }

            base.Update(window, camera, input, dt);

        }

    }
}
