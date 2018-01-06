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

namespace Demos
{
    public class SimpleDemo : Demo
    {
        public unsafe override void Initialize(Camera camera)
        {
            camera.Position = new Vector3(-3f, 3, -3f);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = MathHelper.Pi * 0.1f;
            Simulation = Simulation.Create(BufferPool, new TestCallbacks(),
            new SimulationAllocationSizes
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
            var shapeIndex = Simulation.Shapes.Add(ref shape);
            const int width = 32;
            const int height = 32;
            const int length = 32;
            var latticeSpacing = 5.1f;
            var latticeOffset = -0.5f * width * latticeSpacing;
            SimulationSetup.BuildLattice(
                new RegularGridBuilder(new Vector3(latticeSpacing, 1.5f, latticeSpacing), new Vector3(latticeOffset, 10, latticeOffset), 1f / (shape.Radius * shape.Radius * 2 / 3), shapeIndex),
                new ConstraintlessLatticeBuilder(),
                width, height, length, Simulation, out var bodyHandles, out var constraintHandles);
            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);
            Simulation.Deterministic = false;

            var staticShape = new Sphere(4);
            var staticShapeIndex = Simulation.Shapes.Add(ref staticShape);
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
                    Simulation.Statics.Add(ref staticDescription);
                }
            }

            //ref var velocity = ref Simulation.Bodies.Velocities[Simulation.Bodies.HandleToIndex[bodyHandles[width]]];
            //velocity.Linear = new Vector3(0.1f, 0, 0.1f);
            //velocity.Angular = new Vector3();

            //Simulation.Solver.IterationCount = 100;

        }

        int frameIndex;
        public override void Update(Input input, float dt)
        {
            Console.WriteLine($"Preframe {frameIndex++}");
            if (input.WasPushed(OpenTK.Input.Key.P))
            {
                for (int iterationIndex = 0; iterationIndex < 100; ++iterationIndex)
                {
                    QuickList<int, Buffer<int>>.Create(BufferPool.SpecializeFor<int>(), Simulation.Bodies.ActiveSet.Count, out var bodyIndicestoDeactivate);
                    for (int i = 0; i < Simulation.Bodies.ActiveSet.Count; ++i)
                    {
                        bodyIndicestoDeactivate.AllocateUnsafely() = i;
                    }
                    Simulation.Deactivator.Deactivate(ref bodyIndicestoDeactivate, ThreadDispatcher);

                    bodyIndicestoDeactivate.Dispose(BufferPool.SpecializeFor<int>());

                    QuickList<int, Buffer<int>>.Create(BufferPool.SpecializeFor<int>(), Simulation.Bodies.Sets.Length, out var setsToActivate);
                    for (int i = 1; i < Simulation.Bodies.Sets.Length; ++i)
                    {
                        if (Simulation.Bodies.Sets[i].Allocated)
                        {
                            setsToActivate.AllocateUnsafely() = i;
                        }
                    }

                    Simulation.Activator.ActivateSets(ref setsToActivate, ThreadDispatcher);
                    setsToActivate.Dispose(BufferPool.SpecializeFor<int>());

                }
            }

            //if (input.WasPushed(OpenTK.Input.Key.P))
            //{
            //    QuickList<int, Buffer<int>>.Create(BufferPool.SpecializeFor<int>(), Simulation.Bodies.Sets.Length, out var setsToActivate);
            //    for (int i = 1; i < Simulation.Bodies.Sets.Length; ++i)
            //    {
            //        if (Simulation.Bodies.Sets[i].Allocated)
            //        {
            //            setsToActivate.AllocateUnsafely() = i;
            //        }
            //    }

            //    var start = Stopwatch.GetTimestamp();
            //    Simulation.Activator.ActivateSets(ref setsToActivate, ThreadDispatcher);
            //    var end = Stopwatch.GetTimestamp();
            //    setsToActivate.Dispose(BufferPool.SpecializeFor<int>());
            //    Console.WriteLine($"{setsToActivate.Count} activations, time (ms): {(end - start) * 1e3 / Stopwatch.Frequency}");
            //}
            //if (input.WasPushed(OpenTK.Input.Key.O))
            //{
            //    QuickList<int, Buffer<int>>.Create(BufferPool.SpecializeFor<int>(), Simulation.Bodies.ActiveSet.Count, out var bodyIndicestoDeactivate);
            //    for (int i = 0; i < Simulation.Bodies.ActiveSet.Count; ++i)
            //    {
            //        bodyIndicestoDeactivate.AllocateUnsafely() = i;
            //    }
            //    var start = Stopwatch.GetTimestamp();
            //    Simulation.Deactivator.Deactivate(ref bodyIndicestoDeactivate, ThreadDispatcher);
            //    var end = Stopwatch.GetTimestamp();
            //bodyIndicestoDeactivate.Dispose(BufferPool.SpecializeFor<int>());
            //    Console.WriteLine($"{bodyIndicestoDeactivate.Count} deactivations, time (ms): {(end - start) * 1e3 / Stopwatch.Frequency}");
            //}

            //for (int i = 0; i < Simulation.Bodies.BodyCount; ++i)
            //{
            //    Simulation.Bodies.ValidateExistingHandle(Simulation.Bodies.IndexToHandle[i]);
            //}
            //if (input.WasPushed(OpenTK.Input.Key.P))
            //{
            //    Console.WriteLine("stoppls");
            //}
            base.Update(input, dt);

        }

    }
}
