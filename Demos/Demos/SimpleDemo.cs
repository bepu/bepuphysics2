using BepuUtilities;
using DemoRenderer;
using DemoUtilities;
using BepuPhysics;
using BepuPhysics.Collidables;
using System;
using System.Numerics;

namespace Demos
{
    public class SimpleDemo : Demo
    {
        public unsafe override void Initialize(Camera camera)
        {
            camera.Position = new Vector3(-3f, 3, -3f);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = MathHelper.Pi * 0.1f;
            Simulation = Simulation.Create(BufferPool, new TestCallbacks());
            var shape = new Sphere(0.5f);
            var shapeIndex = Simulation.Shapes.Add(ref shape);
            const int width = 18;
            const int height = 48;
            const int length = 18;
            SimulationSetup.BuildLattice(
                new RegularGridWithKinematicBaseBuilder(new Vector3(1.1f, 1.0f, 1.1f), new Vector3(1, 1, 1), 1f / (shape.Radius * shape.Radius * 2 / 3), shapeIndex),
                new ConstraintlessLatticeBuilder(),
                width, height, length, Simulation, out var bodyHandles, out var constraintHandles);
            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);
            Simulation.Deterministic = true;


            var staticShape = new Sphere(19);
            var staticShapeIndex = Simulation.Shapes.Add(ref staticShape);
            var staticDescription = new StaticDescription
            {
                Collidable = new CollidableDescription
                {
                    Continuity = new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Discrete },
                    Shape = staticShapeIndex,
                    SpeculativeMargin = 0.1f
                },
                Pose = new RigidPose { Position = new Vector3(0, -20, 0), Orientation = BepuUtilities.Quaternion.Identity }
            };
            Simulation.Add(ref staticDescription);

            ref var velocity = ref Simulation.Bodies.Velocities[Simulation.Bodies.HandleToIndex[bodyHandles[width]]];
            velocity.Linear = new Vector3(0.1f, 0, 0.1f);
            velocity.Angular = new Vector3();

            //Simulation.Solver.IterationCount = 100;

            Console.WriteLine(Simulation.Solver.ConstraintCount);

        }

        int frameIndex;
        public override void Update(Input input, float dt)
        {
            //Console.WriteLine($"Preframe {frameIndex++}, mapping count: {Simulation.NarrowPhase.PairCache.Mapping.Count}");

            //for (int i = 0; i < Simulation.Bodies.BodyCount; ++i)
            //{
            //    Simulation.Bodies.ValidateExistingHandle(Simulation.Bodies.IndexToHandle[i]);
            //}
            //if (input.WasPushed(OpenTK.Input.Key.P))
            //{
            //    unsafe { var accessViolationSuppressant = stackalloc int[0]; }
            //    BodyVelocity velocity;
            //    velocity.Linear = new Vector3(.1f, 0, 0.1f);
            //    velocity.Angular = new Vector3();
            //    Simulation.Bodies.SetVelocity(32, ref velocity);
            //}
            base.Update(input, dt);

        }

    }
}
