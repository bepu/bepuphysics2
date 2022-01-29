using BepuUtilities;
using DemoRenderer;
using BepuPhysics;
using BepuPhysics.Collidables;
using System.Numerics;
using DemoContentLoader;
using BepuPhysics.Constraints;
using Demos.Demos;
using System;

namespace Demos.SpecializedTests
{
    public class ConstrainedKinematicIntegrationTest : Demo
    {
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(25, 4, 40);
            camera.Yaw = 0;
            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1), 2, 1), new DemoPoseIntegratorCallbacks(new Vector3(0, -0.1f, 0), 0, 0), new SolveDescription(8, 1));

            var shapeA = new Box(.75f, 1, .5f);
            var shapeIndexA = Simulation.Shapes.Add(shapeA);
            var collidableA = new CollidableDescription(shapeIndexA);
            var shapeB = new Box(.75f, 1, .5f);
            var shapeIndexB = Simulation.Shapes.Add(shapeB);
            var collidableB = new CollidableDescription(shapeIndexB);
            var activity = new BodyActivityDescription(0.01f);
            var inertiaA = shapeA.ComputeInertia(1);
            var inertiaB = shapeB.ComputeInertia(1);

            for (int i = 0; i < 32; ++i)
            {
                var x = 0;
                var z = i * 3;
                var a = Simulation.Bodies.Add(BodyDescription.CreateKinematic(new Vector3(x, 3, z), collidableA, activity));
                var b = Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(x, 5, z), inertiaB, collidableB, activity));
                Simulation.Bodies[a].Velocity.Linear = new Vector3(1, 0, 0);
                Simulation.Solver.Add(a, b, new BallSocket { LocalOffsetA = new Vector3(0, 1, 0), LocalOffsetB = new Vector3(0, -1, 0), SpringSettings = new SpringSettings(30, 1) });
                Simulation.Solver.Add(a, b, new AngularHinge { LocalHingeAxisA = new Vector3(0, 1, 0), LocalHingeAxisB = new Vector3(0, 1, 0), SpringSettings = new SpringSettings(30, 1) });
            }

            for (int i = 0; i < 32; ++i)
            {
                var x = 0;
                var z = i * 3;
                var a = Simulation.Bodies.Add(BodyDescription.CreateKinematic(new Vector3(x, 8, z), collidableA, activity));
                var b = Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(x, 8, z + 2), inertiaB, collidableB, activity));
                Simulation.Bodies[a].Velocity.Linear = new Vector3(1, 0, 0);
                Simulation.Bodies[b].Velocity.Linear = new Vector3(1, 0, 0);
                //Simulation.Solver.Add(a, b, new BallSocket { LocalOffsetA = new Vector3(0, 1, 0), LocalOffsetB = new Vector3(0, -1, 0), SpringSettings = new SpringSettings(30, 1) });
                //Simulation.Solver.Add(a, b, new AngularHinge { LocalHingeAxisA = new Vector3(0, 1, 0), LocalHingeAxisB = new Vector3(0, 1, 0), SpringSettings = new SpringSettings(30, 1) });
            }

            Simulation.Statics.Add(new StaticDescription(new Vector3(), Simulation.Shapes.Add(new Box(8192, 1, 8192))));
        }
    }
}


