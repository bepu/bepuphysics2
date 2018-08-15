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
using BepuPhysics.Constraints;
using Quaternion = BepuUtilities.Quaternion;

namespace Demos.Demos
{
    public class ConstraintTestDemo : Demo
    {
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 4, -10);
            camera.Yaw = MathHelper.Pi;
            Simulation = Simulation.Create(BufferPool, new TestCallbacks());

            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);

            var shapeA = new Box(.75f, 1, .5f);
            var shapeB = new Box(.75f, 1, .5f);
            shapeA.ComputeInertia(1, out var inertiaA);
            shapeA.ComputeInertia(1, out var inertiaB);
            {
                var a = Simulation.Bodies.Add(new BodyDescription(new Vector3(-10, 5, 0), inertiaA, Simulation.Shapes.Add(shapeA), 0.1f, new BodyActivityDescription(0.01f)));
                var b = Simulation.Bodies.Add(new BodyDescription(new Vector3(-10, 3, 0), inertiaB, Simulation.Shapes.Add(shapeB), 0.1f, new BodyActivityDescription(0.01f)));
                Simulation.Solver.Add(a, b, new BallSocket { LocalOffsetA = new Vector3(0, -1, 0), LocalOffsetB = new Vector3(0, 1, 0), SpringSettings = new SpringSettings(30, 1) });
            }
            {
                var a = Simulation.Bodies.Add(new BodyDescription(new Vector3(-7, 5, 0), inertiaA, Simulation.Shapes.Add(shapeA), 0.1f, new BodyActivityDescription(0.01f)));
                var b = Simulation.Bodies.Add(new BodyDescription(new Vector3(-7, 3, 0), inertiaB, Simulation.Shapes.Add(shapeB), 0.1f, new BodyActivityDescription(0.01f)));
                Simulation.Solver.Add(a, b, new BallSocket { LocalOffsetA = new Vector3(0, -1, 0), LocalOffsetB = new Vector3(0, 1, 0), SpringSettings = new SpringSettings(30, 1) });
                Simulation.Solver.Add(a, b, new AngularHinge { HingeAxisLocalA = new Vector3(0, 1, 0), HingeAxisLocalB = new Vector3(0, 1, 0), SpringSettings = new SpringSettings(30, 1) });
            }
            {
                var a = Simulation.Bodies.Add(new BodyDescription(new Vector3(-4, 5, 0), inertiaA, Simulation.Shapes.Add(shapeA), 0.1f, new BodyActivityDescription(0.01f)));
                var b = Simulation.Bodies.Add(new BodyDescription(new Vector3(-4, 3, 0), inertiaB, Simulation.Shapes.Add(shapeB), 0.1f, new BodyActivityDescription(0.01f)));
                Simulation.Solver.Add(a, b, new BallSocket { LocalOffsetA = new Vector3(0, -1, 0), LocalOffsetB = new Vector3(0, 1, 0), SpringSettings = new SpringSettings(30, 1) });
                Simulation.Solver.Add(a, b, new AngularSwivelHinge { SwivelAxisLocalA = new Vector3(0, 1, 0), HingeAxisLocalB = new Vector3(1, 0, 0), SpringSettings = new SpringSettings(30, 1) });
            }
            {
                var a = Simulation.Bodies.Add(new BodyDescription(new Vector3(-1, 5, 0), inertiaA, Simulation.Shapes.Add(shapeA), 0.1f, new BodyActivityDescription(0.01f)));
                var b = Simulation.Bodies.Add(new BodyDescription(new Vector3(-1, 3, 0), inertiaB, Simulation.Shapes.Add(shapeB), 0.1f, new BodyActivityDescription(0.01f)));
                Simulation.Solver.Add(a, b, new BallSocket { LocalOffsetA = new Vector3(0, -1, 0), LocalOffsetB = new Vector3(0, 1, 0), SpringSettings = new SpringSettings(30, 1) });
                Simulation.Solver.Add(a, b, new TwistServo
                {
                    LocalBasisA = Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), MathHelper.PiOver2),
                    LocalBasisB = Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), MathHelper.PiOver2),
                    TargetAngle = 0,
                    SpringSettings = new SpringSettings(30, 1),
                    ServoSettings = new ServoSettings(float.MaxValue, 0, float.MaxValue)
                });
            }
            {
                var a = Simulation.Bodies.Add(new BodyDescription(new Vector3(2, 5, 0), inertiaA, Simulation.Shapes.Add(shapeA), 0.1f, new BodyActivityDescription(0.01f)));
                var b = Simulation.Bodies.Add(new BodyDescription(new Vector3(2, 3, 0), new BodyInertia(), Simulation.Shapes.Add(shapeB), 0.1f, new BodyActivityDescription(0.01f)));
                Simulation.Solver.Add(a, b, new BallSocket { LocalOffsetA = new Vector3(0, -1, 0), LocalOffsetB = new Vector3(0, 1, 0), SpringSettings = new SpringSettings(30, 1) });
                Simulation.Solver.Add(a, b, new TwistLimit
                {
                    LocalBasisA = Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), MathHelper.PiOver2),
                    LocalBasisB = Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), MathHelper.PiOver2),
                    MinimumAngle = -MathHelper.PiOver2,
                    MaximumAngle = MathHelper.PiOver2,
                    SpringSettings = new SpringSettings(30, 1),
                });
                Simulation.Solver.Add(a, b, new AngularHinge { HingeAxisLocalA = new Vector3(0, 1, 0), HingeAxisLocalB = new Vector3(0, 1, 0), SpringSettings = new SpringSettings(30, 1) });
            }

            Simulation.Statics.Add(new StaticDescription(new Vector3(), new CollidableDescription(Simulation.Shapes.Add(new Box(256, 1, 256)), 0.1f)));


        }

    }
}


