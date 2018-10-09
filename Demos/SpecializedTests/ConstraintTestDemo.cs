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
using Demos.Demos;

namespace Demos.SpecializedTests
{
    public class ConstraintTestDemo : Demo
    {
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 4, 15);
            camera.Yaw = 0;
            Simulation = Simulation.Create(BufferPool, new TestCallbacks());

            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);

            var shapeA = new Box(.75f, 1, .5f);
            var shapeB = new Box(.75f, 1, .5f);
            shapeA.ComputeInertia(1, out var inertiaA);
            shapeA.ComputeInertia(1, out var inertiaB);
            {
                var a = Simulation.Bodies.Add(new BodyDescription(new Vector3(-10, 3, 0), inertiaA, Simulation.Shapes.Add(shapeA), 0.1f, new BodyActivityDescription(0.01f)));
                var b = Simulation.Bodies.Add(new BodyDescription(new Vector3(-10, 5, 0), inertiaB, Simulation.Shapes.Add(shapeB), 0.1f, new BodyActivityDescription(0.01f)));
                Simulation.Solver.Add(a, b, new BallSocket { LocalOffsetA = new Vector3(0, 1, 0), LocalOffsetB = new Vector3(0, -1, 0), SpringSettings = new SpringSettings(30, 1) });
            }
            {
                var a = Simulation.Bodies.Add(new BodyDescription(new Vector3(-7, 3, 0), new BodyInertia(), Simulation.Shapes.Add(shapeA), 0.1f, new BodyActivityDescription(0.01f)));
                var b = Simulation.Bodies.Add(new BodyDescription(new Vector3(-7, 5, 0), inertiaB, Simulation.Shapes.Add(shapeB), 0.1f, new BodyActivityDescription(0.01f)));
                Simulation.Solver.Add(a, b, new BallSocket { LocalOffsetA = new Vector3(0, 1, 0), LocalOffsetB = new Vector3(0, -1, 0), SpringSettings = new SpringSettings(30, 1) });
                Simulation.Solver.Add(a, b, new AngularHinge { HingeAxisLocalA = new Vector3(0, 1, 0), HingeAxisLocalB = new Vector3(0, 1, 0), SpringSettings = new SpringSettings(30, 1) });
            }
            {
                var a = Simulation.Bodies.Add(new BodyDescription(new Vector3(-4, 3, 0), new BodyInertia(), Simulation.Shapes.Add(shapeA), 0.1f, new BodyActivityDescription(0.01f)));
                var b = Simulation.Bodies.Add(new BodyDescription(new Vector3(-4, 5, 0), inertiaB, Simulation.Shapes.Add(shapeB), 0.1f, new BodyActivityDescription(0.01f)));
                Simulation.Solver.Add(a, b, new BallSocket { LocalOffsetA = new Vector3(0, 1, 0), LocalOffsetB = new Vector3(0, -1, 0), SpringSettings = new SpringSettings(30, 1) });
                Simulation.Solver.Add(a, b, new AngularSwivelHinge { SwivelAxisLocalA = new Vector3(0, 1, 0), HingeAxisLocalB = new Vector3(1, 0, 0), SpringSettings = new SpringSettings(30, 1) });
                Simulation.Solver.Add(a, b, new SwingLimit { AxisLocalA = new Vector3(0, 1, 0), AxisLocalB = new Vector3(0, 1, 0), MaximumSwingAngle = MathHelper.PiOver2, SpringSettings = new SpringSettings(30, 1) });
            }
            {
                var a = Simulation.Bodies.Add(new BodyDescription(new Vector3(-1, 3, 0), new BodyInertia(), Simulation.Shapes.Add(shapeA), 0.1f, new BodyActivityDescription(0.01f)));
                var b = Simulation.Bodies.Add(new BodyDescription(new Vector3(-1, 5, 0), inertiaB, Simulation.Shapes.Add(shapeB), 0.1f, new BodyActivityDescription(0.01f)));
                Simulation.Solver.Add(a, b, new BallSocket { LocalOffsetA = new Vector3(0, 1, 0), LocalOffsetB = new Vector3(0, -1, 0), SpringSettings = new SpringSettings(30, 1) });
                Simulation.Solver.Add(a, b, new TwistServo
                {
                    LocalBasisA = RagdollDemo.CreateBasis(new Vector3(0, 1, 0), new Vector3(1, 0, 0)),
                    LocalBasisB = RagdollDemo.CreateBasis(new Vector3(0, 1, 0), new Vector3(1, 0, 0)),
                    TargetAngle = MathHelper.PiOver4,
                    SpringSettings = new SpringSettings(30, 1),
                    ServoSettings = new ServoSettings(float.MaxValue, 0, float.MaxValue)
                });
            }
            {
                var a = Simulation.Bodies.Add(new BodyDescription(new Vector3(2, 3, 0), new BodyInertia(), Simulation.Shapes.Add(shapeA), 0.1f, new BodyActivityDescription(0.01f)));
                var b = Simulation.Bodies.Add(new BodyDescription(new Vector3(2, 5, 0), inertiaB, Simulation.Shapes.Add(shapeB), 0.1f, new BodyActivityDescription(0.01f)));
                Simulation.Solver.Add(a, b, new BallSocket { LocalOffsetA = new Vector3(0, 1, 0), LocalOffsetB = new Vector3(0, -1, 0), SpringSettings = new SpringSettings(30, 1) });
                Simulation.Solver.Add(a, b, new TwistLimit
                {
                    LocalBasisA = RagdollDemo.CreateBasis(new Vector3(0, 1, 0), new Vector3(1, 0, 0)),
                    LocalBasisB = RagdollDemo.CreateBasis(new Vector3(0, 1, 0), new Vector3(1, 0, 0)),
                    MinimumAngle = MathHelper.Pi * -0.5f,
                    MaximumAngle = MathHelper.Pi * 0.95f,
                    SpringSettings = new SpringSettings(30, 1),
                });
                Simulation.Solver.Add(a, b, new AngularHinge { HingeAxisLocalA = new Vector3(0, 1, 0), HingeAxisLocalB = new Vector3(0, 1, 0), SpringSettings = new SpringSettings(30, 1) });
            }
            {
                var a = Simulation.Bodies.Add(new BodyDescription(new Vector3(5, 3, 0), new BodyInertia(), Simulation.Shapes.Add(shapeA), 0.1f, new BodyActivityDescription(0.01f)));
                var b = Simulation.Bodies.Add(new BodyDescription(new Vector3(5, 5, 0), inertiaB, Simulation.Shapes.Add(shapeB), 0.1f, new BodyActivityDescription(0.01f)));
                Simulation.Solver.Add(a, b, new BallSocket { LocalOffsetA = new Vector3(0, 1, 0), LocalOffsetB = new Vector3(0, -1, 0), SpringSettings = new SpringSettings(30, 1) });
                Simulation.Solver.Add(a, b, new TwistMotor
                {
                    LocalAxisA = new Vector3(0, 1, 0),
                    LocalAxisB = new Vector3(0, 1, 0),
                    TargetVelocity = MathHelper.Pi * 2,
                    Settings = new MotorSettings(float.MaxValue, 0.1f)
                });
                Simulation.Solver.Add(a, b, new AngularHinge { HingeAxisLocalA = new Vector3(0, 1, 0), HingeAxisLocalB = new Vector3(0, 1, 0), SpringSettings = new SpringSettings(30, 1) });
            }
            {
                var a = Simulation.Bodies.Add(new BodyDescription(new Vector3(8, 3, 0), new BodyInertia(), Simulation.Shapes.Add(shapeA), 0.1f, new BodyActivityDescription(0.01f)));
                var b = Simulation.Bodies.Add(new BodyDescription(new Vector3(8, 5, 0), inertiaB, Simulation.Shapes.Add(shapeB), 0.1f, new BodyActivityDescription(0.01f)));
                Simulation.Solver.Add(a, b, new BallSocket { LocalOffsetA = new Vector3(0, 1, 0), LocalOffsetB = new Vector3(0, -1, 0), SpringSettings = new SpringSettings(30, 1) });
                Simulation.Solver.Add(a, b, new AngularServo
                {
                    TargetRelativeRotationLocalA = Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), MathHelper.PiOver2),
                    ServoSettings = new ServoSettings(float.MaxValue, 0, 12f),
                    SpringSettings = new SpringSettings(30, 1)
                });
            }
            {
                var a = Simulation.Bodies.Add(new BodyDescription(new Vector3(11, 3, 0), inertiaA, Simulation.Shapes.Add(shapeA), 0.1f, new BodyActivityDescription(0.01f)));
                var b = Simulation.Bodies.Add(new BodyDescription(new Vector3(11, 5, 0), inertiaB, Simulation.Shapes.Add(shapeB), 0.1f, new BodyActivityDescription(0.01f)));
                Simulation.Solver.Add(a, b, new BallSocket { LocalOffsetA = new Vector3(0, 1, 0), LocalOffsetB = new Vector3(0, -1, 0), SpringSettings = new SpringSettings(30, 1) });
                Simulation.Solver.Add(a, b, new AngularMotor { TargetVelocityLocalA = new Vector3(0, 1, 0), Settings = new MotorSettings(15, 0.0001f) });
            }
            {
                var aDescription = new BodyDescription(new Vector3(14, 3, 0), inertiaA, Simulation.Shapes.Add(shapeA), 0.1f, new BodyActivityDescription(0.01f));
                var bDescription = new BodyDescription(new Vector3(14, 5, 0), inertiaB, Simulation.Shapes.Add(shapeB), 0.1f, new BodyActivityDescription(0.01f));
                //aDescription.Velocity.Angular = new Vector3(0, 0, 5);
                var a = Simulation.Bodies.Add(aDescription);
                var b = Simulation.Bodies.Add(bDescription);
                Simulation.Solver.Add(a, b, new Weld { LocalOffset = new Vector3(0, 2, 0), LocalOrientation = Quaternion.Identity, SpringSettings = new SpringSettings(30, 1) });
            }
            {
                var aDescription = new BodyDescription(new Vector3(17, 3, 0), inertiaA, Simulation.Shapes.Add(shapeA), 0.1f, new BodyActivityDescription(0.01f));
                var bDescription = new BodyDescription(new Vector3(17, 5, 0), inertiaB, Simulation.Shapes.Add(shapeB), 0.1f, new BodyActivityDescription(0.01f));
                //aDescription.Velocity.Angular = new Vector3(0, 0, 5);
                var a = Simulation.Bodies.Add(aDescription);
                var b = Simulation.Bodies.Add(bDescription);
                Simulation.Solver.Add(a, b, new Weld2 { LocalOffset = new Vector3(0, 2, 0), LocalOrientation = Quaternion.Identity, SpringSettings = new SpringSettings(30, 1) });
            }

            Simulation.Statics.Add(new StaticDescription(new Vector3(), new CollidableDescription(Simulation.Shapes.Add(new Box(256, 1, 256)), 0.1f)));


        }

        public override void Update(Input input, float dt)
        {
            if (input.WasPushed(OpenTK.Input.Key.P))
            {
                Console.Write($"4df");
            }
            base.Update(input, dt);
        }

    }
}


