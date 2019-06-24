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
        static float GetNextPosition(ref float x)
        {
            var toReturn = x;
            x += 3;
            return toReturn;
        }

        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(25, 4, 40);
            camera.Yaw = 0;
            Simulation = Simulation.Create(BufferPool,
                new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));
            
            var shapeA = new Box(.75f, 1, .5f);
            var shapeIndexA = Simulation.Shapes.Add(shapeA);
            var collidableA = new CollidableDescription(shapeIndexA, 0.1f);
            var shapeB = new Box(.75f, 1, .5f);
            var shapeIndexB = Simulation.Shapes.Add(shapeB);
            var collidableB = new CollidableDescription(shapeIndexB, 0.1f);
            var activity = new BodyActivityDescription(0.01f);
            shapeA.ComputeInertia(1, out var inertiaA);
            shapeA.ComputeInertia(1, out var inertiaB);
            var nextX = -10f;
            {
                var x = GetNextPosition(ref nextX);
                var a = Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(x, 3, 0), inertiaA, collidableA, activity));
                var b = Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(x, 5, 0), inertiaB, collidableB, activity));
                Simulation.Solver.Add(a, b, new BallSocket { LocalOffsetA = new Vector3(0, 1, 0), LocalOffsetB = new Vector3(0, -1, 0), SpringSettings = new SpringSettings(30, 1) });
            }
            {
                var x = GetNextPosition(ref nextX);
                var a = Simulation.Bodies.Add(BodyDescription.CreateKinematic(new Vector3(x, 3, 0), collidableA, activity));
                var b = Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(x, 5, 0), inertiaB, collidableB, activity));
                Simulation.Solver.Add(a, b, new BallSocket { LocalOffsetA = new Vector3(0, 1, 0), LocalOffsetB = new Vector3(0, -1, 0), SpringSettings = new SpringSettings(30, 1) });
                Simulation.Solver.Add(a, b, new AngularHinge { LocalHingeAxisA = new Vector3(0, 1, 0), LocalHingeAxisB = new Vector3(0, 1, 0), SpringSettings = new SpringSettings(30, 1) });
            }
            {
                var x = GetNextPosition(ref nextX);
                var a = Simulation.Bodies.Add(BodyDescription.CreateKinematic(new Vector3(x, 3, 0), collidableA, activity));
                var b = Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(x, 5, 0), inertiaB, collidableB, activity));
                Simulation.Solver.Add(a, b, new Hinge
                {
                    LocalOffsetA = new Vector3(0, 1, 0),
                    LocalHingeAxisA = new Vector3(0, 1, 0),
                    LocalOffsetB = new Vector3(0, -1, 0),
                    LocalHingeAxisB = new Vector3(0, 1, 0),
                    SpringSettings = new SpringSettings(30, 1)
                });
            }
            {
                var x = GetNextPosition(ref nextX);
                var a = Simulation.Bodies.Add(BodyDescription.CreateKinematic(new Vector3(x, 3, 0), collidableA, activity));
                var b = Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(x, 5, 0), inertiaB, collidableB, activity));
                Simulation.Solver.Add(a, b, new BallSocket { LocalOffsetA = new Vector3(0, 1, 0), LocalOffsetB = new Vector3(0, -1, 0), SpringSettings = new SpringSettings(30, 1) });
                Simulation.Solver.Add(a, b, new AngularSwivelHinge { LocalSwivelAxisA = new Vector3(1, 0, 0), LocalHingeAxisB = new Vector3(0, 1, 0), SpringSettings = new SpringSettings(30, 1) });
                Simulation.Solver.Add(a, b, new SwingLimit { AxisLocalA = new Vector3(0, 1, 0), AxisLocalB = new Vector3(0, 1, 0), MaximumSwingAngle = MathHelper.PiOver2, SpringSettings = new SpringSettings(30, 1) });
            }
            {
                var x = GetNextPosition(ref nextX);
                var a = Simulation.Bodies.Add(BodyDescription.CreateKinematic(new Vector3(x, 3, 0), collidableA, activity));
                var b = Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(x, 5, 0), inertiaB, collidableB, activity));
                Simulation.Solver.Add(a, b, new SwivelHinge
                {
                    LocalOffsetA = new Vector3(0, 1, 0),
                    LocalSwivelAxisA = new Vector3(1, 0, 0),
                    LocalOffsetB = new Vector3(0, -1, 0),
                    LocalHingeAxisB = new Vector3(0, 1, 0),
                    SpringSettings = new SpringSettings(30, 1)
                });
                Simulation.Solver.Add(a, b, new SwingLimit { AxisLocalA = new Vector3(0, 1, 0), AxisLocalB = new Vector3(0, 1, 0), MaximumSwingAngle = MathHelper.PiOver2, SpringSettings = new SpringSettings(30, 1) });
            }
            {
                var x = GetNextPosition(ref nextX);
                var a = Simulation.Bodies.Add(BodyDescription.CreateKinematic(new Vector3(x, 3, 0), collidableA, activity));
                var b = Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(x, 5, 0), inertiaB, collidableB, activity));
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
                var x = GetNextPosition(ref nextX);
                var a = Simulation.Bodies.Add(BodyDescription.CreateKinematic(new Vector3(x, 3, 0), collidableA, activity));
                var b = Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(x, 5, 0), inertiaB, collidableB, activity));
                Simulation.Solver.Add(a, b, new BallSocket { LocalOffsetA = new Vector3(0, 1, 0), LocalOffsetB = new Vector3(0, -1, 0), SpringSettings = new SpringSettings(30, 1) });
                Simulation.Solver.Add(a, b, new TwistLimit
                {
                    LocalBasisA = RagdollDemo.CreateBasis(new Vector3(0, 1, 0), new Vector3(1, 0, 0)),
                    LocalBasisB = RagdollDemo.CreateBasis(new Vector3(0, 1, 0), new Vector3(1, 0, 0)),
                    MinimumAngle = MathHelper.Pi * -0.5f,
                    MaximumAngle = MathHelper.Pi * 0.95f,
                    SpringSettings = new SpringSettings(30, 1),
                });
                Simulation.Solver.Add(a, b, new AngularHinge { LocalHingeAxisA = new Vector3(0, 1, 0), LocalHingeAxisB = new Vector3(0, 1, 0), SpringSettings = new SpringSettings(30, 1) });
            }
            {
                var x = GetNextPosition(ref nextX);
                var a = Simulation.Bodies.Add(BodyDescription.CreateKinematic(new Vector3(x, 3, 0), collidableA, activity));
                var b = Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(x, 5, 0), inertiaB, collidableB, activity));
                Simulation.Solver.Add(a, b, new BallSocket { LocalOffsetA = new Vector3(0, 1, 0), LocalOffsetB = new Vector3(0, -1, 0), SpringSettings = new SpringSettings(30, 1) });
                Simulation.Solver.Add(a, b, new TwistMotor
                {
                    LocalAxisA = new Vector3(0, 1, 0),
                    LocalAxisB = new Vector3(0, 1, 0),
                    TargetVelocity = MathHelper.Pi * 2,
                    Settings = new MotorSettings(float.MaxValue, 0.1f)
                });
                Simulation.Solver.Add(a, b, new AngularHinge { LocalHingeAxisA = new Vector3(0, 1, 0), LocalHingeAxisB = new Vector3(0, 1, 0), SpringSettings = new SpringSettings(30, 1) });
            }
            {
                var x = GetNextPosition(ref nextX);
                var a = Simulation.Bodies.Add(BodyDescription.CreateKinematic(new Vector3(x, 3, 0), collidableA, activity));
                var b = Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(x, 5, 0), inertiaB, collidableB, activity));
                Simulation.Solver.Add(a, b, new BallSocket { LocalOffsetA = new Vector3(0, 1, 0), LocalOffsetB = new Vector3(0, -1, 0), SpringSettings = new SpringSettings(30, 1) });
                Simulation.Solver.Add(a, b, new AngularServo
                {
                    TargetRelativeRotationLocalA = Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), MathHelper.PiOver2),
                    ServoSettings = new ServoSettings(float.MaxValue, 0, 12f),
                    SpringSettings = new SpringSettings(30, 1)
                });
            }
            {
                var x = GetNextPosition(ref nextX);
                var a = Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(x, 3, 0), inertiaA, collidableA, activity));
                var b = Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(x, 5, 0), inertiaB, collidableB, activity));
                Simulation.Solver.Add(a, b, new BallSocket { LocalOffsetA = new Vector3(0, 1, 0), LocalOffsetB = new Vector3(0, -1, 0), SpringSettings = new SpringSettings(30, 1) });
                Simulation.Solver.Add(a, b, new AngularMotor { TargetVelocityLocalA = new Vector3(0, 1, 0), Settings = new MotorSettings(15, 0.0001f) });
            }
            {
                var x = GetNextPosition(ref nextX);
                var aDescription = BodyDescription.CreateDynamic(new Vector3(x, 3, 0), inertiaA, collidableA, activity);
                var bDescription = BodyDescription.CreateDynamic(new Vector3(x, 5, 0), inertiaB, collidableB, activity);
                //aDescription.Velocity.Angular = new Vector3(0, 0, 5);
                var a = Simulation.Bodies.Add(aDescription);
                var b = Simulation.Bodies.Add(bDescription);
                Simulation.Solver.Add(a, b, new Weld { LocalOffset = new Vector3(0, 2, 0), LocalOrientation = Quaternion.Identity, SpringSettings = new SpringSettings(30, 1) });
            }
            {
                var x = GetNextPosition(ref nextX);
                var sphere = new Sphere(0.125f);
                //Treat each vertex as a point mass that cannot rotate.
                var sphereInertia = new BodyInertia { InverseMass = 1 };
                var sphereCollidable = new CollidableDescription(Simulation.Shapes.Add(sphere), 0.1f);
                var a = new Vector3(x, 3, 0);
                var b = new Vector3(x, 4, 0);
                var c = new Vector3(x, 3, 1);
                var d = new Vector3(x + 1, 3, 0);
                var aDescription = BodyDescription.CreateDynamic(a, sphereInertia, sphereCollidable, activity);
                var bDescription = BodyDescription.CreateDynamic(b, sphereInertia, sphereCollidable, activity);
                var cDescription = BodyDescription.CreateDynamic(c, sphereInertia, sphereCollidable, activity);
                var dDescription = BodyDescription.CreateDynamic(d, sphereInertia, sphereCollidable, activity);
                var aHandle = Simulation.Bodies.Add(aDescription);
                var bHandle = Simulation.Bodies.Add(bDescription);
                var cHandle = Simulation.Bodies.Add(cDescription);
                var dHandle = Simulation.Bodies.Add(dDescription);
                var distanceSpringiness = new SpringSettings(3f, 1);
                Simulation.Solver.Add(aHandle, bHandle, new CenterDistanceConstraint(Vector3.Distance(a, b), distanceSpringiness));
                Simulation.Solver.Add(aHandle, cHandle, new CenterDistanceConstraint(Vector3.Distance(a, c), distanceSpringiness));
                Simulation.Solver.Add(aHandle, dHandle, new CenterDistanceConstraint(Vector3.Distance(a, d), distanceSpringiness));
                Simulation.Solver.Add(bHandle, cHandle, new CenterDistanceConstraint(Vector3.Distance(b, c), distanceSpringiness));
                Simulation.Solver.Add(bHandle, dHandle, new CenterDistanceConstraint(Vector3.Distance(b, d), distanceSpringiness));
                Simulation.Solver.Add(cHandle, dHandle, new CenterDistanceConstraint(Vector3.Distance(c, d), distanceSpringiness));
                Simulation.Solver.Add(aHandle, bHandle, cHandle, dHandle, new VolumeConstraint(a, b, c, d, new SpringSettings(30, 1)));
            }
            {
                var x = GetNextPosition(ref nextX);
                var aDescription = BodyDescription.CreateDynamic(new Vector3(x, 3, 0), inertiaA, collidableA, activity);
                var bDescription = BodyDescription.CreateDynamic(new Vector3(x, 6, 0), inertiaB, collidableB, activity);
                var a = Simulation.Bodies.Add(aDescription);
                var b = Simulation.Bodies.Add(bDescription);
                Simulation.Solver.Add(a, b, new DistanceServo(new Vector3(0, 0.55f, 0), new Vector3(0, -0.55f, 0), 1.9f, new SpringSettings(30, 1), ServoSettings.Default));
            }
            {
                var x = GetNextPosition(ref nextX);
                var aDescription = BodyDescription.CreateDynamic(new Vector3(x, 3, 0), inertiaA, collidableA, activity);
                var bDescription = BodyDescription.CreateDynamic(new Vector3(x, 6, 0), inertiaB, collidableB, activity);
                var a = Simulation.Bodies.Add(aDescription);
                var b = Simulation.Bodies.Add(bDescription);
                Simulation.Solver.Add(a, b, new DistanceLimit(new Vector3(0, 0.55f, 0), new Vector3(0, -0.55f, 0), 1f, 3, new SpringSettings(30, 1)));
            }
            {
                var x = GetNextPosition(ref nextX);
                var sphere = new Sphere(0.125f);
                //Treat each vertex as a point mass that cannot rotate.
                var sphereInertia = new BodyInertia { InverseMass = 1 };
                var sphereCollidable = new CollidableDescription(Simulation.Shapes.Add(sphere), 0.1f);
                var a = new Vector3(x, 3, 0);
                var b = new Vector3(x, 4, 0);
                var c = new Vector3(x + 1, 3, 0);
                var aDescription = BodyDescription.CreateDynamic(a, sphereInertia, sphereCollidable, activity);
                var bDescription = BodyDescription.CreateDynamic(b, sphereInertia, sphereCollidable, activity);
                var cDescription = BodyDescription.CreateDynamic(c, sphereInertia, sphereCollidable, activity);
                var aHandle = Simulation.Bodies.Add(aDescription);
                var bHandle = Simulation.Bodies.Add(bDescription);
                var cHandle = Simulation.Bodies.Add(cDescription);
                var distanceSpringiness = new SpringSettings(3f, 1);
                Simulation.Solver.Add(aHandle, bHandle, new CenterDistanceConstraint(Vector3.Distance(a, b), distanceSpringiness));
                Simulation.Solver.Add(aHandle, cHandle, new CenterDistanceConstraint(Vector3.Distance(a, c), distanceSpringiness));
                Simulation.Solver.Add(bHandle, cHandle, new CenterDistanceConstraint(Vector3.Distance(b, c), distanceSpringiness));
                Simulation.Solver.Add(aHandle, bHandle, cHandle, new AreaConstraint(a, b, c, new SpringSettings(30, 1)));
            }
            {
                var x = GetNextPosition(ref nextX);
                var aDescription = BodyDescription.CreateDynamic(new Vector3(x, 3, 0), default, collidableA, activity);
                var bDescription = BodyDescription.CreateDynamic(new Vector3(x, 6, 0), inertiaB, collidableB, activity);
                var a = Simulation.Bodies.Add(aDescription);
                var b = Simulation.Bodies.Add(bDescription);
                Simulation.Solver.Add(a, b, new PointOnLineServo
                {
                    LocalOffsetA = new Vector3(0, 0.5f, 0),
                    LocalOffsetB = new Vector3(0, -0.5f, 0),
                    LocalDirection = new Vector3(0, 1, 0),
                    SpringSettings = new SpringSettings(30, 1),
                    ServoSettings = ServoSettings.Default
                });
            }
            {
                var x = GetNextPosition(ref nextX);
                var aDescription = BodyDescription.CreateDynamic(new Vector3(x, 3, 0), inertiaA, collidableA, activity);
                var bDescription = BodyDescription.CreateDynamic(new Vector3(x, 6, 0), inertiaB, collidableB, activity);
                var a = Simulation.Bodies.Add(aDescription);
                var b = Simulation.Bodies.Add(bDescription);
                Simulation.Solver.Add(a, b, new LinearAxisServo
                {
                    LocalOffsetA = new Vector3(0, 0.5f, 0),
                    LocalOffsetB = new Vector3(0, -0.5f, 0),
                    LocalPlaneNormal = new Vector3(0, 1, 0),
                    TargetOffset = 2,
                    SpringSettings = new SpringSettings(30, 1),
                    ServoSettings = ServoSettings.Default
                });
            }
            {
                var x = GetNextPosition(ref nextX);
                var aDescription = BodyDescription.CreateDynamic(new Vector3(x, 3, 0), inertiaA, collidableA, activity);
                var bDescription = BodyDescription.CreateDynamic(new Vector3(x, 6, 0), inertiaB, collidableB, activity);
                var a = Simulation.Bodies.Add(aDescription);
                var b = Simulation.Bodies.Add(bDescription);
                Simulation.Solver.Add(a, b, new PointOnLineServo
                {
                    LocalOffsetA = new Vector3(0, 0.5f, 0),
                    LocalOffsetB = new Vector3(0, -0.5f, 0),
                    LocalDirection = new Vector3(0, 1, 0),
                    SpringSettings = new SpringSettings(30, 1),
                    ServoSettings = ServoSettings.Default
                });
                Simulation.Solver.Add(a, b, new LinearAxisMotor
                {
                    LocalOffsetA = new Vector3(0, 0.5f, 0),
                    LocalOffsetB = new Vector3(0, -0.5f, 0),
                    LocalAxis = new Vector3(0, 1, 0),
                    TargetVelocity = -2,
                    Settings = new MotorSettings(15, 0.01f)
                });
            }
            {
                var x = GetNextPosition(ref nextX);
                var aDescription = BodyDescription.CreateDynamic(new Vector3(x, 3, 0), inertiaA, collidableA, activity);
                var bDescription = BodyDescription.CreateDynamic(new Vector3(x, 6, 0), inertiaB, collidableB, activity);
                var a = Simulation.Bodies.Add(aDescription);
                var b = Simulation.Bodies.Add(bDescription);
                Simulation.Solver.Add(a, b, new PointOnLineServo
                {
                    LocalOffsetA = new Vector3(0, 0.5f, 0),
                    LocalOffsetB = new Vector3(0, -0.5f, 0),
                    LocalDirection = new Vector3(0, 1, 0),
                    SpringSettings = new SpringSettings(30, 1),
                    ServoSettings = ServoSettings.Default
                });
                Simulation.Solver.Add(a, b, new LinearAxisLimit
                {
                    LocalOffsetA = new Vector3(0, 0.5f, 0),
                    LocalOffsetB = new Vector3(0, -0.5f, 0),
                    LocalAxis = new Vector3(0, 1, 0),
                    MinimumOffset = 1,
                    MaximumOffset = 2,
                    SpringSettings = new SpringSettings(30, 1)
                });
            }
            {
                var x = GetNextPosition(ref nextX);
                var a = Simulation.Bodies.Add(BodyDescription.CreateKinematic(new Vector3(x, 3, 0), collidableA, activity));
                var b = Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(x, 5, 0), inertiaB, collidableB, activity));
                Simulation.Solver.Add(a, b, new BallSocket { LocalOffsetA = new Vector3(0, 1, 0), LocalOffsetB = new Vector3(0, -1, 0), SpringSettings = new SpringSettings(30, 1) });
                Simulation.Solver.Add(b, a, new AngularAxisMotor
                {
                    LocalAxisA = new Vector3(0, 1, 0),
                    TargetVelocity = MathHelper.Pi * 5,
                    Settings = new MotorSettings(float.MaxValue, 0.1f)
                });
            }
            {
                var x = GetNextPosition(ref nextX);
                var a = Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(x, 3, 0), inertiaA, collidableA, activity));
                Simulation.Solver.Add(a, new OneBodyLinearServo
                {
                    LocalOffset = new Vector3(0, 1, 0),
                    Target = new Vector3(x, 3, 0),
                    ServoSettings = new ServoSettings(2, 0, float.MaxValue),
                    SpringSettings = new SpringSettings(5, 1)
                });
            }
            {
                var x = GetNextPosition(ref nextX);
                var a = Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(x, 3, 0), inertiaA, collidableA, activity));
                var b = Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(x, 5, 0), inertiaB, collidableB, activity));
                Simulation.Solver.Add(a, b, new BallSocket { LocalOffsetA = new Vector3(0, 1, 0), LocalOffsetB = new Vector3(0, -1, 0), SpringSettings = new SpringSettings(30, 1) });
                Simulation.Solver.Add(a, new OneBodyLinearMotor
                {
                    LocalOffset = new Vector3(0, 1, 0),
                    TargetVelocity = new Vector3(0, -1, 0),
                    Settings = new MotorSettings(float.MaxValue, 0.01f),
                });
            }
            {
                var x = GetNextPosition(ref nextX);
                var a = Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(x, 3, 0), inertiaA, collidableA, activity));
                var b = Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(x, 5, 0), inertiaB, collidableB, activity));
                Simulation.Solver.Add(a, b, new BallSocket { LocalOffsetA = new Vector3(0, 1, 0), LocalOffsetB = new Vector3(0, -1, 0), SpringSettings = new SpringSettings(30, 1) });
                Simulation.Solver.Add(a, new OneBodyAngularServo
                {
                    TargetOrientation = Quaternion.Identity,
                    ServoSettings = ServoSettings.Default,
                    SpringSettings = new SpringSettings(30f, 1f)
                });
            }
            {
                var x = GetNextPosition(ref nextX);
                var a = Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(x, 3, 0), inertiaA, collidableA, activity));
                var b = Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(x, 5, 0), inertiaB, collidableB, activity));
                Simulation.Solver.Add(a, b, new BallSocket { LocalOffsetA = new Vector3(0, 1, 0), LocalOffsetB = new Vector3(0, -1, 0), SpringSettings = new SpringSettings(30, 1) });
                Simulation.Solver.Add(a, new OneBodyAngularMotor
                {
                    TargetVelocity = new Vector3(1, 0, 0),
                    Settings = new MotorSettings(float.MaxValue, 0.001f),
                });
            }

            Simulation.Statics.Add(new StaticDescription(new Vector3(), new CollidableDescription(Simulation.Shapes.Add(new Box(256, 1, 256)), 0.1f)));
        }        
    }
}


