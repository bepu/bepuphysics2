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
using BepuPhysics.Constraints;

namespace Demos.Demos
{
    public class ConstraintTestDemo : Demo
    {
        static BodyReference AddBody<TShape>(TShape shape, float mass, RigidPose pose, Simulation simulation) where TShape : struct, IConvexShape
        {
            BodyInertia inertia;
            inertia.InverseMass = mass > 0 ? 1f / mass : 0;
            var shapeIndex = simulation.Shapes.Add(ref shape);
            shape.ComputeLocalInverseInertia(inertia.InverseMass, out inertia.InverseInertiaTensor);
            var description = new BodyDescription
            {
                Activity = new BodyActivityDescription { SleepThreshold = 0, MinimumTimestepCountUnderThreshold = 32 },
                Collidable = new CollidableDescription
                {
                    Continuity = new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Discrete },
                    SpeculativeMargin = .1f,
                    //Note that this always registers a new shape instance. You could be more clever/efficient and share shapes, but the goal here is to show the most basic option.
                    //Also, the cost of registering different shapes isn't that high for tiny implicit shapes.
                    Shape = shapeIndex
                },
                LocalInertia = inertia,
                Pose = pose
            };
            return new BodyReference(simulation.Bodies.Add(ref description), simulation.Bodies);
        }
        public unsafe override void Initialize(Camera camera)
        {
            camera.Position = new Vector3(-20, 10, -20);
            //camera.Yaw = MathHelper.Pi ; 
            camera.Yaw = MathHelper.Pi * 3f / 4;
            //camera.Pitch = MathHelper.PiOver2 * 0.999f;
            Simulation = Simulation.Create(BufferPool, new TestCallbacks());

            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);
            Simulation.Deterministic = false;
            var a = AddBody(new Box(3, 1, 1), 1, new RigidPose { Position = new Vector3(0, 10, 0), Orientation = BepuUtilities.Quaternion.Identity }, Simulation);
            var b = AddBody(new Box(3, 1, 1), 0, new RigidPose { Position = new Vector3(5, 10, 0), Orientation = BepuUtilities.Quaternion.Identity }, Simulation);
            a.Velocity.Angular = new Vector3(1f, 5f, 1f);
            //a.Pose.Orientation = BepuUtilities.Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 1, 1)), MathHelper.PiOver4);
            var springSettings = new SpringSettings(15, 1);
            var ballSocket = new BallSocket
            {
                LocalOffsetA = new Vector3(2.5f, 0, 0),
                LocalOffsetB = new Vector3(-2.5f, 0, 0),
                SpringSettings = springSettings
            };
            Simulation.Solver.Add(a.Handle, b.Handle, ref ballSocket);
            //springSettings = new BepuPhysics.CollisionDetection.SpringSettings { DampingRatio = 0f, NaturalFrequency = MathHelper.Pi * 1 };
            var angularHinge = new AngularHinge
            {
                HingeAxisLocalA = Vector3.Normalize(new Vector3(0, 1, 0)),
                HingeAxisLocalB = Vector3.Normalize(new Vector3(0, 1, 0)),
                SpringSettings = springSettings
            };
            Simulation.Solver.Add(a.Handle, b.Handle, ref angularHinge);
            //var swivelHinge = new AngularSwivelHinge
            //{
            //    SwivelAxisLocalA = new Vector3(1, 0, 0),
            //    HingeAxisLocalB = new Vector3(0, 1, 0),
            //    SpringSettings = springSettings
            //};
            //Simulation.Solver.Add(a.Handle, b.Handle, ref swivelHinge);
            var swingLimit = new SwingLimit
            {
                AxisLocalA = new Vector3(1, 0, 0),
                AxisLocalB = new Vector3(1, 0, 0),
                MinimumDot = -0.5f,
                SpringSettings = new SpringSettings(15, 1)
            };
            Simulation.Solver.Add(a.Handle, b.Handle, ref swingLimit);

            var staticShape = new Box(100, 1, 100);
            var staticShapeIndex = Simulation.Shapes.Add(ref staticShape);

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
            Simulation.Statics.Add(ref staticDescription);


        }

    }
}


