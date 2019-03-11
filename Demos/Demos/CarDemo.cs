using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using DemoUtilities;
using Quaternion = BepuUtilities.Quaternion;

namespace Demos.Demos
{
    struct WheelHandles
    {
        public int Wheel;
        public int SuspensionSpring;
        public int SuspensionTrack;
        public int Hinge;
        public int Motor;
    }

    struct SimpleCar
    {
        public int Body;
        public WheelHandles FrontLeftWheel;
        public WheelHandles FrontRightWheel;
        public WheelHandles BackLeftWheel;
        public WheelHandles BackRightWheel;

        private Vector3 suspensionDirection;
        private AngularHinge hingeDescription;

        public void Steer(Simulation simulation, in WheelHandles wheel, float angle)
        {
            var steeredHinge = hingeDescription;
            Matrix3x3.CreateFromAxisAngle(suspensionDirection, -angle, out var rotation);
            Matrix3x3.Transform(hingeDescription.LocalHingeAxisA, rotation, out steeredHinge.LocalHingeAxisA);
            simulation.Solver.ApplyDescription(wheel.Hinge, ref steeredHinge);
        }

        public void SetSpeed(Simulation simulation, in WheelHandles wheel, float speed, float maximumForce)
        {
            simulation.Solver.ApplyDescription(wheel.Motor, new AngularAxisMotor
            {
                LocalAxisA = new Vector3(0, -1, 0),
                Settings = new MotorSettings(maximumForce, 1e-6f),
                TargetVelocity = speed
            });
        }

        public static WheelHandles CreateWheel(Simulation simulation, BodyProperty<SubgroupCollisionFilter> filters, in RigidPose bodyPose,
            TypedIndex wheelShape, BodyInertia wheelInertia, int bodyHandle, ref SubgroupCollisionFilter bodyFilter, in Vector3 bodyToWheelSuspension, in Vector3 suspensionDirection, float suspensionLength,
            in AngularHinge hingeDescription, in SpringSettings suspensionSettings, in Quaternion localWheelOrientation)
        {
            RigidPose wheelPose;
            RigidPose.Transform(bodyToWheelSuspension + suspensionDirection * suspensionLength, bodyPose, out wheelPose.Position);
            Quaternion.ConcatenateWithoutOverlap(localWheelOrientation, bodyPose.Orientation, out wheelPose.Orientation);
            WheelHandles handles;
            handles.Wheel = simulation.Bodies.Add(BodyDescription.CreateDynamic(wheelPose, wheelInertia, new CollidableDescription(wheelShape, 0.1f), new BodyActivityDescription(0.01f)));

            handles.SuspensionSpring = simulation.Solver.Add(bodyHandle, handles.Wheel, new LinearAxisServo
            {
                LocalPlaneNormal = suspensionDirection,
                TargetOffset = suspensionLength,
                LocalOffsetA = bodyToWheelSuspension,
                LocalOffsetB = default,
                ServoSettings = ServoSettings.Default,
                SpringSettings = suspensionSettings
            });
            handles.SuspensionTrack = simulation.Solver.Add(bodyHandle, handles.Wheel, new PointOnLineServo
            {
                LocalDirection = suspensionDirection,
                LocalOffsetA = bodyToWheelSuspension,
                LocalOffsetB = default,
                ServoSettings = ServoSettings.Default,
                SpringSettings = new SpringSettings(30, 1)
            });
            //We're treating braking and acceleration as the same thing. It is, after all, a *simple* car! Maybe it's electric or something.
            //It would be fairly easy to split brakes and drive motors into different motors.
            handles.Motor = simulation.Solver.Add(handles.Wheel, bodyHandle, new AngularAxisMotor
            {
                LocalAxisA = new Vector3(0, 1, 0),
                Settings = default,
                TargetVelocity = default
            });
            handles.Hinge = simulation.Solver.Add(bodyHandle, handles.Wheel, hingeDescription);
            //The demos SubgroupCollisionFilter is pretty simple and only tests one direction, so we make the non-colliding relationship symmetric.
            ref var wheelFilter = ref filters.Allocate(handles.Wheel);
            wheelFilter = new SubgroupCollisionFilter(bodyHandle, 1);
            SubgroupCollisionFilter.DisableCollision(ref wheelFilter, ref bodyFilter);

            return handles;
        }

        public static SimpleCar Create(Simulation simulation, BodyProperty<SubgroupCollisionFilter> filters, in RigidPose pose,
            TypedIndex bodyShape, BodyInertia bodyInertia, TypedIndex wheelShape, BodyInertia wheelInertia,
            in Vector3 bodyToFrontLeftSuspension, in Vector3 bodyToFrontRightSuspension, in Vector3 bodyToBackLeftSuspension, in Vector3 bodyToBackRightSuspension,
            in Vector3 suspensionDirection, float suspensionLength, in SpringSettings suspensionSettings, in Quaternion localWheelOrientation)
        {
            SimpleCar car;
            car.Body = simulation.Bodies.Add(BodyDescription.CreateDynamic(pose, bodyInertia, new CollidableDescription(bodyShape, 0.1f), new BodyActivityDescription(-0.01f)));
            ref var bodyFilters = ref filters.Allocate(car.Body);
            bodyFilters = new SubgroupCollisionFilter(car.Body, 0);
            Quaternion.TransformUnitY(localWheelOrientation, out var wheelAxis);
            car.hingeDescription = new AngularHinge
            {
                LocalHingeAxisA = wheelAxis,
                LocalHingeAxisB = new Vector3(0, 1, 0),
                SpringSettings = new SpringSettings(30, 1)
            };
            car.suspensionDirection = suspensionDirection;
            car.BackLeftWheel = CreateWheel(simulation, filters, pose, wheelShape, wheelInertia, car.Body, ref bodyFilters, bodyToBackLeftSuspension, suspensionDirection, suspensionLength, car.hingeDescription, suspensionSettings, localWheelOrientation);
            car.BackRightWheel = CreateWheel(simulation, filters, pose, wheelShape, wheelInertia, car.Body, ref bodyFilters, bodyToBackRightSuspension, suspensionDirection, suspensionLength, car.hingeDescription, suspensionSettings, localWheelOrientation);
            car.FrontLeftWheel = CreateWheel(simulation, filters, pose, wheelShape, wheelInertia, car.Body, ref bodyFilters, bodyToFrontLeftSuspension, suspensionDirection, suspensionLength, car.hingeDescription, suspensionSettings, localWheelOrientation);
            car.FrontRightWheel = CreateWheel(simulation, filters, pose, wheelShape, wheelInertia, car.Body, ref bodyFilters, bodyToFrontRightSuspension, suspensionDirection, suspensionLength, car.hingeDescription, suspensionSettings, localWheelOrientation);
            return car;
        }

    }

    public class CarDemo : Demo
    {
        SimpleCar car;
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 5, 10);
            camera.Yaw = 0;
            camera.Pitch = 0;

            var filters = new BodyProperty<SubgroupCollisionFilter>();
            Simulation = Simulation.Create(BufferPool, new SubgroupFilteredCallbacks() { CollisionFilters = filters }, new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));

            var builder = new CompoundBuilder(BufferPool, Simulation.Shapes, 2);
            builder.Add(new Box(1.85f, 0.7f, 4.73f), RigidPose.Identity, 10);
            builder.Add(new Box(1.85f, 0.6f, 2.5f), new RigidPose(new Vector3(0, 0.65f, -0.35f)), 2);
            builder.BuildDynamicCompound(out var children, out var bodyInertia, out _);
            builder.Dispose();
            var bodyShape = new Compound(children);
            var bodyShapeIndex = Simulation.Shapes.Add(bodyShape);
            var wheelShape = new Cylinder(0.70f, .18f);
            wheelShape.ComputeInertia(0.25f, out var wheelInertia);
            var wheelShapeIndex = Simulation.Shapes.Add(wheelShape);

            const float x = 0.85f;
            const float y = -0.4f;
            const float frontZ = 1.7f;
            const float backZ = -1.7f;
            car = SimpleCar.Create(Simulation, filters, new RigidPose(new Vector3(0, 10, 0), Quaternion.Identity), bodyShapeIndex, bodyInertia, wheelShapeIndex, wheelInertia,
                new Vector3(-x, y, frontZ), new Vector3(x, y, frontZ), new Vector3(-x, y, backZ), new Vector3(x, y, backZ), new Vector3(0, -1, 0), 0.25f,
                new SpringSettings(5, 1), Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI * 0.5f));

            Simulation.Statics.Add(new StaticDescription(new Vector3(0, 0, 0), new CollidableDescription(Simulation.Shapes.Add(new Box(1000, 1, 1000)), 0.1f)));

        }
        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            const float steeringAngle = 0.5f;
            float steeringSum = 0;
            if (input.IsDown(OpenTK.Input.Key.H))
            {
                steeringSum += steeringAngle;
            }
            if (input.IsDown(OpenTK.Input.Key.K))
            {
                steeringSum -= steeringAngle;
            }
            car.Steer(Simulation, car.FrontLeftWheel, steeringSum);
            car.Steer(Simulation, car.FrontRightWheel, steeringSum);

            const float forwardSpeed = 40;
            const float forwardForce = 5;
            const float zoomMultiplier = 2;
            const float backwardSpeed = -10;
            const float backwardForce = 4;
            const float idleForce = 0.5f;
            const float brakeForce = 15;
            if (input.IsDown(OpenTK.Input.Key.Space))
            {
                car.SetSpeed(Simulation, car.FrontLeftWheel, 0, brakeForce);
                car.SetSpeed(Simulation, car.FrontRightWheel, 0, brakeForce);
                car.SetSpeed(Simulation, car.BackLeftWheel, 0, brakeForce);
                car.SetSpeed(Simulation, car.BackRightWheel, 0, brakeForce);
            }
            else if (input.IsDown(OpenTK.Input.Key.U))
            {
                var useZoom = input.IsDown(OpenTK.Input.Key.Enter);
                var force = useZoom ? forwardForce * zoomMultiplier : forwardForce;
                var speed = useZoom ? forwardSpeed * zoomMultiplier : forwardSpeed;
                car.SetSpeed(Simulation, car.FrontLeftWheel, speed, force);
                car.SetSpeed(Simulation, car.FrontRightWheel, speed, force);
                car.SetSpeed(Simulation, car.BackLeftWheel, 0, 0);
                car.SetSpeed(Simulation, car.BackRightWheel, 0, 0);
            }
            else if (input.IsDown(OpenTK.Input.Key.J))
            {
                car.SetSpeed(Simulation, car.FrontLeftWheel, backwardSpeed, backwardForce);
                car.SetSpeed(Simulation, car.FrontRightWheel, backwardSpeed, backwardForce);
                car.SetSpeed(Simulation, car.BackLeftWheel, 0, 0);
                car.SetSpeed(Simulation, car.BackRightWheel, 0, 0);
            }
            else
            {
                car.SetSpeed(Simulation, car.FrontLeftWheel, 0, idleForce);
                car.SetSpeed(Simulation, car.FrontRightWheel, 0, idleForce);
                car.SetSpeed(Simulation, car.BackLeftWheel, 0, idleForce);
                car.SetSpeed(Simulation, car.BackRightWheel, 0, idleForce);
            }
            base.Update(window, camera, input, dt);
        }
    }
}