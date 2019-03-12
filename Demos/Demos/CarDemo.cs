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
using DemoRenderer.UI;
using DemoUtilities;
using OpenTK.Input;
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

        public static WheelHandles CreateWheel(Simulation simulation, BodyProperty<CarBodyProperties> properties, in RigidPose bodyPose,
            TypedIndex wheelShape, BodyInertia wheelInertia, float wheelFriction, int bodyHandle, ref SubgroupCollisionFilter bodyFilter, in Vector3 bodyToWheelSuspension, in Vector3 suspensionDirection, float suspensionLength,
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
            ref var wheelProperties = ref properties.Allocate(handles.Wheel);
            wheelProperties = new CarBodyProperties { Filter = new SubgroupCollisionFilter(bodyHandle, 1), Friction = wheelFriction };
            SubgroupCollisionFilter.DisableCollision(ref wheelProperties.Filter, ref bodyFilter);

            return handles;
        }

        public static SimpleCar Create(Simulation simulation, BodyProperty<CarBodyProperties> properties, in RigidPose pose,
            TypedIndex bodyShape, BodyInertia bodyInertia, float bodyFriction, TypedIndex wheelShape, BodyInertia wheelInertia, float wheelFriction,
            in Vector3 bodyToFrontLeftSuspension, in Vector3 bodyToFrontRightSuspension, in Vector3 bodyToBackLeftSuspension, in Vector3 bodyToBackRightSuspension,
            in Vector3 suspensionDirection, float suspensionLength, in SpringSettings suspensionSettings, in Quaternion localWheelOrientation)
        {
            SimpleCar car;
            car.Body = simulation.Bodies.Add(BodyDescription.CreateDynamic(pose, bodyInertia, new CollidableDescription(bodyShape, 0.1f), new BodyActivityDescription(0.01f)));
            ref var bodyProperties = ref properties.Allocate(car.Body);
            bodyProperties = new CarBodyProperties { Friction = bodyFriction, Filter = new SubgroupCollisionFilter(car.Body, 0) };
            Quaternion.TransformUnitY(localWheelOrientation, out var wheelAxis);
            car.hingeDescription = new AngularHinge
            {
                LocalHingeAxisA = wheelAxis,
                LocalHingeAxisB = new Vector3(0, 1, 0),
                SpringSettings = new SpringSettings(30, 1)
            };
            car.suspensionDirection = suspensionDirection;
            car.BackLeftWheel = CreateWheel(simulation, properties, pose, wheelShape, wheelInertia, wheelFriction, car.Body, ref bodyProperties.Filter, bodyToBackLeftSuspension, suspensionDirection, suspensionLength, car.hingeDescription, suspensionSettings, localWheelOrientation);
            car.BackRightWheel = CreateWheel(simulation, properties, pose, wheelShape, wheelInertia, wheelFriction, car.Body, ref bodyProperties.Filter, bodyToBackRightSuspension, suspensionDirection, suspensionLength, car.hingeDescription, suspensionSettings, localWheelOrientation);
            car.FrontLeftWheel = CreateWheel(simulation, properties, pose, wheelShape, wheelInertia, wheelFriction, car.Body, ref bodyProperties.Filter, bodyToFrontLeftSuspension, suspensionDirection, suspensionLength, car.hingeDescription, suspensionSettings, localWheelOrientation);
            car.FrontRightWheel = CreateWheel(simulation, properties, pose, wheelShape, wheelInertia, wheelFriction, car.Body, ref bodyProperties.Filter, bodyToFrontRightSuspension, suspensionDirection, suspensionLength, car.hingeDescription, suspensionSettings, localWheelOrientation);
            return car;
        }

    }

    struct SimpleCarController
    {
        public SimpleCar Car;

        private float steeringAngle;

        public float SteeringAngle { get { return steeringAngle; } }

        public float SteeringSpeed;
        public float MaximumSteeringAngle;

        public float ForwardSpeed;
        public float ForwardForce;
        public float ZoomMultiplier;
        public float BackwardSpeed;
        public float BackwardForce;
        public float IdleForce;
        public float BrakeForce;

        //Track the previous state to force wakeups if the constraint targets have changed.
        private float previousTargetSpeed;
        private float previousTargetForce;

        public SimpleCarController(SimpleCar car,
            float forwardSpeed, float forwardForce, float zoomMultiplier, float backwardSpeed, float backwardForce, float idleForce, float brakeForce,
            float steeringSpeed, float maximumSteeringAngle)
        {
            Car = car;
            ForwardSpeed = forwardSpeed;
            ForwardForce = forwardForce;
            ZoomMultiplier = zoomMultiplier;
            BackwardSpeed = backwardSpeed;
            BackwardForce = backwardForce;
            IdleForce = idleForce;
            BrakeForce = brakeForce;
            SteeringSpeed = steeringSpeed;
            MaximumSteeringAngle = maximumSteeringAngle;

            steeringAngle = 0;
            previousTargetForce = 0;
            previousTargetSpeed = 0;
        }
        public void Update(Simulation simulation, float dt, float targetSteeringAngle, float targetSpeedFraction, bool zoom, bool brake)
        {
            var steeringAngleDifference = targetSteeringAngle - steeringAngle;
            var maximumChange = SteeringSpeed * dt;
            var steeringAngleChange = MathF.Min(maximumChange, MathF.Max(-maximumChange, steeringAngleDifference));
            var previousSteeringAngle = steeringAngle;
            steeringAngle = MathF.Min(MaximumSteeringAngle, MathF.Max(-MaximumSteeringAngle, steeringAngle + steeringAngleChange));
            if (steeringAngle != previousSteeringAngle)
            {
                //By guarding the constraint modifications behind a state test, we avoid waking up the car every single frame.
                //(We could have also used the ApplyDescriptionWithoutWaking function and then explicitly woke the car up when changes occur.)
                Car.Steer(simulation, Car.FrontLeftWheel, steeringAngle);
                Car.Steer(simulation, Car.FrontRightWheel, steeringAngle);
            }
            float newTargetSpeed, newTargetForce;
            bool allWheels;
            if (brake)
            {
                newTargetSpeed = 0;
                newTargetForce = BrakeForce;
                allWheels = true;
            }
            else if (targetSpeedFraction > 0)
            {
                newTargetForce = zoom ? ForwardForce * ZoomMultiplier : ForwardForce;
                newTargetSpeed = targetSpeedFraction * (zoom ? ForwardSpeed * ZoomMultiplier : ForwardSpeed);
                allWheels = false;
            }
            else if (targetSpeedFraction < 0)
            {
                newTargetForce = BackwardForce;
                newTargetSpeed = targetSpeedFraction * BackwardSpeed;
                allWheels = false;
            }
            else
            {
                newTargetForce = IdleForce;
                newTargetSpeed = 0;
                allWheels = true;
            }
            if (previousTargetSpeed != newTargetSpeed || previousTargetForce != newTargetForce)
            {
                previousTargetSpeed = newTargetSpeed;
                previousTargetForce = newTargetForce;
                Car.SetSpeed(simulation, Car.FrontLeftWheel, newTargetSpeed, newTargetForce);
                Car.SetSpeed(simulation, Car.FrontRightWheel, newTargetSpeed, newTargetForce);
                if (allWheels)
                {
                    Car.SetSpeed(simulation, Car.BackLeftWheel, newTargetSpeed, newTargetForce);
                    Car.SetSpeed(simulation, Car.BackRightWheel, newTargetSpeed, newTargetForce);
                }
                else
                {
                    Car.SetSpeed(simulation, Car.BackLeftWheel, 0, 0);
                    Car.SetSpeed(simulation, Car.BackRightWheel, 0, 0);
                }
            }
        }
    }

    struct CarBodyProperties
    {
        public SubgroupCollisionFilter Filter;
        public float Friction;
    }

    /// <summary>
    /// For the car demo, we want both wheel-body collision filtering and different friction for wheels versus the car body.
    /// </summary>
    struct CarCallbacks : INarrowPhaseCallbacks
    {
        public BodyProperty<CarBodyProperties> Properties;
        public void Initialize(Simulation simulation)
        {
            Properties.Initialize(simulation.Bodies);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b)
        {
            //It's impossible for two statics to collide, and pairs are sorted such that bodies always come before statics.
            if (b.Mobility != CollidableMobility.Static)
            {
                return SubgroupCollisionFilter.AllowCollision(Properties[a.Handle].Filter, Properties[b.Handle].Filter);
            }
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
        {
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe void CreateMaterial(CollidablePair pair, out PairMaterialProperties pairMaterial)
        {
            pairMaterial.FrictionCoefficient = Properties[pair.A.Handle].Friction;
            if (pair.B.Mobility != CollidableMobility.Static)
            {
                //If two bodies collide, just average the friction.
                pairMaterial.FrictionCoefficient = (pairMaterial.FrictionCoefficient + Properties[pair.B.Handle].Friction) * 0.5f;
            }
            pairMaterial.MaximumRecoveryVelocity = 2f;
            pairMaterial.SpringSettings = new SpringSettings(30, 1);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, NonconvexContactManifold* manifold, out PairMaterialProperties pairMaterial)
        {
            CreateMaterial(pair, out pairMaterial);
            return true;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, ConvexContactManifold* manifold, out PairMaterialProperties pairMaterial)
        {
            CreateMaterial(pair, out pairMaterial);
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ConvexContactManifold* manifold)
        {
            return true;
        }

        public void Dispose()
        {
            Properties.Dispose();
        }
    }

    public class CarDemo : Demo
    {
        SimpleCarController playerController;

        static Key Forward = Key.W;
        static Key Backward = Key.S;
        static Key Right = Key.D;
        static Key Left = Key.A;
        static Key Zoom = Key.LShift;
        static Key Brake = Key.Space;
        static Key BrakeAlternate = Key.BackSpace; //I have a weird keyboard.
        static Key ToggleCar = Key.C;
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 5, 10);
            camera.Yaw = 0;
            camera.Pitch = 0;

            var properties = new BodyProperty<CarBodyProperties>();
            Simulation = Simulation.Create(BufferPool, new CarCallbacks() { Properties = properties }, new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));

            var builder = new CompoundBuilder(BufferPool, Simulation.Shapes, 2);
            builder.Add(new Box(1.85f, 0.7f, 4.73f), RigidPose.Identity, 10);
            builder.Add(new Box(1.85f, 0.6f, 2.5f), new RigidPose(new Vector3(0, 0.65f, -0.35f)), 0.5f);
            builder.BuildDynamicCompound(out var children, out var bodyInertia, out _);
            builder.Dispose();
            var bodyShape = new Compound(children);
            var bodyShapeIndex = Simulation.Shapes.Add(bodyShape);
            var wheelShape = new Cylinder(0.4f, .18f);
            wheelShape.ComputeInertia(0.25f, out var wheelInertia);
            var wheelShapeIndex = Simulation.Shapes.Add(wheelShape);

            const float x = 0.95f;
            const float y = -0.15f;
            const float frontZ = 1.7f;
            const float backZ = -1.7f;
            playerController = new SimpleCarController(SimpleCar.Create(Simulation, properties, new RigidPose(new Vector3(0, 10, 0), Quaternion.Identity), bodyShapeIndex, bodyInertia, 0.5f, wheelShapeIndex, wheelInertia, 2.5f,
                new Vector3(-x, y, frontZ), new Vector3(x, y, frontZ), new Vector3(-x, y, backZ), new Vector3(x, y, backZ), new Vector3(0, -1, 0), 0.25f,
                new SpringSettings(5, 1), Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI * 0.5f)),
                forwardSpeed: 75, forwardForce: 6, zoomMultiplier: 2, backwardSpeed: 10, backwardForce: 4, idleForce: 0.25f, brakeForce: 7, steeringSpeed: 2.5f, maximumSteeringAngle: MathF.PI * 0.23f);

            const int planeWidth = 257;
            MeshDemo.CreateDeformedPlane(planeWidth, planeWidth,
                (int vX, int vY) =>
                {
                    var octave0 = (MathF.Sin((vX + 5f) * 0.05f) + MathF.Sin((vY + 11) * 0.05f)) * 1.8f;
                    var octave1 = (MathF.Sin((vX + 17) * 0.15f) + MathF.Sin((vY + 19) * 0.15f)) * 0.9f;
                    var octave2 = (MathF.Sin((vX + 37) * 0.35f) + MathF.Sin((vY + 93) * 0.35f)) * 0.4f;
                    var octave3 = (MathF.Sin((vX + 53) * 0.65f) + MathF.Sin((vY + 47) * 0.65f)) * 0.2f;
                    var octave4 = (MathF.Sin((vX + 67) * 1.50f) + MathF.Sin((vY + 13) * 1.5f)) * 0.125f;
                    var distanceToEdge = planeWidth / 2 - Math.Max(Math.Abs(vX - planeWidth / 2), Math.Abs(vY - planeWidth / 2));
                    var edgeRamp = 15f / (distanceToEdge + 1);
                    return new Vector3(vX, octave0 + octave1 + octave2 + octave3 + octave4 + edgeRamp, vY);
                }, new Vector3(3, 1, 3), BufferPool, out var planeMesh);
            Simulation.Statics.Add(new StaticDescription(new Vector3(-100, -15, 100), Quaternion.CreateFromAxisAngle(new Vector3(0, 1, 0), MathF.PI / 2),
                new CollidableDescription(Simulation.Shapes.Add(planeMesh), 0.1f)));

        }
        bool playerControlActive = true;
        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            if (input.WasPushed(ToggleCar))
                playerControlActive = !playerControlActive;
            if (playerControlActive)
            {
                float steeringSum = 0;
                if (input.IsDown(Left))
                {
                    steeringSum += 1;
                }
                if (input.IsDown(Right))
                {
                    steeringSum -= 1;
                }
                //For control purposes, we'll match the fixed update rate of the simulation. Could decouple it- this dt isn't
                //vulnerable to the same instabilities as the simulation itself with variable durations.
                const float controlDt = 1 / 60f;
                var targetSpeedFraction = input.IsDown(Forward) ? 1f : input.IsDown(Backward) ? -1f : 0;
                var zoom = input.IsDown(Zoom);
                playerController.Update(Simulation, controlDt, steeringSum, targetSpeedFraction, zoom, input.IsDown(Brake) || input.IsDown(BrakeAlternate));
            }

            base.Update(window, camera, input, dt);
        }

        void RenderControl(ref Vector2 position, float textHeight, string controlName, string controlValue, TextBuilder text, TextBatcher textBatcher, Font font)
        {
            text.Clear().Append(controlName).Append(": ").Append(controlValue);
            textBatcher.Write(text, position, textHeight, new Vector3(1), font);
            position.Y += textHeight * 1.1f;
        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            if (playerControlActive)
            {
                var carBody = new BodyReference(playerController.Car.Body, Simulation.Bodies);
                Quaternion.TransformUnitY(carBody.Pose.Orientation, out var carUp);
                camera.Position = carBody.Pose.Position + carUp * 1.3f + camera.Backward * 8;
            }

            var textHeight = 16;
            var position = new Vector2(32, renderer.Surface.Resolution.Y - 128);
            RenderControl(ref position, textHeight, nameof(Forward), Forward.ToString(), text, renderer.TextBatcher, font);
            RenderControl(ref position, textHeight, nameof(Backward), Backward.ToString(), text, renderer.TextBatcher, font);
            RenderControl(ref position, textHeight, nameof(Right), Right.ToString(), text, renderer.TextBatcher, font);
            RenderControl(ref position, textHeight, nameof(Left), Left.ToString(), text, renderer.TextBatcher, font);
            RenderControl(ref position, textHeight, nameof(Zoom), Zoom.ToString(), text, renderer.TextBatcher, font);
            RenderControl(ref position, textHeight, nameof(Brake), Brake.ToString(), text, renderer.TextBatcher, font);
            RenderControl(ref position, textHeight, nameof(ToggleCar), ToggleCar.ToString(), text, renderer.TextBatcher, font);
            base.Render(renderer, camera, input, text, font);
        }
    }
}