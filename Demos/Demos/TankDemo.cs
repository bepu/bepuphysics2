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
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using OpenTK.Input;
using Quaternion = BepuUtilities.Quaternion;

namespace Demos.Demos
{
    /// <summary>
    /// Describes properties of a piece of a tank.
    /// </summary>
    public struct TankPartDescription
    {
        /// <summary>
        /// Shape index used by this part's collidable.
        /// </summary>
        public TypedIndex Shape;
        /// <summary>
        /// Inertia of this part's body.
        /// </summary>
        public BodyInertia Inertia;
        /// <summary>
        /// Pose of the part in the tank's local space.
        /// </summary>
        public RigidPose Pose;
        /// <summary>
        /// Friction of the body to be used in pair material calculations.
        /// </summary>
        public float Friction;

        public static TankPartDescription Create<TShape>(float mass, in TShape shape, in RigidPose pose, float friction, Shapes shapes) where TShape : struct, IConvexShape
        {
            TankPartDescription description;
            description.Shape = shapes.Add(shape);
            shape.ComputeInertia(mass, out description.Inertia);
            description.Pose = pose;
            description.Friction = friction;
            return description;
        }
    }

    /// <summary>
    /// Describes a tank's construction.
    /// </summary>
    public struct TankDescription
    {
        /// <summary>
        /// Description of the tank's turret body.
        /// </summary>
        public TankPartDescription Turret;
        /// <summary>
        /// Description of the tank's barrel body.
        /// </summary>
        public TankPartDescription Barrel;
        /// <summary>
        /// Description of the tank's main body.
        /// </summary>
        public TankPartDescription Body;
        /// <summary>
        /// Location of the barrel's anchor in the tank's local space. The barrel will connect to the turret at this location.
        /// </summary>
        public Vector3 BarrelAnchor;
        /// <summary>
        /// Location of the turret's anchor in the tank's local space. The turret will connect to the main body at this location.
        /// </summary>
        public Vector3 TurretAnchor;
        /// <summary>
        /// Basis of the turret and barrel. (0, 0, -1) * TurretBasis in tank local space corresponds to 0 angle for both turret swivel and barrel pitch measurements.
        /// (1, 0, 0) * TurretBasis corresponds to a 90 degree swivel angle.
        /// (0, 1, 0) * TurretBasis corresponds to a 90 degree pitch angle, and is the axis around which the turret can swivel.
        /// </summary>
        public Quaternion TurretBasis;
        /// <summary>
        /// Servo properties for the tank's swivel constraint.
        /// </summary>
        public ServoSettings TurretServo;
        /// <summary>
        /// Spring properties for the tank's swivel constraint.
        /// </summary>
        public SpringSettings TurretSpring;
        /// <summary>
        /// Servo properties for the tank's barrel pitching constraint.
        /// </summary>
        public ServoSettings BarrelServo;
        /// <summary>
        /// Spring properties for the tank's barrel pitching constraint.
        /// </summary>
        public SpringSettings BarrelSpring;


        /// <summary>
        /// Shape used for all wheels.
        /// </summary>
        public TypedIndex WheelShape;
        /// <summary>
        /// Inertia of each wheel body.
        /// </summary>
        public BodyInertia WheelInertia;

        /// <summary>
        /// Local orientation of the wheels. (1,0,0) * WheelOrientation is the suspension direction, (0,1,0) * WheelOrientation is the axis of rotation for the wheels, and (0,0,1) * WheelOrientation is the axis along which the treads will extend.
        /// </summary>
        public Quaternion WheelOrientation;
        /// <summary>
        /// Offset from the tank's local space origin to the left tread's center. The tread will be aligned along (0,0,1) * WheelOrientation.
        /// </summary>
        public Vector3 LeftTreadOffset;
        /// <summary>
        /// Offset from the tank's local space origin to the right tread's center. The tread will be aligned along (0,0,1) * WheelOrientation.
        /// </summary>
        public Vector3 RightTreadOffset;
        /// <summary>
        /// Number of wheels in each tread.
        /// </summary>
        public int WheelCountPerTread;
        /// <summary>
        /// How much space to put in between wheels in the tread.
        /// </summary>
        public float TreadSpacing;
        /// <summary>
        /// Resting length of the suspension for each wheel.
        /// </summary>
        public float SuspensionLength;
        /// <summary>
        /// Spring settings for the wheel suspension.
        /// </summary>
        public SpringSettings SuspensionSettings;
        /// <summary>
        /// Friction for the wheel bodies.
        /// </summary>
        public float WheelFriction;
    }

    /// <summary>
    /// Set of handles and references to a tank instance.
    /// </summary>
    public struct Tank
    {
        /// <summary>
        /// Body handle of the tank's main body.
        /// </summary>
        public int Body;
        /// <summary>
        /// Body handle of the tank's turret.
        /// </summary>
        public int Turret;
        /// <summary>
        /// Body handle of the tank's barrel.
        /// </summary>
        public int Barrel;
        /// <summary>
        /// Constraint handle of the turret swivel servo.
        /// </summary>
        public int TurretServo;
        /// <summary>
        /// Constraint handle of the barrel pitch servo.
        /// </summary>
        public int BarrelServo;
        /// <summary>
        /// List of all wheel body handles associated with the tank.
        /// </summary>
        public Buffer<int> WheelHandles;
        /// <summary>
        /// List of all constraint handles associated with the tank. Includes motors.
        /// </summary>
        public Buffer<int> Constraints;
        /// <summary>
        /// List of constraint handles associated with the left tread's drive motors.
        /// </summary>
        public Buffer<int> LeftMotors;
        /// <summary>
        /// List of constraint handles associated with the right tread's drive motors.
        /// </summary>
        public Buffer<int> RightMotors;

        Quaternion FromBodyLocalToTurretBasisLocal;

        //We cache the motor descriptions so we don't need to recompute the bases.
        TwistServo BarrelServoDescription;
        TwistServo TurretServoDescription;

        public void SetSpeed(Simulation simulation, Buffer<int> motors, float speed, float maximumForce)
        {
            //This sets all properties of a motor at once; it's possible to create a custom description that only assigns a subset of properties if you find this to be somehow expensive.
            var motorDescription = new AngularAxisMotor
            {
                //Assuming the wheels are cylinders oriented in the obvious way.
                LocalAxisA = new Vector3(0, -1, 0),
                Settings = new MotorSettings(maximumForce, 1e-6f),
                TargetVelocity = speed
            };
            for (int i = 0; i < motors.Length; ++i)
            {
                simulation.Solver.ApplyDescription(motors[i], ref motorDescription);
            }
        }

        /// <summary>
        /// Computes the swivel and pitch angles required to aim in a given direction based on the tank's current pose.
        /// </summary>
        /// <param name="simulation">Simulation containing the tank.</param>
        /// <param name="aimDirection">Direction to aim in.</param>
        /// <returns>Swivel and pitch angles to point in the given direction.</returns>
        public (float targetSwivelAngle, float targetPitchAngle) ComputeTurretAngles(Simulation simulation, in Vector3 aimDirection)
        {
            //Decompose the aim direction into target angles for the turret and barrel servos.
            //First, we need to compute the frame of reference and transform the aim direction into the tank's local space.
            //aimDirection * inverse(body.Pose.Orientation) * Tank.LocalBodyPose.Orientation * inverse(Tank.TurretBasis)
            Quaternion.ConcatenateWithoutOverlap(Quaternion.Conjugate(simulation.Bodies.GetBodyReference(Body).Pose.Orientation), FromBodyLocalToTurretBasisLocal, out var toTurretBasis);
            //-Z in the turret basis points along the 0 angle direction for both swivel and pitch.
            //+Y is 90 degrees for pitch.
            //+X is 90 degres for swivel.
            //We'll compute the swivel angle first.
            Quaternion.TransformWithoutOverlap(aimDirection, toTurretBasis, out var aimDirectionInTurretBasis);
            var targetSwivelAngle = MathF.Atan2(-aimDirectionInTurretBasis.Z, aimDirectionInTurretBasis.X);

            //Barrel pitching is measured against the +Y axis and an axis created from the target swivel angle.
            var targetPitchAngle = MathF.Asin(MathF.Max(-1f, MathF.Min(1f, aimDirectionInTurretBasis.Y)));
            return (targetSwivelAngle, targetPitchAngle);
        }

        /// <summary>
        /// Applies a target swivel and pitch angle to the turret's servos.
        /// </summary>
        /// <param name="simulation">Simulation containing the tank.</param>
        /// <param name="targetSwivelAngle">Target swivel angle of the turret.</param>
        /// <param name="targetPitchAngle">Target pitch angle of the barrel.</param>
        public void SetAim(Simulation simulation, float targetSwivelAngle, float targetPitchAngle)
        {
            var turretDescription = TurretServoDescription;
            turretDescription.TargetAngle = targetSwivelAngle;
            simulation.Solver.ApplyDescription(TurretServo, ref turretDescription);
            var barrelDescription = BarrelServoDescription;
            barrelDescription.TargetAngle = targetPitchAngle;
            simulation.Solver.ApplyDescription(BarrelServo, ref barrelDescription);

        }

        static int CreateWheel(Simulation simulation, BodyProperty<TankBodyProperties> properties, in RigidPose tankPose, in RigidPose bodyLocalPose,
            TypedIndex wheelShape, BodyInertia wheelInertia, float wheelFriction, int bodyHandle, ref SubgroupCollisionFilter bodyFilter, in Vector3 bodyToWheelSuspension, float suspensionLength,
            in SpringSettings suspensionSettings, in Quaternion localWheelOrientation,
            ref QuickList<int> wheelHandles, ref QuickList<int> constraints, ref QuickList<int> motors)
        {
            RigidPose wheelPose;
            Quaternion.TransformUnitX(localWheelOrientation, out var suspensionDirection);
            RigidPose.Transform(bodyToWheelSuspension + suspensionDirection * suspensionLength, tankPose, out wheelPose.Position);
            Quaternion.ConcatenateWithoutOverlap(localWheelOrientation, tankPose.Orientation, out wheelPose.Orientation);

            var wheelHandle = simulation.Bodies.Add(BodyDescription.CreateDynamic(wheelPose, wheelInertia, new CollidableDescription(wheelShape, 0.1f), new BodyActivityDescription(0.01f)));
            wheelHandles.AllocateUnsafely() = wheelHandle;

            //We need a LinearAxisServo to act as the suspension spring, pushing the wheel down.
            constraints.AllocateUnsafely() = simulation.Solver.Add(bodyHandle, wheelHandle, new LinearAxisServo
            {
                LocalPlaneNormal = suspensionDirection,
                TargetOffset = suspensionLength,
                LocalOffsetA = bodyToWheelSuspension,
                LocalOffsetB = default,
                ServoSettings = ServoSettings.Default,
                SpringSettings = suspensionSettings
            });
            //A PointOnLineServo keeps the wheel on a fixed track. Note that it does not constrain the angular behavior of the wheel at all.
            constraints.AllocateUnsafely() = simulation.Solver.Add(bodyHandle, wheelHandle, new PointOnLineServo
            {
                LocalDirection = suspensionDirection,
                LocalOffsetA = bodyToWheelSuspension,
                LocalOffsetB = default,
                ServoSettings = ServoSettings.Default,
                SpringSettings = new SpringSettings(30, 1)
            });
            //The angular component is handled by a hinge. Note that we only use the angular component of a hinge constraint here- the PointOnLineServo handles the linear degrees of freedom.
            //We're assuming the wheels will be cylinders. Pretty safe bet. A cylinder rolls around its local Y axis, so the motor will act along that axis.
            Quaternion.TransformUnitY(localWheelOrientation, out var wheelRotationAxis);
            constraints.AllocateUnsafely() = simulation.Solver.Add(bodyHandle, wheelHandle, new AngularHinge
            {
                LocalHingeAxisA = Quaternion.Transform(wheelRotationAxis, Quaternion.Conjugate(bodyLocalPose.Orientation)),
                LocalHingeAxisB = new Vector3(0, 1, 0),
                SpringSettings = new SpringSettings(30, 1)
            });
            //We'll need a velocity motor to actually make the tank move.
            var motorHandle = simulation.Solver.Add(wheelHandle, bodyHandle, new AngularAxisMotor
            {
                //(All these are technically set on the fly during the update right now, but a custom constraint description could set only the Settings and TargetVelocity,
                //leaving the LocalAxisA unchanged, so we'll go ahead and set it to a reasonable value.)
                LocalAxisA = new Vector3(0, 1, 0),
                Settings = default,
                TargetVelocity = default
            });
            motors.AllocateUnsafely() = motorHandle;
            constraints.AllocateUnsafely() = motorHandle;
            ref var wheelProperties = ref properties.Allocate(wheelHandle);
            wheelProperties = new TankBodyProperties { Filter = new SubgroupCollisionFilter(bodyHandle, 3), Friction = wheelFriction };
            //The wheels don't need to be tested against the body or each other.
            SubgroupCollisionFilter.DisableCollision(ref wheelProperties.Filter, ref bodyFilter);
            SubgroupCollisionFilter.DisableCollision(ref wheelProperties.Filter, ref wheelProperties.Filter);
            return wheelHandle;
        }

        static ref SubgroupCollisionFilter CreatePart(Simulation simulation, in TankPartDescription part, RigidPose pose, BodyProperty<TankBodyProperties> properties, out int handle)
        {
            RigidPose.Multiply(part.Pose, pose, out var bodyPose);
            handle = simulation.Bodies.Add(BodyDescription.CreateDynamic(bodyPose, part.Inertia, new CollidableDescription(part.Shape, 0.1f), new BodyActivityDescription(0.01f)));
            ref var partProperties = ref properties.Allocate(handle);
            partProperties = new TankBodyProperties { Friction = part.Friction };
            return ref partProperties.Filter;
        }

        /// <summary>
        /// Creates a tank from a provided tank description.
        /// </summary>
        /// <param name="simulation">Simulation to add the tank to.</param>
        /// <param name="properties">Property set to store per-body information into.</param>
        /// <param name="pool">Buffer pool to allocate tank resources from.</param>
        /// <param name="pose">Pose of the tank.</param>
        /// <param name="description">Description of the tank.</param>
        /// <returns>Tank instance containing references to the simulation tank parts.</returns>
        public static Tank Create(Simulation simulation, BodyProperty<TankBodyProperties> properties, BufferPool pool, in RigidPose pose, in TankDescription description)
        {
            var wheelHandles = new QuickList<int>(description.WheelCountPerTread * 2, pool);
            var constraints = new QuickList<int>(description.WheelCountPerTread * 2 * 6 + 4, pool);
            var leftMotors = new QuickList<int>(description.WheelCountPerTread, pool);
            var rightMotors = new QuickList<int>(description.WheelCountPerTread, pool);
            Tank tank;
            ref var bodyFilter = ref CreatePart(simulation, description.Body, pose, properties, out tank.Body);
            ref var turretFilter = ref CreatePart(simulation, description.Turret, pose, properties, out tank.Turret);
            ref var barrelFilter = ref CreatePart(simulation, description.Barrel, pose, properties, out tank.Barrel);
            //Use the tank's body handle as the group id for collision filters.
            bodyFilter = new SubgroupCollisionFilter(tank.Body, 0);
            turretFilter = new SubgroupCollisionFilter(tank.Body, 1);
            barrelFilter = new SubgroupCollisionFilter(tank.Body, 2);
            SubgroupCollisionFilter.DisableCollision(ref bodyFilter, ref turretFilter);
            SubgroupCollisionFilter.DisableCollision(ref turretFilter, ref barrelFilter);

            Matrix3x3.CreateFromQuaternion(description.TurretBasis, out var turretBasis);

            //Attach the turret to the body.
            Quaternion.Transform(turretBasis.Y, Quaternion.Conjugate(description.Body.Pose.Orientation), out var bodyLocalSwivelAxis);
            Quaternion.Transform(turretBasis.Y, Quaternion.Conjugate(description.Turret.Pose.Orientation), out var turretLocalSwivelAxis);
            RigidPose.TransformByInverse(description.TurretAnchor, description.Body.Pose, out var bodyLocalTurretAnchor);
            RigidPose.TransformByInverse(description.TurretAnchor, description.Turret.Pose, out var turretLocalTurretAnchor);
            constraints.AllocateUnsafely() = simulation.Solver.Add(tank.Body, tank.Turret,
                new Hinge
                {
                    LocalHingeAxisA = bodyLocalSwivelAxis,
                    LocalHingeAxisB = turretLocalSwivelAxis,
                    LocalOffsetA = bodyLocalTurretAnchor,
                    LocalOffsetB = turretLocalTurretAnchor,
                    SpringSettings = new SpringSettings(30, 1)
                });
            //The twist servo might seem like an odd choice to control 1 angular degree of freedom, but servo-like control over 1DOF requires a measurement basis to interpret the target angle.
            //Hence the apparent complexity.
            Matrix3x3 turretSwivelBasis;
            turretSwivelBasis.Z = -turretBasis.Y;
            turretSwivelBasis.X = -turretBasis.Z;
            turretSwivelBasis.Y = turretBasis.X;
            Debug.Assert(turretSwivelBasis.Determinant() > 0.999f && turretSwivelBasis.Determinant() < 1.0001f, "The turret swivel axis and forward axis should be perpendicular and unit length.");
            Quaternion.CreateFromRotationMatrix(turretSwivelBasis, out var turretSwivelBasisQuaternion);
            Quaternion.ConcatenateWithoutOverlap(turretSwivelBasisQuaternion, Quaternion.Conjugate(description.Body.Pose.Orientation), out var bodyLocalTurretBasis);
            Quaternion.ConcatenateWithoutOverlap(turretSwivelBasisQuaternion, Quaternion.Conjugate(description.Turret.Pose.Orientation), out var turretLocalTurretBasis);
            tank.TurretServoDescription = new TwistServo
            {
                LocalBasisA = bodyLocalTurretBasis,
                LocalBasisB = turretLocalTurretBasis,
                SpringSettings = description.TurretSpring,
                ServoSettings = description.TurretServo
            };
            tank.TurretServo = simulation.Solver.Add(tank.Body, tank.Turret, tank.TurretServoDescription);
            constraints.AllocateUnsafely() = tank.TurretServo;

            //Attach the barrel to the turret.
            Quaternion.Transform(turretBasis.X, Quaternion.Conjugate(description.Turret.Pose.Orientation), out var turretLocalPitchAxis);
            Quaternion.Transform(turretBasis.X, Quaternion.Conjugate(description.Barrel.Pose.Orientation), out var barrelLocalPitchAxis);
            RigidPose.TransformByInverse(description.BarrelAnchor, description.Turret.Pose, out var turretLocalBarrelAnchor);
            RigidPose.TransformByInverse(description.BarrelAnchor, description.Barrel.Pose, out var barrelLocalBarrelAnchor);
            constraints.AllocateUnsafely() = simulation.Solver.Add(tank.Turret, tank.Barrel,
                new Hinge
                {
                    LocalHingeAxisA = turretLocalPitchAxis,
                    LocalHingeAxisB = barrelLocalPitchAxis,
                    LocalOffsetA = turretLocalBarrelAnchor,
                    LocalOffsetB = barrelLocalBarrelAnchor,
                    SpringSettings = new SpringSettings(30, 1)
                });
            //The twist servo might seem like an odd choice to control 1 angular degree of freedom, but servo-like control over 1DOF requires a measurement basis to interpret the target angle.
            //Hence the apparent complexity.
            Matrix3x3 barrelPitchBasis;
            barrelPitchBasis.Z = -turretBasis.X;
            barrelPitchBasis.X = -turretBasis.Z;
            barrelPitchBasis.Y = -turretBasis.Y;
            Debug.Assert(barrelPitchBasis.Determinant() > 0.999f && barrelPitchBasis.Determinant() < 1.0001f, "The barrel axis and forward axis should be perpendicular and unit length.");
            Quaternion.CreateFromRotationMatrix(barrelPitchBasis, out var barrelPitchBasisQuaternion);
            Quaternion.ConcatenateWithoutOverlap(barrelPitchBasisQuaternion, Quaternion.Conjugate(description.Turret.Pose.Orientation), out var turretLocalBarrelBasis);
            Quaternion.ConcatenateWithoutOverlap(barrelPitchBasisQuaternion, Quaternion.Conjugate(description.Barrel.Pose.Orientation), out var barrelLocalBarrelBasis);
            tank.BarrelServoDescription = new TwistServo
            {
                LocalBasisA = turretLocalBarrelBasis,
                LocalBasisB = barrelLocalBarrelBasis,
                SpringSettings = description.BarrelSpring,
                ServoSettings = description.BarrelServo
            };
            tank.BarrelServo = simulation.Solver.Add(tank.Turret, tank.Barrel, tank.BarrelServoDescription);
            constraints.AllocateUnsafely() = tank.BarrelServo;

            Quaternion.TransformUnitY(description.WheelOrientation, out var wheelAxis);
            Quaternion.TransformUnitZ(description.WheelOrientation, out var treadDirection);
            var treadStart = description.TreadSpacing * (description.WheelCountPerTread - 1) * -0.5f;
            int previousLeftWheelHandle = 0, previousRightWheelHandle = 0;
            for (int i = 0; i < description.WheelCountPerTread; ++i)
            {
                var wheelOffsetFromTread = treadDirection * (treadStart + i * description.TreadSpacing);
                var rightWheelHandle = CreateWheel(simulation, properties, pose, description.Body.Pose,
                    description.WheelShape, description.WheelInertia, description.WheelFriction, tank.Body, ref properties[tank.Body].Filter,
                    description.RightTreadOffset + wheelOffsetFromTread - description.Body.Pose.Position,
                    description.SuspensionLength, description.SuspensionSettings, description.WheelOrientation,
                    ref wheelHandles, ref constraints, ref rightMotors);
                var leftWheelHandle = CreateWheel(simulation, properties, pose, description.Body.Pose,
                    description.WheelShape, description.WheelInertia, description.WheelFriction, tank.Body, ref properties[tank.Body].Filter,
                    description.LeftTreadOffset + wheelOffsetFromTread - description.Body.Pose.Position,
                    description.SuspensionLength, description.SuspensionSettings, description.WheelOrientation,
                    ref wheelHandles, ref constraints, ref leftMotors);

                if (i >= 1)
                {
                    //Connect wheels in a tread to each other to distribute the drive forces.
                    //The motor will always just target 0 velocity. The wheel orientations will be allowed to drift from each other.
                    //(If you didn't want to allow drift, you could use an AngularServo or TwistServo. AngularServo constrains all 3 degrees of freedom, but for these purposes, that'd be fine.)
                    var motorDescription = new AngularAxisMotor { LocalAxisA = new Vector3(0, 1, 0), Settings = new MotorSettings(float.MaxValue, 1e-4f) };
                    constraints.AllocateUnsafely() = simulation.Solver.Add(previousLeftWheelHandle, leftWheelHandle, ref motorDescription);
                    constraints.AllocateUnsafely() = simulation.Solver.Add(previousRightWheelHandle, rightWheelHandle, ref motorDescription);
                }
                previousLeftWheelHandle = leftWheelHandle;
                previousRightWheelHandle = rightWheelHandle;

            }

            tank.WheelHandles = wheelHandles.Span.Slice(wheelHandles.Count);
            tank.Constraints = constraints.Span.Slice(constraints.Count);
            tank.LeftMotors = leftMotors.Span.Slice(leftMotors.Count);
            tank.RightMotors = rightMotors.Span.Slice(rightMotors.Count);

            //To aim, we transform the aim direction into the turret basis. 
            //aimDirectionInTurretBasis = worldAimDirection * inverse(body.Pose.Orientation) * description.Body.Pose.Orientation * inverse(description.TurretBasis), so we precompute and cache:
            //FromBodyLocalToTurretBasisLocal = description.Body.Pose.Orientation * inverse(description.TurretBasis)
            Quaternion.ConcatenateWithoutOverlap(description.Body.Pose.Orientation, Quaternion.Conjugate(description.TurretBasis), out tank.FromBodyLocalToTurretBasisLocal);
            return tank;
        }

        public void Dispose(BufferPool pool)
        {
            pool.Return(ref WheelHandles);
            pool.Return(ref Constraints);
            pool.Return(ref LeftMotors);
            pool.Return(ref RightMotors);
        }

    }

    /// <summary>
    /// Applies control inputs to a tank instance.
    /// </summary>
    struct TankController
    {
        public Tank Tank;

        //While the Tank instance contains references to all the simulation-contained stuff, none of it actually defines how fast or strong the tank is.
        //We store that here in the controller so it can be modified on the fly.
        public float Speed;
        public float Force;
        public float ZoomMultiplier;
        public float IdleForce;
        public float BrakeForce;

        //Track the previous state to force wakeups if the constraint targets have changed.
        private float previousLeftTargetSpeed;
        private float previousLeftForce;
        private float previousRightTargetSpeed;
        private float previousRightForce;
        private float previousTurretSwivel;
        private float previousBarrelPitch;

        public TankController(Tank tank,
            float speed, float force, float zoomMultiplier, float idleForce, float brakeForce) : this()
        {
            Tank = tank;
            Speed = speed;
            Force = force;
            ZoomMultiplier = zoomMultiplier;
            IdleForce = idleForce;
            BrakeForce = brakeForce;
        }

        /// <summary>
        /// Updates constraint targets for an input state.
        /// </summary>
        /// <param name="simulation">Simulation containing the tank.</param>
        /// <param name="leftTargetSpeedFraction">Target speed fraction of the maximum speed for the left tread.</param>
        /// <param name="rightTargetSpeedFraction">Target speed fraction of the maximum speed for the right tread.</param>
        /// <param name="zoom">Whether or not to use the boost mulitplier.</param>
        /// <param name="brakeLeft"></param>
        /// <param name="brakeRight"></param>
        /// <param name="aimDirection"></param>
        public void Update(Simulation simulation, float leftTargetSpeedFraction, float rightTargetSpeedFraction, bool zoom, bool brakeLeft, bool brakeRight, in Vector3 aimDirection)
        {
            var leftTargetSpeed = brakeLeft ? 0 : leftTargetSpeedFraction * Speed;
            var rightTargetSpeed = brakeRight ? 0 : rightTargetSpeedFraction * Speed;
            if (zoom)
            {
                leftTargetSpeed *= ZoomMultiplier;
                rightTargetSpeed *= ZoomMultiplier;
            }
            var leftForce = brakeLeft ? BrakeForce : leftTargetSpeedFraction == 0 ? IdleForce : Force;
            var rightForce = brakeRight ? BrakeForce : rightTargetSpeedFraction == 0 ? IdleForce : Force;

            var (targetSwivelAngle, targetPitchAngle) = Tank.ComputeTurretAngles(simulation, aimDirection);

            if (leftTargetSpeed != previousLeftTargetSpeed || rightTargetSpeed != previousRightTargetSpeed ||
                leftForce != previousLeftForce || rightForce != previousRightForce ||
                targetSwivelAngle != previousTurretSwivel || targetPitchAngle != previousBarrelPitch)
            {
                //By guarding the constraint modifications behind a state test, we avoid waking up the tank every single frame.
                //(We could have also used the ApplyDescriptionWithoutWaking function and then explicitly woke the tank up when changes occur.)
                Tank.SetSpeed(simulation, Tank.LeftMotors, leftTargetSpeed, leftForce);
                Tank.SetSpeed(simulation, Tank.RightMotors, rightTargetSpeed, rightForce);
                previousLeftTargetSpeed = leftTargetSpeed;
                previousRightTargetSpeed = rightTargetSpeed;
                previousLeftForce = leftForce;
                previousRightForce = rightForce;
                Tank.SetAim(simulation, targetSwivelAngle, targetPitchAngle);
                previousTurretSwivel = targetSwivelAngle;
                previousBarrelPitch = targetPitchAngle;
            }


        }
    }

    public struct TankBodyProperties
    {
        /// <summary>
        /// Controls which collidables the body can collide with.
        /// </summary>
        public SubgroupCollisionFilter Filter;
        /// <summary>
        /// Friction coefficient to use for the body.
        /// </summary>
        public float Friction;
    }

    /// <summary>
    /// For the tank demo, we want both wheel-body collision filtering and different friction for wheels versus the tank body.
    /// </summary>
    struct TankCallbacks : INarrowPhaseCallbacks
    {
        public BodyProperty<TankBodyProperties> Properties;
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
                //If two bodies collide, just average the friction. Other options include min(a, b) or a * b.
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

    public class TankDemo : Demo
    {
        TankController playerController;

        static Key Forward = Key.W;
        static Key Backward = Key.S;
        static Key Right = Key.D;
        static Key Left = Key.A;
        static Key Zoom = Key.LShift;
        static Key Brake = Key.Space;
        static Key BrakeAlternate = Key.BackSpace; //I have a weird keyboard.
        static Key ToggleTank = Key.C;
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 5, 10);
            camera.Yaw = 0;
            camera.Pitch = 0;

            var properties = new BodyProperty<TankBodyProperties>();
            Simulation = Simulation.Create(BufferPool, new TankCallbacks() { Properties = properties }, new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));

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

            var tankDescription = new TankDescription
            {
                Body = TankPartDescription.Create(10, new Box(4f, 1, 5), RigidPose.Identity, 0.5f, Simulation.Shapes),
                Turret = TankPartDescription.Create(1, new Box(1.5f, 0.7f, 2f), new RigidPose(new Vector3(0, 0.85f, 0.4f)), 0.5f, Simulation.Shapes),
                Barrel = TankPartDescription.Create(0.5f, new Box(0.2f, 0.2f, 3f), new RigidPose(new Vector3(0, 0.85f, 0.4f - 1f - 1.5f)), 0.5f, Simulation.Shapes),
                TurretAnchor = new Vector3(0f, 0.5f, 0.4f),
                BarrelAnchor = new Vector3(0, 0.5f + 0.35f, 0.4f - 1f),
                TurretBasis = Quaternion.Identity,
                TurretServo = new ServoSettings(1f, 0f, 40f),
                TurretSpring = new SpringSettings(10f, 1f),
                BarrelServo = new ServoSettings(1f, 0f, 40f),
                BarrelSpring = new SpringSettings(10f, 1f),
                LeftTreadOffset = new Vector3(-1.9f, 0f, 0),
                RightTreadOffset = new Vector3(1.9f, 0f, 0),
                SuspensionLength = 1f,
                SuspensionSettings = new SpringSettings(2.5f, 2f),
                WheelShape = wheelShapeIndex,
                WheelInertia = wheelInertia,
                WheelFriction = 2f,
                TreadSpacing = 1f,
                WheelCountPerTread = 5,
                WheelOrientation = Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI * -0.5f),
            };

            playerController = new TankController(Tank.Create(Simulation, properties, BufferPool, new RigidPose(new Vector3(0, 10, 0), Quaternion.Identity), tankDescription), 20, 5, 2, 1, 3.5f);


            const int planeWidth = 257;
            const float scale = 3;
            var terrainPosition = new Vector2(1 - planeWidth, 1 - planeWidth) * scale * 0.5f;
            var random = new Random(5);

            //Add some building-ish landmarks .
            Vector3 landmarkMin = new Vector3(-planeWidth * scale * 0.45f, 10, -planeWidth * scale * 0.45f);
            Vector3 landmarkSpan = new Vector3(planeWidth * scale * 0.9f, 15, planeWidth * scale * 0.9f);
            for (int j = 0; j < 25; ++j)
            {
                var buildingShape = new Box(10 + (float)random.NextDouble() * 10, 20 + (float)random.NextDouble() * 20, 10 + (float)random.NextDouble() * 10);
                Simulation.Statics.Add(new StaticDescription(
                    new Vector3(0, buildingShape.HalfHeight, 0) + landmarkMin + landmarkSpan * new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()),
                    Quaternion.CreateFromAxisAngle(Vector3.UnitY, (float)random.NextDouble() * MathF.PI),
                    new CollidableDescription(Simulation.Shapes.Add(buildingShape), 0.1f)));
            }




            DemoMeshHelper.CreateDeformedPlane(planeWidth, planeWidth,
                (int vX, int vY) =>
                {
                    var octave0 = (MathF.Sin((vX + 5f) * 0.05f) + MathF.Sin((vY + 11) * 0.05f)) * 1.8f;
                    var octave1 = (MathF.Sin((vX + 17) * 0.15f) + MathF.Sin((vY + 19) * 0.15f)) * 0.9f;
                    var octave2 = (MathF.Sin((vX + 37) * 0.35f) + MathF.Sin((vY + 93) * 0.35f)) * 0.4f;
                    var octave3 = (MathF.Sin((vX + 53) * 0.65f) + MathF.Sin((vY + 47) * 0.65f)) * 0.2f;
                    var octave4 = (MathF.Sin((vX + 67) * 1.50f) + MathF.Sin((vY + 13) * 1.5f)) * 0.125f;
                    var distanceToEdge = planeWidth / 2 - Math.Max(Math.Abs(vX - planeWidth / 2), Math.Abs(vY - planeWidth / 2));
                    var edgeRamp = 25f / (distanceToEdge + 1);
                    var terrainHeight = octave0 + octave1 + octave2 + octave3 + octave4;
                    var vertexPosition = new Vector2(vX * scale, vY * scale) + terrainPosition;
                    return new Vector3(vertexPosition.X, terrainHeight + edgeRamp, vertexPosition.Y);

                }, new Vector3(1, 1, 1), BufferPool, out var planeMesh);
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -15, 0), Quaternion.CreateFromAxisAngle(new Vector3(0, 1, 0), MathF.PI / 2),
                new CollidableDescription(Simulation.Shapes.Add(planeMesh), 0.1f)));


        }

        bool playerControlActive = true;
        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            if (input.WasPushed(ToggleTank))
                playerControlActive = !playerControlActive;
            if (playerControlActive)
            {
                float leftTargetSpeedFraction = 0;
                float rightTargetSpeedFraction = 0;
                var left = input.IsDown(Left);
                var right = input.IsDown(Right);
                var signedTarget = input.IsDown(Forward) ? 1f : input.IsDown(Backward) ? -1f : 0;

                if ((left && right) || (!left && !right))
                {
                    leftTargetSpeedFraction = signedTarget;
                    rightTargetSpeedFraction = signedTarget;
                }
                //If we're trying to move forward and turn, idle the opposing track.
                else if (left)
                {
                    leftTargetSpeedFraction = signedTarget;
                    rightTargetSpeedFraction = 0f;
                }
                else if (right)
                {
                    leftTargetSpeedFraction = 0f;
                    rightTargetSpeedFraction = signedTarget;
                }
                var zoom = input.IsDown(Zoom);
                var brake = input.IsDown(Brake) || input.IsDown(BrakeAlternate);
                playerController.Update(Simulation, leftTargetSpeedFraction, rightTargetSpeedFraction, zoom, brake, brake, camera.Forward);
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
                var tankBody = new BodyReference(playerController.Tank.Body, Simulation.Bodies);
                Quaternion.TransformUnitY(tankBody.Pose.Orientation, out var carUp);
                camera.Position = tankBody.Pose.Position + carUp * 1.3f + camera.Backward * 8;
            }

            var textHeight = 16;
            var position = new Vector2(32, renderer.Surface.Resolution.Y - 128);
            RenderControl(ref position, textHeight, nameof(Forward), Forward.ToString(), text, renderer.TextBatcher, font);
            RenderControl(ref position, textHeight, nameof(Backward), Backward.ToString(), text, renderer.TextBatcher, font);
            RenderControl(ref position, textHeight, nameof(Right), Right.ToString(), text, renderer.TextBatcher, font);
            RenderControl(ref position, textHeight, nameof(Left), Left.ToString(), text, renderer.TextBatcher, font);
            RenderControl(ref position, textHeight, nameof(Zoom), Zoom.ToString(), text, renderer.TextBatcher, font);
            RenderControl(ref position, textHeight, nameof(Brake), Brake.ToString(), text, renderer.TextBatcher, font);
            RenderControl(ref position, textHeight, nameof(ToggleTank), ToggleTank.ToString(), text, renderer.TextBatcher, font);
            base.Render(renderer, camera, input, text, font);
        }
    }
}