using System;
using System.Diagnostics;
using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;

namespace Demos.Demos.Tanks
{
    /// <summary>
    /// Set of handles and references to a tank instance.
    /// </summary>
    public struct Tank
    {
        /// <summary>
        /// Body handle of the tank's main body.
        /// </summary>
        public BodyHandle Body;
        /// <summary>
        /// Body handle of the tank's turret.
        /// </summary>
        public BodyHandle Turret;
        /// <summary>
        /// Body handle of the tank's barrel.
        /// </summary>
        public BodyHandle Barrel;
        /// <summary>
        /// Constraint handle of the turret swivel servo.
        /// </summary>
        public ConstraintHandle TurretServo;
        /// <summary>
        /// Constraint handle of the barrel pitch servo.
        /// </summary>
        public ConstraintHandle BarrelServo;
        /// <summary>
        /// List of all wheel body handles associated with the tank.
        /// </summary>
        public Buffer<BodyHandle> WheelHandles;
        /// <summary>
        /// List of all constraint handles associated with the tank. Includes motors.
        /// </summary>
        public Buffer<ConstraintHandle> Constraints;
        /// <summary>
        /// List of constraint handles associated with the left tread's drive motors.
        /// </summary>
        public Buffer<ConstraintHandle> LeftMotors;
        /// <summary>
        /// List of constraint handles associated with the right tread's drive motors.
        /// </summary>
        public Buffer<ConstraintHandle> RightMotors;

        /// <summary>
        /// Transforms directions from body local space to turret basis local space. Used for computing aiming angles.
        /// </summary>
        Quaternion FromBodyLocalToTurretBasisLocal;
        /// <summary>
        /// Orientation of the body in the tank's local space.
        /// </summary>
        public Quaternion BodyLocalOrientation;
        /// <summary>
        /// Location in the barrel body's local space where projectiles should be created.
        /// </summary>
        Vector3 BarrelLocalProjectileSpawn;
        /// <summary>
        /// Direction in the barrel body's local space along which projectiles should be fired.
        /// </summary>
        Vector3 BarrelLocalDirection;
        /// <summary>
        /// Speed of projectiles fired by the tank.
        /// </summary>
        float ProjectileSpeed;
        /// <summary>
        /// Inertia of projectiles fired by the tank.
        /// </summary>
        BodyInertia ProjectileInertia;
        /// <summary>
        /// Shape of the projectiles fired by the tank.
        /// </summary>
        TypedIndex ProjectileShape;

        //We cache the motor descriptions so we don't need to recompute the bases.
        TwistServo BarrelServoDescription;
        TwistServo TurretServoDescription;

        public void SetSpeed(Simulation simulation, Buffer<ConstraintHandle> motors, float speed, float maximumForce)
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
            QuaternionEx.ConcatenateWithoutOverlap(QuaternionEx.Conjugate(simulation.Bodies.GetBodyReference(Body).Pose.Orientation), FromBodyLocalToTurretBasisLocal, out var toTurretBasis);
            //-Z in the turret basis points along the 0 angle direction for both swivel and pitch.
            //+Y is 90 degrees for pitch.
            //+X is 90 degres for swivel.
            //We'll compute the swivel angle first.
            QuaternionEx.TransformWithoutOverlap(aimDirection, toTurretBasis, out var aimDirectionInTurretBasis);
            var targetSwivelAngle = MathF.Atan2(aimDirectionInTurretBasis.X, -aimDirectionInTurretBasis.Z);

            //Barrel pitching is measured against the +Y axis and an axis created from the target swivel angle.
            var targetPitchAngle = MathF.Asin(MathF.Max(-1f, MathF.Min(1f, -aimDirectionInTurretBasis.Y)));
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

        /// <summary>
        /// Computes the direction along which the barrel points.
        /// </summary>
        /// <param name="simulation">Simulation containing the tank.</param>
        /// <param name="barrelDirection">Direction in which the barrel points.</param>
        public void ComputeBarrelDirection(Simulation simulation, out Vector3 barrelDirection)
        {
            QuaternionEx.Transform(BarrelLocalDirection, simulation.Bodies.GetBodyReference(Barrel).Pose.Orientation, out barrelDirection);
        }

        /// <summary>
        /// Fires a projectile.
        /// </summary>
        /// <param name="simulation">Simulation that contains the tank.</param>
        /// <param name="bodyProperties">Body properties to allocate the projectile's properties in.</param>
        /// <returns>Handle of the created projectile body.</returns>
        public BodyHandle Fire(Simulation simulation, CollidableProperty<TankDemoBodyProperties> bodyProperties)
        {
            var barrel = simulation.Bodies.GetBodyReference(Barrel);
            ref var barrelPose = ref barrel.Pose;
            RigidPose.Transform(BarrelLocalProjectileSpawn, barrelPose, out var projectileSpawn);
            QuaternionEx.Transform(BarrelLocalDirection, barrelPose.Orientation, out var barrelDirection);
            var projectileHandle = simulation.Bodies.Add(BodyDescription.CreateDynamic(projectileSpawn, new BodyVelocity(barrelDirection * ProjectileSpeed + barrel.Velocity.Linear), ProjectileInertia,
                //The projectile moves pretty fast, so we'll use continuous collision detection.
                new CollidableDescription(ProjectileShape, 0.1f, ContinuousDetectionSettings.Continuous(1e-3f, 1e-3f)), new BodyActivityDescription(0.01f)));
            ref var projectileProperties = ref bodyProperties.Allocate(projectileHandle);
            projectileProperties.Friction = 1f;
            //Prevent the projectile from colliding with the firing tank.
            projectileProperties.Filter = new SubgroupCollisionFilter(Body.Value);
            projectileProperties.Filter.CollidableSubgroups = 0;
            projectileProperties.Filter.SubgroupMembership = 0;
            projectileProperties.Projectile = true;

            barrel.Awake = true;
            barrel.ApplyLinearImpulse(barrelDirection * -ProjectileSpeed / ProjectileInertia.InverseMass);
            return projectileHandle;
        }

        static BodyHandle CreateWheel(Simulation simulation, CollidableProperty<TankDemoBodyProperties> properties, in RigidPose tankPose, in RigidPose bodyLocalPose,
            TypedIndex wheelShape, BodyInertia wheelInertia, float wheelFriction, BodyHandle bodyHandle, ref SubgroupCollisionFilter bodyFilter, in Vector3 bodyToWheelSuspension, float suspensionLength,
            in SpringSettings suspensionSettings, in Quaternion localWheelOrientation,
            ref QuickList<BodyHandle> wheelHandles, ref QuickList<ConstraintHandle> constraints, ref QuickList<ConstraintHandle> motors)
        {
            RigidPose wheelPose;
            QuaternionEx.TransformUnitX(localWheelOrientation, out var suspensionDirection);
            RigidPose.Transform(bodyToWheelSuspension + suspensionDirection * suspensionLength, tankPose, out wheelPose.Position);
            QuaternionEx.ConcatenateWithoutOverlap(localWheelOrientation, tankPose.Orientation, out wheelPose.Orientation);

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
            QuaternionEx.TransformUnitY(localWheelOrientation, out var wheelRotationAxis);
            constraints.AllocateUnsafely() = simulation.Solver.Add(bodyHandle, wheelHandle, new AngularHinge
            {
                LocalHingeAxisA = QuaternionEx.Transform(wheelRotationAxis, QuaternionEx.Conjugate(bodyLocalPose.Orientation)),
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
            wheelProperties = new TankDemoBodyProperties { Filter = new SubgroupCollisionFilter(bodyHandle.Value, 3), Friction = wheelFriction, TankPart = true };
            //The wheels don't need to be tested against the body or each other.
            SubgroupCollisionFilter.DisableCollision(ref wheelProperties.Filter, ref bodyFilter);
            SubgroupCollisionFilter.DisableCollision(ref wheelProperties.Filter, ref wheelProperties.Filter);
            return wheelHandle;
        }

        static ref SubgroupCollisionFilter CreatePart(Simulation simulation, in TankPartDescription part, RigidPose pose, CollidableProperty<TankDemoBodyProperties> properties, out BodyHandle handle)
        {
            RigidPose.MultiplyWithoutOverlap(part.Pose, pose, out var bodyPose);
            handle = simulation.Bodies.Add(BodyDescription.CreateDynamic(bodyPose, part.Inertia, new CollidableDescription(part.Shape, 0.1f), new BodyActivityDescription(0.01f)));
            ref var partProperties = ref properties.Allocate(handle);
            partProperties = new TankDemoBodyProperties { Friction = part.Friction, TankPart = true };
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
        public static Tank Create(Simulation simulation, CollidableProperty<TankDemoBodyProperties> properties, BufferPool pool, in RigidPose pose, in TankDescription description)
        {
            var wheelHandles = new QuickList<BodyHandle>(description.WheelCountPerTread * 2, pool);
            var constraints = new QuickList<ConstraintHandle>(description.WheelCountPerTread * 2 * 6 + 4, pool);
            var leftMotors = new QuickList<ConstraintHandle>(description.WheelCountPerTread, pool);
            var rightMotors = new QuickList<ConstraintHandle>(description.WheelCountPerTread, pool);
            Tank tank;
            ref var bodyFilter = ref CreatePart(simulation, description.Body, pose, properties, out tank.Body);
            ref var turretFilter = ref CreatePart(simulation, description.Turret, pose, properties, out tank.Turret);
            ref var barrelFilter = ref CreatePart(simulation, description.Barrel, pose, properties, out tank.Barrel);
            //Use the tank's body handle as the group id for collision filters.
            bodyFilter = new SubgroupCollisionFilter(tank.Body.Value, 0);
            turretFilter = new SubgroupCollisionFilter(tank.Body.Value, 1);
            barrelFilter = new SubgroupCollisionFilter(tank.Body.Value, 2);
            SubgroupCollisionFilter.DisableCollision(ref bodyFilter, ref turretFilter);
            SubgroupCollisionFilter.DisableCollision(ref turretFilter, ref barrelFilter);

            Matrix3x3.CreateFromQuaternion(description.TurretBasis, out var turretBasis);

            //Attach the turret to the body.
            QuaternionEx.Transform(turretBasis.Y, QuaternionEx.Conjugate(description.Body.Pose.Orientation), out var bodyLocalSwivelAxis);
            QuaternionEx.Transform(turretBasis.Y, QuaternionEx.Conjugate(description.Turret.Pose.Orientation), out var turretLocalSwivelAxis);
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
            QuaternionEx.CreateFromRotationMatrix(turretSwivelBasis, out var turretSwivelBasisQuaternion);
            QuaternionEx.ConcatenateWithoutOverlap(turretSwivelBasisQuaternion, QuaternionEx.Conjugate(description.Body.Pose.Orientation), out var bodyLocalTurretBasis);
            QuaternionEx.ConcatenateWithoutOverlap(turretSwivelBasisQuaternion, QuaternionEx.Conjugate(description.Turret.Pose.Orientation), out var turretLocalTurretBasis);
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
            QuaternionEx.Transform(turretBasis.X, QuaternionEx.Conjugate(description.Turret.Pose.Orientation), out var turretLocalPitchAxis);
            QuaternionEx.Transform(turretBasis.X, QuaternionEx.Conjugate(description.Barrel.Pose.Orientation), out var barrelLocalPitchAxis);
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
            QuaternionEx.CreateFromRotationMatrix(barrelPitchBasis, out var barrelPitchBasisQuaternion);
            QuaternionEx.ConcatenateWithoutOverlap(barrelPitchBasisQuaternion, QuaternionEx.Conjugate(description.Turret.Pose.Orientation), out var turretLocalBarrelBasis);
            QuaternionEx.ConcatenateWithoutOverlap(barrelPitchBasisQuaternion, QuaternionEx.Conjugate(description.Barrel.Pose.Orientation), out var barrelLocalBarrelBasis);
            tank.BarrelServoDescription = new TwistServo
            {
                LocalBasisA = turretLocalBarrelBasis,
                LocalBasisB = barrelLocalBarrelBasis,
                SpringSettings = description.BarrelSpring,
                ServoSettings = description.BarrelServo
            };
            tank.BarrelServo = simulation.Solver.Add(tank.Turret, tank.Barrel, tank.BarrelServoDescription);
            constraints.AllocateUnsafely() = tank.BarrelServo;

            QuaternionEx.TransformUnitY(description.WheelOrientation, out var wheelAxis);
            QuaternionEx.TransformUnitZ(description.WheelOrientation, out var treadDirection);
            var treadStart = description.TreadSpacing * (description.WheelCountPerTread - 1) * -0.5f;
            BodyHandle previousLeftWheelHandle = default, previousRightWheelHandle = default;
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
            QuaternionEx.ConcatenateWithoutOverlap(description.Body.Pose.Orientation, QuaternionEx.Conjugate(description.TurretBasis), out tank.FromBodyLocalToTurretBasisLocal);
            tank.BodyLocalOrientation = description.Body.Pose.Orientation;
            tank.BarrelLocalProjectileSpawn = description.BarrelLocalProjectileSpawn;
            QuaternionEx.Transform(-turretBasis.Z, QuaternionEx.Conjugate(description.Barrel.Pose.Orientation), out tank.BarrelLocalDirection);
            tank.ProjectileInertia = description.ProjectileInertia;
            tank.ProjectileShape = description.ProjectileShape;
            tank.ProjectileSpeed = description.ProjectileSpeed;
            return tank;
        }

        void ClearBodyProperties(ref TankDemoBodyProperties properties)
        {
            //After blowing up, all tank parts will collide with each other, and we should no longer flag the pieces as part of a living tank.
            properties.Filter = new SubgroupCollisionFilter(properties.Filter.GroupId);
            properties.TankPart = false;
        }

        public void Explode(Simulation simulation, CollidableProperty<TankDemoBodyProperties> properties, BufferPool pool)
        {
            //When the tank explodes, we just remove all the binding constraints and let it fall apart and reset body properties.
            for (int i =0; i < WheelHandles.Length; ++i)
            {
                ClearBodyProperties(ref properties[WheelHandles[i]]);
            }
            ClearBodyProperties(ref properties[Body]);
            ClearBodyProperties(ref properties[Turret]);
            ClearBodyProperties(ref properties[Barrel]);
            var turret = simulation.Bodies.GetBodyReference(Turret);
            turret.Awake = true;
            turret.Velocity.Linear += new Vector3(0, 10, 0);
            for (int i =0; i < Constraints.Length; ++i)
            {
                simulation.Solver.Remove(Constraints[i]);
            }
            pool.Return(ref WheelHandles);
            pool.Return(ref Constraints);
            pool.Return(ref LeftMotors);
            pool.Return(ref RightMotors);
        }

    }
}