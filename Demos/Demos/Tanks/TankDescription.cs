using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;

namespace Demos.Demos.Tanks
{
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
        /// Location in the barrel body's local space where projectiles should be created.
        /// </summary>
        public Vector3 BarrelLocalProjectileSpawn;
        /// <summary>
        /// Inertia of fired projectiles.
        /// </summary>
        public BodyInertia ProjectileInertia;
        /// <summary>
        /// Shape of fired projectiles.
        /// </summary>
        public TypedIndex ProjectileShape;
        /// <summary>
        /// Speed of fired projectiles.
        /// </summary>
        public float ProjectileSpeed;


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
}