namespace BepuPhysics.Constraints
{
    /// <summary>
    /// Constrains which body properties should be accessed in a body during constraint data gathering/scattering.
    /// </summary>
    public interface IBodyAccessFilter
    {
        /// <summary>
        /// Gets whether position is loaded by the constraint.
        /// </summary>
        public bool GatherPosition { get; }
        /// <summary>
        /// Gets whether orientation is loaded by the constraint.
        /// </summary>
        public bool GatherOrientation { get; }
        /// <summary>
        /// Gets whether body mass is loaded by this constraint.
        /// </summary>
        public bool GatherMass { get; }
        /// <summary>
        /// Gets whether body inertia tensor is loaded by this constraint.
        /// </summary>
        public bool GatherInertiaTensor { get; }
        /// <summary>
        /// Gets whether to load or store body linear velocity in this constraint.
        /// </summary>
        public bool AccessLinearVelocity { get; }
        /// <summary>
        /// Gets whether to load or store body linear velocity in this constraint.
        /// </summary>
        public bool AccessAngularVelocity { get; }
    }


    /// <summary>
    /// Marks all body properties as necessary for gather/scatter.
    /// </summary>
    public struct AccessAll : IBodyAccessFilter
    {
        public bool GatherPosition => true;
        public bool GatherOrientation => true;
        public bool GatherMass => true;
        public bool GatherInertiaTensor => true;
        public bool AccessLinearVelocity => true;
        public bool AccessAngularVelocity => true;
    }

    /// <summary>
    /// Used for kinematic integration; the inertias are known ahead of time and there's no reason to gather them.
    /// </summary>
    public struct AccessNoInertia : IBodyAccessFilter
    {
        public bool GatherPosition => true;
        public bool GatherOrientation => true;
        public bool GatherMass => false;
        public bool GatherInertiaTensor => false;
        public bool AccessLinearVelocity => true;
        public bool AccessAngularVelocity => true;
    }

    public struct AccessNoPose : IBodyAccessFilter
    {
        public bool GatherPosition => false;
        public bool GatherOrientation => false;
        public bool GatherMass => true;
        public bool GatherInertiaTensor => true;
        public bool AccessLinearVelocity => true;
        public bool AccessAngularVelocity => true;
    }
    public struct AccessNoPosition : IBodyAccessFilter
    {
        public bool GatherPosition => false;
        public bool GatherOrientation => true;
        public bool GatherMass => true;
        public bool GatherInertiaTensor => true;
        public bool AccessLinearVelocity => true;
        public bool AccessAngularVelocity => true;
    }
    public struct AccessNoOrientation : IBodyAccessFilter
    {
        public bool GatherPosition => true;
        public bool GatherOrientation => false;
        public bool GatherMass => true;
        public bool GatherInertiaTensor => true;
        public bool AccessLinearVelocity => true;
        public bool AccessAngularVelocity => true;
    }
    public struct AccessOnlyVelocity: IBodyAccessFilter
    {
        public bool GatherPosition => false;
        public bool GatherOrientation => false;
        public bool GatherMass => false;
        public bool GatherInertiaTensor => false;
        public bool AccessLinearVelocity => true;
        public bool AccessAngularVelocity => true;
    }

    public struct AccessOnlyAngular : IBodyAccessFilter
    {
        public bool GatherPosition => false;
        public bool GatherOrientation => true;
        public bool GatherMass => false;
        public bool GatherInertiaTensor => true;
        public bool AccessLinearVelocity => false;
        public bool AccessAngularVelocity => true;
    }

    public struct AccessOnlyAngularWithoutPose : IBodyAccessFilter
    {
        public bool GatherPosition => false;
        public bool GatherOrientation => false;
        public bool GatherMass => false;
        public bool GatherInertiaTensor => true;
        public bool AccessLinearVelocity => false;
        public bool AccessAngularVelocity => true;
    }

    public struct AccessOnlyLinear : IBodyAccessFilter
    {
        public bool GatherPosition => true;
        public bool GatherOrientation => false;
        public bool GatherMass => true;
        public bool GatherInertiaTensor => false;
        public bool AccessLinearVelocity => true;
        public bool AccessAngularVelocity => false;
    }
}
