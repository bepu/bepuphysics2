using BepuPhysics.Collidables;
using System.Numerics;

namespace BepuPhysics
{
    public struct RigidPose
    {
        public Vector3 Position;
        public BepuUtilities.Quaternion Orientation;
    }

    public struct BodyVelocity
    {
        public Vector3 Linear;
        public Vector3 Angular;
    }
    public struct BodyInertia
    {
        public Triangular3x3 InverseInertiaTensor;
        public float InverseMass;
    }

    public struct BodyDescription
    {
        public RigidPose Pose;
        public BodyInertia LocalInertia;
        public BodyVelocity Velocity;
        public CollidableDescription Collidable;

        /// <summary>
        /// Gets the mobility state for a collidable based on this body description's mass and inertia tensor.
        /// If all components of inverse mass and inverse inertia are zero, it is kinematic; otherwise, it is dynamic.
        /// </summary>
        public CollidableMobility Mobility
        {
            get
            {
                return (LocalInertia.InverseMass == 0 &&
                        LocalInertia.InverseInertiaTensor.M11 == 0 &&
                        LocalInertia.InverseInertiaTensor.M21 == 0 &&
                        LocalInertia.InverseInertiaTensor.M22 == 0 &&
                        LocalInertia.InverseInertiaTensor.M31 == 0 &&
                        LocalInertia.InverseInertiaTensor.M32 == 0 &&
                        LocalInertia.InverseInertiaTensor.M33 == 0) ? CollidableMobility.Kinematic : CollidableMobility.Dynamic;
            }
        }

    }

    public struct StaticDescription
    {
        public RigidPose Pose;
        public CollidableDescription Collidable;
    }

    public struct RigidPoses
    {
        public Vector3Wide Position;
        //Note that we store a quaternion rather than a matrix3x3. While this often requires some overhead when performing vector transforms or extracting basis vectors, 
        //systems needing to interact directly with this representation are often terrifically memory bound. Spending the extra ALU time to convert to a basis can actually be faster
        //than loading the extra 5 elements needed to express the full 3x3 rotation matrix. Also, it's marginally easier to keep the rotation normalized over time.
        //There may be an argument for the matrix variant to ALSO be stored for some bandwidth-unconstrained stages, but don't worry about that until there's a reason to worry about it.
        public QuaternionWide Orientation;
    }

    public struct BodyVelocities
    {
        public Vector3Wide LinearVelocity;
        public Vector3Wide AngularVelocity;
    }

    public struct BodyInertias
    {
        public Triangular3x3Wide InverseInertiaTensor;
        //Note that the inverse mass is included in the BodyInertias bundle. InverseMass is rotationally invariant, so it doesn't need to be updated...
        //But it's included alongside the rotated inertia tensor because to split it out would require that constraint presteps suffer another cache miss when they
        //gather the inverse mass in isolation. (From the solver's perspective, inertia/mass gathering is incoherent.)
        public Vector<float> InverseMass;
    }
}
