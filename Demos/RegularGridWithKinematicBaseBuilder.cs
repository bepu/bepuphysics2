using BepuPhysics;
using BepuPhysics.Collidables;
using System.Numerics;

namespace Demos
{
    public struct RegularGridWithKinematicBaseBuilder : IBodyBuilder
    {
        public Vector3 Spacing;
        public Vector3 Origin;
        public float InverseInertiaMultiplier;
        public TypedIndex ShapeIndex;
        public RegularGridWithKinematicBaseBuilder(Vector3 spacing, Vector3 origin, float inverseInertiaMultiplier = 0, TypedIndex shapeIndex = new TypedIndex())
        {
            Spacing = spacing;
            Origin = origin;
            InverseInertiaMultiplier = inverseInertiaMultiplier;
            ShapeIndex = shapeIndex;
        }

        public void Build(int columnIndex, int rowIndex, int sliceIndex, out BodyDescription bodyDescription)
        {
            bodyDescription = new BodyDescription
            {
                Pose = new RigidPose
                {
                    Position = new Vector3(columnIndex, rowIndex, sliceIndex) * Spacing + Origin,
                    Orientation = BepuUtilities.Quaternion.Identity
                },
                LocalInertia = new BodyInertia { InverseMass = rowIndex > 0 ? 1 : 0 },
                Collidable = new CollidableDescription
                {
                    Continuity = new ContinuousDetectionSettings(),
                    SpeculativeMargin = 0.1f,
                    Shape = ShapeIndex
                },
                Activity = new BodyActivityDescription
                {
                    SleepThreshold = 0.1f,
                    MinimumTimestepCountUnderThreshold = 16
                }
            };

            var inverseInertia = bodyDescription.LocalInertia.InverseMass * InverseInertiaMultiplier;
            bodyDescription.LocalInertia.InverseInertiaTensor.XX = inverseInertia;
            bodyDescription.LocalInertia.InverseInertiaTensor.YY = inverseInertia;
            bodyDescription.LocalInertia.InverseInertiaTensor.ZZ = inverseInertia;

        }
    }
}

