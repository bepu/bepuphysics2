using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using System.Numerics;

namespace Demos
{
    public struct RegularGridBuilder : IBodyBuilder
    {
        public Vector3 Spacing;
        public Vector3 Origin;
        public BodyInertia LocalInertia;
        public TypedIndex ShapeIndex;
        public RegularGridBuilder(Vector3 spacing, Vector3 origin, BodyInertia localInertia, TypedIndex shapeIndex = new TypedIndex())
        {
            Spacing = spacing;
            Origin = origin;
            LocalInertia = localInertia;
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
                    //Orientation = BepuUtilities.Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 1, 1)), MathHelper.PiOver4 * 9.83f)
                    //Orientation = BepuUtilities.Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(0, 1, 0)), rowIndex * MathHelper.Pi * 0.1f)
                },
                LocalInertia = LocalInertia,
                Collidable = new CollidableDescription
                {
                    Continuity = new ContinuousDetectionSettings(),
                    SpeculativeMargin = 0.04f,
                    Shape = ShapeIndex
                },
                Activity = new BodyActivityDescription
                {
                    SleepThreshold = .01f,
                    MinimumTimestepCountUnderThreshold = 32
                },
                //Velocity = new BodyVelocity { Angular = new Vector3(0, (rowIndex % 2 - 0.5f) * 20, 0) }
                //Velocity = new BodyVelocity { Angular = new Vector3(1, 0, 0) }
            };

        }
    }
}

