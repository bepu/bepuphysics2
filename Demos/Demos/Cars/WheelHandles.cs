using BepuPhysics;

namespace Demos.Demos.Cars
{
    struct WheelHandles
    {
        public BodyHandle Wheel;
        public ConstraintHandle SuspensionSpring;
        public ConstraintHandle SuspensionTrack;
        public ConstraintHandle Hinge;
        public ConstraintHandle Motor;
    }
}