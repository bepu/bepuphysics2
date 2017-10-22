using System.Numerics;

namespace BepuPhysics
{

    public struct ContactData
    {
        public Vector3Wide OffsetA;
        public Vector3Wide OffsetB;
        public Vector3Wide Normal;
        public Vector<float> PenetrationDepth;
        public SpringSettingsWide SpringSettings;
    }
}
