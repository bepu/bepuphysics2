using System.Numerics;

namespace BepuPhysics.CollisionDetection
{
    public struct ConvexContact2ManifoldWide
    {
        public Vector3Wide OffsetA0;
        public Vector3Wide OffsetA1;
        public Vector3Wide ContactNormal;
        public Vector<float> Depth;
        public Vector<int> Count;
    }

    public struct ConvexContact4ManifoldWide
    {
        public Vector3Wide OffsetA0;
        public Vector3Wide OffsetA1;
        public Vector3Wide OffsetA2;
        public Vector3Wide OffsetA3;
        public Vector3Wide ContactNormal;
        public Vector<float> Depth;
        public Vector<int> Count;
    }

}
