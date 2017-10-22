using System.Numerics;

namespace BepuPhysics.CollisionDetection
{
    public struct ConvexContactManifoldWide
    {
        public Vector3Wide OffsetA0;
        public Vector3Wide OffsetA1;
        public Vector3Wide OffsetA2;
        public Vector3Wide OffsetA3;
        public Vector3Wide OffsetB;
        /// <summary>
        /// Surface basis for the manifolds, defining both normal and tangents.
        /// </summary>
        public Vector3Wide ContactNormal;
        public Vector<float> Depth;
        /// <summary>
        /// The number of contacts in the manifolds.
        /// </summary>
        public Vector<int> Count;
        /// <summary>
        /// The maximum number of contacts that this pair type could ever generate.
        /// </summary>
        public int MaximumCount;
    }

}
