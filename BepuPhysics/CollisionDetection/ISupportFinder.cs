using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace BepuPhysics.CollisionDetection
{    public interface ISupportFinder<TShape, TShapeWide> where TShape : IConvexShape where TShapeWide : IShapeWide<TShape>
    {
        bool HasMargin { get; }
        void GetMargin(in TShapeWide shape, out Vector<float> margin);
        void ComputeLocalSupport(in TShapeWide shape, in Vector3Wide direction, in Vector<int> terminatedLanes, out Vector3Wide support);
        void ComputeSupport(in TShapeWide shape, in Matrix3x3Wide orientation, in Vector3Wide direction, in Vector<int> terminatedLanes, out Vector3Wide support);
    }

}
