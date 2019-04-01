using BepuPhysics.Collidables;
using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace BepuPhysics.CollisionDetection
{    public interface ISupportFinder<TShape, TShapeWide> where TShape : IConvexShape where TShapeWide : IShapeWide<TShape>
    {
        /// <summary>
        /// Gets whether the support finder is sampling a shape with a spherical margin that should be applied after a solution is found for the core shape.
        /// </summary>
        bool HasMargin { get; }
        /// <summary>
        /// Margin associated with the shape according to this support finder.
        /// </summary>
        /// <param name="shape">Shape to find the margin of.</param>
        /// <param name="margin">Margin of the shape.</param>
        void GetMargin(in TShapeWide shape, out Vector<float> margin);
        /// <summary>
        /// Gets whether the support finder maintains information about samples it has taken.
        /// </summary>
        bool HasCache { get; }
        /// <summary>
        /// For support finder types with a cache, informs the support finder that it should attempt to conditionally cache the last sample.
        /// </summary>
        /// <param name="shouldCache">Mask for lanes which should cache the latest sample because it was found to be an improvement by the outer algorithm.</param>
        void CacheLastSample(in Vector<int> shouldCache);
        void ComputeLocalSupport(in TShapeWide shape, in Vector3Wide direction, in Vector<int> terminatedLanes, out Vector3Wide support);
        void ComputeSupport(in TShapeWide shape, in Matrix3x3Wide orientation, in Vector3Wide direction, in Vector<int> terminatedLanes, out Vector3Wide support);
    }

}
