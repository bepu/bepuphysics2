using System.Numerics;
using System.Runtime.CompilerServices;

namespace Demos.Demos.Cars
{
    struct RaceTrack
    {
        public float QuadrantRadius;
        public Vector2 Center;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetClosestPoint(in Vector2 point, float laneOffset, out Vector2 closestPoint, out Vector2 flowDirection)
        {
            var localPoint = point - Center;
            var quadrantCenter = new Vector2(localPoint.X < 0 ? -QuadrantRadius : QuadrantRadius, localPoint.Y < 0 ? -QuadrantRadius : QuadrantRadius);
            var quadrantCenterToPoint = new Vector2(localPoint.X, localPoint.Y) - quadrantCenter;
            var distanceToQuadrantCenter = quadrantCenterToPoint.Length();
            var on01Or10 = localPoint.X * localPoint.Y < 0;
            var signedLaneOffset = on01Or10 ? -laneOffset : laneOffset;
            var toCircleEdgeDirection = distanceToQuadrantCenter > 0 ? quadrantCenterToPoint * (1f / distanceToQuadrantCenter) : new Vector2(QuadrantRadius + signedLaneOffset, 0);
            var offsetFromQuadrantCircle = (QuadrantRadius + signedLaneOffset) * toCircleEdgeDirection;
            closestPoint = quadrantCenter + offsetFromQuadrantCircle;
            var perpendicular = new Vector2(toCircleEdgeDirection.Y, -toCircleEdgeDirection.X);
            flowDirection = on01Or10 ? perpendicular : -perpendicular;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float GetDistance(in Vector2 point)
        {
            GetClosestPoint(point, 0, out var closest, out _);
            return Vector2.Distance(closest, point);
        }
    }
}