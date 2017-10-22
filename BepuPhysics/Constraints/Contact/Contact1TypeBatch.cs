using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Constraints.Contact
{

    public struct ContactManifold1PrestepData
    {
        //NOTE: Prestep data memory layout is relied upon by the constraint description for marginally more efficient setting and getting.
        //If you modify this layout, be sure to update the associated ContactManifold1Constraint.
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public Vector3Wide OffsetA0;
        public Vector3Wide OffsetB;
        public Vector<float> FrictionCoefficient;
        //In a convex manifold, all contacts share the same normal and tangents.
        public Vector3Wide Normal;
        //All contacts also share the spring settings.
        public SpringSettingsWide SpringSettings;
        public Vector<float> MaximumRecoveryVelocity;
        public Vector<float> PenetrationDepth0;
    }

    public struct ContactManifold1AccumulatedImpulses
    {
        public Vector2Wide Tangent;
        public Vector<float> Penetration0;
    }

    public struct ContactManifold1Projection
    {
        public BodyInertias InertiaA;
        public BodyInertias InertiaB;
        public Vector<float> PremultipliedFrictionCoefficient;
        public Vector3Wide Normal;
        public TangentFrictionProjection Tangent;
        public PenetrationLimit1Projection Penetration;
    }

    public struct ContactManifold1Functions :
        IConstraintFunctions<ContactManifold1PrestepData, ContactManifold1Projection, ContactManifold1AccumulatedImpulses>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref TwoBodyReferences bodyReferences, int count,
            float dt, float inverseDt, ref ContactManifold1PrestepData prestep, out ContactManifold1Projection projection)
        {
            bodies.GatherInertia(ref bodyReferences, count, out projection.InertiaA, out projection.InertiaB);
            Vector3Wide.Subtract(ref prestep.OffsetA0, ref prestep.OffsetB, out var offsetToManifoldCenterB);
            projection.PremultipliedFrictionCoefficient = prestep.FrictionCoefficient;
            projection.Normal = prestep.Normal;
            Helpers.BuildOrthnormalBasis(ref prestep.Normal, out var x, out var z);
            TangentFriction.Prestep(ref x, ref z, ref prestep.OffsetA0, ref offsetToManifoldCenterB, ref projection.InertiaA, ref projection.InertiaB, out projection.Tangent);
            PenetrationLimit1.Prestep(ref projection.InertiaA, ref projection.InertiaB, ref prestep.Normal, ref prestep, dt, inverseDt, out projection.Penetration);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocities wsvA, ref BodyVelocities wsvB, ref ContactManifold1Projection projection, ref ContactManifold1AccumulatedImpulses accumulatedImpulses)
        {
            Helpers.BuildOrthnormalBasis(ref projection.Normal, out var x, out var z);
            TangentFriction.WarmStart(ref x, ref z, ref projection.Tangent, ref projection.InertiaA, ref projection.InertiaB, ref accumulatedImpulses.Tangent, ref wsvA, ref wsvB);
            PenetrationLimit1.WarmStart(ref projection.Penetration, ref projection.InertiaA, ref projection.InertiaB,
                ref projection.Normal,
                ref accumulatedImpulses.Penetration0, ref wsvA, ref wsvB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocities wsvA, ref BodyVelocities wsvB, ref ContactManifold1Projection projection, ref ContactManifold1AccumulatedImpulses accumulatedImpulses)
        {
            Helpers.BuildOrthnormalBasis(ref projection.Normal, out var x, out var z);
            var maximumTangentImpulse = projection.PremultipliedFrictionCoefficient * accumulatedImpulses.Penetration0;
            TangentFriction.Solve(ref x, ref z, ref projection.Tangent, ref projection.InertiaA, ref projection.InertiaB, ref maximumTangentImpulse, ref accumulatedImpulses.Tangent, ref wsvA, ref wsvB);
            //Note that we solve the penetration constraints after the friction constraints. 
            //This makes the penetration constraints more authoritative at the cost of the first iteration of the first frame of an impact lacking friction influence.
            //It's a pretty minor effect either way.
            PenetrationLimit1.Solve(ref projection.Penetration, ref projection.InertiaA, ref projection.InertiaB, ref projection.Normal,
                ref accumulatedImpulses.Penetration0, ref wsvA, ref wsvB);
        }

    }

    /// <summary>
    /// Handles the solve iterations of a bunch of 4-contact convex manifold constraints.
    /// </summary>
    public class Contact1TypeBatch :
        //UnposedTwoBodyTypeBatch<ContactManifold4PrestepData, ContactManifold4Projection, ContactManifold4AccumulatedImpulses, ContactManifold4>
        TwoBodyTypeBatch<ContactManifold1PrestepData, ContactManifold1Projection, ContactManifold1AccumulatedImpulses, ContactManifold1Functions>
    {
        public const int BatchTypeId = 8;
    }
}
