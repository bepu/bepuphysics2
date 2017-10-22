using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Constraints.Contact
{

    public struct Contact1OneBodyPrestepData
    {
        //NOTE: Prestep data memory layout is relied upon by the constraint description for marginally more efficient setting and getting.
        //If you modify this layout, be sure to update the associated ContactManifold1Constraint.
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public Vector3Wide OffsetA0;
        public Vector<float> FrictionCoefficient;
        //In a convex manifold, all contacts share the same normal and tangents.
        public Vector3Wide Normal;
        //All contacts also share the spring settings.
        public SpringSettingsWide SpringSettings;
        public Vector<float> MaximumRecoveryVelocity;
        public Vector<float> PenetrationDepth0;
    }
    
    public struct ContactManifold1OneBodyProjection
    {
        public BodyInertias InertiaA;
        public Vector<float> PremultipliedFrictionCoefficient;
        public Vector3Wide Normal;
        public TangentFrictionOneBody.Projection Tangent;
        public PenetrationLimit1OneBody.Projection Penetration;
    }

    public struct Contact1OneBodyFunctions :
        IOneBodyConstraintFunctions<Contact1OneBodyPrestepData, ContactManifold1OneBodyProjection, ContactManifold1AccumulatedImpulses>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref Vector<int> bodyReferences, int count,
            float dt, float inverseDt, ref Contact1OneBodyPrestepData prestep, out ContactManifold1OneBodyProjection projection)
        {
            bodies.GatherInertia(ref bodyReferences, count, out projection.InertiaA);
            projection.PremultipliedFrictionCoefficient = prestep.FrictionCoefficient;
            projection.Normal = prestep.Normal;
            Helpers.BuildOrthnormalBasis(ref prestep.Normal, out var x, out var z);
            TangentFrictionOneBody.Prestep(ref x, ref z, ref prestep.OffsetA0, ref projection.InertiaA, out projection.Tangent);
            PenetrationLimit1OneBody.Prestep(ref projection.InertiaA, ref prestep.Normal, ref prestep, dt, inverseDt, out projection.Penetration);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocities wsvA, ref ContactManifold1OneBodyProjection projection, ref ContactManifold1AccumulatedImpulses accumulatedImpulses)
        {
            Helpers.BuildOrthnormalBasis(ref projection.Normal, out var x, out var z);
            TangentFrictionOneBody.WarmStart(ref x, ref z, ref projection.Tangent, ref projection.InertiaA, ref accumulatedImpulses.Tangent, ref wsvA);
            PenetrationLimit1OneBody.WarmStart(ref projection.Penetration, ref projection.InertiaA,
                ref projection.Normal,
                ref accumulatedImpulses.Penetration0, ref wsvA);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocities wsvA, ref ContactManifold1OneBodyProjection projection, ref ContactManifold1AccumulatedImpulses accumulatedImpulses)
        {
            Helpers.BuildOrthnormalBasis(ref projection.Normal, out var x, out var z);
            var maximumTangentImpulse = projection.PremultipliedFrictionCoefficient * accumulatedImpulses.Penetration0;
            TangentFrictionOneBody.Solve(ref x, ref z, ref projection.Tangent, ref projection.InertiaA, ref maximumTangentImpulse, ref accumulatedImpulses.Tangent, ref wsvA);
            //Note that we solve the penetration constraints after the friction constraints. 
            //This makes the penetration constraints more authoritative at the cost of the first iteration of the first frame of an impact lacking friction influence.
            //It's a pretty minor effect either way.
            PenetrationLimit1OneBody.Solve(ref projection.Penetration, ref projection.InertiaA, ref projection.Normal,
                ref accumulatedImpulses.Penetration0, ref wsvA);
        }

    }

    /// <summary>
    /// Handles the solve iterations of a bunch of 4-contact convex manifold constraints.
    /// </summary>
    public class Contact1OneBodyTypeBatch :
        OneBodyTypeBatch<Contact1OneBodyPrestepData, ContactManifold1OneBodyProjection, ContactManifold1AccumulatedImpulses, Contact1OneBodyFunctions>
    {
        public const int BatchTypeId = 0;
    }
}
