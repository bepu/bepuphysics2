using BepuPhysics.CollisionDetection;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using Quaternion = BepuUtilities.Quaternion;
using static BepuUtilities.GatherScatter;
using BepuUtilities;

namespace BepuPhysics.Constraints.Contact
{
    public struct Contact1OneBody : IConvexOneBodyContactConstraintDescription<Contact1OneBody>
    {
        public ConstraintContactData Contact0;
        public Vector3 Normal;
        public float FrictionCoefficient;
        public SpringSettings SpringSettings;
        public float MaximumRecoveryVelocity;


        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            Debug.Assert(batch.TypeId == ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<Contact1OneBodyPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            GetFirst(ref target.OffsetA0.X) = Contact0.OffsetA.X;
            GetFirst(ref target.OffsetA0.Y) = Contact0.OffsetA.Y;
            GetFirst(ref target.OffsetA0.Z) = Contact0.OffsetA.Z;

            GetFirst(ref target.FrictionCoefficient) = FrictionCoefficient;

            GetFirst(ref target.Normal.X) = Normal.X;
            GetFirst(ref target.Normal.Y) = Normal.Y;
            GetFirst(ref target.Normal.Z) = Normal.Z;

            GetFirst(ref target.SpringSettings.AngularFrequency) = SpringSettings.AngularFrequency;
            GetFirst(ref target.SpringSettings.TwiceDampingRatio) = SpringSettings.TwiceDampingRatio;
            GetFirst(ref target.MaximumRecoveryVelocity) = MaximumRecoveryVelocity;

            GetFirst(ref target.PenetrationDepth0) = Contact0.PenetrationDepth;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact1OneBody description)
        {
            Debug.Assert(batch.TypeId == ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<Contact1OneBodyPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            description.Contact0.OffsetA.X = GetFirst(ref source.OffsetA0.X);
            description.Contact0.OffsetA.Y = GetFirst(ref source.OffsetA0.Y);
            description.Contact0.OffsetA.Z = GetFirst(ref source.OffsetA0.Z);

            description.FrictionCoefficient = GetFirst(ref source.FrictionCoefficient);

            description.Normal.X = GetFirst(ref source.Normal.X);
            description.Normal.Y = GetFirst(ref source.Normal.Y);
            description.Normal.Z = GetFirst(ref source.Normal.Z);

            description.SpringSettings.AngularFrequency = GetFirst(ref source.SpringSettings.AngularFrequency);
            description.SpringSettings.TwiceDampingRatio = GetFirst(ref source.SpringSettings.TwiceDampingRatio);
            description.MaximumRecoveryVelocity = GetFirst(ref source.MaximumRecoveryVelocity);

            description.Contact0.PenetrationDepth = GetFirst(ref source.PenetrationDepth0);

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyManifoldWideProperties(ref Vector3 normal, ref PairMaterialProperties material)
        {
            FrictionCoefficient = material.FrictionCoefficient;
            Normal = normal;
            SpringSettings = material.SpringSettings;
            MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
        }

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact1OneBodyTypeProcessor.BatchTypeId;
        }

        public Type BatchType => typeof(Contact1OneBodyTypeProcessor);
    }

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
        public Vector<float> PremultipliedTwistFrictionCoefficient;
        public TwistFrictionProjection Twist;
    }

    public struct Contact1OneBodyFunctions :
        IOneBodyConstraintFunctions<Contact1OneBodyPrestepData, ContactManifold1OneBodyProjection, Contact1AccumulatedImpulses>
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
            PenetrationLimit1OneBody.Prestep(ref projection.InertiaA, ref prestep, dt, inverseDt, out projection.Penetration);
            //Single contact manifolds have no true surface area, so approximate twist friction lever arm using the penetration depth. 
            projection.PremultipliedTwistFrictionCoefficient = Vector.Max(Vector<float>.Zero, prestep.FrictionCoefficient * prestep.PenetrationDepth0);
            TwistFrictionOneBody.Prestep(ref projection.InertiaA, ref prestep.Normal, out projection.Twist);

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocities wsvA, ref ContactManifold1OneBodyProjection projection, ref Contact1AccumulatedImpulses accumulatedImpulses)
        {
            Helpers.BuildOrthnormalBasis(ref projection.Normal, out var x, out var z);
            TangentFrictionOneBody.WarmStart(ref x, ref z, ref projection.Tangent, ref projection.InertiaA, ref accumulatedImpulses.Tangent, ref wsvA);
            PenetrationLimit1OneBody.WarmStart(ref projection.Penetration, ref projection.InertiaA,
                ref projection.Normal,
                ref accumulatedImpulses.Penetration0, ref wsvA);
            TwistFrictionOneBody.WarmStart(ref projection.Normal, ref projection.InertiaA, ref accumulatedImpulses.Twist, ref wsvA);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocities wsvA, ref ContactManifold1OneBodyProjection projection, ref Contact1AccumulatedImpulses accumulatedImpulses)
        {
            Helpers.BuildOrthnormalBasis(ref projection.Normal, out var x, out var z);
            var maximumTangentImpulse = projection.PremultipliedFrictionCoefficient * accumulatedImpulses.Penetration0;
            TangentFrictionOneBody.Solve(ref x, ref z, ref projection.Tangent, ref projection.InertiaA, ref maximumTangentImpulse, ref accumulatedImpulses.Tangent, ref wsvA);
            //Note that we solve the penetration constraints after the friction constraints. 
            //This makes the penetration constraints more authoritative at the cost of the first iteration of the first frame of an impact lacking friction influence.
            //It's a pretty minor effect either way.
            PenetrationLimit1OneBody.Solve(ref projection.Penetration, ref projection.InertiaA, ref projection.Normal,
                ref accumulatedImpulses.Penetration0, ref wsvA);
            var maximumTwistImpulse = projection.PremultipliedTwistFrictionCoefficient * accumulatedImpulses.Penetration0;
            TwistFrictionOneBody.Solve(ref projection.Normal, ref projection.InertiaA, ref projection.Twist, ref maximumTwistImpulse, ref accumulatedImpulses.Twist, ref wsvA);
        }

    }

    /// <summary>
    /// Handles the solve iterations of a bunch of 4-contact convex manifold constraints.
    /// </summary>
    public class Contact1OneBodyTypeProcessor :
        OneBodyTypeProcessor<Contact1OneBodyPrestepData, ContactManifold1OneBodyProjection, Contact1AccumulatedImpulses, Contact1OneBodyFunctions>
    {
        public const int BatchTypeId = 0;
    }
}
