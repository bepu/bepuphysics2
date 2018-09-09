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
    public struct Contact3 : IConvexTwoBodyContactConstraintDescription<Contact3>
    {
        public ConstraintContactData Contact0;
        public ConstraintContactData Contact1;
        public ConstraintContactData Contact2;
        public Vector3 OffsetB;
        public float FrictionCoefficient;
        public Vector3 Normal;
        public SpringSettings SpringSettings;
        public float MaximumRecoveryVelocity;

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            Debug.Assert(batch.TypeId == ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<Contact3PrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            GetFirst(ref target.OffsetA0.X) = Contact0.OffsetA.X;
            GetFirst(ref target.OffsetA0.Y) = Contact0.OffsetA.Y;
            GetFirst(ref target.OffsetA0.Z) = Contact0.OffsetA.Z;
            GetFirst(ref target.OffsetA1.X) = Contact1.OffsetA.X;
            GetFirst(ref target.OffsetA1.Y) = Contact1.OffsetA.Y;
            GetFirst(ref target.OffsetA1.Z) = Contact1.OffsetA.Z;
            GetFirst(ref target.OffsetA2.X) = Contact2.OffsetA.X;
            GetFirst(ref target.OffsetA2.Y) = Contact2.OffsetA.Y;
            GetFirst(ref target.OffsetA2.Z) = Contact2.OffsetA.Z;

            GetFirst(ref target.OffsetB.X) = OffsetB.X;
            GetFirst(ref target.OffsetB.Y) = OffsetB.Y;
            GetFirst(ref target.OffsetB.Z) = OffsetB.Z;

            GetFirst(ref target.FrictionCoefficient) = FrictionCoefficient;

            GetFirst(ref target.Normal.X) = Normal.X;
            GetFirst(ref target.Normal.Y) = Normal.Y;
            GetFirst(ref target.Normal.Z) = Normal.Z;

            GetFirst(ref target.SpringSettings.AngularFrequency) = SpringSettings.AngularFrequency;
            GetFirst(ref target.SpringSettings.TwiceDampingRatio) = SpringSettings.TwiceDampingRatio;
            GetFirst(ref target.MaximumRecoveryVelocity) = MaximumRecoveryVelocity;

            GetFirst(ref target.PenetrationDepth0) = Contact0.PenetrationDepth;
            GetFirst(ref target.PenetrationDepth1) = Contact1.PenetrationDepth;
            GetFirst(ref target.PenetrationDepth2) = Contact2.PenetrationDepth;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact3 description)
        {
            Debug.Assert(batch.TypeId == ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<Contact3PrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            description.Contact0.OffsetA.X = GetFirst(ref source.OffsetA0.X);
            description.Contact0.OffsetA.Y = GetFirst(ref source.OffsetA0.Y);
            description.Contact0.OffsetA.Z = GetFirst(ref source.OffsetA0.Z);
            description.Contact1.OffsetA.X = GetFirst(ref source.OffsetA1.X);
            description.Contact1.OffsetA.Y = GetFirst(ref source.OffsetA1.Y);
            description.Contact1.OffsetA.Z = GetFirst(ref source.OffsetA1.Z);
            description.Contact2.OffsetA.X = GetFirst(ref source.OffsetA2.X);
            description.Contact2.OffsetA.Y = GetFirst(ref source.OffsetA2.Y);
            description.Contact2.OffsetA.Z = GetFirst(ref source.OffsetA2.Z);

            description.OffsetB.X = GetFirst(ref source.OffsetB.X);
            description.OffsetB.Y = GetFirst(ref source.OffsetB.Y);
            description.OffsetB.Z = GetFirst(ref source.OffsetB.Z);

            description.FrictionCoefficient = GetFirst(ref source.FrictionCoefficient);

            description.Normal.X = GetFirst(ref source.Normal.X);
            description.Normal.Y = GetFirst(ref source.Normal.Y);
            description.Normal.Z = GetFirst(ref source.Normal.Z);

            description.SpringSettings.AngularFrequency = GetFirst(ref source.SpringSettings.AngularFrequency);
            description.SpringSettings.TwiceDampingRatio = GetFirst(ref source.SpringSettings.TwiceDampingRatio);
            description.MaximumRecoveryVelocity = GetFirst(ref source.MaximumRecoveryVelocity);

            description.Contact0.PenetrationDepth = GetFirst(ref source.PenetrationDepth0);
            description.Contact1.PenetrationDepth = GetFirst(ref source.PenetrationDepth1);
            description.Contact2.PenetrationDepth = GetFirst(ref source.PenetrationDepth2);

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyManifoldWideProperties(ref Vector3 offsetB, ref Vector3 normal, ref PairMaterialProperties material)
        {
            OffsetB = offsetB;
            FrictionCoefficient = material.FrictionCoefficient;
            Normal = normal;
            SpringSettings = material.SpringSettings;
            MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
        }

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact3TypeProcessor.BatchTypeId;
        }

        public Type BatchType => typeof(Contact3TypeProcessor);
    }

    public struct Contact3PrestepData
    {
        //NOTE: Prestep data memory layout is relied upon by the constraint description for marginally more efficient setting and getting.
        //If you modify this layout, be sure to update the associated ContactManifold4Constraint.
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public Vector3Wide OffsetA0;
        public Vector3Wide OffsetA1;
        public Vector3Wide OffsetA2;
        public Vector3Wide OffsetB;
        public Vector<float> FrictionCoefficient;
        //In a convex manifold, all contacts share the same normal and tangents.
        public Vector3Wide Normal;
        //All contacts also share the spring settings.
        public SpringSettingsWide SpringSettings;
        public Vector<float> MaximumRecoveryVelocity;
        public Vector<float> PenetrationDepth0;
        public Vector<float> PenetrationDepth1;
        public Vector<float> PenetrationDepth2;
    }

    public struct Contact3AccumulatedImpulses
    {
        public Vector2Wide Tangent;
        public Vector<float> Penetration0;
        public Vector<float> Penetration1;
        public Vector<float> Penetration2;
        public Vector<float> Twist;
    }

    public struct Contact3Projection
    {
        public BodyInertias InertiaA;
        public BodyInertias InertiaB;
        public Vector<float> PremultipliedFrictionCoefficient;
        public Vector3Wide Normal;
        public TangentFriction.Projection Tangent;
        public PenetrationLimit3.Projection Penetration;
        //Lever arms aren't included in the twist projection because the number of arms required varies independently of the twist projection itself.
        public Vector<float> LeverArm0;
        public Vector<float> LeverArm1;
        public Vector<float> LeverArm2;
        public TwistFrictionProjection Twist;
    }

    public struct Contact3Functions :
        IConstraintFunctions<Contact3PrestepData, Contact3Projection, Contact3AccumulatedImpulses>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeFrictionCenter(in Vector3Wide offsetA0, in Vector3Wide offsetA1, in Vector3Wide offsetA2, 
            in Vector<float> depth0, in Vector<float> depth1, in Vector<float> depth2, out Vector3Wide center)
        {
            //This can sometimes cause a weird center of friction. That's a bit strange, but the alternative is often stranger:
            //Without this, if one contact is active and the other is speculative, friction will use the manifold center as halfway between the two points. If something is holding 
            //the inactive contact side up and swinging it around, the existence of speculative contacts would make friction work against the free swinging.
            var weight0 = Vector.ConditionalSelect(Vector.LessThan(depth0, Vector<float>.Zero), Vector<float>.Zero, Vector<float>.One);
            var weight1 = Vector.ConditionalSelect(Vector.LessThan(depth1, Vector<float>.Zero), Vector<float>.Zero, Vector<float>.One);
            var weight2 = Vector.ConditionalSelect(Vector.LessThan(depth2, Vector<float>.Zero), Vector<float>.Zero, Vector<float>.One);
            var weightSum = weight0 + weight1 + weight2;
            var useFallback = Vector.Equals(weightSum, Vector<float>.Zero);
            weightSum = Vector.ConditionalSelect(useFallback, new Vector<float>(3), weightSum);
            var inverseWeightSum = Vector<float>.One / weightSum;
            weight0 = Vector.ConditionalSelect(useFallback, inverseWeightSum, weight0 * inverseWeightSum);
            weight1 = Vector.ConditionalSelect(useFallback, inverseWeightSum, weight1 * inverseWeightSum);
            weight2 = Vector.ConditionalSelect(useFallback, inverseWeightSum, weight2 * inverseWeightSum);
            Vector3Wide.Scale(offsetA0, weight0, out var a0Contribution);
            Vector3Wide.Scale(offsetA1, weight1, out var a1Contribution);
            Vector3Wide.Scale(offsetA2, weight2, out var a2Contribution);
            Vector3Wide.Add(a0Contribution, a1Contribution, out center);
            Vector3Wide.Add(a2Contribution, center, out center);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref TwoBodyReferences bodyReferences, int count,
            float dt, float inverseDt, ref BodyInertias inertiaA, ref BodyInertias inertiaB, ref Contact3PrestepData prestep, out Contact3Projection projection)
        {
            projection.InertiaA = inertiaA;
            projection.InertiaB = inertiaB;
            ComputeFrictionCenter(prestep.OffsetA0, prestep.OffsetA1, prestep.OffsetA2, prestep.PenetrationDepth0, prestep.PenetrationDepth1, prestep.PenetrationDepth2, out var offsetToManifoldCenterA);
            Vector3Wide.Subtract(offsetToManifoldCenterA, prestep.OffsetB, out var offsetToManifoldCenterB);
            projection.PremultipliedFrictionCoefficient = (1f/3f) * prestep.FrictionCoefficient;
            projection.Normal = prestep.Normal;
            Helpers.BuildOrthnormalBasis(ref prestep.Normal, out var x, out var z);
            TangentFriction.Prestep(ref x, ref z, ref offsetToManifoldCenterA, ref offsetToManifoldCenterB, ref projection.InertiaA, ref projection.InertiaB, out projection.Tangent);
            PenetrationLimit3.Prestep(ref projection.InertiaA, ref projection.InertiaB, ref prestep.Normal, ref prestep, dt, inverseDt, out projection.Penetration);
            //Just assume the lever arms for B are the same. It's a good guess. (The only reason we computed the offset B is because we didn't want to go into world space.)
            Vector3Wide.Distance(prestep.OffsetA0, offsetToManifoldCenterA, out projection.LeverArm0);
            Vector3Wide.Distance(prestep.OffsetA1, offsetToManifoldCenterA, out projection.LeverArm1);
            Vector3Wide.Distance(prestep.OffsetA2, offsetToManifoldCenterA, out projection.LeverArm2);
            TwistFriction.Prestep(ref projection.InertiaA, ref projection.InertiaB, ref prestep.Normal, out projection.Twist);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocities wsvA, ref BodyVelocities wsvB, ref Contact3Projection projection, ref Contact3AccumulatedImpulses accumulatedImpulses)
        {
            Helpers.BuildOrthnormalBasis(ref projection.Normal, out var x, out var z);
            TangentFriction.WarmStart(ref x, ref z, ref projection.Tangent, ref projection.InertiaA, ref projection.InertiaB, ref accumulatedImpulses.Tangent, ref wsvA, ref wsvB);
            PenetrationLimit3.WarmStart(ref projection.Penetration, ref projection.InertiaA, ref projection.InertiaB,
                ref projection.Normal,
                ref accumulatedImpulses.Penetration0,
                ref accumulatedImpulses.Penetration1,
                ref accumulatedImpulses.Penetration2, ref wsvA, ref wsvB);
            TwistFriction.WarmStart(ref projection.Normal, ref projection.InertiaA, ref projection.InertiaB, ref accumulatedImpulses.Twist, ref wsvA, ref wsvB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocities wsvA, ref BodyVelocities wsvB, ref Contact3Projection projection, ref Contact3AccumulatedImpulses accumulatedImpulses)
        {
            Helpers.BuildOrthnormalBasis(ref projection.Normal, out var x, out var z);
            var maximumTangentImpulse = projection.PremultipliedFrictionCoefficient *
                (accumulatedImpulses.Penetration0 + accumulatedImpulses.Penetration1 + accumulatedImpulses.Penetration2);
            TangentFriction.Solve(ref x, ref z, ref projection.Tangent, ref projection.InertiaA, ref projection.InertiaB, ref maximumTangentImpulse, ref accumulatedImpulses.Tangent, ref wsvA, ref wsvB);
            //Note that we solve the penetration constraints after the friction constraints. 
            //This makes the penetration constraints more authoritative at the cost of the first iteration of the first frame of an impact lacking friction influence.
            //It's a pretty minor effect either way.
            PenetrationLimit3.Solve(ref projection.Penetration, ref projection.InertiaA, ref projection.InertiaB, ref projection.Normal,
                ref accumulatedImpulses.Penetration0,
                ref accumulatedImpulses.Penetration1,
                ref accumulatedImpulses.Penetration2, ref wsvA, ref wsvB);
            var maximumTwistImpulse = projection.PremultipliedFrictionCoefficient * (
                accumulatedImpulses.Penetration0 * projection.LeverArm0 +
                accumulatedImpulses.Penetration1 * projection.LeverArm1 +
                accumulatedImpulses.Penetration2 * projection.LeverArm2);
            TwistFriction.Solve(ref projection.Normal, ref projection.InertiaA, ref projection.InertiaB, ref projection.Twist, ref maximumTwistImpulse, ref accumulatedImpulses.Twist, ref wsvA, ref wsvB);
        }
    }

    /// <summary>
    /// Handles the solve iterations of a bunch of 3-contact convex manifold constraints.
    /// </summary>
    public class Contact3TypeProcessor :
        TwoBodyTypeProcessor<Contact3PrestepData, Contact3Projection, Contact3AccumulatedImpulses, Contact3Functions>
    {
        public const int BatchTypeId = 6;
    }
}
