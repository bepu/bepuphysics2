using BepuPhysics.CollisionDetection;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Diagnostics;
using BepuUtilities;
using BepuUtilities.Memory;
using static BepuUtilities.GatherScatter;
namespace BepuPhysics.Constraints.Contact
{  
    public struct Contact1AccumulatedImpulses : IConvexContactAccumulatedImpulses<Contact1AccumulatedImpulses>
    {
        public Vector2Wide Tangent;
        public Vector<float> Penetration0;
        public Vector<float> Twist;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector2Wide GetTangentFriction(ref Contact1AccumulatedImpulses impulses)
        {
            return ref impulses.Tangent;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector<float> GetTwistFriction(ref Contact1AccumulatedImpulses impulses)
        {
            return ref impulses.Twist;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector<float> GetPenetrationImpulseForContact(ref Contact1AccumulatedImpulses impulses, int index)
        {
            Debug.Assert(index >= 0 && index < 1);
            return ref Unsafe.Add(ref impulses.Penetration0, index);
        }
        public int ContactCount => 1;
    }

    public struct Contact2AccumulatedImpulses : IConvexContactAccumulatedImpulses<Contact2AccumulatedImpulses>
    {
        public Vector2Wide Tangent;
        public Vector<float> Penetration0;
        public Vector<float> Penetration1;
        public Vector<float> Twist;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector2Wide GetTangentFriction(ref Contact2AccumulatedImpulses impulses)
        {
            return ref impulses.Tangent;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector<float> GetTwistFriction(ref Contact2AccumulatedImpulses impulses)
        {
            return ref impulses.Twist;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector<float> GetPenetrationImpulseForContact(ref Contact2AccumulatedImpulses impulses, int index)
        {
            Debug.Assert(index >= 0 && index < 2);
            return ref Unsafe.Add(ref impulses.Penetration0, index);
        }
        public int ContactCount => 2;
    }

    public struct Contact3AccumulatedImpulses : IConvexContactAccumulatedImpulses<Contact3AccumulatedImpulses>
    {
        public Vector2Wide Tangent;
        public Vector<float> Penetration0;
        public Vector<float> Penetration1;
        public Vector<float> Penetration2;
        public Vector<float> Twist;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector2Wide GetTangentFriction(ref Contact3AccumulatedImpulses impulses)
        {
            return ref impulses.Tangent;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector<float> GetTwistFriction(ref Contact3AccumulatedImpulses impulses)
        {
            return ref impulses.Twist;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector<float> GetPenetrationImpulseForContact(ref Contact3AccumulatedImpulses impulses, int index)
        {
            Debug.Assert(index >= 0 && index < 3);
            return ref Unsafe.Add(ref impulses.Penetration0, index);
        }
        public int ContactCount => 3;
    }

    public struct Contact4AccumulatedImpulses : IConvexContactAccumulatedImpulses<Contact4AccumulatedImpulses>
    {
        public Vector2Wide Tangent;
        public Vector<float> Penetration0;
        public Vector<float> Penetration1;
        public Vector<float> Penetration2;
        public Vector<float> Penetration3;
        public Vector<float> Twist;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector2Wide GetTangentFriction(ref Contact4AccumulatedImpulses impulses)
        {
            return ref impulses.Tangent;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector<float> GetTwistFriction(ref Contact4AccumulatedImpulses impulses)
        {
            return ref impulses.Twist;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector<float> GetPenetrationImpulseForContact(ref Contact4AccumulatedImpulses impulses, int index)
        {
            Debug.Assert(index >= 0 && index < 4);
            return ref Unsafe.Add(ref impulses.Penetration0, index);
        }
        public int ContactCount => 4;
    }

    internal static class FrictionHelpers
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeFrictionCenter(
            in Vector3Wide offsetA0, in Vector3Wide offsetA1, 
            in Vector<float> depth0,in Vector<float> depth1, out Vector3Wide center)
        {
            //This can sometimes cause a weird center of friction. That's a bit strange, but the alternative is often stranger:
            //Without this, if one contact is active and the other is speculative, friction will use the manifold center as halfway between the two points. If something is holding 
            //the inactive contact side up and swinging it around, the existence of speculative contacts would make friction work against the free swinging.
            var weight0 = Vector.ConditionalSelect(Vector.LessThan(depth0, Vector<float>.Zero), Vector<float>.Zero, Vector<float>.One);
            var weight1 = Vector.ConditionalSelect(Vector.LessThan(depth1, Vector<float>.Zero), Vector<float>.Zero, Vector<float>.One);
            var weightSum = weight0 + weight1;
            var useFallback = Vector.Equals(weightSum, Vector<float>.Zero);
            weightSum = Vector.ConditionalSelect(useFallback, new Vector<float>(2), weightSum);
            var inverseWeightSum = Vector<float>.One / weightSum;
            weight0 = Vector.ConditionalSelect(useFallback, inverseWeightSum, weight0 * inverseWeightSum);
            weight1 = Vector.ConditionalSelect(useFallback, inverseWeightSum, weight1 * inverseWeightSum);
            Vector3Wide.Scale(offsetA0, weight0, out var a0Contribution);
            Vector3Wide.Scale(offsetA1, weight1, out var a1Contribution);
            Vector3Wide.Add(a0Contribution, a1Contribution, out center);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeFrictionCenter(
            in Vector3Wide offsetA0, in Vector3Wide offsetA1, in Vector3Wide offsetA2, 
            in Vector<float> depth0,in Vector<float> depth1,in Vector<float> depth2, out Vector3Wide center)
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
            Vector3Wide.Add(a0Contribution, a1Contribution, out var a0a1);
            Vector3Wide.Add(a0a1, a2Contribution, out center);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeFrictionCenter(
            in Vector3Wide offsetA0, in Vector3Wide offsetA1, in Vector3Wide offsetA2, in Vector3Wide offsetA3, 
            in Vector<float> depth0,in Vector<float> depth1,in Vector<float> depth2,in Vector<float> depth3, out Vector3Wide center)
        {
            //This can sometimes cause a weird center of friction. That's a bit strange, but the alternative is often stranger:
            //Without this, if one contact is active and the other is speculative, friction will use the manifold center as halfway between the two points. If something is holding 
            //the inactive contact side up and swinging it around, the existence of speculative contacts would make friction work against the free swinging.
            var weight0 = Vector.ConditionalSelect(Vector.LessThan(depth0, Vector<float>.Zero), Vector<float>.Zero, Vector<float>.One);
            var weight1 = Vector.ConditionalSelect(Vector.LessThan(depth1, Vector<float>.Zero), Vector<float>.Zero, Vector<float>.One);
            var weight2 = Vector.ConditionalSelect(Vector.LessThan(depth2, Vector<float>.Zero), Vector<float>.Zero, Vector<float>.One);
            var weight3 = Vector.ConditionalSelect(Vector.LessThan(depth3, Vector<float>.Zero), Vector<float>.Zero, Vector<float>.One);
            var weightSum = weight0 + weight1 + weight2 + weight3;
            var useFallback = Vector.Equals(weightSum, Vector<float>.Zero);
            weightSum = Vector.ConditionalSelect(useFallback, new Vector<float>(4), weightSum);
            var inverseWeightSum = Vector<float>.One / weightSum;
            weight0 = Vector.ConditionalSelect(useFallback, inverseWeightSum, weight0 * inverseWeightSum);
            weight1 = Vector.ConditionalSelect(useFallback, inverseWeightSum, weight1 * inverseWeightSum);
            weight2 = Vector.ConditionalSelect(useFallback, inverseWeightSum, weight2 * inverseWeightSum);
            weight3 = Vector.ConditionalSelect(useFallback, inverseWeightSum, weight3 * inverseWeightSum);
            Vector3Wide.Scale(offsetA0, weight0, out var a0Contribution);
            Vector3Wide.Scale(offsetA1, weight1, out var a1Contribution);
            Vector3Wide.Scale(offsetA2, weight2, out var a2Contribution);
            Vector3Wide.Scale(offsetA3, weight3, out var a3Contribution);
            Vector3Wide.Add(a0Contribution, a1Contribution, out var a0a1);
            Vector3Wide.Add(a2Contribution, a3Contribution, out var a2a3);
            Vector3Wide.Add(a0a1, a2a3, out center);
        }

    }
    public struct Contact1OneBody : IConvexOneBodyContactConstraintDescription<Contact1OneBody>
    {
        public ConstraintContactData Contact0;
        public Vector3 Normal;
        public float FrictionCoefficient;
        public SpringSettings SpringSettings;
        public float MaximumRecoveryVelocity;

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            Debug.Assert(batch.TypeId == ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<Contact1OneBodyPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(Contact0.OffsetA, ref target.Contact0.OffsetA);
            GetFirst(ref target.Contact0.Depth) = Contact0.PenetrationDepth;
            
            Vector3Wide.WriteFirst(Normal, ref target.Normal);
            GetFirst(ref target.MaterialProperties.FrictionCoefficient) = FrictionCoefficient;
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.MaterialProperties.SpringSettings);
            GetFirst(ref target.MaterialProperties.MaximumRecoveryVelocity) = MaximumRecoveryVelocity;
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact1OneBody description)
        {    
            Debug.Assert(batch.TypeId == ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<Contact1OneBodyPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.Contact0.OffsetA, out description.Contact0.OffsetA);
            description.Contact0.PenetrationDepth = GetFirst(ref source.Contact0.Depth);
            Vector3Wide.ReadFirst(source.Normal, out description.Normal);
            description.FrictionCoefficient = GetFirst(ref source.MaterialProperties.FrictionCoefficient);
            SpringSettingsWide.ReadFirst(source.MaterialProperties.SpringSettings, out description.SpringSettings);
            description.MaximumRecoveryVelocity = GetFirst(ref source.MaterialProperties.MaximumRecoveryVelocity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyManifoldWideProperties(ref Vector3 normal, ref PairMaterialProperties material)
        {
            FrictionCoefficient = material.FrictionCoefficient;
            Normal = normal;
            SpringSettings = material.SpringSettings;
            MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ConstraintContactData GetFirstContact(ref Contact1OneBody description)
        {
            return ref description.Contact0;
        }
        
        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact1OneBodyTypeProcessor.BatchTypeId;
        }

        public readonly Type TypeProcessorType => typeof(Contact1OneBodyTypeProcessor);

    }

    public struct Contact1OneBodyPrestepData : IConvexContactPrestep<Contact1OneBodyPrestepData>
    {
        //NOTE: Prestep data memory layout is relied upon by the constraint description for marginally more efficient setting and getting.
        //If you modify this layout, be sure to update the associated ContactManifold4Constraint.
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public ConvexContactWide Contact0;
        //In a convex manifold, all contacts share the same normal and tangents.
        public Vector3Wide Normal;
        public MaterialPropertiesWide MaterialProperties;
		
        public readonly int BodyCount => 1;
        public readonly int ContactCount => 1;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector3Wide GetNormal(ref Contact1OneBodyPrestepData prestep)
        {
            return ref prestep.Normal;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ConvexContactWide GetContact(ref Contact1OneBodyPrestepData prestep, int index)
        {
            return ref Unsafe.Add(ref prestep.Contact0, index);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref MaterialPropertiesWide GetMaterialProperties(ref Contact1OneBodyPrestepData prestep)
        {
            return ref prestep.MaterialProperties;
        }

    }


    public struct Contact1OneBodyFunctions : IOneBodyConstraintFunctions<Contact1OneBodyPrestepData, Contact1AccumulatedImpulses>
    {       
        public bool RequiresIncrementalSubstepUpdates => true;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide velocityA, ref Contact1OneBodyPrestepData prestep)
        {
            PenetrationLimitOneBody.UpdatePenetrationDepth(dt, prestep.Contact0.OffsetA, prestep.Normal, velocityA, ref prestep.Contact0.Depth);
        }
               
        [MethodImpl(MethodImplOptions.AggressiveInlining)] 
        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, ref Contact1OneBodyPrestepData prestep, ref Contact1AccumulatedImpulses accumulatedImpulses, ref BodyVelocityWide wsvA)
        {
            Helpers.BuildOrthonormalBasis(prestep.Normal, out var x, out var z);
            TangentFrictionOneBody.WarmStart(x, z, prestep.Contact0.OffsetA, inertiaA, accumulatedImpulses.Tangent, ref wsvA);
            PenetrationLimitOneBody.WarmStart(inertiaA, prestep.Normal, prestep.Contact0.OffsetA, accumulatedImpulses.Penetration0, ref wsvA);
            TwistFrictionOneBody.WarmStart(prestep.Normal, inertiaA, accumulatedImpulses.Twist, ref wsvA);
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, float dt, float inverseDt, ref Contact1OneBodyPrestepData prestep, ref Contact1AccumulatedImpulses accumulatedImpulses, ref BodyVelocityWide wsvA)
        {            
            //Note that we solve the penetration constraints before the friction constraints. 
            //This makes the friction constraints more authoritative, since they happen last.
            //It's a pretty minor effect either way, but penetration constraints have error correction feedback- penetration depth.
            //Friction is velocity only and has no error correction, so introducing error there might cause drift.
            SpringSettingsWide.ComputeSpringiness(prestep.MaterialProperties.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);
            var inverseDtWide = new Vector<float>(inverseDt);
            PenetrationLimitOneBody.Solve(inertiaA, prestep.Normal, prestep.Contact0.OffsetA, prestep.Contact0.Depth, positionErrorToVelocity, effectiveMassCFMScale, prestep.MaterialProperties.MaximumRecoveryVelocity, inverseDtWide, softnessImpulseScale, ref accumulatedImpulses.Penetration0, ref wsvA);
            Helpers.BuildOrthonormalBasis(prestep.Normal, out var x, out var z);
            var maximumTangentImpulse = prestep.MaterialProperties.FrictionCoefficient * (accumulatedImpulses.Penetration0);
            TangentFrictionOneBody.Solve(x, z, prestep.Contact0.OffsetA, inertiaA, maximumTangentImpulse, ref accumulatedImpulses.Tangent, ref wsvA);
            //If there's only one contact, then the contact patch as determined by contact distance would be zero.
            //That can cause some subtle behavioral issues sometimes, so we approximate lever arm with the contact depth, assuming that the contact surface area will increase as the depth increases.
            var maximumTwistImpulse = prestep.MaterialProperties.FrictionCoefficient * accumulatedImpulses.Penetration0 * Vector.Max(Vector<float>.Zero, prestep.Contact0.Depth);
            TwistFrictionOneBody.Solve(prestep.Normal, inertiaA, maximumTwistImpulse, ref accumulatedImpulses.Twist, ref wsvA);
        }               
    }
    
    /// <summary>
    /// Handles the solve iterations of a bunch of 1-contact one body manifold constraints.
    /// </summary>
    public class Contact1OneBodyTypeProcessor :
        OneBodyContactTypeProcessor<Contact1OneBodyPrestepData, Contact1AccumulatedImpulses, Contact1OneBodyFunctions>
    {
        //Matches UpdateConstraintForManifold's manifoldTypeAsConstraintType computation.
        public const int BatchTypeId = 0;
    }


    public struct Contact2OneBody : IConvexOneBodyContactConstraintDescription<Contact2OneBody>
    {
        public ConstraintContactData Contact0;
        public ConstraintContactData Contact1;
        public Vector3 Normal;
        public float FrictionCoefficient;
        public SpringSettings SpringSettings;
        public float MaximumRecoveryVelocity;

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            Debug.Assert(batch.TypeId == ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<Contact2OneBodyPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(Contact0.OffsetA, ref target.Contact0.OffsetA);
            GetFirst(ref target.Contact0.Depth) = Contact0.PenetrationDepth;
            Vector3Wide.WriteFirst(Contact1.OffsetA, ref target.Contact1.OffsetA);
            GetFirst(ref target.Contact1.Depth) = Contact1.PenetrationDepth;
            
            Vector3Wide.WriteFirst(Normal, ref target.Normal);
            GetFirst(ref target.MaterialProperties.FrictionCoefficient) = FrictionCoefficient;
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.MaterialProperties.SpringSettings);
            GetFirst(ref target.MaterialProperties.MaximumRecoveryVelocity) = MaximumRecoveryVelocity;
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact2OneBody description)
        {    
            Debug.Assert(batch.TypeId == ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<Contact2OneBodyPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.Contact0.OffsetA, out description.Contact0.OffsetA);
            description.Contact0.PenetrationDepth = GetFirst(ref source.Contact0.Depth);
            Vector3Wide.ReadFirst(source.Contact1.OffsetA, out description.Contact1.OffsetA);
            description.Contact1.PenetrationDepth = GetFirst(ref source.Contact1.Depth);
            Vector3Wide.ReadFirst(source.Normal, out description.Normal);
            description.FrictionCoefficient = GetFirst(ref source.MaterialProperties.FrictionCoefficient);
            SpringSettingsWide.ReadFirst(source.MaterialProperties.SpringSettings, out description.SpringSettings);
            description.MaximumRecoveryVelocity = GetFirst(ref source.MaterialProperties.MaximumRecoveryVelocity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyManifoldWideProperties(ref Vector3 normal, ref PairMaterialProperties material)
        {
            FrictionCoefficient = material.FrictionCoefficient;
            Normal = normal;
            SpringSettings = material.SpringSettings;
            MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ConstraintContactData GetFirstContact(ref Contact2OneBody description)
        {
            return ref description.Contact0;
        }
        
        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact2OneBodyTypeProcessor.BatchTypeId;
        }

        public readonly Type TypeProcessorType => typeof(Contact2OneBodyTypeProcessor);

    }

    public struct Contact2OneBodyPrestepData : IConvexContactPrestep<Contact2OneBodyPrestepData>
    {
        //NOTE: Prestep data memory layout is relied upon by the constraint description for marginally more efficient setting and getting.
        //If you modify this layout, be sure to update the associated ContactManifold4Constraint.
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public ConvexContactWide Contact0;
        public ConvexContactWide Contact1;
        //In a convex manifold, all contacts share the same normal and tangents.
        public Vector3Wide Normal;
        public MaterialPropertiesWide MaterialProperties;
		
        public readonly int BodyCount => 1;
        public readonly int ContactCount => 2;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector3Wide GetNormal(ref Contact2OneBodyPrestepData prestep)
        {
            return ref prestep.Normal;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ConvexContactWide GetContact(ref Contact2OneBodyPrestepData prestep, int index)
        {
            return ref Unsafe.Add(ref prestep.Contact0, index);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref MaterialPropertiesWide GetMaterialProperties(ref Contact2OneBodyPrestepData prestep)
        {
            return ref prestep.MaterialProperties;
        }

    }


    public struct Contact2OneBodyFunctions : IOneBodyConstraintFunctions<Contact2OneBodyPrestepData, Contact2AccumulatedImpulses>
    {       
        public bool RequiresIncrementalSubstepUpdates => true;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide velocityA, ref Contact2OneBodyPrestepData prestep)
        {
            PenetrationLimitOneBody.UpdatePenetrationDepth(dt, prestep.Contact0.OffsetA, prestep.Normal, velocityA, ref prestep.Contact0.Depth);
            PenetrationLimitOneBody.UpdatePenetrationDepth(dt, prestep.Contact1.OffsetA, prestep.Normal, velocityA, ref prestep.Contact1.Depth);
        }
               
        [MethodImpl(MethodImplOptions.AggressiveInlining)] 
        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, ref Contact2OneBodyPrestepData prestep, ref Contact2AccumulatedImpulses accumulatedImpulses, ref BodyVelocityWide wsvA)
        {
            Helpers.BuildOrthonormalBasis(prestep.Normal, out var x, out var z);
            FrictionHelpers.ComputeFrictionCenter(prestep.Contact0.OffsetA, prestep.Contact1.OffsetA, prestep.Contact0.Depth, prestep.Contact1.Depth, out var offsetToManifoldCenterA);
            TangentFrictionOneBody.WarmStart(x, z, offsetToManifoldCenterA, inertiaA, accumulatedImpulses.Tangent, ref wsvA);
            PenetrationLimitOneBody.WarmStart(inertiaA, prestep.Normal, prestep.Contact0.OffsetA, accumulatedImpulses.Penetration0, ref wsvA);
            PenetrationLimitOneBody.WarmStart(inertiaA, prestep.Normal, prestep.Contact1.OffsetA, accumulatedImpulses.Penetration1, ref wsvA);
            TwistFrictionOneBody.WarmStart(prestep.Normal, inertiaA, accumulatedImpulses.Twist, ref wsvA);
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, float dt, float inverseDt, ref Contact2OneBodyPrestepData prestep, ref Contact2AccumulatedImpulses accumulatedImpulses, ref BodyVelocityWide wsvA)
        {            
            //Note that we solve the penetration constraints before the friction constraints. 
            //This makes the friction constraints more authoritative, since they happen last.
            //It's a pretty minor effect either way, but penetration constraints have error correction feedback- penetration depth.
            //Friction is velocity only and has no error correction, so introducing error there might cause drift.
            SpringSettingsWide.ComputeSpringiness(prestep.MaterialProperties.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);
            var inverseDtWide = new Vector<float>(inverseDt);
            PenetrationLimitOneBody.Solve(inertiaA, prestep.Normal, prestep.Contact0.OffsetA, prestep.Contact0.Depth, positionErrorToVelocity, effectiveMassCFMScale, prestep.MaterialProperties.MaximumRecoveryVelocity, inverseDtWide, softnessImpulseScale, ref accumulatedImpulses.Penetration0, ref wsvA);
            PenetrationLimitOneBody.Solve(inertiaA, prestep.Normal, prestep.Contact1.OffsetA, prestep.Contact1.Depth, positionErrorToVelocity, effectiveMassCFMScale, prestep.MaterialProperties.MaximumRecoveryVelocity, inverseDtWide, softnessImpulseScale, ref accumulatedImpulses.Penetration1, ref wsvA);
            Helpers.BuildOrthonormalBasis(prestep.Normal, out var x, out var z);
            var premultipliedFrictionCoefficient = new Vector<float>(1f / 2f) * prestep.MaterialProperties.FrictionCoefficient;
            var maximumTangentImpulse = premultipliedFrictionCoefficient * (accumulatedImpulses.Penetration0 + accumulatedImpulses.Penetration1);
            FrictionHelpers.ComputeFrictionCenter(prestep.Contact0.OffsetA, prestep.Contact1.OffsetA, prestep.Contact0.Depth, prestep.Contact1.Depth, out var offsetToManifoldCenterA);
            TangentFrictionOneBody.Solve(x, z, offsetToManifoldCenterA, inertiaA, maximumTangentImpulse, ref accumulatedImpulses.Tangent, ref wsvA);
            var maximumTwistImpulse = premultipliedFrictionCoefficient * (
                accumulatedImpulses.Penetration0 * Vector3Wide.Distance(offsetToManifoldCenterA, prestep.Contact0.OffsetA) +
                accumulatedImpulses.Penetration1 * Vector3Wide.Distance(offsetToManifoldCenterA, prestep.Contact1.OffsetA));
            TwistFrictionOneBody.Solve(prestep.Normal, inertiaA, maximumTwistImpulse, ref accumulatedImpulses.Twist, ref wsvA);
        }               
    }
    
    /// <summary>
    /// Handles the solve iterations of a bunch of 2-contact one body manifold constraints.
    /// </summary>
    public class Contact2OneBodyTypeProcessor :
        OneBodyContactTypeProcessor<Contact2OneBodyPrestepData, Contact2AccumulatedImpulses, Contact2OneBodyFunctions>
    {
        //Matches UpdateConstraintForManifold's manifoldTypeAsConstraintType computation.
        public const int BatchTypeId = 1;
    }


    public struct Contact3OneBody : IConvexOneBodyContactConstraintDescription<Contact3OneBody>
    {
        public ConstraintContactData Contact0;
        public ConstraintContactData Contact1;
        public ConstraintContactData Contact2;
        public Vector3 Normal;
        public float FrictionCoefficient;
        public SpringSettings SpringSettings;
        public float MaximumRecoveryVelocity;

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            Debug.Assert(batch.TypeId == ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<Contact3OneBodyPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(Contact0.OffsetA, ref target.Contact0.OffsetA);
            GetFirst(ref target.Contact0.Depth) = Contact0.PenetrationDepth;
            Vector3Wide.WriteFirst(Contact1.OffsetA, ref target.Contact1.OffsetA);
            GetFirst(ref target.Contact1.Depth) = Contact1.PenetrationDepth;
            Vector3Wide.WriteFirst(Contact2.OffsetA, ref target.Contact2.OffsetA);
            GetFirst(ref target.Contact2.Depth) = Contact2.PenetrationDepth;
            
            Vector3Wide.WriteFirst(Normal, ref target.Normal);
            GetFirst(ref target.MaterialProperties.FrictionCoefficient) = FrictionCoefficient;
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.MaterialProperties.SpringSettings);
            GetFirst(ref target.MaterialProperties.MaximumRecoveryVelocity) = MaximumRecoveryVelocity;
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact3OneBody description)
        {    
            Debug.Assert(batch.TypeId == ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<Contact3OneBodyPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.Contact0.OffsetA, out description.Contact0.OffsetA);
            description.Contact0.PenetrationDepth = GetFirst(ref source.Contact0.Depth);
            Vector3Wide.ReadFirst(source.Contact1.OffsetA, out description.Contact1.OffsetA);
            description.Contact1.PenetrationDepth = GetFirst(ref source.Contact1.Depth);
            Vector3Wide.ReadFirst(source.Contact2.OffsetA, out description.Contact2.OffsetA);
            description.Contact2.PenetrationDepth = GetFirst(ref source.Contact2.Depth);
            Vector3Wide.ReadFirst(source.Normal, out description.Normal);
            description.FrictionCoefficient = GetFirst(ref source.MaterialProperties.FrictionCoefficient);
            SpringSettingsWide.ReadFirst(source.MaterialProperties.SpringSettings, out description.SpringSettings);
            description.MaximumRecoveryVelocity = GetFirst(ref source.MaterialProperties.MaximumRecoveryVelocity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyManifoldWideProperties(ref Vector3 normal, ref PairMaterialProperties material)
        {
            FrictionCoefficient = material.FrictionCoefficient;
            Normal = normal;
            SpringSettings = material.SpringSettings;
            MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ConstraintContactData GetFirstContact(ref Contact3OneBody description)
        {
            return ref description.Contact0;
        }
        
        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact3OneBodyTypeProcessor.BatchTypeId;
        }

        public readonly Type TypeProcessorType => typeof(Contact3OneBodyTypeProcessor);

    }

    public struct Contact3OneBodyPrestepData : IConvexContactPrestep<Contact3OneBodyPrestepData>
    {
        //NOTE: Prestep data memory layout is relied upon by the constraint description for marginally more efficient setting and getting.
        //If you modify this layout, be sure to update the associated ContactManifold4Constraint.
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public ConvexContactWide Contact0;
        public ConvexContactWide Contact1;
        public ConvexContactWide Contact2;
        //In a convex manifold, all contacts share the same normal and tangents.
        public Vector3Wide Normal;
        public MaterialPropertiesWide MaterialProperties;
		
        public readonly int BodyCount => 1;
        public readonly int ContactCount => 3;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector3Wide GetNormal(ref Contact3OneBodyPrestepData prestep)
        {
            return ref prestep.Normal;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ConvexContactWide GetContact(ref Contact3OneBodyPrestepData prestep, int index)
        {
            return ref Unsafe.Add(ref prestep.Contact0, index);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref MaterialPropertiesWide GetMaterialProperties(ref Contact3OneBodyPrestepData prestep)
        {
            return ref prestep.MaterialProperties;
        }

    }


    public struct Contact3OneBodyFunctions : IOneBodyConstraintFunctions<Contact3OneBodyPrestepData, Contact3AccumulatedImpulses>
    {       
        public bool RequiresIncrementalSubstepUpdates => true;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide velocityA, ref Contact3OneBodyPrestepData prestep)
        {
            PenetrationLimitOneBody.UpdatePenetrationDepth(dt, prestep.Contact0.OffsetA, prestep.Normal, velocityA, ref prestep.Contact0.Depth);
            PenetrationLimitOneBody.UpdatePenetrationDepth(dt, prestep.Contact1.OffsetA, prestep.Normal, velocityA, ref prestep.Contact1.Depth);
            PenetrationLimitOneBody.UpdatePenetrationDepth(dt, prestep.Contact2.OffsetA, prestep.Normal, velocityA, ref prestep.Contact2.Depth);
        }
               
        [MethodImpl(MethodImplOptions.AggressiveInlining)] 
        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, ref Contact3OneBodyPrestepData prestep, ref Contact3AccumulatedImpulses accumulatedImpulses, ref BodyVelocityWide wsvA)
        {
            Helpers.BuildOrthonormalBasis(prestep.Normal, out var x, out var z);
            FrictionHelpers.ComputeFrictionCenter(prestep.Contact0.OffsetA, prestep.Contact1.OffsetA, prestep.Contact2.OffsetA, prestep.Contact0.Depth, prestep.Contact1.Depth, prestep.Contact2.Depth, out var offsetToManifoldCenterA);
            TangentFrictionOneBody.WarmStart(x, z, offsetToManifoldCenterA, inertiaA, accumulatedImpulses.Tangent, ref wsvA);
            PenetrationLimitOneBody.WarmStart(inertiaA, prestep.Normal, prestep.Contact0.OffsetA, accumulatedImpulses.Penetration0, ref wsvA);
            PenetrationLimitOneBody.WarmStart(inertiaA, prestep.Normal, prestep.Contact1.OffsetA, accumulatedImpulses.Penetration1, ref wsvA);
            PenetrationLimitOneBody.WarmStart(inertiaA, prestep.Normal, prestep.Contact2.OffsetA, accumulatedImpulses.Penetration2, ref wsvA);
            TwistFrictionOneBody.WarmStart(prestep.Normal, inertiaA, accumulatedImpulses.Twist, ref wsvA);
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, float dt, float inverseDt, ref Contact3OneBodyPrestepData prestep, ref Contact3AccumulatedImpulses accumulatedImpulses, ref BodyVelocityWide wsvA)
        {            
            //Note that we solve the penetration constraints before the friction constraints. 
            //This makes the friction constraints more authoritative, since they happen last.
            //It's a pretty minor effect either way, but penetration constraints have error correction feedback- penetration depth.
            //Friction is velocity only and has no error correction, so introducing error there might cause drift.
            SpringSettingsWide.ComputeSpringiness(prestep.MaterialProperties.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);
            var inverseDtWide = new Vector<float>(inverseDt);
            PenetrationLimitOneBody.Solve(inertiaA, prestep.Normal, prestep.Contact0.OffsetA, prestep.Contact0.Depth, positionErrorToVelocity, effectiveMassCFMScale, prestep.MaterialProperties.MaximumRecoveryVelocity, inverseDtWide, softnessImpulseScale, ref accumulatedImpulses.Penetration0, ref wsvA);
            PenetrationLimitOneBody.Solve(inertiaA, prestep.Normal, prestep.Contact1.OffsetA, prestep.Contact1.Depth, positionErrorToVelocity, effectiveMassCFMScale, prestep.MaterialProperties.MaximumRecoveryVelocity, inverseDtWide, softnessImpulseScale, ref accumulatedImpulses.Penetration1, ref wsvA);
            PenetrationLimitOneBody.Solve(inertiaA, prestep.Normal, prestep.Contact2.OffsetA, prestep.Contact2.Depth, positionErrorToVelocity, effectiveMassCFMScale, prestep.MaterialProperties.MaximumRecoveryVelocity, inverseDtWide, softnessImpulseScale, ref accumulatedImpulses.Penetration2, ref wsvA);
            Helpers.BuildOrthonormalBasis(prestep.Normal, out var x, out var z);
            var premultipliedFrictionCoefficient = new Vector<float>(1f / 3f) * prestep.MaterialProperties.FrictionCoefficient;
            var maximumTangentImpulse = premultipliedFrictionCoefficient * (accumulatedImpulses.Penetration0 + accumulatedImpulses.Penetration1 + accumulatedImpulses.Penetration2);
            FrictionHelpers.ComputeFrictionCenter(prestep.Contact0.OffsetA, prestep.Contact1.OffsetA, prestep.Contact2.OffsetA, prestep.Contact0.Depth, prestep.Contact1.Depth, prestep.Contact2.Depth, out var offsetToManifoldCenterA);
            TangentFrictionOneBody.Solve(x, z, offsetToManifoldCenterA, inertiaA, maximumTangentImpulse, ref accumulatedImpulses.Tangent, ref wsvA);
            var maximumTwistImpulse = premultipliedFrictionCoefficient * (
                accumulatedImpulses.Penetration0 * Vector3Wide.Distance(offsetToManifoldCenterA, prestep.Contact0.OffsetA) +
                accumulatedImpulses.Penetration1 * Vector3Wide.Distance(offsetToManifoldCenterA, prestep.Contact1.OffsetA) +
                accumulatedImpulses.Penetration2 * Vector3Wide.Distance(offsetToManifoldCenterA, prestep.Contact2.OffsetA));
            TwistFrictionOneBody.Solve(prestep.Normal, inertiaA, maximumTwistImpulse, ref accumulatedImpulses.Twist, ref wsvA);
        }               
    }
    
    /// <summary>
    /// Handles the solve iterations of a bunch of 3-contact one body manifold constraints.
    /// </summary>
    public class Contact3OneBodyTypeProcessor :
        OneBodyContactTypeProcessor<Contact3OneBodyPrestepData, Contact3AccumulatedImpulses, Contact3OneBodyFunctions>
    {
        //Matches UpdateConstraintForManifold's manifoldTypeAsConstraintType computation.
        public const int BatchTypeId = 2;
    }


    public struct Contact4OneBody : IConvexOneBodyContactConstraintDescription<Contact4OneBody>
    {
        public ConstraintContactData Contact0;
        public ConstraintContactData Contact1;
        public ConstraintContactData Contact2;
        public ConstraintContactData Contact3;
        public Vector3 Normal;
        public float FrictionCoefficient;
        public SpringSettings SpringSettings;
        public float MaximumRecoveryVelocity;

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            Debug.Assert(batch.TypeId == ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<Contact4OneBodyPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(Contact0.OffsetA, ref target.Contact0.OffsetA);
            GetFirst(ref target.Contact0.Depth) = Contact0.PenetrationDepth;
            Vector3Wide.WriteFirst(Contact1.OffsetA, ref target.Contact1.OffsetA);
            GetFirst(ref target.Contact1.Depth) = Contact1.PenetrationDepth;
            Vector3Wide.WriteFirst(Contact2.OffsetA, ref target.Contact2.OffsetA);
            GetFirst(ref target.Contact2.Depth) = Contact2.PenetrationDepth;
            Vector3Wide.WriteFirst(Contact3.OffsetA, ref target.Contact3.OffsetA);
            GetFirst(ref target.Contact3.Depth) = Contact3.PenetrationDepth;
            
            Vector3Wide.WriteFirst(Normal, ref target.Normal);
            GetFirst(ref target.MaterialProperties.FrictionCoefficient) = FrictionCoefficient;
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.MaterialProperties.SpringSettings);
            GetFirst(ref target.MaterialProperties.MaximumRecoveryVelocity) = MaximumRecoveryVelocity;
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact4OneBody description)
        {    
            Debug.Assert(batch.TypeId == ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<Contact4OneBodyPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.Contact0.OffsetA, out description.Contact0.OffsetA);
            description.Contact0.PenetrationDepth = GetFirst(ref source.Contact0.Depth);
            Vector3Wide.ReadFirst(source.Contact1.OffsetA, out description.Contact1.OffsetA);
            description.Contact1.PenetrationDepth = GetFirst(ref source.Contact1.Depth);
            Vector3Wide.ReadFirst(source.Contact2.OffsetA, out description.Contact2.OffsetA);
            description.Contact2.PenetrationDepth = GetFirst(ref source.Contact2.Depth);
            Vector3Wide.ReadFirst(source.Contact3.OffsetA, out description.Contact3.OffsetA);
            description.Contact3.PenetrationDepth = GetFirst(ref source.Contact3.Depth);
            Vector3Wide.ReadFirst(source.Normal, out description.Normal);
            description.FrictionCoefficient = GetFirst(ref source.MaterialProperties.FrictionCoefficient);
            SpringSettingsWide.ReadFirst(source.MaterialProperties.SpringSettings, out description.SpringSettings);
            description.MaximumRecoveryVelocity = GetFirst(ref source.MaterialProperties.MaximumRecoveryVelocity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyManifoldWideProperties(ref Vector3 normal, ref PairMaterialProperties material)
        {
            FrictionCoefficient = material.FrictionCoefficient;
            Normal = normal;
            SpringSettings = material.SpringSettings;
            MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ConstraintContactData GetFirstContact(ref Contact4OneBody description)
        {
            return ref description.Contact0;
        }
        
        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact4OneBodyTypeProcessor.BatchTypeId;
        }

        public readonly Type TypeProcessorType => typeof(Contact4OneBodyTypeProcessor);

    }

    public struct Contact4OneBodyPrestepData : IConvexContactPrestep<Contact4OneBodyPrestepData>
    {
        //NOTE: Prestep data memory layout is relied upon by the constraint description for marginally more efficient setting and getting.
        //If you modify this layout, be sure to update the associated ContactManifold4Constraint.
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public ConvexContactWide Contact0;
        public ConvexContactWide Contact1;
        public ConvexContactWide Contact2;
        public ConvexContactWide Contact3;
        //In a convex manifold, all contacts share the same normal and tangents.
        public Vector3Wide Normal;
        public MaterialPropertiesWide MaterialProperties;
		
        public readonly int BodyCount => 1;
        public readonly int ContactCount => 4;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector3Wide GetNormal(ref Contact4OneBodyPrestepData prestep)
        {
            return ref prestep.Normal;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ConvexContactWide GetContact(ref Contact4OneBodyPrestepData prestep, int index)
        {
            return ref Unsafe.Add(ref prestep.Contact0, index);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref MaterialPropertiesWide GetMaterialProperties(ref Contact4OneBodyPrestepData prestep)
        {
            return ref prestep.MaterialProperties;
        }

    }


    public struct Contact4OneBodyFunctions : IOneBodyConstraintFunctions<Contact4OneBodyPrestepData, Contact4AccumulatedImpulses>
    {       
        public bool RequiresIncrementalSubstepUpdates => true;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide velocityA, ref Contact4OneBodyPrestepData prestep)
        {
            PenetrationLimitOneBody.UpdatePenetrationDepth(dt, prestep.Contact0.OffsetA, prestep.Normal, velocityA, ref prestep.Contact0.Depth);
            PenetrationLimitOneBody.UpdatePenetrationDepth(dt, prestep.Contact1.OffsetA, prestep.Normal, velocityA, ref prestep.Contact1.Depth);
            PenetrationLimitOneBody.UpdatePenetrationDepth(dt, prestep.Contact2.OffsetA, prestep.Normal, velocityA, ref prestep.Contact2.Depth);
            PenetrationLimitOneBody.UpdatePenetrationDepth(dt, prestep.Contact3.OffsetA, prestep.Normal, velocityA, ref prestep.Contact3.Depth);
        }
               
        [MethodImpl(MethodImplOptions.AggressiveInlining)] 
        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, ref Contact4OneBodyPrestepData prestep, ref Contact4AccumulatedImpulses accumulatedImpulses, ref BodyVelocityWide wsvA)
        {
            Helpers.BuildOrthonormalBasis(prestep.Normal, out var x, out var z);
            FrictionHelpers.ComputeFrictionCenter(prestep.Contact0.OffsetA, prestep.Contact1.OffsetA, prestep.Contact2.OffsetA, prestep.Contact3.OffsetA, prestep.Contact0.Depth, prestep.Contact1.Depth, prestep.Contact2.Depth, prestep.Contact3.Depth, out var offsetToManifoldCenterA);
            TangentFrictionOneBody.WarmStart(x, z, offsetToManifoldCenterA, inertiaA, accumulatedImpulses.Tangent, ref wsvA);
            PenetrationLimitOneBody.WarmStart(inertiaA, prestep.Normal, prestep.Contact0.OffsetA, accumulatedImpulses.Penetration0, ref wsvA);
            PenetrationLimitOneBody.WarmStart(inertiaA, prestep.Normal, prestep.Contact1.OffsetA, accumulatedImpulses.Penetration1, ref wsvA);
            PenetrationLimitOneBody.WarmStart(inertiaA, prestep.Normal, prestep.Contact2.OffsetA, accumulatedImpulses.Penetration2, ref wsvA);
            PenetrationLimitOneBody.WarmStart(inertiaA, prestep.Normal, prestep.Contact3.OffsetA, accumulatedImpulses.Penetration3, ref wsvA);
            TwistFrictionOneBody.WarmStart(prestep.Normal, inertiaA, accumulatedImpulses.Twist, ref wsvA);
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, float dt, float inverseDt, ref Contact4OneBodyPrestepData prestep, ref Contact4AccumulatedImpulses accumulatedImpulses, ref BodyVelocityWide wsvA)
        {            
            //Note that we solve the penetration constraints before the friction constraints. 
            //This makes the friction constraints more authoritative, since they happen last.
            //It's a pretty minor effect either way, but penetration constraints have error correction feedback- penetration depth.
            //Friction is velocity only and has no error correction, so introducing error there might cause drift.
            SpringSettingsWide.ComputeSpringiness(prestep.MaterialProperties.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);
            var inverseDtWide = new Vector<float>(inverseDt);
            PenetrationLimitOneBody.Solve(inertiaA, prestep.Normal, prestep.Contact0.OffsetA, prestep.Contact0.Depth, positionErrorToVelocity, effectiveMassCFMScale, prestep.MaterialProperties.MaximumRecoveryVelocity, inverseDtWide, softnessImpulseScale, ref accumulatedImpulses.Penetration0, ref wsvA);
            PenetrationLimitOneBody.Solve(inertiaA, prestep.Normal, prestep.Contact1.OffsetA, prestep.Contact1.Depth, positionErrorToVelocity, effectiveMassCFMScale, prestep.MaterialProperties.MaximumRecoveryVelocity, inverseDtWide, softnessImpulseScale, ref accumulatedImpulses.Penetration1, ref wsvA);
            PenetrationLimitOneBody.Solve(inertiaA, prestep.Normal, prestep.Contact2.OffsetA, prestep.Contact2.Depth, positionErrorToVelocity, effectiveMassCFMScale, prestep.MaterialProperties.MaximumRecoveryVelocity, inverseDtWide, softnessImpulseScale, ref accumulatedImpulses.Penetration2, ref wsvA);
            PenetrationLimitOneBody.Solve(inertiaA, prestep.Normal, prestep.Contact3.OffsetA, prestep.Contact3.Depth, positionErrorToVelocity, effectiveMassCFMScale, prestep.MaterialProperties.MaximumRecoveryVelocity, inverseDtWide, softnessImpulseScale, ref accumulatedImpulses.Penetration3, ref wsvA);
            Helpers.BuildOrthonormalBasis(prestep.Normal, out var x, out var z);
            var premultipliedFrictionCoefficient = new Vector<float>(1f / 4f) * prestep.MaterialProperties.FrictionCoefficient;
            var maximumTangentImpulse = premultipliedFrictionCoefficient * (accumulatedImpulses.Penetration0 + accumulatedImpulses.Penetration1 + accumulatedImpulses.Penetration2 + accumulatedImpulses.Penetration3);
            FrictionHelpers.ComputeFrictionCenter(prestep.Contact0.OffsetA, prestep.Contact1.OffsetA, prestep.Contact2.OffsetA, prestep.Contact3.OffsetA, prestep.Contact0.Depth, prestep.Contact1.Depth, prestep.Contact2.Depth, prestep.Contact3.Depth, out var offsetToManifoldCenterA);
            TangentFrictionOneBody.Solve(x, z, offsetToManifoldCenterA, inertiaA, maximumTangentImpulse, ref accumulatedImpulses.Tangent, ref wsvA);
            var maximumTwistImpulse = premultipliedFrictionCoefficient * (
                accumulatedImpulses.Penetration0 * Vector3Wide.Distance(offsetToManifoldCenterA, prestep.Contact0.OffsetA) +
                accumulatedImpulses.Penetration1 * Vector3Wide.Distance(offsetToManifoldCenterA, prestep.Contact1.OffsetA) +
                accumulatedImpulses.Penetration2 * Vector3Wide.Distance(offsetToManifoldCenterA, prestep.Contact2.OffsetA) +
                accumulatedImpulses.Penetration3 * Vector3Wide.Distance(offsetToManifoldCenterA, prestep.Contact3.OffsetA));
            TwistFrictionOneBody.Solve(prestep.Normal, inertiaA, maximumTwistImpulse, ref accumulatedImpulses.Twist, ref wsvA);
        }               
    }
    
    /// <summary>
    /// Handles the solve iterations of a bunch of 4-contact one body manifold constraints.
    /// </summary>
    public class Contact4OneBodyTypeProcessor :
        OneBodyContactTypeProcessor<Contact4OneBodyPrestepData, Contact4AccumulatedImpulses, Contact4OneBodyFunctions>
    {
        //Matches UpdateConstraintForManifold's manifoldTypeAsConstraintType computation.
        public const int BatchTypeId = 3;
    }


    public struct Contact1 : IConvexTwoBodyContactConstraintDescription<Contact1>
    {
        public ConstraintContactData Contact0;
        public Vector3 OffsetB;
        public Vector3 Normal;
        public float FrictionCoefficient;
        public SpringSettings SpringSettings;
        public float MaximumRecoveryVelocity;

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            Debug.Assert(batch.TypeId == ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<Contact1PrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(Contact0.OffsetA, ref target.Contact0.OffsetA);
            GetFirst(ref target.Contact0.Depth) = Contact0.PenetrationDepth;
            
            Vector3Wide.WriteFirst(OffsetB, ref target.OffsetB);
            Vector3Wide.WriteFirst(Normal, ref target.Normal);
            GetFirst(ref target.MaterialProperties.FrictionCoefficient) = FrictionCoefficient;
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.MaterialProperties.SpringSettings);
            GetFirst(ref target.MaterialProperties.MaximumRecoveryVelocity) = MaximumRecoveryVelocity;
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact1 description)
        {    
            Debug.Assert(batch.TypeId == ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<Contact1PrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.Contact0.OffsetA, out description.Contact0.OffsetA);
            description.Contact0.PenetrationDepth = GetFirst(ref source.Contact0.Depth);
            
            Vector3Wide.ReadFirst(source.OffsetB, out description.OffsetB);
            Vector3Wide.ReadFirst(source.Normal, out description.Normal);
            description.FrictionCoefficient = GetFirst(ref source.MaterialProperties.FrictionCoefficient);
            SpringSettingsWide.ReadFirst(source.MaterialProperties.SpringSettings, out description.SpringSettings);
            description.MaximumRecoveryVelocity = GetFirst(ref source.MaterialProperties.MaximumRecoveryVelocity);
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ConstraintContactData GetFirstContact(ref Contact1 description)
        {
            return ref description.Contact0;
        }
        
        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact1TypeProcessor.BatchTypeId;
        }

        public readonly Type TypeProcessorType => typeof(Contact1TypeProcessor);

    }

    public struct Contact1PrestepData : ITwoBodyConvexContactPrestep<Contact1PrestepData>
    {
        //NOTE: Prestep data memory layout is relied upon by the constraint description for marginally more efficient setting and getting.
        //If you modify this layout, be sure to update the associated ContactManifold4Constraint.
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public ConvexContactWide Contact0;
        public Vector3Wide OffsetB;
        //In a convex manifold, all contacts share the same normal and tangents.
        public Vector3Wide Normal;
        public MaterialPropertiesWide MaterialProperties;
		
        public readonly int BodyCount => 2;
        public readonly int ContactCount => 1;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector3Wide GetNormal(ref Contact1PrestepData prestep)
        {
            return ref prestep.Normal;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ConvexContactWide GetContact(ref Contact1PrestepData prestep, int index)
        {
            return ref Unsafe.Add(ref prestep.Contact0, index);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref MaterialPropertiesWide GetMaterialProperties(ref Contact1PrestepData prestep)
        {
            return ref prestep.MaterialProperties;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector3Wide GetOffsetB(ref Contact1PrestepData prestep)
        {
            return ref prestep.OffsetB;
        }
    }


    public struct Contact1Functions : ITwoBodyConstraintFunctions<Contact1PrestepData, Contact1AccumulatedImpulses>
    {       
        public bool RequiresIncrementalSubstepUpdates => true;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide velocityA, in BodyVelocityWide velocityB, ref Contact1PrestepData prestep)
        {
            PenetrationLimit.UpdatePenetrationDepth(dt, prestep.Contact0.OffsetA, prestep.OffsetB, prestep.Normal, velocityA, velocityB, ref prestep.Contact0.Depth);
        }
               
        [MethodImpl(MethodImplOptions.AggressiveInlining)] 
        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, ref Contact1PrestepData prestep, ref Contact1AccumulatedImpulses accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            Helpers.BuildOrthonormalBasis(prestep.Normal, out var x, out var z);
            Vector3Wide.Subtract(prestep.Contact0.OffsetA, prestep.OffsetB, out var offsetToManifoldCenterB);
            TangentFriction.WarmStart(x, z, prestep.Contact0.OffsetA, offsetToManifoldCenterB, inertiaA, inertiaB, accumulatedImpulses.Tangent, ref wsvA, ref wsvB);
            PenetrationLimit.WarmStart(inertiaA, inertiaB, prestep.Normal, prestep.Contact0.OffsetA, prestep.Contact0.OffsetA - prestep.OffsetB, accumulatedImpulses.Penetration0, ref wsvA, ref wsvB);
            TwistFriction.WarmStart(prestep.Normal, inertiaA, inertiaB, accumulatedImpulses.Twist, ref wsvA, ref wsvB);
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, ref Contact1PrestepData prestep, ref Contact1AccumulatedImpulses accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {            
            //Note that we solve the penetration constraints before the friction constraints. 
            //This makes the friction constraints more authoritative, since they happen last.
            //It's a pretty minor effect either way, but penetration constraints have error correction feedback- penetration depth.
            //Friction is velocity only and has no error correction, so introducing error there might cause drift.
            SpringSettingsWide.ComputeSpringiness(prestep.MaterialProperties.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);
            var inverseDtWide = new Vector<float>(inverseDt);
            PenetrationLimit.Solve(inertiaA, inertiaB, prestep.Normal, prestep.Contact0.OffsetA, prestep.Contact0.OffsetA - prestep.OffsetB, prestep.Contact0.Depth, positionErrorToVelocity, effectiveMassCFMScale, prestep.MaterialProperties.MaximumRecoveryVelocity, inverseDtWide, softnessImpulseScale, ref accumulatedImpulses.Penetration0, ref wsvA, ref wsvB);
            Helpers.BuildOrthonormalBasis(prestep.Normal, out var x, out var z);
            var maximumTangentImpulse = prestep.MaterialProperties.FrictionCoefficient * (accumulatedImpulses.Penetration0);
            Vector3Wide.Subtract(prestep.Contact0.OffsetA, prestep.OffsetB, out var offsetToManifoldCenterB);
            TangentFriction.Solve(x, z, prestep.Contact0.OffsetA, offsetToManifoldCenterB, inertiaA, inertiaB, maximumTangentImpulse, ref accumulatedImpulses.Tangent, ref wsvA, ref wsvB);
            //If there's only one contact, then the contact patch as determined by contact distance would be zero.
            //That can cause some subtle behavioral issues sometimes, so we approximate lever arm with the contact depth, assuming that the contact surface area will increase as the depth increases.
            var maximumTwistImpulse = prestep.MaterialProperties.FrictionCoefficient * accumulatedImpulses.Penetration0 * Vector.Max(Vector<float>.Zero, prestep.Contact0.Depth);
            TwistFriction.Solve(prestep.Normal, inertiaA, inertiaB, maximumTwistImpulse, ref accumulatedImpulses.Twist, ref wsvA, ref wsvB);
        }               
    }
    
    /// <summary>
    /// Handles the solve iterations of a bunch of 1-contact two body manifold constraints.
    /// </summary>
    public class Contact1TypeProcessor :
        TwoBodyContactTypeProcessor<Contact1PrestepData, Contact1AccumulatedImpulses, Contact1Functions>
    {
        //Matches UpdateConstraintForManifold's manifoldTypeAsConstraintType computation.
        public const int BatchTypeId = 4;
    }


    public struct Contact2 : IConvexTwoBodyContactConstraintDescription<Contact2>
    {
        public ConstraintContactData Contact0;
        public ConstraintContactData Contact1;
        public Vector3 OffsetB;
        public Vector3 Normal;
        public float FrictionCoefficient;
        public SpringSettings SpringSettings;
        public float MaximumRecoveryVelocity;

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            Debug.Assert(batch.TypeId == ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<Contact2PrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(Contact0.OffsetA, ref target.Contact0.OffsetA);
            GetFirst(ref target.Contact0.Depth) = Contact0.PenetrationDepth;
            Vector3Wide.WriteFirst(Contact1.OffsetA, ref target.Contact1.OffsetA);
            GetFirst(ref target.Contact1.Depth) = Contact1.PenetrationDepth;
            
            Vector3Wide.WriteFirst(OffsetB, ref target.OffsetB);
            Vector3Wide.WriteFirst(Normal, ref target.Normal);
            GetFirst(ref target.MaterialProperties.FrictionCoefficient) = FrictionCoefficient;
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.MaterialProperties.SpringSettings);
            GetFirst(ref target.MaterialProperties.MaximumRecoveryVelocity) = MaximumRecoveryVelocity;
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact2 description)
        {    
            Debug.Assert(batch.TypeId == ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<Contact2PrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.Contact0.OffsetA, out description.Contact0.OffsetA);
            description.Contact0.PenetrationDepth = GetFirst(ref source.Contact0.Depth);
            Vector3Wide.ReadFirst(source.Contact1.OffsetA, out description.Contact1.OffsetA);
            description.Contact1.PenetrationDepth = GetFirst(ref source.Contact1.Depth);
            
            Vector3Wide.ReadFirst(source.OffsetB, out description.OffsetB);
            Vector3Wide.ReadFirst(source.Normal, out description.Normal);
            description.FrictionCoefficient = GetFirst(ref source.MaterialProperties.FrictionCoefficient);
            SpringSettingsWide.ReadFirst(source.MaterialProperties.SpringSettings, out description.SpringSettings);
            description.MaximumRecoveryVelocity = GetFirst(ref source.MaterialProperties.MaximumRecoveryVelocity);
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ConstraintContactData GetFirstContact(ref Contact2 description)
        {
            return ref description.Contact0;
        }
        
        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact2TypeProcessor.BatchTypeId;
        }

        public readonly Type TypeProcessorType => typeof(Contact2TypeProcessor);

    }

    public struct Contact2PrestepData : ITwoBodyConvexContactPrestep<Contact2PrestepData>
    {
        //NOTE: Prestep data memory layout is relied upon by the constraint description for marginally more efficient setting and getting.
        //If you modify this layout, be sure to update the associated ContactManifold4Constraint.
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public ConvexContactWide Contact0;
        public ConvexContactWide Contact1;
        public Vector3Wide OffsetB;
        //In a convex manifold, all contacts share the same normal and tangents.
        public Vector3Wide Normal;
        public MaterialPropertiesWide MaterialProperties;
		
        public readonly int BodyCount => 2;
        public readonly int ContactCount => 2;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector3Wide GetNormal(ref Contact2PrestepData prestep)
        {
            return ref prestep.Normal;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ConvexContactWide GetContact(ref Contact2PrestepData prestep, int index)
        {
            return ref Unsafe.Add(ref prestep.Contact0, index);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref MaterialPropertiesWide GetMaterialProperties(ref Contact2PrestepData prestep)
        {
            return ref prestep.MaterialProperties;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector3Wide GetOffsetB(ref Contact2PrestepData prestep)
        {
            return ref prestep.OffsetB;
        }
    }


    public struct Contact2Functions : ITwoBodyConstraintFunctions<Contact2PrestepData, Contact2AccumulatedImpulses>
    {       
        public bool RequiresIncrementalSubstepUpdates => true;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide velocityA, in BodyVelocityWide velocityB, ref Contact2PrestepData prestep)
        {
            PenetrationLimit.UpdatePenetrationDepth(dt, prestep.Contact0.OffsetA, prestep.OffsetB, prestep.Normal, velocityA, velocityB, ref prestep.Contact0.Depth);
            PenetrationLimit.UpdatePenetrationDepth(dt, prestep.Contact1.OffsetA, prestep.OffsetB, prestep.Normal, velocityA, velocityB, ref prestep.Contact1.Depth);
        }
               
        [MethodImpl(MethodImplOptions.AggressiveInlining)] 
        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, ref Contact2PrestepData prestep, ref Contact2AccumulatedImpulses accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            Helpers.BuildOrthonormalBasis(prestep.Normal, out var x, out var z);
            FrictionHelpers.ComputeFrictionCenter(prestep.Contact0.OffsetA, prestep.Contact1.OffsetA, prestep.Contact0.Depth, prestep.Contact1.Depth, out var offsetToManifoldCenterA);
            Vector3Wide.Subtract(offsetToManifoldCenterA, prestep.OffsetB, out var offsetToManifoldCenterB);
            TangentFriction.WarmStart(x, z, offsetToManifoldCenterA, offsetToManifoldCenterB, inertiaA, inertiaB, accumulatedImpulses.Tangent, ref wsvA, ref wsvB);
            PenetrationLimit.WarmStart(inertiaA, inertiaB, prestep.Normal, prestep.Contact0.OffsetA, prestep.Contact0.OffsetA - prestep.OffsetB, accumulatedImpulses.Penetration0, ref wsvA, ref wsvB);
            PenetrationLimit.WarmStart(inertiaA, inertiaB, prestep.Normal, prestep.Contact1.OffsetA, prestep.Contact1.OffsetA - prestep.OffsetB, accumulatedImpulses.Penetration1, ref wsvA, ref wsvB);
            TwistFriction.WarmStart(prestep.Normal, inertiaA, inertiaB, accumulatedImpulses.Twist, ref wsvA, ref wsvB);
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, ref Contact2PrestepData prestep, ref Contact2AccumulatedImpulses accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {            
            //Note that we solve the penetration constraints before the friction constraints. 
            //This makes the friction constraints more authoritative, since they happen last.
            //It's a pretty minor effect either way, but penetration constraints have error correction feedback- penetration depth.
            //Friction is velocity only and has no error correction, so introducing error there might cause drift.
            SpringSettingsWide.ComputeSpringiness(prestep.MaterialProperties.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);
            var inverseDtWide = new Vector<float>(inverseDt);
            PenetrationLimit.Solve(inertiaA, inertiaB, prestep.Normal, prestep.Contact0.OffsetA, prestep.Contact0.OffsetA - prestep.OffsetB, prestep.Contact0.Depth, positionErrorToVelocity, effectiveMassCFMScale, prestep.MaterialProperties.MaximumRecoveryVelocity, inverseDtWide, softnessImpulseScale, ref accumulatedImpulses.Penetration0, ref wsvA, ref wsvB);
            PenetrationLimit.Solve(inertiaA, inertiaB, prestep.Normal, prestep.Contact1.OffsetA, prestep.Contact1.OffsetA - prestep.OffsetB, prestep.Contact1.Depth, positionErrorToVelocity, effectiveMassCFMScale, prestep.MaterialProperties.MaximumRecoveryVelocity, inverseDtWide, softnessImpulseScale, ref accumulatedImpulses.Penetration1, ref wsvA, ref wsvB);
            Helpers.BuildOrthonormalBasis(prestep.Normal, out var x, out var z);
            var premultipliedFrictionCoefficient = new Vector<float>(1f / 2f) * prestep.MaterialProperties.FrictionCoefficient;
            var maximumTangentImpulse = premultipliedFrictionCoefficient * (accumulatedImpulses.Penetration0 + accumulatedImpulses.Penetration1);
            FrictionHelpers.ComputeFrictionCenter(prestep.Contact0.OffsetA, prestep.Contact1.OffsetA, prestep.Contact0.Depth, prestep.Contact1.Depth, out var offsetToManifoldCenterA);
            Vector3Wide.Subtract(offsetToManifoldCenterA, prestep.OffsetB, out var offsetToManifoldCenterB);
            TangentFriction.Solve(x, z, offsetToManifoldCenterA, offsetToManifoldCenterB, inertiaA, inertiaB, maximumTangentImpulse, ref accumulatedImpulses.Tangent, ref wsvA, ref wsvB);
            var maximumTwistImpulse = premultipliedFrictionCoefficient * (
                accumulatedImpulses.Penetration0 * Vector3Wide.Distance(offsetToManifoldCenterA, prestep.Contact0.OffsetA) +
                accumulatedImpulses.Penetration1 * Vector3Wide.Distance(offsetToManifoldCenterA, prestep.Contact1.OffsetA));
            TwistFriction.Solve(prestep.Normal, inertiaA, inertiaB, maximumTwistImpulse, ref accumulatedImpulses.Twist, ref wsvA, ref wsvB);
        }               
    }
    
    /// <summary>
    /// Handles the solve iterations of a bunch of 2-contact two body manifold constraints.
    /// </summary>
    public class Contact2TypeProcessor :
        TwoBodyContactTypeProcessor<Contact2PrestepData, Contact2AccumulatedImpulses, Contact2Functions>
    {
        //Matches UpdateConstraintForManifold's manifoldTypeAsConstraintType computation.
        public const int BatchTypeId = 5;
    }


    public struct Contact3 : IConvexTwoBodyContactConstraintDescription<Contact3>
    {
        public ConstraintContactData Contact0;
        public ConstraintContactData Contact1;
        public ConstraintContactData Contact2;
        public Vector3 OffsetB;
        public Vector3 Normal;
        public float FrictionCoefficient;
        public SpringSettings SpringSettings;
        public float MaximumRecoveryVelocity;

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            Debug.Assert(batch.TypeId == ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<Contact3PrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(Contact0.OffsetA, ref target.Contact0.OffsetA);
            GetFirst(ref target.Contact0.Depth) = Contact0.PenetrationDepth;
            Vector3Wide.WriteFirst(Contact1.OffsetA, ref target.Contact1.OffsetA);
            GetFirst(ref target.Contact1.Depth) = Contact1.PenetrationDepth;
            Vector3Wide.WriteFirst(Contact2.OffsetA, ref target.Contact2.OffsetA);
            GetFirst(ref target.Contact2.Depth) = Contact2.PenetrationDepth;
            
            Vector3Wide.WriteFirst(OffsetB, ref target.OffsetB);
            Vector3Wide.WriteFirst(Normal, ref target.Normal);
            GetFirst(ref target.MaterialProperties.FrictionCoefficient) = FrictionCoefficient;
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.MaterialProperties.SpringSettings);
            GetFirst(ref target.MaterialProperties.MaximumRecoveryVelocity) = MaximumRecoveryVelocity;
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact3 description)
        {    
            Debug.Assert(batch.TypeId == ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<Contact3PrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.Contact0.OffsetA, out description.Contact0.OffsetA);
            description.Contact0.PenetrationDepth = GetFirst(ref source.Contact0.Depth);
            Vector3Wide.ReadFirst(source.Contact1.OffsetA, out description.Contact1.OffsetA);
            description.Contact1.PenetrationDepth = GetFirst(ref source.Contact1.Depth);
            Vector3Wide.ReadFirst(source.Contact2.OffsetA, out description.Contact2.OffsetA);
            description.Contact2.PenetrationDepth = GetFirst(ref source.Contact2.Depth);
            
            Vector3Wide.ReadFirst(source.OffsetB, out description.OffsetB);
            Vector3Wide.ReadFirst(source.Normal, out description.Normal);
            description.FrictionCoefficient = GetFirst(ref source.MaterialProperties.FrictionCoefficient);
            SpringSettingsWide.ReadFirst(source.MaterialProperties.SpringSettings, out description.SpringSettings);
            description.MaximumRecoveryVelocity = GetFirst(ref source.MaterialProperties.MaximumRecoveryVelocity);
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ConstraintContactData GetFirstContact(ref Contact3 description)
        {
            return ref description.Contact0;
        }
        
        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact3TypeProcessor.BatchTypeId;
        }

        public readonly Type TypeProcessorType => typeof(Contact3TypeProcessor);

    }

    public struct Contact3PrestepData : ITwoBodyConvexContactPrestep<Contact3PrestepData>
    {
        //NOTE: Prestep data memory layout is relied upon by the constraint description for marginally more efficient setting and getting.
        //If you modify this layout, be sure to update the associated ContactManifold4Constraint.
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public ConvexContactWide Contact0;
        public ConvexContactWide Contact1;
        public ConvexContactWide Contact2;
        public Vector3Wide OffsetB;
        //In a convex manifold, all contacts share the same normal and tangents.
        public Vector3Wide Normal;
        public MaterialPropertiesWide MaterialProperties;
		
        public readonly int BodyCount => 2;
        public readonly int ContactCount => 3;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector3Wide GetNormal(ref Contact3PrestepData prestep)
        {
            return ref prestep.Normal;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ConvexContactWide GetContact(ref Contact3PrestepData prestep, int index)
        {
            return ref Unsafe.Add(ref prestep.Contact0, index);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref MaterialPropertiesWide GetMaterialProperties(ref Contact3PrestepData prestep)
        {
            return ref prestep.MaterialProperties;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector3Wide GetOffsetB(ref Contact3PrestepData prestep)
        {
            return ref prestep.OffsetB;
        }
    }


    public struct Contact3Functions : ITwoBodyConstraintFunctions<Contact3PrestepData, Contact3AccumulatedImpulses>
    {       
        public bool RequiresIncrementalSubstepUpdates => true;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide velocityA, in BodyVelocityWide velocityB, ref Contact3PrestepData prestep)
        {
            PenetrationLimit.UpdatePenetrationDepth(dt, prestep.Contact0.OffsetA, prestep.OffsetB, prestep.Normal, velocityA, velocityB, ref prestep.Contact0.Depth);
            PenetrationLimit.UpdatePenetrationDepth(dt, prestep.Contact1.OffsetA, prestep.OffsetB, prestep.Normal, velocityA, velocityB, ref prestep.Contact1.Depth);
            PenetrationLimit.UpdatePenetrationDepth(dt, prestep.Contact2.OffsetA, prestep.OffsetB, prestep.Normal, velocityA, velocityB, ref prestep.Contact2.Depth);
        }
               
        [MethodImpl(MethodImplOptions.AggressiveInlining)] 
        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, ref Contact3PrestepData prestep, ref Contact3AccumulatedImpulses accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            Helpers.BuildOrthonormalBasis(prestep.Normal, out var x, out var z);
            FrictionHelpers.ComputeFrictionCenter(prestep.Contact0.OffsetA, prestep.Contact1.OffsetA, prestep.Contact2.OffsetA, prestep.Contact0.Depth, prestep.Contact1.Depth, prestep.Contact2.Depth, out var offsetToManifoldCenterA);
            Vector3Wide.Subtract(offsetToManifoldCenterA, prestep.OffsetB, out var offsetToManifoldCenterB);
            TangentFriction.WarmStart(x, z, offsetToManifoldCenterA, offsetToManifoldCenterB, inertiaA, inertiaB, accumulatedImpulses.Tangent, ref wsvA, ref wsvB);
            PenetrationLimit.WarmStart(inertiaA, inertiaB, prestep.Normal, prestep.Contact0.OffsetA, prestep.Contact0.OffsetA - prestep.OffsetB, accumulatedImpulses.Penetration0, ref wsvA, ref wsvB);
            PenetrationLimit.WarmStart(inertiaA, inertiaB, prestep.Normal, prestep.Contact1.OffsetA, prestep.Contact1.OffsetA - prestep.OffsetB, accumulatedImpulses.Penetration1, ref wsvA, ref wsvB);
            PenetrationLimit.WarmStart(inertiaA, inertiaB, prestep.Normal, prestep.Contact2.OffsetA, prestep.Contact2.OffsetA - prestep.OffsetB, accumulatedImpulses.Penetration2, ref wsvA, ref wsvB);
            TwistFriction.WarmStart(prestep.Normal, inertiaA, inertiaB, accumulatedImpulses.Twist, ref wsvA, ref wsvB);
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, ref Contact3PrestepData prestep, ref Contact3AccumulatedImpulses accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {            
            //Note that we solve the penetration constraints before the friction constraints. 
            //This makes the friction constraints more authoritative, since they happen last.
            //It's a pretty minor effect either way, but penetration constraints have error correction feedback- penetration depth.
            //Friction is velocity only and has no error correction, so introducing error there might cause drift.
            SpringSettingsWide.ComputeSpringiness(prestep.MaterialProperties.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);
            var inverseDtWide = new Vector<float>(inverseDt);
            PenetrationLimit.Solve(inertiaA, inertiaB, prestep.Normal, prestep.Contact0.OffsetA, prestep.Contact0.OffsetA - prestep.OffsetB, prestep.Contact0.Depth, positionErrorToVelocity, effectiveMassCFMScale, prestep.MaterialProperties.MaximumRecoveryVelocity, inverseDtWide, softnessImpulseScale, ref accumulatedImpulses.Penetration0, ref wsvA, ref wsvB);
            PenetrationLimit.Solve(inertiaA, inertiaB, prestep.Normal, prestep.Contact1.OffsetA, prestep.Contact1.OffsetA - prestep.OffsetB, prestep.Contact1.Depth, positionErrorToVelocity, effectiveMassCFMScale, prestep.MaterialProperties.MaximumRecoveryVelocity, inverseDtWide, softnessImpulseScale, ref accumulatedImpulses.Penetration1, ref wsvA, ref wsvB);
            PenetrationLimit.Solve(inertiaA, inertiaB, prestep.Normal, prestep.Contact2.OffsetA, prestep.Contact2.OffsetA - prestep.OffsetB, prestep.Contact2.Depth, positionErrorToVelocity, effectiveMassCFMScale, prestep.MaterialProperties.MaximumRecoveryVelocity, inverseDtWide, softnessImpulseScale, ref accumulatedImpulses.Penetration2, ref wsvA, ref wsvB);
            Helpers.BuildOrthonormalBasis(prestep.Normal, out var x, out var z);
            var premultipliedFrictionCoefficient = new Vector<float>(1f / 3f) * prestep.MaterialProperties.FrictionCoefficient;
            var maximumTangentImpulse = premultipliedFrictionCoefficient * (accumulatedImpulses.Penetration0 + accumulatedImpulses.Penetration1 + accumulatedImpulses.Penetration2);
            FrictionHelpers.ComputeFrictionCenter(prestep.Contact0.OffsetA, prestep.Contact1.OffsetA, prestep.Contact2.OffsetA, prestep.Contact0.Depth, prestep.Contact1.Depth, prestep.Contact2.Depth, out var offsetToManifoldCenterA);
            Vector3Wide.Subtract(offsetToManifoldCenterA, prestep.OffsetB, out var offsetToManifoldCenterB);
            TangentFriction.Solve(x, z, offsetToManifoldCenterA, offsetToManifoldCenterB, inertiaA, inertiaB, maximumTangentImpulse, ref accumulatedImpulses.Tangent, ref wsvA, ref wsvB);
            var maximumTwistImpulse = premultipliedFrictionCoefficient * (
                accumulatedImpulses.Penetration0 * Vector3Wide.Distance(offsetToManifoldCenterA, prestep.Contact0.OffsetA) +
                accumulatedImpulses.Penetration1 * Vector3Wide.Distance(offsetToManifoldCenterA, prestep.Contact1.OffsetA) +
                accumulatedImpulses.Penetration2 * Vector3Wide.Distance(offsetToManifoldCenterA, prestep.Contact2.OffsetA));
            TwistFriction.Solve(prestep.Normal, inertiaA, inertiaB, maximumTwistImpulse, ref accumulatedImpulses.Twist, ref wsvA, ref wsvB);
        }               
    }
    
    /// <summary>
    /// Handles the solve iterations of a bunch of 3-contact two body manifold constraints.
    /// </summary>
    public class Contact3TypeProcessor :
        TwoBodyContactTypeProcessor<Contact3PrestepData, Contact3AccumulatedImpulses, Contact3Functions>
    {
        //Matches UpdateConstraintForManifold's manifoldTypeAsConstraintType computation.
        public const int BatchTypeId = 6;
    }


    public struct Contact4 : IConvexTwoBodyContactConstraintDescription<Contact4>
    {
        public ConstraintContactData Contact0;
        public ConstraintContactData Contact1;
        public ConstraintContactData Contact2;
        public ConstraintContactData Contact3;
        public Vector3 OffsetB;
        public Vector3 Normal;
        public float FrictionCoefficient;
        public SpringSettings SpringSettings;
        public float MaximumRecoveryVelocity;

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            Debug.Assert(batch.TypeId == ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<Contact4PrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(Contact0.OffsetA, ref target.Contact0.OffsetA);
            GetFirst(ref target.Contact0.Depth) = Contact0.PenetrationDepth;
            Vector3Wide.WriteFirst(Contact1.OffsetA, ref target.Contact1.OffsetA);
            GetFirst(ref target.Contact1.Depth) = Contact1.PenetrationDepth;
            Vector3Wide.WriteFirst(Contact2.OffsetA, ref target.Contact2.OffsetA);
            GetFirst(ref target.Contact2.Depth) = Contact2.PenetrationDepth;
            Vector3Wide.WriteFirst(Contact3.OffsetA, ref target.Contact3.OffsetA);
            GetFirst(ref target.Contact3.Depth) = Contact3.PenetrationDepth;
            
            Vector3Wide.WriteFirst(OffsetB, ref target.OffsetB);
            Vector3Wide.WriteFirst(Normal, ref target.Normal);
            GetFirst(ref target.MaterialProperties.FrictionCoefficient) = FrictionCoefficient;
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.MaterialProperties.SpringSettings);
            GetFirst(ref target.MaterialProperties.MaximumRecoveryVelocity) = MaximumRecoveryVelocity;
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact4 description)
        {    
            Debug.Assert(batch.TypeId == ConstraintTypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<Contact4PrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.Contact0.OffsetA, out description.Contact0.OffsetA);
            description.Contact0.PenetrationDepth = GetFirst(ref source.Contact0.Depth);
            Vector3Wide.ReadFirst(source.Contact1.OffsetA, out description.Contact1.OffsetA);
            description.Contact1.PenetrationDepth = GetFirst(ref source.Contact1.Depth);
            Vector3Wide.ReadFirst(source.Contact2.OffsetA, out description.Contact2.OffsetA);
            description.Contact2.PenetrationDepth = GetFirst(ref source.Contact2.Depth);
            Vector3Wide.ReadFirst(source.Contact3.OffsetA, out description.Contact3.OffsetA);
            description.Contact3.PenetrationDepth = GetFirst(ref source.Contact3.Depth);
            
            Vector3Wide.ReadFirst(source.OffsetB, out description.OffsetB);
            Vector3Wide.ReadFirst(source.Normal, out description.Normal);
            description.FrictionCoefficient = GetFirst(ref source.MaterialProperties.FrictionCoefficient);
            SpringSettingsWide.ReadFirst(source.MaterialProperties.SpringSettings, out description.SpringSettings);
            description.MaximumRecoveryVelocity = GetFirst(ref source.MaterialProperties.MaximumRecoveryVelocity);
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ConstraintContactData GetFirstContact(ref Contact4 description)
        {
            return ref description.Contact0;
        }
        
        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact4TypeProcessor.BatchTypeId;
        }

        public readonly Type TypeProcessorType => typeof(Contact4TypeProcessor);

    }

    public struct Contact4PrestepData : ITwoBodyConvexContactPrestep<Contact4PrestepData>
    {
        //NOTE: Prestep data memory layout is relied upon by the constraint description for marginally more efficient setting and getting.
        //If you modify this layout, be sure to update the associated ContactManifold4Constraint.
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public ConvexContactWide Contact0;
        public ConvexContactWide Contact1;
        public ConvexContactWide Contact2;
        public ConvexContactWide Contact3;
        public Vector3Wide OffsetB;
        //In a convex manifold, all contacts share the same normal and tangents.
        public Vector3Wide Normal;
        public MaterialPropertiesWide MaterialProperties;
		
        public readonly int BodyCount => 2;
        public readonly int ContactCount => 4;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector3Wide GetNormal(ref Contact4PrestepData prestep)
        {
            return ref prestep.Normal;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ConvexContactWide GetContact(ref Contact4PrestepData prestep, int index)
        {
            return ref Unsafe.Add(ref prestep.Contact0, index);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref MaterialPropertiesWide GetMaterialProperties(ref Contact4PrestepData prestep)
        {
            return ref prestep.MaterialProperties;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector3Wide GetOffsetB(ref Contact4PrestepData prestep)
        {
            return ref prestep.OffsetB;
        }
    }


    public struct Contact4Functions : ITwoBodyConstraintFunctions<Contact4PrestepData, Contact4AccumulatedImpulses>
    {       
        public bool RequiresIncrementalSubstepUpdates => true;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide velocityA, in BodyVelocityWide velocityB, ref Contact4PrestepData prestep)
        {
            PenetrationLimit.UpdatePenetrationDepth(dt, prestep.Contact0.OffsetA, prestep.OffsetB, prestep.Normal, velocityA, velocityB, ref prestep.Contact0.Depth);
            PenetrationLimit.UpdatePenetrationDepth(dt, prestep.Contact1.OffsetA, prestep.OffsetB, prestep.Normal, velocityA, velocityB, ref prestep.Contact1.Depth);
            PenetrationLimit.UpdatePenetrationDepth(dt, prestep.Contact2.OffsetA, prestep.OffsetB, prestep.Normal, velocityA, velocityB, ref prestep.Contact2.Depth);
            PenetrationLimit.UpdatePenetrationDepth(dt, prestep.Contact3.OffsetA, prestep.OffsetB, prestep.Normal, velocityA, velocityB, ref prestep.Contact3.Depth);
        }
               
        [MethodImpl(MethodImplOptions.AggressiveInlining)] 
        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, ref Contact4PrestepData prestep, ref Contact4AccumulatedImpulses accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            Helpers.BuildOrthonormalBasis(prestep.Normal, out var x, out var z);
            FrictionHelpers.ComputeFrictionCenter(prestep.Contact0.OffsetA, prestep.Contact1.OffsetA, prestep.Contact2.OffsetA, prestep.Contact3.OffsetA, prestep.Contact0.Depth, prestep.Contact1.Depth, prestep.Contact2.Depth, prestep.Contact3.Depth, out var offsetToManifoldCenterA);
            Vector3Wide.Subtract(offsetToManifoldCenterA, prestep.OffsetB, out var offsetToManifoldCenterB);
            TangentFriction.WarmStart(x, z, offsetToManifoldCenterA, offsetToManifoldCenterB, inertiaA, inertiaB, accumulatedImpulses.Tangent, ref wsvA, ref wsvB);
            PenetrationLimit.WarmStart(inertiaA, inertiaB, prestep.Normal, prestep.Contact0.OffsetA, prestep.Contact0.OffsetA - prestep.OffsetB, accumulatedImpulses.Penetration0, ref wsvA, ref wsvB);
            PenetrationLimit.WarmStart(inertiaA, inertiaB, prestep.Normal, prestep.Contact1.OffsetA, prestep.Contact1.OffsetA - prestep.OffsetB, accumulatedImpulses.Penetration1, ref wsvA, ref wsvB);
            PenetrationLimit.WarmStart(inertiaA, inertiaB, prestep.Normal, prestep.Contact2.OffsetA, prestep.Contact2.OffsetA - prestep.OffsetB, accumulatedImpulses.Penetration2, ref wsvA, ref wsvB);
            PenetrationLimit.WarmStart(inertiaA, inertiaB, prestep.Normal, prestep.Contact3.OffsetA, prestep.Contact3.OffsetA - prestep.OffsetB, accumulatedImpulses.Penetration3, ref wsvA, ref wsvB);
            TwistFriction.WarmStart(prestep.Normal, inertiaA, inertiaB, accumulatedImpulses.Twist, ref wsvA, ref wsvB);
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, ref Contact4PrestepData prestep, ref Contact4AccumulatedImpulses accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {            
            //Note that we solve the penetration constraints before the friction constraints. 
            //This makes the friction constraints more authoritative, since they happen last.
            //It's a pretty minor effect either way, but penetration constraints have error correction feedback- penetration depth.
            //Friction is velocity only and has no error correction, so introducing error there might cause drift.
            SpringSettingsWide.ComputeSpringiness(prestep.MaterialProperties.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);
            var inverseDtWide = new Vector<float>(inverseDt);
            PenetrationLimit.Solve(inertiaA, inertiaB, prestep.Normal, prestep.Contact0.OffsetA, prestep.Contact0.OffsetA - prestep.OffsetB, prestep.Contact0.Depth, positionErrorToVelocity, effectiveMassCFMScale, prestep.MaterialProperties.MaximumRecoveryVelocity, inverseDtWide, softnessImpulseScale, ref accumulatedImpulses.Penetration0, ref wsvA, ref wsvB);
            PenetrationLimit.Solve(inertiaA, inertiaB, prestep.Normal, prestep.Contact1.OffsetA, prestep.Contact1.OffsetA - prestep.OffsetB, prestep.Contact1.Depth, positionErrorToVelocity, effectiveMassCFMScale, prestep.MaterialProperties.MaximumRecoveryVelocity, inverseDtWide, softnessImpulseScale, ref accumulatedImpulses.Penetration1, ref wsvA, ref wsvB);
            PenetrationLimit.Solve(inertiaA, inertiaB, prestep.Normal, prestep.Contact2.OffsetA, prestep.Contact2.OffsetA - prestep.OffsetB, prestep.Contact2.Depth, positionErrorToVelocity, effectiveMassCFMScale, prestep.MaterialProperties.MaximumRecoveryVelocity, inverseDtWide, softnessImpulseScale, ref accumulatedImpulses.Penetration2, ref wsvA, ref wsvB);
            PenetrationLimit.Solve(inertiaA, inertiaB, prestep.Normal, prestep.Contact3.OffsetA, prestep.Contact3.OffsetA - prestep.OffsetB, prestep.Contact3.Depth, positionErrorToVelocity, effectiveMassCFMScale, prestep.MaterialProperties.MaximumRecoveryVelocity, inverseDtWide, softnessImpulseScale, ref accumulatedImpulses.Penetration3, ref wsvA, ref wsvB);
            Helpers.BuildOrthonormalBasis(prestep.Normal, out var x, out var z);
            var premultipliedFrictionCoefficient = new Vector<float>(1f / 4f) * prestep.MaterialProperties.FrictionCoefficient;
            var maximumTangentImpulse = premultipliedFrictionCoefficient * (accumulatedImpulses.Penetration0 + accumulatedImpulses.Penetration1 + accumulatedImpulses.Penetration2 + accumulatedImpulses.Penetration3);
            FrictionHelpers.ComputeFrictionCenter(prestep.Contact0.OffsetA, prestep.Contact1.OffsetA, prestep.Contact2.OffsetA, prestep.Contact3.OffsetA, prestep.Contact0.Depth, prestep.Contact1.Depth, prestep.Contact2.Depth, prestep.Contact3.Depth, out var offsetToManifoldCenterA);
            Vector3Wide.Subtract(offsetToManifoldCenterA, prestep.OffsetB, out var offsetToManifoldCenterB);
            TangentFriction.Solve(x, z, offsetToManifoldCenterA, offsetToManifoldCenterB, inertiaA, inertiaB, maximumTangentImpulse, ref accumulatedImpulses.Tangent, ref wsvA, ref wsvB);
            var maximumTwistImpulse = premultipliedFrictionCoefficient * (
                accumulatedImpulses.Penetration0 * Vector3Wide.Distance(offsetToManifoldCenterA, prestep.Contact0.OffsetA) +
                accumulatedImpulses.Penetration1 * Vector3Wide.Distance(offsetToManifoldCenterA, prestep.Contact1.OffsetA) +
                accumulatedImpulses.Penetration2 * Vector3Wide.Distance(offsetToManifoldCenterA, prestep.Contact2.OffsetA) +
                accumulatedImpulses.Penetration3 * Vector3Wide.Distance(offsetToManifoldCenterA, prestep.Contact3.OffsetA));
            TwistFriction.Solve(prestep.Normal, inertiaA, inertiaB, maximumTwistImpulse, ref accumulatedImpulses.Twist, ref wsvA, ref wsvB);
        }               
    }
    
    /// <summary>
    /// Handles the solve iterations of a bunch of 4-contact two body manifold constraints.
    /// </summary>
    public class Contact4TypeProcessor :
        TwoBodyContactTypeProcessor<Contact4PrestepData, Contact4AccumulatedImpulses, Contact4Functions>
    {
        //Matches UpdateConstraintForManifold's manifoldTypeAsConstraintType computation.
        public const int BatchTypeId = 7;
    }


}
