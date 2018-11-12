using BepuPhysics.CollisionDetection;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
namespace BepuPhysics.Constraints.Contact
{  
    public struct Contact2Nonconvex : INonconvexTwoBodyContactConstraintDescription<Contact2Nonconvex>
    {
        public NonconvexTwoBodyManifoldConstraintProperties Common;
        public NonconvexConstraintContactData Contact0;
        public NonconvexConstraintContactData Contact1;

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            NonconvexConstraintHelpers.ApplyTwoBodyDescription<Contact2Nonconvex, Contact2NonconvexPrestepData>(ref this, ref batch, bundleIndex, innerIndex);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact2Nonconvex description)
        {
            NonconvexConstraintHelpers.BuildTwoBodyDescription<Contact2Nonconvex, Contact2NonconvexPrestepData>(ref batch, bundleIndex, innerIndex, out description);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyManifoldWideProperties(ref Vector3 offsetB, ref PairMaterialProperties material)
        {
            Common.OffsetB = offsetB;
            Common.FrictionCoefficient = material.FrictionCoefficient;
            Common.SpringSettings = material.SpringSettings;
            Common.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexTwoBodyManifoldConstraintProperties GetCommonProperties(ref Contact2Nonconvex description)
        {
            return ref description.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexConstraintContactData GetFirstContact(ref Contact2Nonconvex description)
        {
            return ref description.Contact0;
        }

        public int ContactCount => 2;

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact2NonconvexTypeProcessor.BatchTypeId;
        }

        public Type BatchType => typeof(Contact2NonconvexTypeProcessor);

    }

    public struct Contact2NonconvexPrestepData : INonconvexTwoBodyContactPrestepWide<Contact2NonconvexPrestepData>
    {
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public NonconvexTwoBodyContactPrestepCommon Common;
        public NonconvexPrestepData Contact0;
        public NonconvexPrestepData Contact1;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexTwoBodyContactPrestepCommon GetCommonProperties(ref Contact2NonconvexPrestepData prestep)
        {
            return ref prestep.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexPrestepData GetFirstContact(ref Contact2NonconvexPrestepData prestep)
        {
            return ref prestep.Contact0;
        }
    }

    public struct Contact2NonconvexAccumulatedImpulses
    {
        public NonconvexAccumulatedImpulses Contact0;
        public NonconvexAccumulatedImpulses Contact1;
    }

    public unsafe struct Contact2NonconvexProjection : INonconvexTwoBodyProjection<Contact2NonconvexProjection>
    {
        public NonconvexTwoBodyProjectionCommon Common;
        //Nonprimitive fixed would be pretty convenient.
        public ContactNonconvexTwoBodyProjection Contact0;
        public ContactNonconvexTwoBodyProjection Contact1;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ContactNonconvexTwoBodyProjection GetFirstContact(ref Contact2NonconvexProjection projection)
        {
            return ref projection.Contact0;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexTwoBodyProjectionCommon GetCommonProperties(ref Contact2NonconvexProjection projection)
        {
            return ref projection.Common;
        }

        public int ContactCount => 2;
    }
	
    /// <summary>
    /// Handles the solve iterations of a bunch of 2-contact nonconvex two body manifold constraints.
    /// </summary>
    public class Contact2NonconvexTypeProcessor :
        TwoBodyTypeProcessor<Contact2NonconvexPrestepData, Contact2NonconvexProjection, Contact2NonconvexAccumulatedImpulses,
            ContactNonconvexTwoBodyFunctions<Contact2NonconvexPrestepData, Contact2NonconvexProjection, Contact2NonconvexAccumulatedImpulses>>
    {
        //Matches UpdateConstraintForManifold's manifoldTypeAsConstraintType computation.
        public const int BatchTypeId = 15;
    }

	public struct Contact2NonconvexOneBody : INonconvexOneBodyContactConstraintDescription<Contact2NonconvexOneBody>
    {
        public NonconvexOneBodyManifoldConstraintProperties Common;
        public NonconvexConstraintContactData Contact0;
        public NonconvexConstraintContactData Contact1;

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            NonconvexConstraintHelpers.ApplyOneBodyDescription<Contact2NonconvexOneBody, Contact2NonconvexOneBodyPrestepData>(ref this, ref batch, bundleIndex, innerIndex);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact2NonconvexOneBody description)
        {
            NonconvexConstraintHelpers.BuildOneBodyDescription<Contact2NonconvexOneBody, Contact2NonconvexOneBodyPrestepData>(ref batch, bundleIndex, innerIndex, out description);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyManifoldWideProperties(ref PairMaterialProperties material)
        {
            Common.FrictionCoefficient = material.FrictionCoefficient;
            Common.SpringSettings = material.SpringSettings;
            Common.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexOneBodyManifoldConstraintProperties GetCommonProperties(ref Contact2NonconvexOneBody description)
        {
            return ref description.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexConstraintContactData GetFirstContact(ref Contact2NonconvexOneBody description)
        {
            return ref description.Contact0;
        }

        public int ContactCount => 2;

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact2NonconvexOneBodyTypeProcessor.BatchTypeId;
        }

        public Type BatchType => typeof(Contact2NonconvexOneBodyTypeProcessor);

    }

    public struct Contact2NonconvexOneBodyPrestepData : INonconvexOneBodyContactPrestepWide<Contact2NonconvexOneBodyPrestepData>    
	{
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public NonconvexOneBodyContactPrestepCommon Common;
        public NonconvexPrestepData Contact0;
        public NonconvexPrestepData Contact1;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexOneBodyContactPrestepCommon GetCommonProperties(ref Contact2NonconvexOneBodyPrestepData prestep)
        {
            return ref prestep.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexPrestepData GetFirstContact(ref Contact2NonconvexOneBodyPrestepData prestep)
        {
            return ref prestep.Contact0;
        }
    }

    public unsafe struct Contact2NonconvexOneBodyProjection : INonconvexOneBodyProjection<Contact2NonconvexOneBodyProjection>
    {
        public NonconvexOneBodyProjectionCommon Common;
        //Nonprimitive fixed would be pretty convenient.
        public ContactNonconvexOneBodyProjection Contact0;
        public ContactNonconvexOneBodyProjection Contact1;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ContactNonconvexOneBodyProjection GetFirstContact(ref Contact2NonconvexOneBodyProjection projection)
        {
            return ref projection.Contact0;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexOneBodyProjectionCommon GetCommonProperties(ref Contact2NonconvexOneBodyProjection projection)
        {
            return ref projection.Common;
        }

        public int ContactCount => 2;
    }

	/// <summary>
    /// Handles the solve iterations of a bunch of 2-contact nonconvex one body manifold constraints.
    /// </summary>
    public class Contact2NonconvexOneBodyTypeProcessor :
        OneBodyTypeProcessor<Contact2NonconvexOneBodyPrestepData, Contact2NonconvexOneBodyProjection, Contact2NonconvexAccumulatedImpulses,
            ContactNonconvexOneBodyFunctions<Contact2NonconvexOneBodyPrestepData, Contact2NonconvexOneBodyProjection, Contact2NonconvexAccumulatedImpulses>>
    {
        //Matches UpdateConstraintForManifold's manifoldTypeAsConstraintType computation.
        public const int BatchTypeId = 8;
    }


    public struct Contact3Nonconvex : INonconvexTwoBodyContactConstraintDescription<Contact3Nonconvex>
    {
        public NonconvexTwoBodyManifoldConstraintProperties Common;
        public NonconvexConstraintContactData Contact0;
        public NonconvexConstraintContactData Contact1;
        public NonconvexConstraintContactData Contact2;

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            NonconvexConstraintHelpers.ApplyTwoBodyDescription<Contact3Nonconvex, Contact3NonconvexPrestepData>(ref this, ref batch, bundleIndex, innerIndex);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact3Nonconvex description)
        {
            NonconvexConstraintHelpers.BuildTwoBodyDescription<Contact3Nonconvex, Contact3NonconvexPrestepData>(ref batch, bundleIndex, innerIndex, out description);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyManifoldWideProperties(ref Vector3 offsetB, ref PairMaterialProperties material)
        {
            Common.OffsetB = offsetB;
            Common.FrictionCoefficient = material.FrictionCoefficient;
            Common.SpringSettings = material.SpringSettings;
            Common.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexTwoBodyManifoldConstraintProperties GetCommonProperties(ref Contact3Nonconvex description)
        {
            return ref description.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexConstraintContactData GetFirstContact(ref Contact3Nonconvex description)
        {
            return ref description.Contact0;
        }

        public int ContactCount => 3;

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact3NonconvexTypeProcessor.BatchTypeId;
        }

        public Type BatchType => typeof(Contact3NonconvexTypeProcessor);

    }

    public struct Contact3NonconvexPrestepData : INonconvexTwoBodyContactPrestepWide<Contact3NonconvexPrestepData>
    {
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public NonconvexTwoBodyContactPrestepCommon Common;
        public NonconvexPrestepData Contact0;
        public NonconvexPrestepData Contact1;
        public NonconvexPrestepData Contact2;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexTwoBodyContactPrestepCommon GetCommonProperties(ref Contact3NonconvexPrestepData prestep)
        {
            return ref prestep.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexPrestepData GetFirstContact(ref Contact3NonconvexPrestepData prestep)
        {
            return ref prestep.Contact0;
        }
    }

    public struct Contact3NonconvexAccumulatedImpulses
    {
        public NonconvexAccumulatedImpulses Contact0;
        public NonconvexAccumulatedImpulses Contact1;
        public NonconvexAccumulatedImpulses Contact2;
    }

    public unsafe struct Contact3NonconvexProjection : INonconvexTwoBodyProjection<Contact3NonconvexProjection>
    {
        public NonconvexTwoBodyProjectionCommon Common;
        //Nonprimitive fixed would be pretty convenient.
        public ContactNonconvexTwoBodyProjection Contact0;
        public ContactNonconvexTwoBodyProjection Contact1;
        public ContactNonconvexTwoBodyProjection Contact2;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ContactNonconvexTwoBodyProjection GetFirstContact(ref Contact3NonconvexProjection projection)
        {
            return ref projection.Contact0;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexTwoBodyProjectionCommon GetCommonProperties(ref Contact3NonconvexProjection projection)
        {
            return ref projection.Common;
        }

        public int ContactCount => 3;
    }
	
    /// <summary>
    /// Handles the solve iterations of a bunch of 3-contact nonconvex two body manifold constraints.
    /// </summary>
    public class Contact3NonconvexTypeProcessor :
        TwoBodyTypeProcessor<Contact3NonconvexPrestepData, Contact3NonconvexProjection, Contact3NonconvexAccumulatedImpulses,
            ContactNonconvexTwoBodyFunctions<Contact3NonconvexPrestepData, Contact3NonconvexProjection, Contact3NonconvexAccumulatedImpulses>>
    {
        //Matches UpdateConstraintForManifold's manifoldTypeAsConstraintType computation.
        public const int BatchTypeId = 16;
    }

	public struct Contact3NonconvexOneBody : INonconvexOneBodyContactConstraintDescription<Contact3NonconvexOneBody>
    {
        public NonconvexOneBodyManifoldConstraintProperties Common;
        public NonconvexConstraintContactData Contact0;
        public NonconvexConstraintContactData Contact1;
        public NonconvexConstraintContactData Contact2;

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            NonconvexConstraintHelpers.ApplyOneBodyDescription<Contact3NonconvexOneBody, Contact3NonconvexOneBodyPrestepData>(ref this, ref batch, bundleIndex, innerIndex);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact3NonconvexOneBody description)
        {
            NonconvexConstraintHelpers.BuildOneBodyDescription<Contact3NonconvexOneBody, Contact3NonconvexOneBodyPrestepData>(ref batch, bundleIndex, innerIndex, out description);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyManifoldWideProperties(ref PairMaterialProperties material)
        {
            Common.FrictionCoefficient = material.FrictionCoefficient;
            Common.SpringSettings = material.SpringSettings;
            Common.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexOneBodyManifoldConstraintProperties GetCommonProperties(ref Contact3NonconvexOneBody description)
        {
            return ref description.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexConstraintContactData GetFirstContact(ref Contact3NonconvexOneBody description)
        {
            return ref description.Contact0;
        }

        public int ContactCount => 3;

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact3NonconvexOneBodyTypeProcessor.BatchTypeId;
        }

        public Type BatchType => typeof(Contact3NonconvexOneBodyTypeProcessor);

    }

    public struct Contact3NonconvexOneBodyPrestepData : INonconvexOneBodyContactPrestepWide<Contact3NonconvexOneBodyPrestepData>    
	{
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public NonconvexOneBodyContactPrestepCommon Common;
        public NonconvexPrestepData Contact0;
        public NonconvexPrestepData Contact1;
        public NonconvexPrestepData Contact2;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexOneBodyContactPrestepCommon GetCommonProperties(ref Contact3NonconvexOneBodyPrestepData prestep)
        {
            return ref prestep.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexPrestepData GetFirstContact(ref Contact3NonconvexOneBodyPrestepData prestep)
        {
            return ref prestep.Contact0;
        }
    }

    public unsafe struct Contact3NonconvexOneBodyProjection : INonconvexOneBodyProjection<Contact3NonconvexOneBodyProjection>
    {
        public NonconvexOneBodyProjectionCommon Common;
        //Nonprimitive fixed would be pretty convenient.
        public ContactNonconvexOneBodyProjection Contact0;
        public ContactNonconvexOneBodyProjection Contact1;
        public ContactNonconvexOneBodyProjection Contact2;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ContactNonconvexOneBodyProjection GetFirstContact(ref Contact3NonconvexOneBodyProjection projection)
        {
            return ref projection.Contact0;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexOneBodyProjectionCommon GetCommonProperties(ref Contact3NonconvexOneBodyProjection projection)
        {
            return ref projection.Common;
        }

        public int ContactCount => 3;
    }

	/// <summary>
    /// Handles the solve iterations of a bunch of 3-contact nonconvex one body manifold constraints.
    /// </summary>
    public class Contact3NonconvexOneBodyTypeProcessor :
        OneBodyTypeProcessor<Contact3NonconvexOneBodyPrestepData, Contact3NonconvexOneBodyProjection, Contact3NonconvexAccumulatedImpulses,
            ContactNonconvexOneBodyFunctions<Contact3NonconvexOneBodyPrestepData, Contact3NonconvexOneBodyProjection, Contact3NonconvexAccumulatedImpulses>>
    {
        //Matches UpdateConstraintForManifold's manifoldTypeAsConstraintType computation.
        public const int BatchTypeId = 9;
    }


    public struct Contact4Nonconvex : INonconvexTwoBodyContactConstraintDescription<Contact4Nonconvex>
    {
        public NonconvexTwoBodyManifoldConstraintProperties Common;
        public NonconvexConstraintContactData Contact0;
        public NonconvexConstraintContactData Contact1;
        public NonconvexConstraintContactData Contact2;
        public NonconvexConstraintContactData Contact3;

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            NonconvexConstraintHelpers.ApplyTwoBodyDescription<Contact4Nonconvex, Contact4NonconvexPrestepData>(ref this, ref batch, bundleIndex, innerIndex);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact4Nonconvex description)
        {
            NonconvexConstraintHelpers.BuildTwoBodyDescription<Contact4Nonconvex, Contact4NonconvexPrestepData>(ref batch, bundleIndex, innerIndex, out description);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyManifoldWideProperties(ref Vector3 offsetB, ref PairMaterialProperties material)
        {
            Common.OffsetB = offsetB;
            Common.FrictionCoefficient = material.FrictionCoefficient;
            Common.SpringSettings = material.SpringSettings;
            Common.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexTwoBodyManifoldConstraintProperties GetCommonProperties(ref Contact4Nonconvex description)
        {
            return ref description.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexConstraintContactData GetFirstContact(ref Contact4Nonconvex description)
        {
            return ref description.Contact0;
        }

        public int ContactCount => 4;

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact4NonconvexTypeProcessor.BatchTypeId;
        }

        public Type BatchType => typeof(Contact4NonconvexTypeProcessor);

    }

    public struct Contact4NonconvexPrestepData : INonconvexTwoBodyContactPrestepWide<Contact4NonconvexPrestepData>
    {
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public NonconvexTwoBodyContactPrestepCommon Common;
        public NonconvexPrestepData Contact0;
        public NonconvexPrestepData Contact1;
        public NonconvexPrestepData Contact2;
        public NonconvexPrestepData Contact3;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexTwoBodyContactPrestepCommon GetCommonProperties(ref Contact4NonconvexPrestepData prestep)
        {
            return ref prestep.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexPrestepData GetFirstContact(ref Contact4NonconvexPrestepData prestep)
        {
            return ref prestep.Contact0;
        }
    }

    public struct Contact4NonconvexAccumulatedImpulses
    {
        public NonconvexAccumulatedImpulses Contact0;
        public NonconvexAccumulatedImpulses Contact1;
        public NonconvexAccumulatedImpulses Contact2;
        public NonconvexAccumulatedImpulses Contact3;
    }

    public unsafe struct Contact4NonconvexProjection : INonconvexTwoBodyProjection<Contact4NonconvexProjection>
    {
        public NonconvexTwoBodyProjectionCommon Common;
        //Nonprimitive fixed would be pretty convenient.
        public ContactNonconvexTwoBodyProjection Contact0;
        public ContactNonconvexTwoBodyProjection Contact1;
        public ContactNonconvexTwoBodyProjection Contact2;
        public ContactNonconvexTwoBodyProjection Contact3;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ContactNonconvexTwoBodyProjection GetFirstContact(ref Contact4NonconvexProjection projection)
        {
            return ref projection.Contact0;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexTwoBodyProjectionCommon GetCommonProperties(ref Contact4NonconvexProjection projection)
        {
            return ref projection.Common;
        }

        public int ContactCount => 4;
    }
	
    /// <summary>
    /// Handles the solve iterations of a bunch of 4-contact nonconvex two body manifold constraints.
    /// </summary>
    public class Contact4NonconvexTypeProcessor :
        TwoBodyTypeProcessor<Contact4NonconvexPrestepData, Contact4NonconvexProjection, Contact4NonconvexAccumulatedImpulses,
            ContactNonconvexTwoBodyFunctions<Contact4NonconvexPrestepData, Contact4NonconvexProjection, Contact4NonconvexAccumulatedImpulses>>
    {
        //Matches UpdateConstraintForManifold's manifoldTypeAsConstraintType computation.
        public const int BatchTypeId = 17;
    }

	public struct Contact4NonconvexOneBody : INonconvexOneBodyContactConstraintDescription<Contact4NonconvexOneBody>
    {
        public NonconvexOneBodyManifoldConstraintProperties Common;
        public NonconvexConstraintContactData Contact0;
        public NonconvexConstraintContactData Contact1;
        public NonconvexConstraintContactData Contact2;
        public NonconvexConstraintContactData Contact3;

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            NonconvexConstraintHelpers.ApplyOneBodyDescription<Contact4NonconvexOneBody, Contact4NonconvexOneBodyPrestepData>(ref this, ref batch, bundleIndex, innerIndex);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact4NonconvexOneBody description)
        {
            NonconvexConstraintHelpers.BuildOneBodyDescription<Contact4NonconvexOneBody, Contact4NonconvexOneBodyPrestepData>(ref batch, bundleIndex, innerIndex, out description);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyManifoldWideProperties(ref PairMaterialProperties material)
        {
            Common.FrictionCoefficient = material.FrictionCoefficient;
            Common.SpringSettings = material.SpringSettings;
            Common.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexOneBodyManifoldConstraintProperties GetCommonProperties(ref Contact4NonconvexOneBody description)
        {
            return ref description.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexConstraintContactData GetFirstContact(ref Contact4NonconvexOneBody description)
        {
            return ref description.Contact0;
        }

        public int ContactCount => 4;

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact4NonconvexOneBodyTypeProcessor.BatchTypeId;
        }

        public Type BatchType => typeof(Contact4NonconvexOneBodyTypeProcessor);

    }

    public struct Contact4NonconvexOneBodyPrestepData : INonconvexOneBodyContactPrestepWide<Contact4NonconvexOneBodyPrestepData>    
	{
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public NonconvexOneBodyContactPrestepCommon Common;
        public NonconvexPrestepData Contact0;
        public NonconvexPrestepData Contact1;
        public NonconvexPrestepData Contact2;
        public NonconvexPrestepData Contact3;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexOneBodyContactPrestepCommon GetCommonProperties(ref Contact4NonconvexOneBodyPrestepData prestep)
        {
            return ref prestep.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexPrestepData GetFirstContact(ref Contact4NonconvexOneBodyPrestepData prestep)
        {
            return ref prestep.Contact0;
        }
    }

    public unsafe struct Contact4NonconvexOneBodyProjection : INonconvexOneBodyProjection<Contact4NonconvexOneBodyProjection>
    {
        public NonconvexOneBodyProjectionCommon Common;
        //Nonprimitive fixed would be pretty convenient.
        public ContactNonconvexOneBodyProjection Contact0;
        public ContactNonconvexOneBodyProjection Contact1;
        public ContactNonconvexOneBodyProjection Contact2;
        public ContactNonconvexOneBodyProjection Contact3;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ContactNonconvexOneBodyProjection GetFirstContact(ref Contact4NonconvexOneBodyProjection projection)
        {
            return ref projection.Contact0;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexOneBodyProjectionCommon GetCommonProperties(ref Contact4NonconvexOneBodyProjection projection)
        {
            return ref projection.Common;
        }

        public int ContactCount => 4;
    }

	/// <summary>
    /// Handles the solve iterations of a bunch of 4-contact nonconvex one body manifold constraints.
    /// </summary>
    public class Contact4NonconvexOneBodyTypeProcessor :
        OneBodyTypeProcessor<Contact4NonconvexOneBodyPrestepData, Contact4NonconvexOneBodyProjection, Contact4NonconvexAccumulatedImpulses,
            ContactNonconvexOneBodyFunctions<Contact4NonconvexOneBodyPrestepData, Contact4NonconvexOneBodyProjection, Contact4NonconvexAccumulatedImpulses>>
    {
        //Matches UpdateConstraintForManifold's manifoldTypeAsConstraintType computation.
        public const int BatchTypeId = 10;
    }


    public struct Contact5Nonconvex : INonconvexTwoBodyContactConstraintDescription<Contact5Nonconvex>
    {
        public NonconvexTwoBodyManifoldConstraintProperties Common;
        public NonconvexConstraintContactData Contact0;
        public NonconvexConstraintContactData Contact1;
        public NonconvexConstraintContactData Contact2;
        public NonconvexConstraintContactData Contact3;
        public NonconvexConstraintContactData Contact4;

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            NonconvexConstraintHelpers.ApplyTwoBodyDescription<Contact5Nonconvex, Contact5NonconvexPrestepData>(ref this, ref batch, bundleIndex, innerIndex);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact5Nonconvex description)
        {
            NonconvexConstraintHelpers.BuildTwoBodyDescription<Contact5Nonconvex, Contact5NonconvexPrestepData>(ref batch, bundleIndex, innerIndex, out description);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyManifoldWideProperties(ref Vector3 offsetB, ref PairMaterialProperties material)
        {
            Common.OffsetB = offsetB;
            Common.FrictionCoefficient = material.FrictionCoefficient;
            Common.SpringSettings = material.SpringSettings;
            Common.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexTwoBodyManifoldConstraintProperties GetCommonProperties(ref Contact5Nonconvex description)
        {
            return ref description.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexConstraintContactData GetFirstContact(ref Contact5Nonconvex description)
        {
            return ref description.Contact0;
        }

        public int ContactCount => 5;

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact5NonconvexTypeProcessor.BatchTypeId;
        }

        public Type BatchType => typeof(Contact5NonconvexTypeProcessor);

    }

    public struct Contact5NonconvexPrestepData : INonconvexTwoBodyContactPrestepWide<Contact5NonconvexPrestepData>
    {
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public NonconvexTwoBodyContactPrestepCommon Common;
        public NonconvexPrestepData Contact0;
        public NonconvexPrestepData Contact1;
        public NonconvexPrestepData Contact2;
        public NonconvexPrestepData Contact3;
        public NonconvexPrestepData Contact4;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexTwoBodyContactPrestepCommon GetCommonProperties(ref Contact5NonconvexPrestepData prestep)
        {
            return ref prestep.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexPrestepData GetFirstContact(ref Contact5NonconvexPrestepData prestep)
        {
            return ref prestep.Contact0;
        }
    }

    public struct Contact5NonconvexAccumulatedImpulses
    {
        public NonconvexAccumulatedImpulses Contact0;
        public NonconvexAccumulatedImpulses Contact1;
        public NonconvexAccumulatedImpulses Contact2;
        public NonconvexAccumulatedImpulses Contact3;
        public NonconvexAccumulatedImpulses Contact4;
    }

    public unsafe struct Contact5NonconvexProjection : INonconvexTwoBodyProjection<Contact5NonconvexProjection>
    {
        public NonconvexTwoBodyProjectionCommon Common;
        //Nonprimitive fixed would be pretty convenient.
        public ContactNonconvexTwoBodyProjection Contact0;
        public ContactNonconvexTwoBodyProjection Contact1;
        public ContactNonconvexTwoBodyProjection Contact2;
        public ContactNonconvexTwoBodyProjection Contact3;
        public ContactNonconvexTwoBodyProjection Contact4;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ContactNonconvexTwoBodyProjection GetFirstContact(ref Contact5NonconvexProjection projection)
        {
            return ref projection.Contact0;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexTwoBodyProjectionCommon GetCommonProperties(ref Contact5NonconvexProjection projection)
        {
            return ref projection.Common;
        }

        public int ContactCount => 5;
    }
	
    /// <summary>
    /// Handles the solve iterations of a bunch of 5-contact nonconvex two body manifold constraints.
    /// </summary>
    public class Contact5NonconvexTypeProcessor :
        TwoBodyTypeProcessor<Contact5NonconvexPrestepData, Contact5NonconvexProjection, Contact5NonconvexAccumulatedImpulses,
            ContactNonconvexTwoBodyFunctions<Contact5NonconvexPrestepData, Contact5NonconvexProjection, Contact5NonconvexAccumulatedImpulses>>
    {
        //Matches UpdateConstraintForManifold's manifoldTypeAsConstraintType computation.
        public const int BatchTypeId = 18;
    }

	public struct Contact5NonconvexOneBody : INonconvexOneBodyContactConstraintDescription<Contact5NonconvexOneBody>
    {
        public NonconvexOneBodyManifoldConstraintProperties Common;
        public NonconvexConstraintContactData Contact0;
        public NonconvexConstraintContactData Contact1;
        public NonconvexConstraintContactData Contact2;
        public NonconvexConstraintContactData Contact3;
        public NonconvexConstraintContactData Contact4;

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            NonconvexConstraintHelpers.ApplyOneBodyDescription<Contact5NonconvexOneBody, Contact5NonconvexOneBodyPrestepData>(ref this, ref batch, bundleIndex, innerIndex);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact5NonconvexOneBody description)
        {
            NonconvexConstraintHelpers.BuildOneBodyDescription<Contact5NonconvexOneBody, Contact5NonconvexOneBodyPrestepData>(ref batch, bundleIndex, innerIndex, out description);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyManifoldWideProperties(ref PairMaterialProperties material)
        {
            Common.FrictionCoefficient = material.FrictionCoefficient;
            Common.SpringSettings = material.SpringSettings;
            Common.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexOneBodyManifoldConstraintProperties GetCommonProperties(ref Contact5NonconvexOneBody description)
        {
            return ref description.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexConstraintContactData GetFirstContact(ref Contact5NonconvexOneBody description)
        {
            return ref description.Contact0;
        }

        public int ContactCount => 5;

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact5NonconvexOneBodyTypeProcessor.BatchTypeId;
        }

        public Type BatchType => typeof(Contact5NonconvexOneBodyTypeProcessor);

    }

    public struct Contact5NonconvexOneBodyPrestepData : INonconvexOneBodyContactPrestepWide<Contact5NonconvexOneBodyPrestepData>    
	{
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public NonconvexOneBodyContactPrestepCommon Common;
        public NonconvexPrestepData Contact0;
        public NonconvexPrestepData Contact1;
        public NonconvexPrestepData Contact2;
        public NonconvexPrestepData Contact3;
        public NonconvexPrestepData Contact4;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexOneBodyContactPrestepCommon GetCommonProperties(ref Contact5NonconvexOneBodyPrestepData prestep)
        {
            return ref prestep.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexPrestepData GetFirstContact(ref Contact5NonconvexOneBodyPrestepData prestep)
        {
            return ref prestep.Contact0;
        }
    }

    public unsafe struct Contact5NonconvexOneBodyProjection : INonconvexOneBodyProjection<Contact5NonconvexOneBodyProjection>
    {
        public NonconvexOneBodyProjectionCommon Common;
        //Nonprimitive fixed would be pretty convenient.
        public ContactNonconvexOneBodyProjection Contact0;
        public ContactNonconvexOneBodyProjection Contact1;
        public ContactNonconvexOneBodyProjection Contact2;
        public ContactNonconvexOneBodyProjection Contact3;
        public ContactNonconvexOneBodyProjection Contact4;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ContactNonconvexOneBodyProjection GetFirstContact(ref Contact5NonconvexOneBodyProjection projection)
        {
            return ref projection.Contact0;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexOneBodyProjectionCommon GetCommonProperties(ref Contact5NonconvexOneBodyProjection projection)
        {
            return ref projection.Common;
        }

        public int ContactCount => 5;
    }

	/// <summary>
    /// Handles the solve iterations of a bunch of 5-contact nonconvex one body manifold constraints.
    /// </summary>
    public class Contact5NonconvexOneBodyTypeProcessor :
        OneBodyTypeProcessor<Contact5NonconvexOneBodyPrestepData, Contact5NonconvexOneBodyProjection, Contact5NonconvexAccumulatedImpulses,
            ContactNonconvexOneBodyFunctions<Contact5NonconvexOneBodyPrestepData, Contact5NonconvexOneBodyProjection, Contact5NonconvexAccumulatedImpulses>>
    {
        //Matches UpdateConstraintForManifold's manifoldTypeAsConstraintType computation.
        public const int BatchTypeId = 11;
    }


    public struct Contact6Nonconvex : INonconvexTwoBodyContactConstraintDescription<Contact6Nonconvex>
    {
        public NonconvexTwoBodyManifoldConstraintProperties Common;
        public NonconvexConstraintContactData Contact0;
        public NonconvexConstraintContactData Contact1;
        public NonconvexConstraintContactData Contact2;
        public NonconvexConstraintContactData Contact3;
        public NonconvexConstraintContactData Contact4;
        public NonconvexConstraintContactData Contact5;

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            NonconvexConstraintHelpers.ApplyTwoBodyDescription<Contact6Nonconvex, Contact6NonconvexPrestepData>(ref this, ref batch, bundleIndex, innerIndex);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact6Nonconvex description)
        {
            NonconvexConstraintHelpers.BuildTwoBodyDescription<Contact6Nonconvex, Contact6NonconvexPrestepData>(ref batch, bundleIndex, innerIndex, out description);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyManifoldWideProperties(ref Vector3 offsetB, ref PairMaterialProperties material)
        {
            Common.OffsetB = offsetB;
            Common.FrictionCoefficient = material.FrictionCoefficient;
            Common.SpringSettings = material.SpringSettings;
            Common.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexTwoBodyManifoldConstraintProperties GetCommonProperties(ref Contact6Nonconvex description)
        {
            return ref description.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexConstraintContactData GetFirstContact(ref Contact6Nonconvex description)
        {
            return ref description.Contact0;
        }

        public int ContactCount => 6;

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact6NonconvexTypeProcessor.BatchTypeId;
        }

        public Type BatchType => typeof(Contact6NonconvexTypeProcessor);

    }

    public struct Contact6NonconvexPrestepData : INonconvexTwoBodyContactPrestepWide<Contact6NonconvexPrestepData>
    {
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public NonconvexTwoBodyContactPrestepCommon Common;
        public NonconvexPrestepData Contact0;
        public NonconvexPrestepData Contact1;
        public NonconvexPrestepData Contact2;
        public NonconvexPrestepData Contact3;
        public NonconvexPrestepData Contact4;
        public NonconvexPrestepData Contact5;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexTwoBodyContactPrestepCommon GetCommonProperties(ref Contact6NonconvexPrestepData prestep)
        {
            return ref prestep.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexPrestepData GetFirstContact(ref Contact6NonconvexPrestepData prestep)
        {
            return ref prestep.Contact0;
        }
    }

    public struct Contact6NonconvexAccumulatedImpulses
    {
        public NonconvexAccumulatedImpulses Contact0;
        public NonconvexAccumulatedImpulses Contact1;
        public NonconvexAccumulatedImpulses Contact2;
        public NonconvexAccumulatedImpulses Contact3;
        public NonconvexAccumulatedImpulses Contact4;
        public NonconvexAccumulatedImpulses Contact5;
    }

    public unsafe struct Contact6NonconvexProjection : INonconvexTwoBodyProjection<Contact6NonconvexProjection>
    {
        public NonconvexTwoBodyProjectionCommon Common;
        //Nonprimitive fixed would be pretty convenient.
        public ContactNonconvexTwoBodyProjection Contact0;
        public ContactNonconvexTwoBodyProjection Contact1;
        public ContactNonconvexTwoBodyProjection Contact2;
        public ContactNonconvexTwoBodyProjection Contact3;
        public ContactNonconvexTwoBodyProjection Contact4;
        public ContactNonconvexTwoBodyProjection Contact5;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ContactNonconvexTwoBodyProjection GetFirstContact(ref Contact6NonconvexProjection projection)
        {
            return ref projection.Contact0;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexTwoBodyProjectionCommon GetCommonProperties(ref Contact6NonconvexProjection projection)
        {
            return ref projection.Common;
        }

        public int ContactCount => 6;
    }
	
    /// <summary>
    /// Handles the solve iterations of a bunch of 6-contact nonconvex two body manifold constraints.
    /// </summary>
    public class Contact6NonconvexTypeProcessor :
        TwoBodyTypeProcessor<Contact6NonconvexPrestepData, Contact6NonconvexProjection, Contact6NonconvexAccumulatedImpulses,
            ContactNonconvexTwoBodyFunctions<Contact6NonconvexPrestepData, Contact6NonconvexProjection, Contact6NonconvexAccumulatedImpulses>>
    {
        //Matches UpdateConstraintForManifold's manifoldTypeAsConstraintType computation.
        public const int BatchTypeId = 19;
    }

	public struct Contact6NonconvexOneBody : INonconvexOneBodyContactConstraintDescription<Contact6NonconvexOneBody>
    {
        public NonconvexOneBodyManifoldConstraintProperties Common;
        public NonconvexConstraintContactData Contact0;
        public NonconvexConstraintContactData Contact1;
        public NonconvexConstraintContactData Contact2;
        public NonconvexConstraintContactData Contact3;
        public NonconvexConstraintContactData Contact4;
        public NonconvexConstraintContactData Contact5;

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            NonconvexConstraintHelpers.ApplyOneBodyDescription<Contact6NonconvexOneBody, Contact6NonconvexOneBodyPrestepData>(ref this, ref batch, bundleIndex, innerIndex);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact6NonconvexOneBody description)
        {
            NonconvexConstraintHelpers.BuildOneBodyDescription<Contact6NonconvexOneBody, Contact6NonconvexOneBodyPrestepData>(ref batch, bundleIndex, innerIndex, out description);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyManifoldWideProperties(ref PairMaterialProperties material)
        {
            Common.FrictionCoefficient = material.FrictionCoefficient;
            Common.SpringSettings = material.SpringSettings;
            Common.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexOneBodyManifoldConstraintProperties GetCommonProperties(ref Contact6NonconvexOneBody description)
        {
            return ref description.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexConstraintContactData GetFirstContact(ref Contact6NonconvexOneBody description)
        {
            return ref description.Contact0;
        }

        public int ContactCount => 6;

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact6NonconvexOneBodyTypeProcessor.BatchTypeId;
        }

        public Type BatchType => typeof(Contact6NonconvexOneBodyTypeProcessor);

    }

    public struct Contact6NonconvexOneBodyPrestepData : INonconvexOneBodyContactPrestepWide<Contact6NonconvexOneBodyPrestepData>    
	{
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public NonconvexOneBodyContactPrestepCommon Common;
        public NonconvexPrestepData Contact0;
        public NonconvexPrestepData Contact1;
        public NonconvexPrestepData Contact2;
        public NonconvexPrestepData Contact3;
        public NonconvexPrestepData Contact4;
        public NonconvexPrestepData Contact5;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexOneBodyContactPrestepCommon GetCommonProperties(ref Contact6NonconvexOneBodyPrestepData prestep)
        {
            return ref prestep.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexPrestepData GetFirstContact(ref Contact6NonconvexOneBodyPrestepData prestep)
        {
            return ref prestep.Contact0;
        }
    }

    public unsafe struct Contact6NonconvexOneBodyProjection : INonconvexOneBodyProjection<Contact6NonconvexOneBodyProjection>
    {
        public NonconvexOneBodyProjectionCommon Common;
        //Nonprimitive fixed would be pretty convenient.
        public ContactNonconvexOneBodyProjection Contact0;
        public ContactNonconvexOneBodyProjection Contact1;
        public ContactNonconvexOneBodyProjection Contact2;
        public ContactNonconvexOneBodyProjection Contact3;
        public ContactNonconvexOneBodyProjection Contact4;
        public ContactNonconvexOneBodyProjection Contact5;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ContactNonconvexOneBodyProjection GetFirstContact(ref Contact6NonconvexOneBodyProjection projection)
        {
            return ref projection.Contact0;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexOneBodyProjectionCommon GetCommonProperties(ref Contact6NonconvexOneBodyProjection projection)
        {
            return ref projection.Common;
        }

        public int ContactCount => 6;
    }

	/// <summary>
    /// Handles the solve iterations of a bunch of 6-contact nonconvex one body manifold constraints.
    /// </summary>
    public class Contact6NonconvexOneBodyTypeProcessor :
        OneBodyTypeProcessor<Contact6NonconvexOneBodyPrestepData, Contact6NonconvexOneBodyProjection, Contact6NonconvexAccumulatedImpulses,
            ContactNonconvexOneBodyFunctions<Contact6NonconvexOneBodyPrestepData, Contact6NonconvexOneBodyProjection, Contact6NonconvexAccumulatedImpulses>>
    {
        //Matches UpdateConstraintForManifold's manifoldTypeAsConstraintType computation.
        public const int BatchTypeId = 12;
    }


    public struct Contact7Nonconvex : INonconvexTwoBodyContactConstraintDescription<Contact7Nonconvex>
    {
        public NonconvexTwoBodyManifoldConstraintProperties Common;
        public NonconvexConstraintContactData Contact0;
        public NonconvexConstraintContactData Contact1;
        public NonconvexConstraintContactData Contact2;
        public NonconvexConstraintContactData Contact3;
        public NonconvexConstraintContactData Contact4;
        public NonconvexConstraintContactData Contact5;
        public NonconvexConstraintContactData Contact6;

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            NonconvexConstraintHelpers.ApplyTwoBodyDescription<Contact7Nonconvex, Contact7NonconvexPrestepData>(ref this, ref batch, bundleIndex, innerIndex);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact7Nonconvex description)
        {
            NonconvexConstraintHelpers.BuildTwoBodyDescription<Contact7Nonconvex, Contact7NonconvexPrestepData>(ref batch, bundleIndex, innerIndex, out description);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyManifoldWideProperties(ref Vector3 offsetB, ref PairMaterialProperties material)
        {
            Common.OffsetB = offsetB;
            Common.FrictionCoefficient = material.FrictionCoefficient;
            Common.SpringSettings = material.SpringSettings;
            Common.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexTwoBodyManifoldConstraintProperties GetCommonProperties(ref Contact7Nonconvex description)
        {
            return ref description.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexConstraintContactData GetFirstContact(ref Contact7Nonconvex description)
        {
            return ref description.Contact0;
        }

        public int ContactCount => 7;

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact7NonconvexTypeProcessor.BatchTypeId;
        }

        public Type BatchType => typeof(Contact7NonconvexTypeProcessor);

    }

    public struct Contact7NonconvexPrestepData : INonconvexTwoBodyContactPrestepWide<Contact7NonconvexPrestepData>
    {
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public NonconvexTwoBodyContactPrestepCommon Common;
        public NonconvexPrestepData Contact0;
        public NonconvexPrestepData Contact1;
        public NonconvexPrestepData Contact2;
        public NonconvexPrestepData Contact3;
        public NonconvexPrestepData Contact4;
        public NonconvexPrestepData Contact5;
        public NonconvexPrestepData Contact6;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexTwoBodyContactPrestepCommon GetCommonProperties(ref Contact7NonconvexPrestepData prestep)
        {
            return ref prestep.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexPrestepData GetFirstContact(ref Contact7NonconvexPrestepData prestep)
        {
            return ref prestep.Contact0;
        }
    }

    public struct Contact7NonconvexAccumulatedImpulses
    {
        public NonconvexAccumulatedImpulses Contact0;
        public NonconvexAccumulatedImpulses Contact1;
        public NonconvexAccumulatedImpulses Contact2;
        public NonconvexAccumulatedImpulses Contact3;
        public NonconvexAccumulatedImpulses Contact4;
        public NonconvexAccumulatedImpulses Contact5;
        public NonconvexAccumulatedImpulses Contact6;
    }

    public unsafe struct Contact7NonconvexProjection : INonconvexTwoBodyProjection<Contact7NonconvexProjection>
    {
        public NonconvexTwoBodyProjectionCommon Common;
        //Nonprimitive fixed would be pretty convenient.
        public ContactNonconvexTwoBodyProjection Contact0;
        public ContactNonconvexTwoBodyProjection Contact1;
        public ContactNonconvexTwoBodyProjection Contact2;
        public ContactNonconvexTwoBodyProjection Contact3;
        public ContactNonconvexTwoBodyProjection Contact4;
        public ContactNonconvexTwoBodyProjection Contact5;
        public ContactNonconvexTwoBodyProjection Contact6;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ContactNonconvexTwoBodyProjection GetFirstContact(ref Contact7NonconvexProjection projection)
        {
            return ref projection.Contact0;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexTwoBodyProjectionCommon GetCommonProperties(ref Contact7NonconvexProjection projection)
        {
            return ref projection.Common;
        }

        public int ContactCount => 7;
    }
	
    /// <summary>
    /// Handles the solve iterations of a bunch of 7-contact nonconvex two body manifold constraints.
    /// </summary>
    public class Contact7NonconvexTypeProcessor :
        TwoBodyTypeProcessor<Contact7NonconvexPrestepData, Contact7NonconvexProjection, Contact7NonconvexAccumulatedImpulses,
            ContactNonconvexTwoBodyFunctions<Contact7NonconvexPrestepData, Contact7NonconvexProjection, Contact7NonconvexAccumulatedImpulses>>
    {
        //Matches UpdateConstraintForManifold's manifoldTypeAsConstraintType computation.
        public const int BatchTypeId = 20;
    }

	public struct Contact7NonconvexOneBody : INonconvexOneBodyContactConstraintDescription<Contact7NonconvexOneBody>
    {
        public NonconvexOneBodyManifoldConstraintProperties Common;
        public NonconvexConstraintContactData Contact0;
        public NonconvexConstraintContactData Contact1;
        public NonconvexConstraintContactData Contact2;
        public NonconvexConstraintContactData Contact3;
        public NonconvexConstraintContactData Contact4;
        public NonconvexConstraintContactData Contact5;
        public NonconvexConstraintContactData Contact6;

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            NonconvexConstraintHelpers.ApplyOneBodyDescription<Contact7NonconvexOneBody, Contact7NonconvexOneBodyPrestepData>(ref this, ref batch, bundleIndex, innerIndex);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact7NonconvexOneBody description)
        {
            NonconvexConstraintHelpers.BuildOneBodyDescription<Contact7NonconvexOneBody, Contact7NonconvexOneBodyPrestepData>(ref batch, bundleIndex, innerIndex, out description);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyManifoldWideProperties(ref PairMaterialProperties material)
        {
            Common.FrictionCoefficient = material.FrictionCoefficient;
            Common.SpringSettings = material.SpringSettings;
            Common.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexOneBodyManifoldConstraintProperties GetCommonProperties(ref Contact7NonconvexOneBody description)
        {
            return ref description.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexConstraintContactData GetFirstContact(ref Contact7NonconvexOneBody description)
        {
            return ref description.Contact0;
        }

        public int ContactCount => 7;

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact7NonconvexOneBodyTypeProcessor.BatchTypeId;
        }

        public Type BatchType => typeof(Contact7NonconvexOneBodyTypeProcessor);

    }

    public struct Contact7NonconvexOneBodyPrestepData : INonconvexOneBodyContactPrestepWide<Contact7NonconvexOneBodyPrestepData>    
	{
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public NonconvexOneBodyContactPrestepCommon Common;
        public NonconvexPrestepData Contact0;
        public NonconvexPrestepData Contact1;
        public NonconvexPrestepData Contact2;
        public NonconvexPrestepData Contact3;
        public NonconvexPrestepData Contact4;
        public NonconvexPrestepData Contact5;
        public NonconvexPrestepData Contact6;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexOneBodyContactPrestepCommon GetCommonProperties(ref Contact7NonconvexOneBodyPrestepData prestep)
        {
            return ref prestep.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexPrestepData GetFirstContact(ref Contact7NonconvexOneBodyPrestepData prestep)
        {
            return ref prestep.Contact0;
        }
    }

    public unsafe struct Contact7NonconvexOneBodyProjection : INonconvexOneBodyProjection<Contact7NonconvexOneBodyProjection>
    {
        public NonconvexOneBodyProjectionCommon Common;
        //Nonprimitive fixed would be pretty convenient.
        public ContactNonconvexOneBodyProjection Contact0;
        public ContactNonconvexOneBodyProjection Contact1;
        public ContactNonconvexOneBodyProjection Contact2;
        public ContactNonconvexOneBodyProjection Contact3;
        public ContactNonconvexOneBodyProjection Contact4;
        public ContactNonconvexOneBodyProjection Contact5;
        public ContactNonconvexOneBodyProjection Contact6;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ContactNonconvexOneBodyProjection GetFirstContact(ref Contact7NonconvexOneBodyProjection projection)
        {
            return ref projection.Contact0;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexOneBodyProjectionCommon GetCommonProperties(ref Contact7NonconvexOneBodyProjection projection)
        {
            return ref projection.Common;
        }

        public int ContactCount => 7;
    }

	/// <summary>
    /// Handles the solve iterations of a bunch of 7-contact nonconvex one body manifold constraints.
    /// </summary>
    public class Contact7NonconvexOneBodyTypeProcessor :
        OneBodyTypeProcessor<Contact7NonconvexOneBodyPrestepData, Contact7NonconvexOneBodyProjection, Contact7NonconvexAccumulatedImpulses,
            ContactNonconvexOneBodyFunctions<Contact7NonconvexOneBodyPrestepData, Contact7NonconvexOneBodyProjection, Contact7NonconvexAccumulatedImpulses>>
    {
        //Matches UpdateConstraintForManifold's manifoldTypeAsConstraintType computation.
        public const int BatchTypeId = 13;
    }


    public struct Contact8Nonconvex : INonconvexTwoBodyContactConstraintDescription<Contact8Nonconvex>
    {
        public NonconvexTwoBodyManifoldConstraintProperties Common;
        public NonconvexConstraintContactData Contact0;
        public NonconvexConstraintContactData Contact1;
        public NonconvexConstraintContactData Contact2;
        public NonconvexConstraintContactData Contact3;
        public NonconvexConstraintContactData Contact4;
        public NonconvexConstraintContactData Contact5;
        public NonconvexConstraintContactData Contact6;
        public NonconvexConstraintContactData Contact7;

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            NonconvexConstraintHelpers.ApplyTwoBodyDescription<Contact8Nonconvex, Contact8NonconvexPrestepData>(ref this, ref batch, bundleIndex, innerIndex);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact8Nonconvex description)
        {
            NonconvexConstraintHelpers.BuildTwoBodyDescription<Contact8Nonconvex, Contact8NonconvexPrestepData>(ref batch, bundleIndex, innerIndex, out description);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyManifoldWideProperties(ref Vector3 offsetB, ref PairMaterialProperties material)
        {
            Common.OffsetB = offsetB;
            Common.FrictionCoefficient = material.FrictionCoefficient;
            Common.SpringSettings = material.SpringSettings;
            Common.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexTwoBodyManifoldConstraintProperties GetCommonProperties(ref Contact8Nonconvex description)
        {
            return ref description.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexConstraintContactData GetFirstContact(ref Contact8Nonconvex description)
        {
            return ref description.Contact0;
        }

        public int ContactCount => 8;

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact8NonconvexTypeProcessor.BatchTypeId;
        }

        public Type BatchType => typeof(Contact8NonconvexTypeProcessor);

    }

    public struct Contact8NonconvexPrestepData : INonconvexTwoBodyContactPrestepWide<Contact8NonconvexPrestepData>
    {
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public NonconvexTwoBodyContactPrestepCommon Common;
        public NonconvexPrestepData Contact0;
        public NonconvexPrestepData Contact1;
        public NonconvexPrestepData Contact2;
        public NonconvexPrestepData Contact3;
        public NonconvexPrestepData Contact4;
        public NonconvexPrestepData Contact5;
        public NonconvexPrestepData Contact6;
        public NonconvexPrestepData Contact7;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexTwoBodyContactPrestepCommon GetCommonProperties(ref Contact8NonconvexPrestepData prestep)
        {
            return ref prestep.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexPrestepData GetFirstContact(ref Contact8NonconvexPrestepData prestep)
        {
            return ref prestep.Contact0;
        }
    }

    public struct Contact8NonconvexAccumulatedImpulses
    {
        public NonconvexAccumulatedImpulses Contact0;
        public NonconvexAccumulatedImpulses Contact1;
        public NonconvexAccumulatedImpulses Contact2;
        public NonconvexAccumulatedImpulses Contact3;
        public NonconvexAccumulatedImpulses Contact4;
        public NonconvexAccumulatedImpulses Contact5;
        public NonconvexAccumulatedImpulses Contact6;
        public NonconvexAccumulatedImpulses Contact7;
    }

    public unsafe struct Contact8NonconvexProjection : INonconvexTwoBodyProjection<Contact8NonconvexProjection>
    {
        public NonconvexTwoBodyProjectionCommon Common;
        //Nonprimitive fixed would be pretty convenient.
        public ContactNonconvexTwoBodyProjection Contact0;
        public ContactNonconvexTwoBodyProjection Contact1;
        public ContactNonconvexTwoBodyProjection Contact2;
        public ContactNonconvexTwoBodyProjection Contact3;
        public ContactNonconvexTwoBodyProjection Contact4;
        public ContactNonconvexTwoBodyProjection Contact5;
        public ContactNonconvexTwoBodyProjection Contact6;
        public ContactNonconvexTwoBodyProjection Contact7;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ContactNonconvexTwoBodyProjection GetFirstContact(ref Contact8NonconvexProjection projection)
        {
            return ref projection.Contact0;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexTwoBodyProjectionCommon GetCommonProperties(ref Contact8NonconvexProjection projection)
        {
            return ref projection.Common;
        }

        public int ContactCount => 8;
    }
	
    /// <summary>
    /// Handles the solve iterations of a bunch of 8-contact nonconvex two body manifold constraints.
    /// </summary>
    public class Contact8NonconvexTypeProcessor :
        TwoBodyTypeProcessor<Contact8NonconvexPrestepData, Contact8NonconvexProjection, Contact8NonconvexAccumulatedImpulses,
            ContactNonconvexTwoBodyFunctions<Contact8NonconvexPrestepData, Contact8NonconvexProjection, Contact8NonconvexAccumulatedImpulses>>
    {
        //Matches UpdateConstraintForManifold's manifoldTypeAsConstraintType computation.
        public const int BatchTypeId = 21;
    }

	public struct Contact8NonconvexOneBody : INonconvexOneBodyContactConstraintDescription<Contact8NonconvexOneBody>
    {
        public NonconvexOneBodyManifoldConstraintProperties Common;
        public NonconvexConstraintContactData Contact0;
        public NonconvexConstraintContactData Contact1;
        public NonconvexConstraintContactData Contact2;
        public NonconvexConstraintContactData Contact3;
        public NonconvexConstraintContactData Contact4;
        public NonconvexConstraintContactData Contact5;
        public NonconvexConstraintContactData Contact6;
        public NonconvexConstraintContactData Contact7;

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            NonconvexConstraintHelpers.ApplyOneBodyDescription<Contact8NonconvexOneBody, Contact8NonconvexOneBodyPrestepData>(ref this, ref batch, bundleIndex, innerIndex);
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact8NonconvexOneBody description)
        {
            NonconvexConstraintHelpers.BuildOneBodyDescription<Contact8NonconvexOneBody, Contact8NonconvexOneBodyPrestepData>(ref batch, bundleIndex, innerIndex, out description);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void CopyManifoldWideProperties(ref PairMaterialProperties material)
        {
            Common.FrictionCoefficient = material.FrictionCoefficient;
            Common.SpringSettings = material.SpringSettings;
            Common.MaximumRecoveryVelocity = material.MaximumRecoveryVelocity;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexOneBodyManifoldConstraintProperties GetCommonProperties(ref Contact8NonconvexOneBody description)
        {
            return ref description.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexConstraintContactData GetFirstContact(ref Contact8NonconvexOneBody description)
        {
            return ref description.Contact0;
        }

        public int ContactCount => 8;

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact8NonconvexOneBodyTypeProcessor.BatchTypeId;
        }

        public Type BatchType => typeof(Contact8NonconvexOneBodyTypeProcessor);

    }

    public struct Contact8NonconvexOneBodyPrestepData : INonconvexOneBodyContactPrestepWide<Contact8NonconvexOneBodyPrestepData>    
	{
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public NonconvexOneBodyContactPrestepCommon Common;
        public NonconvexPrestepData Contact0;
        public NonconvexPrestepData Contact1;
        public NonconvexPrestepData Contact2;
        public NonconvexPrestepData Contact3;
        public NonconvexPrestepData Contact4;
        public NonconvexPrestepData Contact5;
        public NonconvexPrestepData Contact6;
        public NonconvexPrestepData Contact7;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexOneBodyContactPrestepCommon GetCommonProperties(ref Contact8NonconvexOneBodyPrestepData prestep)
        {
            return ref prestep.Common;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexPrestepData GetFirstContact(ref Contact8NonconvexOneBodyPrestepData prestep)
        {
            return ref prestep.Contact0;
        }
    }

    public unsafe struct Contact8NonconvexOneBodyProjection : INonconvexOneBodyProjection<Contact8NonconvexOneBodyProjection>
    {
        public NonconvexOneBodyProjectionCommon Common;
        //Nonprimitive fixed would be pretty convenient.
        public ContactNonconvexOneBodyProjection Contact0;
        public ContactNonconvexOneBodyProjection Contact1;
        public ContactNonconvexOneBodyProjection Contact2;
        public ContactNonconvexOneBodyProjection Contact3;
        public ContactNonconvexOneBodyProjection Contact4;
        public ContactNonconvexOneBodyProjection Contact5;
        public ContactNonconvexOneBodyProjection Contact6;
        public ContactNonconvexOneBodyProjection Contact7;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref ContactNonconvexOneBodyProjection GetFirstContact(ref Contact8NonconvexOneBodyProjection projection)
        {
            return ref projection.Contact0;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexOneBodyProjectionCommon GetCommonProperties(ref Contact8NonconvexOneBodyProjection projection)
        {
            return ref projection.Common;
        }

        public int ContactCount => 8;
    }

	/// <summary>
    /// Handles the solve iterations of a bunch of 8-contact nonconvex one body manifold constraints.
    /// </summary>
    public class Contact8NonconvexOneBodyTypeProcessor :
        OneBodyTypeProcessor<Contact8NonconvexOneBodyPrestepData, Contact8NonconvexOneBodyProjection, Contact8NonconvexAccumulatedImpulses,
            ContactNonconvexOneBodyFunctions<Contact8NonconvexOneBodyPrestepData, Contact8NonconvexOneBodyProjection, Contact8NonconvexAccumulatedImpulses>>
    {
        //Matches UpdateConstraintForManifold's manifoldTypeAsConstraintType computation.
        public const int BatchTypeId = 14;
    }


}
