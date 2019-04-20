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

        public Type TypeProcessorType => typeof(Contact2NonconvexTypeProcessor);

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
        
        public int ContactCount => 2;
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
        TwoBodyContactTypeProcessor<Contact2NonconvexPrestepData, Contact2NonconvexProjection, Contact2NonconvexAccumulatedImpulses,
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

        public Type TypeProcessorType => typeof(Contact2NonconvexOneBodyTypeProcessor);

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
                
        public int ContactCount => 2;
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
        OneBodyContactTypeProcessor<Contact2NonconvexOneBodyPrestepData, Contact2NonconvexOneBodyProjection, Contact2NonconvexAccumulatedImpulses,
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

        public Type TypeProcessorType => typeof(Contact3NonconvexTypeProcessor);

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
        
        public int ContactCount => 3;
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
        TwoBodyContactTypeProcessor<Contact3NonconvexPrestepData, Contact3NonconvexProjection, Contact3NonconvexAccumulatedImpulses,
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

        public Type TypeProcessorType => typeof(Contact3NonconvexOneBodyTypeProcessor);

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
                
        public int ContactCount => 3;
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
        OneBodyContactTypeProcessor<Contact3NonconvexOneBodyPrestepData, Contact3NonconvexOneBodyProjection, Contact3NonconvexAccumulatedImpulses,
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

        public Type TypeProcessorType => typeof(Contact4NonconvexTypeProcessor);

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
        
        public int ContactCount => 4;
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
        TwoBodyContactTypeProcessor<Contact4NonconvexPrestepData, Contact4NonconvexProjection, Contact4NonconvexAccumulatedImpulses,
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

        public Type TypeProcessorType => typeof(Contact4NonconvexOneBodyTypeProcessor);

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
                
        public int ContactCount => 4;
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
        OneBodyContactTypeProcessor<Contact4NonconvexOneBodyPrestepData, Contact4NonconvexOneBodyProjection, Contact4NonconvexAccumulatedImpulses,
            ContactNonconvexOneBodyFunctions<Contact4NonconvexOneBodyPrestepData, Contact4NonconvexOneBodyProjection, Contact4NonconvexAccumulatedImpulses>>
    {
        //Matches UpdateConstraintForManifold's manifoldTypeAsConstraintType computation.
        public const int BatchTypeId = 10;
    }


}
