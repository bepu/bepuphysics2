using BepuPhysics.CollisionDetection;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuUtilities;
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

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact2Nonconvex description)
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

        public readonly int ContactCount => 2;

        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact2NonconvexTypeProcessor.BatchTypeId;
        }

        public readonly Type TypeProcessorType => typeof(Contact2NonconvexTypeProcessor);

    }

    public struct Contact2NonconvexPrestepData : ITwoBodyNonconvexContactPrestep<Contact2NonconvexPrestepData>
    {
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public MaterialPropertiesWide MaterialProperties;
        public Vector3Wide OffsetB;
        public NonconvexContactPrestepData Contact0;
        public NonconvexContactPrestepData Contact1;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref MaterialPropertiesWide GetMaterialProperties(ref Contact2NonconvexPrestepData prestep)
        {
            return ref prestep.MaterialProperties;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector3Wide GetOffsetB(ref Contact2NonconvexPrestepData prestep)
        {
            return ref prestep.OffsetB;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexContactPrestepData GetContact(ref Contact2NonconvexPrestepData prestep, int index)
        {
            return ref Unsafe.Add(ref prestep.Contact0, index);
        }
        
        public readonly int ContactCount => 2;
        public readonly int BodyCount => 2;
    }

    public struct Contact2NonconvexAccumulatedImpulses : INonconvexContactAccumulatedImpulses<Contact2NonconvexAccumulatedImpulses>
    {
        public NonconvexAccumulatedImpulses Contact0;
        public NonconvexAccumulatedImpulses Contact1;
        public readonly int ContactCount => 2;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexAccumulatedImpulses GetImpulsesForContact(ref Contact2NonconvexAccumulatedImpulses impulses, int index)
        {
            return ref Unsafe.Add(ref impulses.Contact0, index);
        }
    }
        
    /// <summary>
    /// Handles the solve iterations of a bunch of 2-contact nonconvex two body manifold constraints.
    /// </summary>
    public class Contact2NonconvexTypeProcessor :
        TwoBodyContactTypeProcessor<Contact2NonconvexPrestepData, Contact2NonconvexAccumulatedImpulses,
            ContactNonconvexTwoBodyFunctions<Contact2NonconvexPrestepData, Contact2NonconvexAccumulatedImpulses>>
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

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact2NonconvexOneBody description)
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

        public readonly int ContactCount => 2;

        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact2NonconvexOneBodyTypeProcessor.BatchTypeId;
        }

        public readonly Type TypeProcessorType => typeof(Contact2NonconvexOneBodyTypeProcessor);

    }

    public struct Contact2NonconvexOneBodyPrestepData : INonconvexContactPrestep<Contact2NonconvexOneBodyPrestepData>    
    {
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public MaterialPropertiesWide MaterialProperties;
        public NonconvexContactPrestepData Contact0;
        public NonconvexContactPrestepData Contact1;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref MaterialPropertiesWide GetMaterialProperties(ref Contact2NonconvexOneBodyPrestepData prestep)
        {
            return ref prestep.MaterialProperties;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexContactPrestepData GetContact(ref Contact2NonconvexOneBodyPrestepData prestep, int index)
        {
            return ref Unsafe.Add(ref prestep.Contact0, index);
        }
                
        public readonly int ContactCount => 2;
        public readonly int BodyCount => 1;
    }    

    /// <summary>
    /// Handles the solve iterations of a bunch of 2-contact nonconvex one body manifold constraints.
    /// </summary>
    public class Contact2NonconvexOneBodyTypeProcessor :
        OneBodyContactTypeProcessor<Contact2NonconvexOneBodyPrestepData, Contact2NonconvexAccumulatedImpulses,
            ContactNonconvexOneBodyFunctions<Contact2NonconvexOneBodyPrestepData, Contact2NonconvexAccumulatedImpulses>>
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

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact3Nonconvex description)
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

        public readonly int ContactCount => 3;

        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact3NonconvexTypeProcessor.BatchTypeId;
        }

        public readonly Type TypeProcessorType => typeof(Contact3NonconvexTypeProcessor);

    }

    public struct Contact3NonconvexPrestepData : ITwoBodyNonconvexContactPrestep<Contact3NonconvexPrestepData>
    {
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public MaterialPropertiesWide MaterialProperties;
        public Vector3Wide OffsetB;
        public NonconvexContactPrestepData Contact0;
        public NonconvexContactPrestepData Contact1;
        public NonconvexContactPrestepData Contact2;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref MaterialPropertiesWide GetMaterialProperties(ref Contact3NonconvexPrestepData prestep)
        {
            return ref prestep.MaterialProperties;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector3Wide GetOffsetB(ref Contact3NonconvexPrestepData prestep)
        {
            return ref prestep.OffsetB;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexContactPrestepData GetContact(ref Contact3NonconvexPrestepData prestep, int index)
        {
            return ref Unsafe.Add(ref prestep.Contact0, index);
        }
        
        public readonly int ContactCount => 3;
        public readonly int BodyCount => 2;
    }

    public struct Contact3NonconvexAccumulatedImpulses : INonconvexContactAccumulatedImpulses<Contact3NonconvexAccumulatedImpulses>
    {
        public NonconvexAccumulatedImpulses Contact0;
        public NonconvexAccumulatedImpulses Contact1;
        public NonconvexAccumulatedImpulses Contact2;
        public readonly int ContactCount => 3;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexAccumulatedImpulses GetImpulsesForContact(ref Contact3NonconvexAccumulatedImpulses impulses, int index)
        {
            return ref Unsafe.Add(ref impulses.Contact0, index);
        }
    }
        
    /// <summary>
    /// Handles the solve iterations of a bunch of 3-contact nonconvex two body manifold constraints.
    /// </summary>
    public class Contact3NonconvexTypeProcessor :
        TwoBodyContactTypeProcessor<Contact3NonconvexPrestepData, Contact3NonconvexAccumulatedImpulses,
            ContactNonconvexTwoBodyFunctions<Contact3NonconvexPrestepData, Contact3NonconvexAccumulatedImpulses>>
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

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact3NonconvexOneBody description)
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

        public readonly int ContactCount => 3;

        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact3NonconvexOneBodyTypeProcessor.BatchTypeId;
        }

        public readonly Type TypeProcessorType => typeof(Contact3NonconvexOneBodyTypeProcessor);

    }

    public struct Contact3NonconvexOneBodyPrestepData : INonconvexContactPrestep<Contact3NonconvexOneBodyPrestepData>    
    {
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public MaterialPropertiesWide MaterialProperties;
        public NonconvexContactPrestepData Contact0;
        public NonconvexContactPrestepData Contact1;
        public NonconvexContactPrestepData Contact2;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref MaterialPropertiesWide GetMaterialProperties(ref Contact3NonconvexOneBodyPrestepData prestep)
        {
            return ref prestep.MaterialProperties;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexContactPrestepData GetContact(ref Contact3NonconvexOneBodyPrestepData prestep, int index)
        {
            return ref Unsafe.Add(ref prestep.Contact0, index);
        }
                
        public readonly int ContactCount => 3;
        public readonly int BodyCount => 1;
    }    

    /// <summary>
    /// Handles the solve iterations of a bunch of 3-contact nonconvex one body manifold constraints.
    /// </summary>
    public class Contact3NonconvexOneBodyTypeProcessor :
        OneBodyContactTypeProcessor<Contact3NonconvexOneBodyPrestepData, Contact3NonconvexAccumulatedImpulses,
            ContactNonconvexOneBodyFunctions<Contact3NonconvexOneBodyPrestepData, Contact3NonconvexAccumulatedImpulses>>
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

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact4Nonconvex description)
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

        public readonly int ContactCount => 4;

        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact4NonconvexTypeProcessor.BatchTypeId;
        }

        public readonly Type TypeProcessorType => typeof(Contact4NonconvexTypeProcessor);

    }

    public struct Contact4NonconvexPrestepData : ITwoBodyNonconvexContactPrestep<Contact4NonconvexPrestepData>
    {
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public MaterialPropertiesWide MaterialProperties;
        public Vector3Wide OffsetB;
        public NonconvexContactPrestepData Contact0;
        public NonconvexContactPrestepData Contact1;
        public NonconvexContactPrestepData Contact2;
        public NonconvexContactPrestepData Contact3;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref MaterialPropertiesWide GetMaterialProperties(ref Contact4NonconvexPrestepData prestep)
        {
            return ref prestep.MaterialProperties;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref Vector3Wide GetOffsetB(ref Contact4NonconvexPrestepData prestep)
        {
            return ref prestep.OffsetB;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexContactPrestepData GetContact(ref Contact4NonconvexPrestepData prestep, int index)
        {
            return ref Unsafe.Add(ref prestep.Contact0, index);
        }
        
        public readonly int ContactCount => 4;
        public readonly int BodyCount => 2;
    }

    public struct Contact4NonconvexAccumulatedImpulses : INonconvexContactAccumulatedImpulses<Contact4NonconvexAccumulatedImpulses>
    {
        public NonconvexAccumulatedImpulses Contact0;
        public NonconvexAccumulatedImpulses Contact1;
        public NonconvexAccumulatedImpulses Contact2;
        public NonconvexAccumulatedImpulses Contact3;
        public readonly int ContactCount => 4;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexAccumulatedImpulses GetImpulsesForContact(ref Contact4NonconvexAccumulatedImpulses impulses, int index)
        {
            return ref Unsafe.Add(ref impulses.Contact0, index);
        }
    }
        
    /// <summary>
    /// Handles the solve iterations of a bunch of 4-contact nonconvex two body manifold constraints.
    /// </summary>
    public class Contact4NonconvexTypeProcessor :
        TwoBodyContactTypeProcessor<Contact4NonconvexPrestepData, Contact4NonconvexAccumulatedImpulses,
            ContactNonconvexTwoBodyFunctions<Contact4NonconvexPrestepData, Contact4NonconvexAccumulatedImpulses>>
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

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Contact4NonconvexOneBody description)
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

        public readonly int ContactCount => 4;

        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Contact4NonconvexOneBodyTypeProcessor.BatchTypeId;
        }

        public readonly Type TypeProcessorType => typeof(Contact4NonconvexOneBodyTypeProcessor);

    }

    public struct Contact4NonconvexOneBodyPrestepData : INonconvexContactPrestep<Contact4NonconvexOneBodyPrestepData>    
    {
        //Note that this layout is defined by the execution order in the prestep. The function accesses it sequentially to ensure the prefetcher can do its job.
        public MaterialPropertiesWide MaterialProperties;
        public NonconvexContactPrestepData Contact0;
        public NonconvexContactPrestepData Contact1;
        public NonconvexContactPrestepData Contact2;
        public NonconvexContactPrestepData Contact3;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref MaterialPropertiesWide GetMaterialProperties(ref Contact4NonconvexOneBodyPrestepData prestep)
        {
            return ref prestep.MaterialProperties;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref NonconvexContactPrestepData GetContact(ref Contact4NonconvexOneBodyPrestepData prestep, int index)
        {
            return ref Unsafe.Add(ref prestep.Contact0, index);
        }
                
        public readonly int ContactCount => 4;
        public readonly int BodyCount => 1;
    }    

    /// <summary>
    /// Handles the solve iterations of a bunch of 4-contact nonconvex one body manifold constraints.
    /// </summary>
    public class Contact4NonconvexOneBodyTypeProcessor :
        OneBodyContactTypeProcessor<Contact4NonconvexOneBodyPrestepData, Contact4NonconvexAccumulatedImpulses,
            ContactNonconvexOneBodyFunctions<Contact4NonconvexOneBodyPrestepData, Contact4NonconvexAccumulatedImpulses>>
    {
        //Matches UpdateConstraintForManifold's manifoldTypeAsConstraintType computation.
        public const int BatchTypeId = 10;
    }


}
