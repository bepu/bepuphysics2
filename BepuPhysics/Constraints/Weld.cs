using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using static BepuUtilities.GatherScatter;
using Quaternion = BepuUtilities.Quaternion;
namespace BepuPhysics.Constraints
{
    /// <summary>
    /// Constrains two bodies to maintain a relative position and orientation. All six degrees of freedom are solved simultaneously.
    /// </summary>
    public struct Weld : ITwoBodyConstraintDescription<Weld>
    {
        /// <summary>
        /// Offset from body A to body B in the local space of A.
        /// </summary>
        public Vector3 LocalOffset;
        /// <summary>
        /// Target orientation of body B in body A's local space. 
        /// </summary>
        public Quaternion LocalOrientation;

        /// <summary>
        /// Springiness of the position and orientation constraints.
        /// </summary>
        public SpringSettings SpringSettings;

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return WeldTypeProcessor.BatchTypeId;
            }
        }

        public Type TypeProcessorType => typeof(WeldTypeProcessor);

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertUnitLength(LocalOrientation, nameof(Weld), nameof(LocalOrientation));
            ConstraintChecker.AssertValid(SpringSettings, nameof(Weld));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<WeldPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(LocalOffset, ref target.LocalOffset);
            QuaternionWide.WriteFirst(LocalOrientation, ref target.LocalOrientation);
            SpringSettingsWide.WriteFirst(SpringSettings, ref target.SpringSettings);
            GetFirst(ref target.SpringSettings.AngularFrequency) = SpringSettings.AngularFrequency;
            GetFirst(ref target.SpringSettings.TwiceDampingRatio) = SpringSettings.TwiceDampingRatio;
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Weld description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<WeldPrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.LocalOffset, out description.LocalOffset);
            QuaternionWide.ReadFirst(source.LocalOrientation, out description.LocalOrientation);
            SpringSettingsWide.ReadFirst(source.SpringSettings, out description.SpringSettings);
        }
    }

    public struct WeldPrestepData
    {
        public Vector3Wide LocalOffset;
        public QuaternionWide LocalOrientation;
        public SpringSettingsWide SpringSettings;
    }

    public struct WeldProjection
    {
        public Vector3Wide Offset;
        public Vector3Wide OffsetBiasVelocity;
        public Vector3Wide OrientationBiasVelocity;
        public Symmetric6x6Wide EffectiveMass;
        public Vector<float> SoftnessImpulseScale;
        public BodyInertias InertiaA;
        public BodyInertias InertiaB;
    }

    public struct WeldAccumulatedImpulses
    {
        public Vector3Wide Orientation;
        public Vector3Wide Offset;
    }

    public struct WeldFunctions : IConstraintFunctions<WeldPrestepData, WeldProjection, WeldAccumulatedImpulses>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref TwoBodyReferences bodyReferences, int count, float dt, float inverseDt, ref BodyInertias inertiaA, ref BodyInertias inertiaB,
            ref WeldPrestepData prestep, out WeldProjection projection)
        {
            bodies.GatherPose(ref bodyReferences, count, out var localPositionB, out var orientationA, out var orientationB);
            projection.InertiaA = inertiaA;
            projection.InertiaB = inertiaB;

            //The weld constraint handles 6 degrees of freedom simultaneously. The constraints are:
            //localOrientation * orientationA = orientationB
            //positionA + localOffset * orientationA = positionB
            //The velocity derivatives:
            //angularVelocityA = angularVelocityB
            //linearVelocityA + angularVelocityA x (localOffset * orientationA) = linearVelocityB
            //Note that the position constraint is similar a ball socket joint, except the anchor point is on top of the center of mass of object B.

            //From the above, the jacobians ordered as [linearA, angularA, linearB, angularB] are:
            //J = [ 0, I,                                          0, -I ]
            //    [ I, skewSymmetric(localOffset * orientationA), -I,  0 ]
            //where I is the 3x3 identity matrix.
            //Effective mass = (J * M^-1 * JT)^-1, which is going to be a little tricky because J * M^-1 * JT is a 6x6 matrix:
            //J * M^-1 * JT = [ Ia^-1 + Ib^-1,                                     Ia^-1 * transpose(skewSymmetric(localOffset * orientationA))                                                             ]
            //                [ skewSymmetric(localOffset * orientationA) * Ia^-1, Ma^-1 + Mb^-1 + skewSymmetric(localOffset * orientationA) * Ia^-1 * transpose(skewSymmetric(localOffset * orientationA)) ]
            //where Ia^-1 and Ib^-1 are the inverse inertia tensors for a and b and Ma^-1 and Mb^-1 are the inverse masses of A and B expanded to 3x3 diagonal matrices.
            //It's worth noting that the effective mass- and its inverse- are symmetric, so we can cut down the inverse computations.
            Symmetric3x3Wide.Add(inertiaA.InverseInertiaTensor, inertiaB.InverseInertiaTensor, out var jmjtA);
            QuaternionWide.TransformWithoutOverlap(prestep.LocalOffset, orientationA, out projection.Offset);
            Matrix3x3Wide.CreateCrossProduct(projection.Offset, out var xAB);
            Symmetric3x3Wide.Multiply(inertiaA.InverseInertiaTensor, xAB, out var jmjtB);
            Symmetric3x3Wide.CompleteMatrixSandwichTranspose(xAB, jmjtB, out var jmjtD);
            var diagonalAdd = inertiaA.InverseMass + inertiaB.InverseMass;
            jmjtD.XX += diagonalAdd;
            jmjtD.YY += diagonalAdd;
            jmjtD.ZZ += diagonalAdd;
            Symmetric6x6Wide.Invert(jmjtA, jmjtB, jmjtD, out projection.EffectiveMass);

            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);
            Symmetric6x6Wide.Scale(projection.EffectiveMass, effectiveMassCFMScale, out projection.EffectiveMass);

            //Compute the current constraint error for all 6 degrees of freedom.
            //Compute the position error and bias velocities. Note the order of subtraction when calculating error- we want the bias velocity to counteract the separation.
            Vector3Wide.Subtract(localPositionB, projection.Offset, out var positionError);
            QuaternionWide.ConcatenateWithoutOverlap(prestep.LocalOrientation, orientationA, out var targetOrientationB);
            QuaternionWide.Conjugate(targetOrientationB, out var inverseTarget);
            QuaternionWide.ConcatenateWithoutOverlap(inverseTarget, orientationB, out var rotationError);
            QuaternionWide.GetApproximateAxisAngleFromQuaternion(rotationError, out var rotationErrorAxis, out var rotationErrorLength);

            Vector3Wide.Scale(positionError, positionErrorToVelocity, out projection.OffsetBiasVelocity);
            Vector3Wide.Scale(rotationErrorAxis, rotationErrorLength * positionErrorToVelocity, out projection.OrientationBiasVelocity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ApplyImpulse(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref WeldProjection projection, ref Vector3Wide orientationCSI, ref Vector3Wide offsetCSI)
        {
            //Recall the jacobians:
            //J = [ 0, I,                                          0, -I ]
            //    [ I, skewSymmetric(localOffset * orientationA), -I,  0 ]
            //The velocity changes are: 
            // csi * J * I^-1
            //linearImpulseA = offsetCSI
            //angularImpulseA = orientationCSI + worldOffset x offsetCSI
            //linearImpulseB = -offsetCSI
            //angularImpulseB = -orientationCSI
            Vector3Wide.Scale(offsetCSI, projection.InertiaA.InverseMass, out var linearChangeA);
            Vector3Wide.Add(velocityA.Linear, linearChangeA, out velocityA.Linear);

            //Note order of cross relative to the SolveIteration. 
            //SolveIteration transforms velocity into constraint space velocity using JT, while this converts constraint space to world space using J.
            //The elements are transposed, and transposed skew symmetric matrices are negated. Flipping the cross product is equivalent to a negation.
            Vector3Wide.CrossWithoutOverlap(projection.Offset, offsetCSI, out var offsetWorldImpulse);
            Vector3Wide.Add(offsetWorldImpulse, orientationCSI, out var angularImpulseA);
            Symmetric3x3Wide.TransformWithoutOverlap(angularImpulseA, projection.InertiaA.InverseInertiaTensor, out var angularChangeA);
            Vector3Wide.Add(velocityA.Angular, angularChangeA, out velocityA.Angular);

            Vector3Wide.Scale(offsetCSI, projection.InertiaB.InverseMass, out var negatedLinearChangeB);
            Vector3Wide.Subtract(velocityB.Linear, negatedLinearChangeB, out velocityB.Linear); //note subtraction; the jacobian is -I

            Symmetric3x3Wide.TransformWithoutOverlap(orientationCSI, projection.InertiaB.InverseInertiaTensor, out var negatedAngularChangeB);
            Vector3Wide.Subtract(velocityB.Angular, negatedAngularChangeB, out velocityB.Angular); //note subtraction; the jacobian is -I        
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref WeldProjection projection, ref WeldAccumulatedImpulses accumulatedImpulse)
        {
            ApplyImpulse(ref velocityA, ref velocityB, ref projection, ref accumulatedImpulse.Orientation, ref accumulatedImpulse.Offset);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref WeldProjection projection, ref WeldAccumulatedImpulses accumulatedImpulse)
        {
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            //csv = V * JT 
            Vector3Wide.Subtract(velocityA.Angular, velocityB.Angular, out var orientationCSV);
            Vector3Wide.Subtract(velocityA.Linear, velocityB.Linear, out var offsetCSV);

            Vector3Wide.CrossWithoutOverlap(velocityA.Angular, projection.Offset,  out var offsetAngularCSV);
            Vector3Wide.Add(offsetCSV, offsetAngularCSV, out offsetCSV);

            //Note subtraction: this is computing biasVelocity - csv, and later we'll compute (biasVelocity-csv) - softness.
            Vector3Wide.Subtract(projection.OrientationBiasVelocity, orientationCSV, out orientationCSV);
            Vector3Wide.Subtract(projection.OffsetBiasVelocity, offsetCSV, out offsetCSV);

            Symmetric6x6Wide.TransformWithoutOverlap(orientationCSV, offsetCSV, projection.EffectiveMass, out var orientationCSI, out var offsetCSI);
            Vector3Wide.Scale(accumulatedImpulse.Offset, projection.SoftnessImpulseScale, out var offsetSoftness);
            Vector3Wide.Scale(accumulatedImpulse.Orientation, projection.SoftnessImpulseScale, out var orientationSoftness);
            Vector3Wide.Subtract(offsetCSI, offsetSoftness, out offsetCSI);
            Vector3Wide.Subtract(orientationCSI, orientationSoftness, out orientationCSI);
            Vector3Wide.Add(accumulatedImpulse.Orientation, orientationCSI, out accumulatedImpulse.Orientation);
            Vector3Wide.Add(accumulatedImpulse.Offset, offsetCSI, out accumulatedImpulse.Offset);

            ApplyImpulse(ref velocityA, ref velocityB, ref projection, ref orientationCSI, ref offsetCSI);
        }

    }


    /// <summary>
    /// Handles the solve iterations of a bunch of ball socket constraints.
    /// </summary>
    public class WeldTypeProcessor : TwoBodyTypeProcessor<WeldPrestepData, WeldProjection, WeldAccumulatedImpulses, WeldFunctions>
    {
        public const int BatchTypeId = 31;
    }
}

