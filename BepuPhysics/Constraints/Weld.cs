using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using static BepuUtilities.QuaternionWide;
using static BepuUtilities.Vector3Wide;
using static BepuUtilities.Symmetric3x3Wide;
using static BepuUtilities.Matrix3x3Wide;
using static BepuUtilities.GatherScatter;


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

        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return WeldTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(WeldTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
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

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out Weld description)
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
        public void Prestep(in QuaternionWide orientationA, in BodyInertias inertiaA, in Vector3Wide ab, in QuaternionWide orientationB, in BodyInertias inertiaB, float dt, float inverseDt,
            ref WeldPrestepData prestep, out WeldProjection projection)
        {
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
            Vector3Wide.Subtract(ab, projection.Offset, out var positionError);
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

            Vector3Wide.CrossWithoutOverlap(velocityA.Angular, projection.Offset, out var offsetAngularCSV);
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


        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ApplyImpulse(in BodyInertias inertiaA, in BodyInertias inertiaB, in Vector3Wide offset, in Vector3Wide orientationCSI, in Vector3Wide offsetCSI, ref BodyVelocities velocityA, ref BodyVelocities velocityB)
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
            velocityA.Linear += offsetCSI * inertiaA.InverseMass;

            //Note order of cross relative to the SolveIteration. 
            //SolveIteration transforms velocity into constraint space velocity using JT, while this converts constraint space to world space using J.
            //The elements are transposed, and transposed skew symmetric matrices are negated. Flipping the cross product is equivalent to a negation.
            velocityA.Angular += (Cross(offset, offsetCSI) + orientationCSI) * inertiaA.InverseInertiaTensor;

            velocityB.Linear -= offsetCSI * inertiaB.InverseMass;//note subtraction; the jacobian is -I
            velocityB.Angular -= orientationCSI * inertiaB.InverseInertiaTensor;//note subtraction; the jacobian is -I     
        }

        //[MethodImpl(MethodImplOptions.NoInlining)]
        public void WarmStart2(in QuaternionWide orientationA, in BodyInertias inertiaA, in Vector3Wide ab, in QuaternionWide orientationB, in BodyInertias inertiaB, float dt, float inverseDt,
            in WeldPrestepData prestep, in WeldAccumulatedImpulses accumulatedImpulses, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
            Transform(prestep.LocalOffset, orientationA, out var offset);
            ApplyImpulse(inertiaA, inertiaB, offset, accumulatedImpulses.Orientation, accumulatedImpulses.Offset, ref wsvA, ref wsvB);
            //var offset = prestep.LocalOffset * orientationA;
            //ApplyImpulse(inertiaA, inertiaB, prestep.LocalOffset * orientationA, accumulatedImpulses.Orientation, accumulatedImpulses.Offset, ref wsvA, ref wsvB);
        }
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve2(in QuaternionWide orientationA, in BodyInertias inertiaA, in Vector3Wide ab, in QuaternionWide orientationB, in BodyInertias inertiaB, float dt, float inverseDt,
            in WeldPrestepData prestep, ref WeldAccumulatedImpulses accumulatedImpulses, ref BodyVelocities wsvA, ref BodyVelocities wsvB)
        {
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

            //Compute the position error and bias velocities. Note the order of subtraction when calculating error- we want the bias velocity to counteract the separation.
            //var offset = prestep.LocalOffset * orientationA;
            QuaternionWide.Transform(prestep.LocalOffset, orientationA, out var offset);
            var positionError = ab - offset;
            var targetOrientationB = prestep.LocalOrientation * orientationA;
            //ConcatenateWithoutOverlap(prestep.LocalOrientation, orientationA, out var targetOrientationB);
            ConcatenateWithoutOverlap(Conjugate(targetOrientationB), orientationB, out var rotationError);
            GetApproximateAxisAngleFromQuaternion(rotationError, out var rotationErrorAxis, out var rotationErrorLength);

            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);
            var offsetBiasVelocity = positionError * positionErrorToVelocity;
            var orientationBiasVelocity = rotationErrorAxis * (rotationErrorLength * positionErrorToVelocity);

            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            //csi = -accumulatedImpulse * projection.SoftnessImpulseScale - (-biasVelocity + csvaLinear + csvaAngular + csvbLinear + csvbAngular) * effectiveMass;
            //csi = (biasVelocity - csvaLinear - csvaAngular - csvbLinear - csvbAngular) * effectiveMass - accumulatedImpulse * projection.SoftnessImpulseScale;
            //csv = V * JT 
            var orientationCSV = orientationBiasVelocity - (wsvA.Angular - wsvB.Angular);
            var offsetCSV = offsetBiasVelocity - (wsvA.Linear - wsvB.Linear + Cross(wsvA.Angular, offset));

            //Effective mass = (J * M^-1 * JT)^-1, which is going to be a little tricky because J * M^-1 * JT is a 6x6 matrix:
            //J * M^-1 * JT = [ Ia^-1 + Ib^-1,                                     Ia^-1 * transpose(skewSymmetric(localOffset * orientationA))                                                             ]
            //                [ skewSymmetric(localOffset * orientationA) * Ia^-1, Ma^-1 + Mb^-1 + skewSymmetric(localOffset * orientationA) * Ia^-1 * transpose(skewSymmetric(localOffset * orientationA)) ]
            //where Ia^-1 and Ib^-1 are the inverse inertia tensors for a and b and Ma^-1 and Mb^-1 are the inverse masses of A and B expanded to 3x3 diagonal matrices.
            //var jmjtA = inertiaA.InverseInertiaTensor + inertiaB.InverseInertiaTensor;
            Symmetric3x3Wide.Add(inertiaA.InverseInertiaTensor, inertiaB.InverseInertiaTensor, out var jmjtA);
            CreateCrossProduct(offset, out var xAB);
            Multiply(inertiaA.InverseInertiaTensor, xAB, out var jmjtB);
            //var jmjtB = inertiaA.InverseInertiaTensor * xAB;
            CompleteMatrixSandwichTranspose(xAB, jmjtB, out var jmjtD);
            var diagonalAdd = inertiaA.InverseMass + inertiaB.InverseMass;
            jmjtD.XX += diagonalAdd;
            jmjtD.YY += diagonalAdd;
            jmjtD.ZZ += diagonalAdd;
            //Note that there is no need to invert the 6x6 inverse effective mass matrix chonk. We want to convert a constraint space velocity into a constraint space impulse, csi = csv * effectiveMass.
            //This is equivalent to solving csi * effectiveMass^-1 = csv for csi, and since effectiveMass^-1 is symmetric positive semidefinite, we can use an LDLT decomposition to quickly solve it.
            Symmetric6x6Wide.LDLTSolve(orientationCSV, offsetCSV, jmjtA, jmjtB, jmjtD, out var orientationCSI, out var offsetCSI);
            //Symmetric6x6Wide.Invert(jmjtA, jmjtB, jmjtD, out var testEffectiveMass);
            //Symmetric6x6Wide.TransformWithoutOverlap(orientationCSV, offsetCSV, testEffectiveMass, out var orientationCSI2, out var offsetCSI2);
            //Scale(orientationCSI, effectiveMassCFMScale, out orientationCSI);
            //Scale(accumulatedImpulses.Orientation, softnessImpulseScale, out var orientationSoftness);
            //Subtract(orientationCSI, orientationSoftness, out orientationCSI);
            //Add(accumulatedImpulses.Orientation, orientationCSI, out accumulatedImpulses.Orientation);
            //Scale(offsetCSI, effectiveMassCFMScale, out offsetCSI);
            //Scale(accumulatedImpulses.Offset, softnessImpulseScale, out var offsetSoftness);
            //Subtract(offsetCSI, offsetSoftness, out offsetCSI);
            //Add(accumulatedImpulses.Offset, offsetCSI, out accumulatedImpulses.Offset);
            orientationCSI = orientationCSI * effectiveMassCFMScale - accumulatedImpulses.Orientation * softnessImpulseScale;
            offsetCSI = offsetCSI * effectiveMassCFMScale - accumulatedImpulses.Offset * softnessImpulseScale;
            accumulatedImpulses.Orientation += orientationCSI;
            accumulatedImpulses.Offset += offsetCSI;

            ApplyImpulse(inertiaA, inertiaB, ab, orientationCSI, offsetCSI, ref wsvA, ref wsvB);
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

