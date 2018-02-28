using BepuPhysics.CollisionDetection;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using static BepuPhysics.GatherScatter;
namespace BepuPhysics.Constraints
{
    public struct AngularHinge : IConstraintDescription<AngularHinge>
    {
        public Vector3 ConstrainedAxisXLocalA;
        public Vector3 ConstrainedAxisYLocalA;
        public Vector3 HingeAxisLocalB;
        public SpringSettings SpringSettings;

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return AngularHingeTypeProcessor.BatchTypeId;
            }
        }

        public Type BatchType => typeof(AngularHingeTypeProcessor);

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<AngularHingePrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            GetFirst(ref target.ConstrainedAxisXLocalA.X) = ConstrainedAxisXLocalA.X;
            GetFirst(ref target.ConstrainedAxisXLocalA.Y) = ConstrainedAxisXLocalA.Y;
            GetFirst(ref target.ConstrainedAxisXLocalA.Z) = ConstrainedAxisXLocalA.Z;
            GetFirst(ref target.ConstrainedAxisYLocalA.X) = ConstrainedAxisYLocalA.X;
            GetFirst(ref target.ConstrainedAxisYLocalA.Y) = ConstrainedAxisYLocalA.Y;
            GetFirst(ref target.ConstrainedAxisYLocalA.Z) = ConstrainedAxisYLocalA.Z;
            GetFirst(ref target.HingeAxisLocalB.X) = HingeAxisLocalB.X;
            GetFirst(ref target.HingeAxisLocalB.Y) = HingeAxisLocalB.Y;
            GetFirst(ref target.HingeAxisLocalB.Z) = HingeAxisLocalB.Z;
            GetFirst(ref target.SpringSettings.NaturalFrequency) = SpringSettings.NaturalFrequency;
            GetFirst(ref target.SpringSettings.DampingRatio) = SpringSettings.DampingRatio;
        }

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out AngularHinge description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<AngularHingePrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            description.ConstrainedAxisXLocalA.X = GetFirst(ref source.ConstrainedAxisXLocalA.X);
            description.ConstrainedAxisXLocalA.Y = GetFirst(ref source.ConstrainedAxisXLocalA.Y);
            description.ConstrainedAxisXLocalA.Z = GetFirst(ref source.ConstrainedAxisXLocalA.Z);
            description.ConstrainedAxisYLocalA.X = GetFirst(ref source.ConstrainedAxisYLocalA.X);
            description.ConstrainedAxisYLocalA.Y = GetFirst(ref source.ConstrainedAxisYLocalA.Y);
            description.ConstrainedAxisYLocalA.Z = GetFirst(ref source.ConstrainedAxisYLocalA.Z);
            description.HingeAxisLocalB.X = GetFirst(ref source.HingeAxisLocalB.X);
            description.HingeAxisLocalB.Y = GetFirst(ref source.HingeAxisLocalB.Y);
            description.HingeAxisLocalB.Z = GetFirst(ref source.HingeAxisLocalB.Z);
            description.SpringSettings.NaturalFrequency = GetFirst(ref source.SpringSettings.NaturalFrequency);
            description.SpringSettings.DampingRatio = GetFirst(ref source.SpringSettings.DampingRatio);
        }
    }

    public struct AngularHingePrestepData
    {
        public Vector3Wide ConstrainedAxisXLocalA;
        public Vector3Wide ConstrainedAxisYLocalA;
        public Vector3Wide HingeAxisLocalB;
        public SpringSettingsWide SpringSettings;
    }

    public struct AngularHingeProjection
    {
        //JacobianB = -JacobianA, so no need to store it explicitly.
        public Matrix2x3Wide VelocityToImpulseA;
        public Vector2Wide BiasImpulse;
        public Vector<float> SoftnessImpulseScale;
        public Matrix2x3Wide ImpulseToVelocityA;
        public Matrix2x3Wide NegatedImpulseToVelocityB;
    }

    public struct AngularHingeFunctions : IConstraintFunctions<AngularHingePrestepData, AngularHingeProjection, Vector2Wide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref TwoBodyReferences bodyReferences, int count, float dt, float inverseDt, ref AngularHingePrestepData prestep,
            out AngularHingeProjection projection)
        {
            bodies.GatherInertiaAndPose(ref bodyReferences, count,
                out var localPositionB, out var orientationA, out var orientationB,
                out var inertiaA, out var inertiaB);

            //The free rotation axis attached to B is stopped from leaning in the direction of ConstrainedAxisX or ConstrainedAxisY attached to A, yielding a 2DOF constraint:
            //C = [dot(hingeAxisB, constrainedAX)] = [0]
            //    [dot(hingeAxisB, constrainedAY)]   [0]
            //C' = [dot(d/dt(hingeAxisB), constrainedAX) + dot(hingeAxisB, d/dt(constrainedAX))] = [0]
            //     [dot(d/dt(hingeAxisB), constrainedAY) + dot(hingeAxisB, d/dt(constrainedAY))]   [0]
            //C' = [dot(angularVelocityB x hingeAxisB, constrainedAX) + dot(hingeAxisB, angularVelocityA x constrainedAX)] = [0]
            //     [dot(angularVelocityB x hingeAxisB, constrainedAY) + dot(hingeAxisB, angularVelocityA x constrainedAY)]   [0]
            //C' = [dot(hingeAxisB x constrainedAX, angularVelocityB) + dot(angularVelocityA, constrainedAX x hingeAxisB)] = [0]
            //     [dot(hingeAxisB x constrainedAY, angularVelocityB) + dot(angularVelocityA, constrainedAY x hingeAxisB)]   [0]
            //Providing jacobians of:
            //JA = [constrainedAxisAX x hingeAxisB]
            //     [constrainedAxisAY x hingeAxisB]
            //JB = [hingeAxisB x constrainedAxisAX]
            //     [hingeAxisB x constrainedAxisAY]
            //a x b == -b x a, so JB == -JA.

            //Now, we choose the storage representation. The default approach would be to store JA, the effective mass, and both inverse inertias, requiring 6 + 4 + 6 + 6 scalars.  
            //The alternative is to store JAT * effectiveMass, and then also JA * inverseInertiaTensor(A/B), requiring only 6 + 6 + 6 scalars.
            //So, overall, prebaking saves us 4 scalars and a bit of iteration-time ALU.
            Matrix3x3Wide.CreateFromQuaternion(ref orientationA, out var orientationMatrixA);
            Matrix3x3Wide.TransformWithoutOverlap(ref prestep.ConstrainedAxisXLocalA, ref orientationMatrixA, out var constrainedAxisX);
            Matrix3x3Wide.TransformWithoutOverlap(ref prestep.ConstrainedAxisYLocalA, ref orientationMatrixA, out var constrainedAxisY);
            QuaternionWide.TransformWithoutOverlap(ref prestep.HingeAxisLocalB, ref orientationB, out var hingeAxis);
            Matrix2x3Wide jacobianA;
            Vector3Wide.CrossWithoutOverlap(ref constrainedAxisX, ref hingeAxis, out jacobianA.X);
            Vector3Wide.CrossWithoutOverlap(ref constrainedAxisY, ref hingeAxis, out jacobianA.Y);

            //Note that JA = -JB, but for the purposes of calculating the effective mass the sign is irrelevant.

            //This computes the effective mass using the usual (J * M^-1 * JT)^-1 formulation, but we actually make use of the intermediate result J * M^-1 so we compute it directly.
            Triangular3x3Wide.MultiplyBySymmetricWithoutOverlap(ref jacobianA, ref inertiaA.InverseInertiaTensor, out projection.ImpulseToVelocityA);
            //Note that we don't use -jacobianA here, so we're actually storing out the negated version of the transform. That's fine; we'll simply subtract in the iteration.
            Triangular3x3Wide.MultiplyBySymmetricWithoutOverlap(ref jacobianA, ref inertiaB.InverseInertiaTensor, out projection.NegatedImpulseToVelocityB);
            Triangular2x2Wide.CompleteMatrixSandwich(ref projection.ImpulseToVelocityA, ref jacobianA, out var angularA);
            Triangular2x2Wide.CompleteMatrixSandwich(ref projection.NegatedImpulseToVelocityB, ref jacobianA, out var angularB);
            Triangular2x2Wide.Add(ref angularA, ref angularB, out var inverseEffectiveMass);
            Triangular2x2Wide.InvertSymmetricWithoutOverlap(ref inverseEffectiveMass, out var effectiveMass);


            Springiness.ComputeSpringiness(ref prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);
            Triangular2x2Wide.Scale(ref effectiveMass, ref effectiveMassCFMScale, out effectiveMass);
            Triangular2x2Wide.MultiplyTransposedBySymmetric(ref jacobianA, ref effectiveMass, out projection.VelocityToImpulseA);

            //Compute the position error and bias velocities.
            Vector2Wide error;
            Vector3Wide.Dot(ref hingeAxis, ref constrainedAxisX, out error.X);
            Vector3Wide.Dot(ref hingeAxis, ref constrainedAxisY, out error.Y);
            //Note the negation: we want to oppose the separation. TODO: arguably, should bake the negation into positionErrorToVelocity, given its name.
            positionErrorToVelocity = -positionErrorToVelocity;
            Vector2Wide.Scale(ref error, ref positionErrorToVelocity, out var biasVelocity);
            Triangular2x2Wide.TransformBySymmetricWithoutOverlap(ref biasVelocity, ref effectiveMass, out projection.BiasImpulse);

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ApplyImpulse(ref Vector3Wide angularVelocityA, ref Vector3Wide angularVelocityB, ref AngularHingeProjection projection, ref Vector2Wide csi)
        {
            Matrix2x3Wide.Transform(ref csi, ref projection.ImpulseToVelocityA, out var velocityChangeA);
            Vector3Wide.Add(ref angularVelocityA, ref velocityChangeA, out angularVelocityA);
            Matrix2x3Wide.Transform(ref csi, ref projection.NegatedImpulseToVelocityB, out var negatedVelocityChangeB);
            Vector3Wide.Subtract(ref angularVelocityB, ref negatedVelocityChangeB, out angularVelocityB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WarmStart(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref AngularHingeProjection projection, ref Vector2Wide accumulatedImpulse)
        {
            ApplyImpulse(ref velocityA.Angular, ref velocityB.Angular, ref projection, ref accumulatedImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Solve(ref BodyVelocities velocityA, ref BodyVelocities velocityB, ref AngularHingeProjection projection, ref Vector2Wide accumulatedImpulse)
        {
            //JB = -JA. This is (angularVelocityA * JA + angularVelocityB * JB) * effectiveMass => (angularVelocityA - angularVelocityB) * (JA * effectiveMass)
            Vector3Wide.Subtract(ref velocityA.Angular, ref velocityB.Angular, out var difference);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(ref difference, ref projection.VelocityToImpulseA, out var csi);
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            Vector2Wide.Scale(ref accumulatedImpulse, ref projection.SoftnessImpulseScale, out var softnessContribution);
            Vector2Wide.Add(ref softnessContribution, ref csi, out csi);
            Vector2Wide.Subtract(ref projection.BiasImpulse, ref csi, out csi);

            Vector2Wide.Add(ref accumulatedImpulse, ref csi, out accumulatedImpulse);

            ApplyImpulse(ref velocityA.Angular, ref velocityB.Angular, ref projection, ref csi);
        }

    }

    public class AngularHingeTypeProcessor : TwoBodyTypeProcessor<AngularHingePrestepData, AngularHingeProjection, Vector2Wide, AngularHingeFunctions>
    {
        public const int BatchTypeId = 17;
    }
}

