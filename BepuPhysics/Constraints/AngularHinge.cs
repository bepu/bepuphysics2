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
        public Vector3 HingeAxisLocalA;
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
            GetFirst(ref target.HingeAxisLocalA.X) = HingeAxisLocalA.X;
            GetFirst(ref target.HingeAxisLocalA.Y) = HingeAxisLocalA.Y;
            GetFirst(ref target.HingeAxisLocalA.Z) = HingeAxisLocalA.Z;
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
            description.HingeAxisLocalA.X = GetFirst(ref source.HingeAxisLocalA.X);
            description.HingeAxisLocalA.Y = GetFirst(ref source.HingeAxisLocalA.Y);
            description.HingeAxisLocalA.Z = GetFirst(ref source.HingeAxisLocalA.Z);
            description.HingeAxisLocalB.X = GetFirst(ref source.HingeAxisLocalB.X);
            description.HingeAxisLocalB.Y = GetFirst(ref source.HingeAxisLocalB.Y);
            description.HingeAxisLocalB.Z = GetFirst(ref source.HingeAxisLocalB.Z);
            description.SpringSettings.NaturalFrequency = GetFirst(ref source.SpringSettings.NaturalFrequency);
            description.SpringSettings.DampingRatio = GetFirst(ref source.SpringSettings.DampingRatio);
        }
    }

    public struct AngularHingePrestepData
    {
        public Vector3Wide HingeAxisLocalA;
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
            //Note that we build the basis in local space first. This helps keep the basis consistent during rotation.
            Helpers.BuildOrthnormalBasis(ref prestep.HingeAxisLocalA, out var localAX, out var localAY);
            Matrix3x3Wide.CreateFromQuaternion(ref orientationA, out var orientationMatrixA);
            Matrix3x3Wide.TransformWithoutOverlap(ref localAX, ref orientationMatrixA, out var constrainedAxisX);
            Matrix3x3Wide.TransformWithoutOverlap(ref localAY, ref orientationMatrixA, out var constrainedAxisY);
            QuaternionWide.TransformWithoutOverlap(ref prestep.HingeAxisLocalB, ref orientationB, out var hingeAxis);
            Matrix2x3Wide jacobianA;
            Vector3Wide.CrossWithoutOverlap(ref constrainedAxisX, ref hingeAxis, out jacobianA.X);
            Vector3Wide.CrossWithoutOverlap(ref constrainedAxisY, ref hingeAxis, out jacobianA.Y);
            //If hingeB is on the constraint axis plane, the jacobians lose independence. Arbitrarily use fallback axes.
            Vector3Wide.CrossWithoutOverlap(ref jacobianA.X, ref jacobianA.Y, out var crossTest);
            //Note that this causes a discontinuity in jacobian length. We just don't worry about it.
            Vector3Wide.Dot(ref crossTest, ref crossTest, out var crossTestLengthSquared);
            var useFallback = Vector.LessThan(crossTestLengthSquared, new Vector<float>(1e-10f));
            Vector3Wide.ConditionalSelect(ref useFallback, ref constrainedAxisY, ref jacobianA.X, out jacobianA.X);
            Vector3Wide.ConditionalSelect(ref useFallback, ref constrainedAxisX, ref jacobianA.Y, out jacobianA.Y);

            ////ALTERNATIVE FORMULATION
            ////C = atan2(dot(constraintAxisAX, hingeAxisB), dot(hingeAxisA, hingeAxisB)) = 0
            ////JAX = ((constraintAxisAX * hingeAxisB) * (hingeAxisA x hingeAxisB) - (hingeAxisA * hingeAxisB) * (constraintAxisAX x hingeAxisB)) * denom
            ////denom = 1 / ((hingeAxisA * hingeAxisB)^2 + (constraintAxisAX * hingeAxisB)^2)
            //Vector3Wide.CrossWithoutOverlap(ref constrainedAxisX, ref constrainedAxisY, out var hingeAxisA);
            //Vector3Wide.Dot(ref hingeAxisA, ref hingeAxis, out var hahb);
            //Vector3Wide.Dot(ref constrainedAxisX, ref hingeAxis, out var caxhb);
            //var denomX = Vector<float>.One / (hahb * hahb + caxhb * caxhb);
            //Vector3Wide.CrossWithoutOverlap(ref hingeAxisA, ref hingeAxis, out var haxhb);
            //Vector3Wide.CrossWithoutOverlap(ref constrainedAxisX, ref hingeAxis, out var caxxhb);
            //Vector3Wide.Scale(ref haxhb, ref caxhb, out var leftX);
            //Vector3Wide.Scale(ref caxxhb, ref hahb, out var rightX);
            //Vector3Wide.Subtract(ref leftX, ref rightX, out var numeratorX);
            //Vector3Wide.Scale(ref numeratorX, ref denomX, out var alternativeJacobianAX);
            
            //Vector3Wide.Dot(ref constrainedAxisY, ref hingeAxis, out var cayhb);
            //var denomY = Vector<float>.One / (hahb * hahb + cayhb * cayhb);
            //Vector3Wide.CrossWithoutOverlap(ref constrainedAxisY, ref hingeAxis, out var cayxhb);
            //Vector3Wide.Scale(ref haxhb, ref cayhb, out var leftY);
            //Vector3Wide.Scale(ref cayxhb, ref hahb, out var rightY);
            //Vector3Wide.Subtract(ref leftY, ref rightY, out var numeratorY);
            //Vector3Wide.Scale(ref numeratorY, ref denomY, out var alternativeJacobianAY);


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

