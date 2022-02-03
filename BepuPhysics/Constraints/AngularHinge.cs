using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using static BepuUtilities.GatherScatter;
namespace BepuPhysics.Constraints
{
    /// <summary>
    /// Angular component of a hinge. Constrains the angular degrees of freedom of two bodies such that they can only rotate relative to each other around the hinge's axis.
    /// </summary>
    public struct AngularHinge : ITwoBodyConstraintDescription<AngularHinge>
    {
        /// <summary>
        /// Hinge axis in the local space of A.
        /// </summary>
        public Vector3 LocalHingeAxisA;
        /// <summary>
        /// Hinge axis in the local space of B.
        /// </summary>
        public Vector3 LocalHingeAxisB;
        /// <summary>
        /// Spring frequency and damping parameters.
        /// </summary>
        public SpringSettings SpringSettings;

        public readonly int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return AngularHingeTypeProcessor.BatchTypeId;
            }
        }

        public readonly Type TypeProcessorType => typeof(AngularHingeTypeProcessor);

        public readonly void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
        {
            ConstraintChecker.AssertUnitLength(LocalHingeAxisA, nameof(AngularHinge), nameof(LocalHingeAxisA));
            ConstraintChecker.AssertUnitLength(LocalHingeAxisB, nameof(AngularHinge), nameof(LocalHingeAxisB));
            ConstraintChecker.AssertValid(SpringSettings, nameof(AngularHinge));
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var target = ref GetOffsetInstance(ref Buffer<AngularHingePrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.WriteFirst(LocalHingeAxisA, ref target.LocalHingeAxisA);
            Vector3Wide.WriteFirst(LocalHingeAxisB, ref target.LocalHingeAxisB);
            GetFirst(ref target.SpringSettings.AngularFrequency) = SpringSettings.AngularFrequency;
            GetFirst(ref target.SpringSettings.TwiceDampingRatio) = SpringSettings.TwiceDampingRatio;
        }

        public readonly void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out AngularHinge description)
        {
            Debug.Assert(ConstraintTypeId == batch.TypeId, "The type batch passed to the description must match the description's expected type.");
            ref var source = ref GetOffsetInstance(ref Buffer<AngularHingePrestepData>.Get(ref batch.PrestepData, bundleIndex), innerIndex);
            Vector3Wide.ReadFirst(source.LocalHingeAxisA, out description.LocalHingeAxisA);
            Vector3Wide.ReadFirst(source.LocalHingeAxisB, out description.LocalHingeAxisB);
            description.SpringSettings.AngularFrequency = GetFirst(ref source.SpringSettings.AngularFrequency);
            description.SpringSettings.TwiceDampingRatio = GetFirst(ref source.SpringSettings.TwiceDampingRatio);
        }
    }

    public struct AngularHingePrestepData
    {
        public Vector3Wide LocalHingeAxisA;
        public Vector3Wide LocalHingeAxisB;
        public SpringSettingsWide SpringSettings;
    }

    public struct AngularHingeFunctions : ITwoBodyConstraintFunctions<AngularHingePrestepData, Vector2Wide>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void GetErrorAngles(in Vector3Wide hingeAxisA, in Vector3Wide hingeAxisB, in Matrix2x3Wide jacobianA, out Vector2Wide errorAngles)
        {
            //Compute the position error and bias velocities.
            //Now we just have the slight annoyance that our error function contains inverse trigonometry.
            //We'll just use:
            //atan(dot(hingeAxisBOnPlaneX, hingeAxisA), dot(hingeAxisBOnPlaneX, constraintAxisAY)) = 
            //sign(dot(hingeAxisBOnPlaneX, constraintAxisAY)) * acos(dot(hingeAxisA, hingeAxisBOnPlaneX / ||hingeAxisBOnPlaneX||))
            //TODO: You could probably speed this up a bunch with some alternative approximations if you're willing to accept more error.
            //V1 abandoned the inverse trig and just used a dot product equivalent, which... had issues, but hey it's cheap.
            Vector3Wide.Dot(hingeAxisB, jacobianA.X, out var hingeAxisBDotX);
            Vector3Wide.Dot(hingeAxisB, jacobianA.Y, out var hingeAxisBDotY);
            Vector3Wide.Scale(jacobianA.X, hingeAxisBDotX, out var toRemoveX);
            Vector3Wide.Scale(jacobianA.Y, hingeAxisBDotY, out var toRemoveY);
            Vector3Wide.Subtract(hingeAxisB, toRemoveX, out var hingeAxisBOnPlaneX);
            Vector3Wide.Subtract(hingeAxisB, toRemoveY, out var hingeAxisBOnPlaneY);
            Vector3Wide.Length(hingeAxisBOnPlaneX, out var xLength);
            Vector3Wide.Length(hingeAxisBOnPlaneY, out var yLength);
            var scaleX = Vector<float>.One / xLength;
            var scaleY = Vector<float>.One / yLength;
            Vector3Wide.Scale(hingeAxisBOnPlaneX, scaleX, out hingeAxisBOnPlaneX);
            Vector3Wide.Scale(hingeAxisBOnPlaneY, scaleY, out hingeAxisBOnPlaneY);
            //If the axis is parallel with the normal of the plane, just arbitrarily pick 0 angle.
            var epsilon = new Vector<float>(1e-7f);
            var useFallbackX = Vector.LessThan(xLength, epsilon);
            var useFallbackY = Vector.LessThan(yLength, epsilon);
            Vector3Wide.ConditionalSelect(useFallbackX, hingeAxisA, hingeAxisBOnPlaneX, out hingeAxisBOnPlaneX);
            Vector3Wide.ConditionalSelect(useFallbackY, hingeAxisA, hingeAxisBOnPlaneY, out hingeAxisBOnPlaneY);

            Vector3Wide.Dot(hingeAxisBOnPlaneX, hingeAxisA, out var hbxha);
            Vector3Wide.Dot(hingeAxisBOnPlaneY, hingeAxisA, out var hbyha);
            //We could probably get away with an acos approximation of something like (1 - x) * pi/2, but we'll do just a little more work:
            errorAngles.X = MathHelper.Acos(hbxha);
            errorAngles.Y = MathHelper.Acos(hbyha);
            Vector3Wide.Dot(hingeAxisBOnPlaneX, jacobianA.Y, out var hbxay);
            Vector3Wide.Dot(hingeAxisBOnPlaneY, jacobianA.X, out var hbyax);
            errorAngles.X = Vector.ConditionalSelect(Vector.LessThan(hbxay, Vector<float>.Zero), errorAngles.X, -errorAngles.X);
            errorAngles.Y = Vector.ConditionalSelect(Vector.LessThan(hbyax, Vector<float>.Zero), -errorAngles.Y, errorAngles.Y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ApplyImpulse(in Matrix2x3Wide impulseToVelocityA, in Matrix2x3Wide negatedImpulseToVelocityB, in Vector2Wide csi, ref Vector3Wide angularVelocityA, ref Vector3Wide angularVelocityB)
        {
            Matrix2x3Wide.Transform(csi, impulseToVelocityA, out var velocityChangeA);
            Vector3Wide.Add(angularVelocityA, velocityChangeA, out angularVelocityA);
            Matrix2x3Wide.Transform(csi, negatedImpulseToVelocityB, out var negatedVelocityChangeB);
            Vector3Wide.Subtract(angularVelocityB, negatedVelocityChangeB, out angularVelocityB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ComputeJacobians(in Vector3Wide localHingeAxisA, in QuaternionWide orientationA, out Vector3Wide hingeAxisA, out Matrix2x3Wide jacobianA)
        {
            //Note that we build the tangents in local space first to avoid inconsistencies.
            Helpers.BuildOrthonormalBasis(localHingeAxisA, out var localAX, out var localAY);
            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var orientationMatrixA);
            Matrix3x3Wide.TransformWithoutOverlap(localHingeAxisA, orientationMatrixA, out hingeAxisA);
            Matrix3x3Wide.TransformWithoutOverlap(localAX, orientationMatrixA, out jacobianA.X);
            Matrix3x3Wide.TransformWithoutOverlap(localAY, orientationMatrixA, out jacobianA.Y);
        }

        public void WarmStart(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, ref AngularHingePrestepData prestep, ref Vector2Wide accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            ComputeJacobians(prestep.LocalHingeAxisA, orientationA, out _, out var jacobianA);
            Symmetric3x3Wide.MultiplyWithoutOverlap(jacobianA, inertiaA.InverseInertiaTensor, out var impulseToVelocityA);
            Symmetric3x3Wide.MultiplyWithoutOverlap(jacobianA, inertiaB.InverseInertiaTensor, out var negatedImpulseToVelocityB);
            ApplyImpulse(impulseToVelocityA, negatedImpulseToVelocityB, accumulatedImpulses, ref wsvA.Angular, ref wsvB.Angular);
        }

        public void Solve(in Vector3Wide positionA, in QuaternionWide orientationA, in BodyInertiaWide inertiaA, in Vector3Wide positionB, in QuaternionWide orientationB, in BodyInertiaWide inertiaB, float dt, float inverseDt, ref AngularHingePrestepData prestep, ref Vector2Wide accumulatedImpulses, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            //Note that we build the tangents in local space first to avoid inconsistencies.
            ComputeJacobians(prestep.LocalHingeAxisA, orientationA, out var hingeAxisA, out var jacobianA);
            QuaternionWide.TransformWithoutOverlap(prestep.LocalHingeAxisB, orientationB, out var hingeAxisB);

            //We project hingeAxisB onto the planes defined by A's axis X and and axis Y, and treat them as constant with respect to A's velocity. 
            //This hand waves away a bit of complexity related to the fact that A's axes have velocity too, but it works out pretty nicely in the end.
            //hingeAxisBOnPlaneX = hingeAxisB - dot(constraintAxisX, hingeAxisB) * constraintAxisX
            //hingeAxisBOnPlaneY = hingeAxisB - dot(constraintAxisY, hingeAxisB) * constraintAxisY
            //Note that we actually make use of inverse trig here. This is largely for the sake of the formulation, and the derivative will end up collapsing nicely.
            //C = [atan(dot(hingeAxisBOnPlaneX, hingeAxisA), dot(hingeAxisBOnPlaneX, constraintAxisAY))] = [0]
            //    [atan(dot(hingeAxisBOnPlaneY, hingeAxisA), dot(hingeAxisBOnPlaneY, constraintAxisAX))]   [0]
            //Focusing on the hingeAxisOnPlaneX jacobian:
            //C' = (dot(hingeAxisBOnPlaneX, hingeAxisA) * d/dt(dot(hingeAxisBOnPlaneX, constraintAxisAY)) - 
            //      d/dt(dot(hingeAxisBOnPlaneX, hingeAxisA)) * dot(hingeAxisBOnPlaneX, constraintAxisAY)) * denom
            //where denom = 1f / (dot(hingeAxisBOnPlaneX, hingeAxisA)^2 + dot(hingeAxisBOnPlaneX, constraintAxisAY)^2)
            //C' = (dot(hingeAxisBOnPlaneX, hingeAxisA) * (dot(d/dt(hingeAxisBOnPlaneX), constraintAxisAY) + dot(hingeAxisBOnPlaneX, d/dt(constraintAxisAY))) - 
            //     (dot(d/dt(hingeAxisBOnPlaneX), hingeAxisA) + dot(hingeAxisBOnPlaneX, d/dt(hingeAxisA))) * dot(hingeAxisBOnPlaneX, constraintAxisAY)) * denom
            //C' = (dot(hingeAxisBOnPlaneX, hingeAxisA) * (dot(wB x hingeAxisBOnPlaneX, constraintAxisAY) + dot(hingeAxisBOnPlaneX, wA x constraintAxisAY)) - 
            //     (dot(wB x hingeAxisBOnPlaneX, hingeAxisA) + dot(hingeAxisBOnPlaneX, wA x hingeAxisA)) * dot(hingeAxisBOnPlaneX, constraintAxisAY)) * denom
            //C' = (dot(hingeAxisBOnPlaneX, hingeAxisA) * (dot(hingeAxisBOnPlaneX x constraintAxisAY, wB) + dot(wA, constraintAxisAY x hingeAxisBOnPlaneX)) - 
            //     (dot(hingeAxisBOnPlaneX x hingeAxisA, wB) + dot(wA, hingeAxisA x hingeAxisBOnPlaneX)) * dot(hingeAxisBOnPlaneX, constraintAxisAY)) * denom
            //C' = ((dot(dot(hingeAxisBOnPlaneX, hingeAxisA) * (hingeAxisBOnPlaneX x constraintAxisAY), wB) + dot(wA, dot(hingeAxisBOnPlaneX, hingeAxisA) * (constraintAxisAY x hingeAxisBOnPlaneX))) - 
            //      (dot((hingeAxisBOnPlaneX x hingeAxisA) * dot(hingeAxisBOnPlaneX, constraintAxisAY), wB) + dot(wA, (hingeAxisA x hingeAxisBOnPlaneX) * dot(hingeAxisBOnPlaneX, constraintAxisAY)))) * denom

            //C' = ((dot(dot(hingeAxisBOnPlaneX, hingeAxisA) * (hingeAxisBOnPlaneX x constraintAxisAY) - (hingeAxisBOnPlaneX x hingeAxisA) * dot(hingeAxisBOnPlaneX, constraintAxisAY), wB) + 
            //       dot(wA, dot(hingeAxisBOnPlaneX, hingeAxisA) * (constraintAxisAY x hingeAxisBOnPlaneX) - (hingeAxisA x hingeAxisBOnPlaneX) * dot(hingeAxisBOnPlaneX, constraintAxisAY)))) * denom
            //C' = (dot(wB, dot(hingeAxisBOnPlaneX, hingeAxisA) * (hingeAxisBOnPlaneX x constraintAxisAY) - 
            //              dot(hingeAxisBOnPlaneX, constraintAxisAY) * (hingeAxisBOnPlaneX x hingeAxisA) + 
            //      dot(wA, dot(hingeAxisBOnPlaneX, hingeAxisA) * (constraintAxisAY x hingeAxisBOnPlaneX) - 
            //              dot(hingeAxisBOnPlaneX, constraintAxisAY) * (hingeAxisA x hingeAxisBOnPlaneX)) * denom
            //Note that both contributing vectors of jacobian A, constraintAxisAY x hingeAxisBOnPlaneX and hingeAxisA x hingeAxisBOnPlaneX, are aligned with constraintAxisAX.
            //The only remaining question is the scale. Measure it by dotting with the constraintAxisAX.
            //(Switching notation for conciseness here: a = hingeAxisA, b = hingeAxisBOnPlaneX, x = constraintAxisAX, y = constraintAxisAY)
            //(dot(b, a) * cross(y, b) - dot(b, y) * cross(a, b)) / (dot(b, a)^2 + dot(b,y)^2)
            //scale = dot((dot(b, a) * cross(y, b) - dot(b, y) * cross(a, b)) / (dot(b, a)^2 + dot(b,y)^2), x)
            //scale = (dot(b, a) * dot(x, cross(y, b)) - dot(b, y) * dot(x, cross(a, b))) / (dot(b, a)^2 + dot(b,y)^2)
            //scale = (dot(b, a) * dot(b, cross(x, y)) - dot(b, y) * dot(b, cross(x, a))) / (dot(b, a)^2 + dot(b,y)^2)
            //scale = (dot(b, a) * dot(b, a) - dot(b, y) * dot(b, -y)) / (dot(b, a)^2 + dot(b,y)^2)
            //scale = (dot(b, a) * dot(b, a) + dot(b, y) * dot(b, y)) / (dot(b, a)^2 + dot(b,y)^2)
            //scale = 1
            //How convenient!
            //jacobianA = [constraintAxisAX]
            //            [constraintAxisAY]
            //jacobianB = -jacobianA

            //Note that JA = -JB, but for the purposes of calculating the effective mass the sign is irrelevant.
            //This computes the effective mass using the usual (J * M^-1 * JT)^-1 formulation, but we actually make use of the intermediate result J * M^-1 so we compute it directly.
            Symmetric3x3Wide.MultiplyWithoutOverlap(jacobianA, inertiaA.InverseInertiaTensor, out var impulseToVelocityA);
            //Note that we don't use -jacobianA here, so we're actually storing out the negated version of the transform. That's fine; we'll simply subtract in the iteration.
            Symmetric3x3Wide.MultiplyWithoutOverlap(jacobianA, inertiaB.InverseInertiaTensor, out var negatedImpulseToVelocityB);
            Symmetric2x2Wide.CompleteMatrixSandwich(impulseToVelocityA, jacobianA, out var angularA);
            Symmetric2x2Wide.CompleteMatrixSandwich(negatedImpulseToVelocityB, jacobianA, out var angularB);
            Symmetric2x2Wide.Add(angularA, angularB, out var inverseEffectiveMass);
            Symmetric2x2Wide.InvertWithoutOverlap(inverseEffectiveMass, out var effectiveMass);

            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out var softnessImpulseScale);
            //Note the effective mass is not scaled directly. Instead, we scale the csi to save a multiply.
            GetErrorAngles(hingeAxisA, hingeAxisB, jacobianA, out var errorAngle);
            //Note the negation: we want to oppose the separation. TODO: arguably, should bake the negation into positionErrorToVelocity, given its name.
            Vector2Wide.Scale(errorAngle, -positionErrorToVelocity, out var biasVelocity);
            Symmetric2x2Wide.TransformWithoutOverlap(biasVelocity, effectiveMass, out var biasImpulse);

            //JB = -JA. This is (angularVelocityA * JA + angularVelocityB * JB) * effectiveMass
            Vector3Wide.Subtract(wsvA.Angular, wsvB.Angular, out var difference);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(difference, jacobianA, out var csv);
            Symmetric2x2Wide.TransformWithoutOverlap(csv, effectiveMass, out var csi);
            Vector2Wide.Scale(csi, effectiveMassCFMScale, out csi);
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            Vector2Wide.Scale(accumulatedImpulses, softnessImpulseScale, out var softnessContribution);
            Vector2Wide.Add(softnessContribution, csi, out csi);
            Vector2Wide.Subtract(biasImpulse, csi, out csi);

            Vector2Wide.Add(accumulatedImpulses, csi, out accumulatedImpulses);

            ApplyImpulse(impulseToVelocityA, negatedImpulseToVelocityB, csi, ref wsvA.Angular, ref wsvB.Angular);
        }

        public bool RequiresIncrementalSubstepUpdates => false;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IncrementallyUpdateForSubstep(in Vector<float> dt, in BodyVelocityWide wsvA, in BodyVelocityWide wsvB, ref AngularHingePrestepData prestepData) { }
    }

    public class AngularHingeTypeProcessor : TwoBodyTypeProcessor<AngularHingePrestepData, Vector2Wide, AngularHingeFunctions, AccessOnlyAngular, AccessOnlyAngularWithoutPose, AccessOnlyAngular, AccessOnlyAngular>
    {
        public const int BatchTypeId = 23;
    }
}

