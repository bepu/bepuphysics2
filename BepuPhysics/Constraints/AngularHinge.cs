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

        public int ConstraintTypeId
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return AngularHingeTypeProcessor.BatchTypeId;
            }
        }

        public Type TypeProcessorType => typeof(AngularHingeTypeProcessor);

        public void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex)
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

        public void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out AngularHinge description)
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
            MathHelper.ApproximateAcos(hbxha, out errorAngles.X);
            MathHelper.ApproximateAcos(hbyha, out errorAngles.Y);
            Vector3Wide.Dot(hingeAxisBOnPlaneX, jacobianA.Y, out var hbxay);
            Vector3Wide.Dot(hingeAxisBOnPlaneY, jacobianA.X, out var hbyax);
            errorAngles.X = Vector.ConditionalSelect(Vector.LessThan(hbxay, Vector<float>.Zero), errorAngles.X, -errorAngles.X);
            errorAngles.Y = Vector.ConditionalSelect(Vector.LessThan(hbyax, Vector<float>.Zero), -errorAngles.Y, errorAngles.Y);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref TwoBodyReferences bodyReferences, int count, float dt, float inverseDt, ref BodyInertias inertiaA, ref BodyInertias inertiaB,
            ref AngularHingePrestepData prestep, out AngularHingeProjection projection)
        {
            bodies.GatherOrientation(ref bodyReferences, count, out var orientationA, out var orientationB);

            //Note that we build the tangents in local space first to avoid inconsistencies.
            Helpers.BuildOrthnormalBasis(prestep.LocalHingeAxisA, out var localAX, out var localAY);
            Matrix3x3Wide.CreateFromQuaternion(orientationA, out var orientationMatrixA);
            Matrix3x3Wide.TransformWithoutOverlap(prestep.LocalHingeAxisA, orientationMatrixA, out var hingeAxisA);
            Matrix2x3Wide jacobianA;
            Matrix3x3Wide.TransformWithoutOverlap(localAX, orientationMatrixA, out jacobianA.X);
            Matrix3x3Wide.TransformWithoutOverlap(localAY, orientationMatrixA, out jacobianA.Y);
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
            Symmetric3x3Wide.MultiplyWithoutOverlap(jacobianA, inertiaA.InverseInertiaTensor, out projection.ImpulseToVelocityA);
            //Note that we don't use -jacobianA here, so we're actually storing out the negated version of the transform. That's fine; we'll simply subtract in the iteration.
            Symmetric3x3Wide.MultiplyWithoutOverlap(jacobianA, inertiaB.InverseInertiaTensor, out projection.NegatedImpulseToVelocityB);
            Symmetric2x2Wide.CompleteMatrixSandwich(projection.ImpulseToVelocityA, jacobianA, out var angularA);
            Symmetric2x2Wide.CompleteMatrixSandwich(projection.NegatedImpulseToVelocityB, jacobianA, out var angularB);
            Symmetric2x2Wide.Add(angularA, angularB, out var inverseEffectiveMass);
            Symmetric2x2Wide.InvertWithoutOverlap(inverseEffectiveMass, out var effectiveMass);

            SpringSettingsWide.ComputeSpringiness(prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);
            Symmetric2x2Wide.Scale(effectiveMass, effectiveMassCFMScale, out effectiveMass);
            Symmetric2x2Wide.MultiplyTransposed(jacobianA, effectiveMass, out projection.VelocityToImpulseA);
            GetErrorAngles(hingeAxisA, hingeAxisB, jacobianA, out var errorAngle);
            //Note the negation: we want to oppose the separation. TODO: arguably, should bake the negation into positionErrorToVelocity, given its name.
            Vector2Wide.Scale(errorAngle, -positionErrorToVelocity, out var biasVelocity);
            Symmetric2x2Wide.TransformWithoutOverlap(biasVelocity, effectiveMass, out projection.BiasImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void ApplyImpulse(ref Vector3Wide angularVelocityA, ref Vector3Wide angularVelocityB, ref AngularHingeProjection projection, ref Vector2Wide csi)
        {
            Matrix2x3Wide.Transform(csi, projection.ImpulseToVelocityA, out var velocityChangeA);
            Vector3Wide.Add(angularVelocityA, velocityChangeA, out angularVelocityA);
            Matrix2x3Wide.Transform(csi, projection.NegatedImpulseToVelocityB, out var negatedVelocityChangeB);
            Vector3Wide.Subtract(angularVelocityB, negatedVelocityChangeB, out angularVelocityB);
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
            Vector3Wide.Subtract(velocityA.Angular, velocityB.Angular, out var difference);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(difference, projection.VelocityToImpulseA, out var csi);
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            Vector2Wide.Scale(accumulatedImpulse, projection.SoftnessImpulseScale, out var softnessContribution);
            Vector2Wide.Add(softnessContribution, csi, out csi);
            Vector2Wide.Subtract(projection.BiasImpulse, csi, out csi);

            Vector2Wide.Add(accumulatedImpulse, csi, out accumulatedImpulse);

            ApplyImpulse(ref velocityA.Angular, ref velocityB.Angular, ref projection, ref csi);
        }

    }

    public class AngularHingeTypeProcessor : TwoBodyTypeProcessor<AngularHingePrestepData, AngularHingeProjection, Vector2Wide, AngularHingeFunctions>
    {
        public const int BatchTypeId = 23;
    }
}

