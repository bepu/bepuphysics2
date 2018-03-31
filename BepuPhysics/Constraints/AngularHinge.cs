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
            GetFirst(ref target.SpringSettings.AngularFrequency) = SpringSettings.AngularFrequency;
            GetFirst(ref target.SpringSettings.TwiceDampingRatio) = SpringSettings.TwiceDampingRatio;
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
            description.SpringSettings.AngularFrequency = GetFirst(ref source.SpringSettings.AngularFrequency);
            description.SpringSettings.TwiceDampingRatio = GetFirst(ref source.SpringSettings.TwiceDampingRatio);
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
        static void ApproximateAcos(ref Vector<float> x, out Vector<float> acos)
        {
            //acos(x) ~= (pi / (2 * sqrt(2))) * sqrt(2 - 2 * x), for 0<=x<=1
            //acos(x) ~= pi - (pi / (2 * sqrt(2))) * sqrt(2 + 2 * x), for -1<=x<=0
            var two = new Vector<float>(2f);
            acos = new Vector<float>(1.11072073454f) * Vector.SquareRoot(Vector.Max(Vector<float>.Zero, two - two * Vector.Abs(x)));
            acos = Vector.ConditionalSelect(Vector.LessThan(x, Vector<float>.Zero), new Vector<float>(MathHelper.Pi) - acos, acos);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Prestep(Bodies bodies, ref TwoBodyReferences bodyReferences, int count, float dt, float inverseDt, ref AngularHingePrestepData prestep,
            out AngularHingeProjection projection)
        {
            bodies.GatherInertiaAndPose(ref bodyReferences, count,
                out var localPositionB, out var orientationA, out var orientationB,
                out var inertiaA, out var inertiaB);

            //Note that we build the tangents in local space first to avoid inconsistencies.
            Helpers.BuildOrthnormalBasis(ref prestep.HingeAxisLocalA, out var localAX, out var localAY);
            Matrix3x3Wide.CreateFromQuaternion(ref orientationA, out var orientationMatrixA);
            Matrix3x3Wide.TransformWithoutOverlap(ref prestep.HingeAxisLocalA, ref orientationMatrixA, out var hingeAxisA);
            Matrix2x3Wide jacobianA;
            Matrix3x3Wide.TransformWithoutOverlap(ref localAX, ref orientationMatrixA, out jacobianA.X);
            Matrix3x3Wide.TransformWithoutOverlap(ref localAY, ref orientationMatrixA, out jacobianA.Y);
            QuaternionWide.TransformWithoutOverlap(ref prestep.HingeAxisLocalB, ref orientationB, out var hingeAxisB);

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
            Triangular3x3Wide.MultiplyBySymmetricWithoutOverlap(ref jacobianA, ref inertiaA.InverseInertiaTensor, out projection.ImpulseToVelocityA);
            //Note that we don't use -jacobianA here, so we're actually storing out the negated version of the transform. That's fine; we'll simply subtract in the iteration.
            Triangular3x3Wide.MultiplyBySymmetricWithoutOverlap(ref jacobianA, ref inertiaB.InverseInertiaTensor, out projection.NegatedImpulseToVelocityB);
            Triangular2x2Wide.CompleteMatrixSandwich(ref projection.ImpulseToVelocityA, ref jacobianA, out var angularA);
            Triangular2x2Wide.CompleteMatrixSandwich(ref projection.NegatedImpulseToVelocityB, ref jacobianA, out var angularB);
            Triangular2x2Wide.Add(ref angularA, ref angularB, out var inverseEffectiveMass);
            Triangular2x2Wide.InvertSymmetricWithoutOverlap(ref inverseEffectiveMass, out var effectiveMass);

            SpringSettings.ComputeSpringiness(ref prestep.SpringSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);
            Triangular2x2Wide.Scale(ref effectiveMass, ref effectiveMassCFMScale, out effectiveMass);
            Triangular2x2Wide.MultiplyTransposedBySymmetric(ref jacobianA, ref effectiveMass, out projection.VelocityToImpulseA);

            //Compute the position error and bias velocities.
            //Now we just have the slight annoyance that our error function contains inverse trigonometry.
            //We'll just use:
            //atan(dot(hingeAxisBOnPlaneX, hingeAxisA), dot(hingeAxisBOnPlaneX, constraintAxisAY)) = 
            //sign(dot(hingeAxisBOnPlaneX, constraintAxisAY)) * acos(dot(hingeAxisA, hingeAxisBOnPlaneX / ||hingeAxisBOnPlaneX||))
            //TODO: You could probably speed this up a bunch with some alternative approximations if you're willing to accept more error.
            //V1 abandoned the inverse trig and just used a dot product equivalent, which... had issues, but hey it's cheap.
            Vector3Wide.Dot(ref hingeAxisB, ref jacobianA.X, out var hingeAxisBDotX);
            Vector3Wide.Dot(ref hingeAxisB, ref jacobianA.Y, out var hingeAxisBDotY);
            Vector3Wide.Scale(ref jacobianA.X, ref hingeAxisBDotX, out var toRemoveX);
            Vector3Wide.Scale(ref jacobianA.Y, ref hingeAxisBDotY, out var toRemoveY);
            Vector3Wide.Subtract(ref hingeAxisB, ref toRemoveX, out var hingeAxisBOnPlaneX);
            Vector3Wide.Subtract(ref hingeAxisB, ref toRemoveY, out var hingeAxisBOnPlaneY);
            Vector3Wide.Length(ref hingeAxisBOnPlaneX, out var xLength);
            Vector3Wide.Length(ref hingeAxisBOnPlaneY, out var yLength);
            var scaleX = Vector<float>.One / xLength;
            var scaleY = Vector<float>.One / yLength;
            Vector3Wide.Scale(ref hingeAxisBOnPlaneX, ref scaleX, out hingeAxisBOnPlaneX);
            Vector3Wide.Scale(ref hingeAxisBOnPlaneY, ref scaleY, out hingeAxisBOnPlaneY);
            //If the axis is parallel with the normal of the plane, just arbitrarily pick 0 angle.
            var epsilon = new Vector<float>(1e-7f);
            var useFallbackX = Vector.LessThan(xLength, epsilon);
            var useFallbackY = Vector.LessThan(yLength, epsilon);
            Vector3Wide.ConditionalSelect(ref useFallbackX, ref hingeAxisA, ref hingeAxisBOnPlaneX, out hingeAxisBOnPlaneX);
            Vector3Wide.ConditionalSelect(ref useFallbackY, ref hingeAxisA, ref hingeAxisBOnPlaneY, out hingeAxisBOnPlaneY);

            Vector3Wide.Dot(ref hingeAxisBOnPlaneX, ref hingeAxisA, out var hbxha);
            Vector3Wide.Dot(ref hingeAxisBOnPlaneY, ref hingeAxisA, out var hbyha);
            Vector2Wide errorAngle;
            //We could probably get away with an acos approximation of something like (1 - x) * pi/2, but we'll do just a little more work:
            ApproximateAcos(ref hbxha, out errorAngle.X);
            ApproximateAcos(ref hbyha, out errorAngle.Y);
            Vector3Wide.Dot(ref hingeAxisBOnPlaneX, ref jacobianA.Y, out var hbxay);
            Vector3Wide.Dot(ref hingeAxisBOnPlaneY, ref jacobianA.X, out var hbyax);
            errorAngle.X = Vector.ConditionalSelect(Vector.LessThan(hbxay, Vector<float>.Zero), errorAngle.X, -errorAngle.X);
            errorAngle.Y = Vector.ConditionalSelect(Vector.LessThan(hbyax, Vector<float>.Zero), -errorAngle.Y, errorAngle.Y);
            //Note the negation: we want to oppose the separation. TODO: arguably, should bake the negation into positionErrorToVelocity, given its name.
            positionErrorToVelocity = -positionErrorToVelocity;
            Vector2Wide.Scale(ref errorAngle, ref positionErrorToVelocity, out var biasVelocity);
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
        public const int BatchTypeId = 23;
    }
}

