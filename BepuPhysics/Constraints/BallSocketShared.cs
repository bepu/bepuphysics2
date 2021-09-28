using BepuUtilities;
using System.Numerics;
using System.Runtime.CompilerServices;
namespace BepuPhysics.Constraints
{
    /// <summary>
    /// Provides shared functionality for constraints with jacobians similar to the BallSocket.
    /// </summary>
    public static class BallSocketShared
    {
        //TODO: There may be an argument for some extra level of abstraction here. If we gave the prestep function the data it needed (i.e. pose and inertia)
        //directly, it would be easier to share the implementation across different constraints. For example, it may be extremely common to use a particular set of 
        //constraints together- like ball socket, swing limit, and revolute angular joint- and you wouldn't want to regather inertia and pose for all of them.
        //(In that specific case, using a simultaneously solved ball socket + revolute angular joint would be preferred for stability, but the point stands.)
        //(Also... any time you use combo constraints, it is extremely likely that you need to modify the projections for optimal memory packing.
        //There are very few cases where a combo constraint will have less than 3DOFs...)
        //The only reason not to do that is codegen concerns. But we may want to stop holding back just because of some hopefully-not-permanent quirks in the JIT.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeEffectiveMass(in BodyInertiaWide inertiaA, in BodyInertiaWide inertiaB,
            in Vector3Wide offsetA, in Vector3Wide offsetB, in Vector<float> effectiveMassCFMScale, out Symmetric3x3Wide effectiveMass)
        {
            //Anchor points attached to each body are constrained to stay in the same position, yielding a position constraint of:
            //C = positionA + anchorOffsetA - (positionB + anchorOffsetB) = 0
            //C' = velocityA + d/dt(anchorOffsetA) - (velocityB + d/dt(anchorOffsetB)) = 0
            //C' = velocityA + angularVelocity x anchorOffsetA - (velocityB + angularVelocityB x anchorOffsetB) = 0
            //C' = velocityA * I + angularVelocity * skewSymmetric(anchorOffsetA) - velocityB * I - angularVelocityB * skewSymmetric(anchorOffsetB) = 0
            //So, the jacobians:
            //LinearA: I
            //AngularA: skewSymmetric(anchorOffsetA)
            //LinearB: -I
            //AngularB: skewSymmetric(-anchorOffsetB)
            //Each of these is a 3x3 matrix. However, we don't need to explicitly compute or store any of these.
            //Not storing the identity matrix is obvious enough, but we can also just store the offset rather than the full skew symmetric matrix for the angular jacobians.
            //The skew symmetric matrix is replaced by a simple cross product.

            //The projection should strive to contain the smallest possible representation necessary for the constraint to work.
            //We can sorta-kinda make use of the baking-inertia-into-jacobian trick, because I * softenedEffectiveMass is... softenedEffectiveMass.
            //Likewise, J * inverseInertia is just the scalar inverseMasses for the two linear contributions.
            //Note that there's no reason to try to bake the softenedEffectiveMass into the angular jacobian, because we already have to perform at least one 
            //softenedEffectiveMass multiply due to the linear components. We can just store the raw angular jacobians and piggyback on the linear component effective mass multiply.

            //So, the question becomes... is there a way to bundle inverseInertia into the angularJacobians?
            //Yes, but it's not useful. If we premultiply the skew(offset) * inverseInertia for CSIToWSV, the result is no longer symmetric.
            //That means we would have to store 3 additional scalars per body compared to the symmetric inverse inertias. That's not particularly useful;
            //it only saves a cross product. Loading 6 more scalars to save 2 cross products (12 multiplies, 6 adds) is a terrible trade, even at SIMD128.

            Symmetric3x3Wide.SkewSandwichWithoutOverlap(offsetA, inertiaA.InverseInertiaTensor, out var inverseEffectiveMass);
            //Note that the jacobian is technically skewSymmetric(-OffsetB), but the sign doesn't matter due to the sandwich.
            Symmetric3x3Wide.SkewSandwichWithoutOverlap(offsetB, inertiaB.InverseInertiaTensor, out var angularBContribution);
            Symmetric3x3Wide.Add(inverseEffectiveMass, angularBContribution, out inverseEffectiveMass);

            //Linear contributions are simply I * inverseMass * I, which is just boosting the diagonal.
            var linearContribution = inertiaA.InverseMass + inertiaB.InverseMass;
            inverseEffectiveMass.XX += linearContribution;
            inverseEffectiveMass.YY += linearContribution;
            inverseEffectiveMass.ZZ += linearContribution;
            Symmetric3x3Wide.Invert(inverseEffectiveMass, out effectiveMass);
            Symmetric3x3Wide.Scale(effectiveMass, effectiveMassCFMScale, out effectiveMass);


            //NOTE:
            //It's possible to use local inertia tensors diagonalized to only 3 scalars per body. Given that we load orientations as a part of the prestep and so could reconstruct 
            //the world inertia tensor, this would save 6 scalar loads. In isolation, that's a likely win. However, there are some other considerations:
            //1) Some common constraints, notably including the contact constraints, do not load pose. They can either load diagonalized inertia with orientation (7 total scalars per body)
            //or a symmetric world inertia tensor (6 scalars). Given the relatively high cost of incoherent gathers, the 6 scalar world inertia gather would be used.
            //2) If constraints are loading from both local and world inertias rather than one or the other, the number of cache misses increases.
            //3) It requires that the local space of shapes is aligned with the moments of inertia. While this could be done automatically when creating shapes, and while
            //all primitive shapes are built with diagonal inertia tensors, it is likely that the local 'reorientation' required by diagonalization applied to things like
            //convex hulls or meshes would be confusing for users. Recentering certainly was in V1.
            //4) It cannot be used to save space on explicitly stored inertias in constraints, because the warmstart/solve do not load the pose (and, as above, loading orientation and
            //local inertia is 1 scalar more than just the world inertia).
            //5) While the local diagonal representation does offer some possibilities for ALU-level optimizations in the prestep, the effective mass matrix will be in world space,
            //and in general it will not be any smaller than the usual symmetric representation.
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB,
            in Vector3Wide offsetA, in Vector3Wide offsetB, in BodyInertiaWide inertiaA, in BodyInertiaWide inertiaB, in Vector3Wide constraintSpaceImpulse)
        {
            Vector3Wide.CrossWithoutOverlap(offsetA, constraintSpaceImpulse, out var wsi);
            Symmetric3x3Wide.TransformWithoutOverlap(wsi, inertiaA.InverseInertiaTensor, out var change);
            Vector3Wide.Add(velocityA.Angular, change, out velocityA.Angular);

            Vector3Wide.Scale(constraintSpaceImpulse, inertiaA.InverseMass, out change);
            Vector3Wide.Add(velocityA.Linear, change, out velocityA.Linear);

            Vector3Wide.CrossWithoutOverlap(constraintSpaceImpulse, offsetB, out wsi); //note flip-negation
            Symmetric3x3Wide.TransformWithoutOverlap(wsi, inertiaB.InverseInertiaTensor, out change);
            Vector3Wide.Add(velocityB.Angular, change, out velocityB.Angular);

            Vector3Wide.Scale(constraintSpaceImpulse, inertiaB.InverseMass, out change);
            Vector3Wide.Subtract(velocityB.Linear, change, out velocityB.Linear); //note subtraction; the jacobian is -I
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeCorrectiveImpulse(ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB, in Vector3Wide offsetA, in Vector3Wide offsetB,
            in Vector3Wide biasVelocity, in Symmetric3x3Wide effectiveMass, in Vector<float> softnessImpulseScale, in Vector3Wide accumulatedImpulse, out Vector3Wide correctiveImpulse)
        {
            //csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);
            //Note subtraction; jLinearB = -I.
            Vector3Wide.Subtract(velocityA.Linear, velocityB.Linear, out var csv);
            Vector3Wide.CrossWithoutOverlap(velocityA.Angular, offsetA, out var angularCSV);
            Vector3Wide.Add(csv, angularCSV, out csv);
            //Note reversed cross order; matches the jacobian -CrossMatrix(offsetB).
            Vector3Wide.CrossWithoutOverlap(offsetB, velocityB.Angular, out angularCSV);
            Vector3Wide.Add(csv, angularCSV, out csv);
            Vector3Wide.Subtract(biasVelocity, csv, out csv);

            Symmetric3x3Wide.TransformWithoutOverlap(csv, effectiveMass, out correctiveImpulse);
            Vector3Wide.Scale(accumulatedImpulse, softnessImpulseScale, out var softness);
            Vector3Wide.Subtract(correctiveImpulse, softness, out correctiveImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Solve(ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB, in Vector3Wide offsetA, in Vector3Wide offsetB,
            in Vector3Wide biasVelocity, in Symmetric3x3Wide effectiveMass, in Vector<float> softnessImpulseScale, ref Vector3Wide accumulatedImpulse, in BodyInertiaWide inertiaA, in BodyInertiaWide inertiaB)
        {
            ComputeCorrectiveImpulse(ref velocityA, ref velocityB, offsetA, offsetB, biasVelocity, effectiveMass, softnessImpulseScale, accumulatedImpulse, out var correctiveImpulse);
            //This function does not have a maximum impulse limit, so no clamping is required.
            Vector3Wide.Add(accumulatedImpulse, correctiveImpulse, out accumulatedImpulse);

            ApplyImpulse(ref velocityA, ref velocityB, offsetA, offsetB, inertiaA, inertiaB, correctiveImpulse);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Solve(ref BodyVelocityWide velocityA, ref BodyVelocityWide velocityB, in Vector3Wide offsetA, in Vector3Wide offsetB,
            in Vector3Wide biasVelocity, in Symmetric3x3Wide effectiveMass, in Vector<float> softnessImpulseScale, in Vector<float> maximumImpulse, ref Vector3Wide accumulatedImpulse, in BodyInertiaWide inertiaA, in BodyInertiaWide inertiaB)
        {
            ComputeCorrectiveImpulse(ref velocityA, ref velocityB, offsetA, offsetB, biasVelocity, effectiveMass, softnessImpulseScale, accumulatedImpulse, out var correctiveImpulse);
            //This function DOES have a maximum impulse limit.
            ServoSettingsWide.ClampImpulse(maximumImpulse, ref accumulatedImpulse, ref correctiveImpulse);

            ApplyImpulse(ref velocityA, ref velocityB, offsetA, offsetB, inertiaA, inertiaB, correctiveImpulse);
        }

    }
}
