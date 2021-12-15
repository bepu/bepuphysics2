using BepuUtilities;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Constraints.Contact
{
    /// <summary>
    /// Handles the tangent friction implementation.
    /// </summary>
    public static class TangentFriction
    {
        public struct Jacobians
        {
            public Matrix2x3Wide LinearA;
            public Matrix2x3Wide AngularA;
            public Matrix2x3Wide AngularB;
        }
        //Since this is an unshared specialized implementation, the jacobian calculation is kept in here rather than in the batch.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeJacobians(in Vector3Wide tangentX, in Vector3Wide tangentY, in Vector3Wide offsetA, in Vector3Wide offsetB,
            out Jacobians jacobians)
        {
            //Two velocity constraints: 
            //dot(velocity(p, A), tangentX) = dot(velocity(p, B), tangentX)
            //dot(velocity(p, A), tangentY) = dot(velocity(p, B), tangentY)
            //where velocity(p, A) is the velocity of a point p attached to object A.
            //velocity(p, A) = linearVelocityA + angularVelocityA x (p - positionA) = linearVelocityA + angularVelocityA x offsetA
            //so:
            //dot(velocity(p, A), tangentX) = dot(linearVelocityA, tangentX) + dot(angularVelocityA x offsetA, tangentX)
            //dot(velocity(p, A), tangentX) = dot(linearVelocityA, tangentX) + dot(offsetA x tangentX, angularVelocityA)
            //Restating the two constraints:
            //dot(linearVelocityA, tangentX) + dot(offsetA x tangentX, angularVelocityA) = dot(linearVelocityB, tangentX) + dot(offsetB x tangentX, angularVelocityB)
            //dot(linearVelocityA, tangentY) + dot(offsetA x tangentY, angularVelocityA) = dot(linearVelocityB, tangentY) + dot(offsetB x tangentY, angularVelocityB)
            //dot(linearVelocityA, tangentX) + dot(offsetA x tangentX, angularVelocityA) - dot(linearVelocityB, tangentX) - dot(offsetB x tangentX, angularVelocityB) = 0
            //dot(linearVelocityA, tangentY) + dot(offsetA x tangentY, angularVelocityA) - dot(linearVelocityB, tangentY) - dot(offsetB x tangentY, angularVelocityB) = 0

            //Since there are two constraints (2DOFs), there are two rows in the jacobian, which based on the above is:
            //jLinearA = [ tangentX ]
            //           [ tangentY ]
            //jAngularA = [ offsetA x tangentX ]
            //            [ offsetA x tangentY ]
            //jLinearB = [ -tangentX ]
            //           [ -tangentY ]
            //jAngularB = [ -offsetB x tangentX ] = [ tangentX x offsetB ]
            //            [ -offsetB x tangentY ]   [ tangentY x offsetB ]
            //TODO: there would be a minor benefit in eliminating this copy manually, since it's very likely that the compiler won't. And it's probably also introducing more locals init.
            jacobians.LinearA.X = tangentX;
            jacobians.LinearA.Y = tangentY;
            Vector3Wide.CrossWithoutOverlap(offsetA, tangentX, out jacobians.AngularA.X);
            Vector3Wide.CrossWithoutOverlap(offsetA, tangentY, out jacobians.AngularA.Y);
            Vector3Wide.CrossWithoutOverlap(tangentX, offsetB, out jacobians.AngularB.X);
            Vector3Wide.CrossWithoutOverlap(tangentY, offsetB, out jacobians.AngularB.Y);
        }

        /// <summary>
        /// Transforms an impulse from constraint space to world space, uses it to modify the cached world space velocities of the bodies.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(in Jacobians jacobians, in BodyInertiaWide inertiaA, in BodyInertiaWide inertiaB,
            in Vector2Wide correctiveImpulse, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            Matrix2x3Wide.Transform(correctiveImpulse, jacobians.LinearA, out var linearImpulseA);
            Matrix2x3Wide.Transform(correctiveImpulse, jacobians.AngularA, out var angularImpulseA);
            Matrix2x3Wide.Transform(correctiveImpulse, jacobians.AngularB, out var angularImpulseB);
            BodyVelocityWide correctiveVelocityA, correctiveVelocityB;
            Vector3Wide.Scale(linearImpulseA, inertiaA.InverseMass, out correctiveVelocityA.Linear);
            Symmetric3x3Wide.TransformWithoutOverlap(angularImpulseA, inertiaA.InverseInertiaTensor, out correctiveVelocityA.Angular);
            Vector3Wide.Scale(linearImpulseA, inertiaB.InverseMass, out correctiveVelocityB.Linear);
            Symmetric3x3Wide.TransformWithoutOverlap(angularImpulseB, inertiaB.InverseInertiaTensor, out correctiveVelocityB.Angular);
            Vector3Wide.Add(wsvA.Linear, correctiveVelocityA.Linear, out wsvA.Linear);
            Vector3Wide.Add(wsvA.Angular, correctiveVelocityA.Angular, out wsvA.Angular);
            Vector3Wide.Subtract(wsvB.Linear, correctiveVelocityB.Linear, out wsvB.Linear); //note subtract- we based it on the LinearA jacobian.
            Vector3Wide.Add(wsvB.Angular, correctiveVelocityB.Angular, out wsvB.Angular);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeCorrectiveImpulse(in BodyVelocityWide wsvA, in BodyVelocityWide wsvB, in Symmetric2x2Wide effectiveMass, in Jacobians jacobians,
            in Vector<float> maximumImpulse, ref Vector2Wide accumulatedImpulse, out Vector2Wide correctiveCSI)
        {
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(wsvA.Linear, jacobians.LinearA, out var csvaLinear);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(wsvA.Angular, jacobians.AngularA, out var csvaAngular);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(wsvB.Linear, jacobians.LinearA, out var csvbLinear);
            Matrix2x3Wide.TransformByTransposeWithoutOverlap(wsvB.Angular, jacobians.AngularB, out var csvbAngular);
            //Note that the velocity in constraint space is (csvaLinear - csvbLinear + csvaAngular + csvbAngular).
            //The subtraction there is due to sharing the linear jacobian between both bodies.
            //In the following, we need to compute the constraint space *violating* velocity- which is the negation of the above velocity in constraint space.
            //So, (csvbLinear - csvaLinear - (csvaAngular + csvbAngular)).
            Vector2Wide.Subtract(csvbLinear, csvaLinear, out var csvLinear);
            Vector2Wide.Add(csvaAngular, csvbAngular, out var csvAngular);
            Vector2Wide.Subtract(csvLinear, csvAngular, out var csv);

            Symmetric2x2Wide.TransformWithoutOverlap(csv, effectiveMass, out var csi);

            var previousAccumulated = accumulatedImpulse;
            Vector2Wide.Add(accumulatedImpulse, csi, out accumulatedImpulse);
            //The maximum force of friction depends upon the normal impulse. The maximum is supplied per iteration.
            Vector2Wide.Length(accumulatedImpulse, out var accumulatedMagnitude);
            //Note division by zero guard.
            var scale = Vector.Min(Vector<float>.One, maximumImpulse / Vector.Max(new Vector<float>(1e-16f), accumulatedMagnitude));
            Vector2Wide.Scale(accumulatedImpulse, scale, out accumulatedImpulse);

            Vector2Wide.Subtract(accumulatedImpulse, previousAccumulated, out correctiveCSI);

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WarmStart(in Vector3Wide tangentX, in Vector3Wide tangentY, in Vector3Wide offsetToManifoldCenterA, in Vector3Wide offsetToManifoldCenterB, in BodyInertiaWide inertiaA, in BodyInertiaWide inertiaB,
            in Vector2Wide accumulatedImpulse, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            ComputeJacobians(tangentX, tangentY, offsetToManifoldCenterA, offsetToManifoldCenterB, out var jacobians);
            //TODO: If the previous frame and current frame are associated with different time steps, the previous frame's solution won't be a good solution anymore.
            //To compensate for this, the accumulated impulse should be scaled if dt changes.
            ApplyImpulse(jacobians, inertiaA, inertiaB, accumulatedImpulse, ref wsvA, ref wsvB);
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Solve(in Vector3Wide tangentX, in Vector3Wide tangentY, in Vector3Wide offsetToManifoldCenterA, in Vector3Wide offsetToManifoldCenterB, in BodyInertiaWide inertiaA, in BodyInertiaWide inertiaB,
            in Vector<float> maximumImpulse, ref Vector2Wide accumulatedImpulse, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            ComputeJacobians(tangentX, tangentY, offsetToManifoldCenterA, offsetToManifoldCenterB, out var jacobians);
            //Compute effective mass matrix contributions.
            Symmetric2x2Wide.SandwichScale(jacobians.LinearA, inertiaA.InverseMass, out var linearContributionA);
            Symmetric2x2Wide.SandwichScale(jacobians.LinearA, inertiaB.InverseMass, out var linearContributionB);

            Symmetric3x3Wide.MatrixSandwich(jacobians.AngularA, inertiaA.InverseInertiaTensor, out var angularContributionA);
            Symmetric3x3Wide.MatrixSandwich(jacobians.AngularB, inertiaB.InverseInertiaTensor, out var angularContributionB);

            //No softening; this constraint is rigid by design. (It does support a maximum force, but that is distinct from a proper damping ratio/natural frequency.)
            Symmetric2x2Wide.Add(linearContributionA, linearContributionB, out var linear);
            Symmetric2x2Wide.Add(angularContributionA, angularContributionB, out var angular);
            Symmetric2x2Wide.Add(linear, angular, out var inverseEffectiveMass);
            Symmetric2x2Wide.InvertWithoutOverlap(inverseEffectiveMass, out var effectiveMass);

            ComputeCorrectiveImpulse(wsvA, wsvB, effectiveMass, jacobians, maximumImpulse, ref accumulatedImpulse, out var correctiveCSI);
            ApplyImpulse(jacobians, inertiaA, inertiaB, correctiveCSI, ref wsvA, ref wsvB);
        }
    }
}
