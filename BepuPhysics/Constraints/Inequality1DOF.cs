using BepuUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace BepuPhysics.Constraints
{ 
    //TODO: These are notes about the mathy bits underlying constraints. They were written for the original pre-2.4 version of the solver.
    //Most of it's still applicable, but 2.4 and up no longer have separate 'projection' states, and while packing is still important, it applies to *prestep data*.
    //There's a lot less room for tricky premultiplication to save memory bandwidth, simply because those quantities are all in registers/L1 cache during a constraint solve in 2.4+.
    //Would be nice to update those parts.

    internal struct TwoBody1DOFJacobians
    {
        public Vector3Wide LinearA;
        public Vector3Wide AngularA;
        public Vector3Wide LinearB;
        public Vector3Wide AngularB;
    }

    internal struct Projection2Body1DOF
    {
        //Rather than projecting from world space to constraint space *velocity* using JT, we precompute JT * effective mass
        //and go directly from world space velocity to constraint space impulse.
        public Vector3Wide WSVtoCSILinearA;
        public Vector3Wide WSVtoCSIAngularA;
        public Vector3Wide WSVtoCSILinearB;
        public Vector3Wide WSVtoCSIAngularB;

        //Since we jump directly from world space velocity to constraint space impulse, the velocity bias needs to be precomputed into an impulse offset too.
        public Vector<float> BiasImpulse;
        //And once again, CFM becomes CFM * EffectiveMass- massively cancels out due to the derivation of CFM. (See prestep notes.)
        public Vector<float> SoftnessImpulseScale;

        //It also needs to project from constraint space to world space.
        //We bundle this with the inertia/mass multiplier, so rather than taking a constraint impulse to world impulse and then to world velocity change,
        //we just go directly from constraint impulse to world velocity change.
        //For constraints with lower DOF counts, using this format also saves us some memory bandwidth- 
        //the inverse inertia tensor and inverse mass for a 2 body constraint cost 20 floats, compared to this implementation's 12.
        //(Note that even in an implementation where we use the body inertias, we should still cache it constraint-locally to avoid big gathers.)
        public Vector3Wide CSIToWSVLinearA;
        public Vector3Wide CSIToWSVAngularA;
        public Vector3Wide CSIToWSVLinearB;
        public Vector3Wide CSIToWSVAngularB;
    }

    internal static class Inequality2Body1DOF
    {

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Prestep(ref BodyInertiaWide inertiaA, ref BodyInertiaWide inertiaB, ref TwoBody1DOFJacobians jacobians, ref SpringSettingsWide springSettings, ref Vector<float> maximumRecoveryVelocity,
            ref Vector<float> positionError, float dt, float inverseDt, out Projection2Body1DOF projection)
        {
            //unsoftened effective mass = (J * M^-1 * JT)^-1
            //where J is a constraintDOF x bodyCount*6 sized matrix, JT is its transpose, and for two bodies M^-1 is:
            //[inverseMassA,    0, 0, 0]
            //[0, inverseInertiaA, 0, 0]
            //[0, 0, inverseMassB,    0]
            //[0, 0, 0, inverseInertiaB]
            //The entries of J match up to this convention, containing the linear and angular components of each body in sequence, so for a 2 body 1DOF constraint J would look like:
            //[linearA 1x3, angularA 1x3, linearB 1x3, angularB 1x3]
            //Note that it is a row vector by convention. When transforming velocities from world space into constraint space, it is assumed that the velocity vector is organized as a
            //row vector matching up to the jacobian (that is, [linearA 1x3, angularA 1x3, linearB 1x3, angularB 1x3]), so for a 2 body 2 DOF constraint,
            //worldVelocity * JT would be a [worldVelocity: 1x12] * [JT: 12x2], resulting in a 1x2 constraint space velocity row vector.
            //Similarly, when going from constraint space impulse to world space impulse in the above example, we would do [csi: 1x2] * [J: 2x12] to get a 1x12 world impulse row vector.

            //Note that the engine uses row vectors for all velocities and positions and so on. Rotation and inertia tensors are constructed for premultiplication. 
            //In other words, unlike many of the presentations in the space, we use v * JT and csi * J instead of J * v and JT * csi.
            //There is no meaningful difference- the two conventions are just transpositions of each other.

            //(If you want to know how this stuff works, go read the constraint related presentations: http://box2d.org/downloads/
            //Be mindful of the difference in conventions. You'll see J * v instead of v * JT, for example. Everything is still fundamentally the same, though.)

            //Due to the block structure of the mass matrix, we can handle each component separately and then sum the results.
            //For this 1DOF constraint, the result is a simple scalar.
            //Note that we store the intermediate results of J * M^-1 for use when projecting from constraint space impulses to world velocity changes. 
            //If we didn't store those intermediate values, we could just scale the dot product of jacobians.LinearA with itself to save 4 multiplies.
            Vector3Wide.Scale(jacobians.LinearA, inertiaA.InverseMass, out projection.CSIToWSVLinearA);
            Vector3Wide.Scale(jacobians.LinearB, inertiaB.InverseMass, out projection.CSIToWSVLinearB);
            Vector3Wide.Dot(projection.CSIToWSVLinearA, jacobians.LinearA, out var linearA);
            Vector3Wide.Dot(projection.CSIToWSVLinearB, jacobians.LinearB, out var linearB);

            //The angular components are a little more involved; (J * I^-1) * JT is explicitly computed.
            Symmetric3x3Wide.TransformWithoutOverlap(jacobians.AngularA, inertiaA.InverseInertiaTensor, out projection.CSIToWSVAngularA);
            Symmetric3x3Wide.TransformWithoutOverlap(jacobians.AngularB, inertiaB.InverseInertiaTensor, out projection.CSIToWSVAngularB);
            Vector3Wide.Dot(projection.CSIToWSVAngularA, jacobians.AngularA, out var angularA);
            Vector3Wide.Dot(projection.CSIToWSVAngularB, jacobians.AngularB, out var angularB);

            //Now for a digression!
            //Softness is applied along the diagonal (which, for a 1DOF constraint, is just the only element).
            //Check the the ODE reference for a bit more information: http://ode.org/ode-latest-userguide.html#sec_3_8_0
            //And also see Erin Catto's Soft Constraints presentation for more details: http://box2d.org/files/GDC2011/GDC2011_Catto_Erin_Soft_Constraints.pdf)

            //There are some very interesting tricks you can use here, though.
            //Our core tuning variables are the damping ratio and natural frequency.
            //Our runtime used variables are softness and an error reduction feedback scale..
            //(For the following, I'll use the ODE terms CFM and ERP, constraint force mixing and error reduction parameter.)
            //So first, we need to get from damping ratio and natural frequency to stiffness and damping spring constants.
            //From there, we'll go to CFM/ERP.
            //Then, we'll create an expression for a softened effective mass matrix (i.e. one that takes into account the CFM term),
            //and an expression for the contraint force mixing term in the solve iteration.
            //Finally, compute ERP.
            //(And then some tricks.)

            //1) Convert from damping ratio and natural frequency to stiffness and damping constants.
            //The raw expressions are:
            //stiffness = effectiveMass * naturalFrequency^2
            //damping = effectiveMass * 2 * dampingRatio * naturalFrequency
            //Rather than using any single object as the reference for the 'mass' term involved in this conversion, use the effective mass of the constraint.
            //In other words, we're dynamically picking the spring constants necessary to achieve the desired behavior for the current constraint configuration.
            //(See Erin Catto's presentation above for more details on this.)

            //(Note that this is different from BEPUphysics v1. There, users configured stiffness and damping constants. That worked okay, but people often got confused about
            //why constraints didn't behave the same when they changed masses. Usually it manifested as someone creating an incredibly high mass object relative to the default
            //stiffness/damping, and they'd post on the forum wondering why constraints were so soft. Basically, the defaults were another sneaky tuning factor to get wrong.
            //Since damping ratio and natural frequency define the behavior independent of the mass, this problem goes away- and it makes some other interesting things happen...)

            //2) Convert from stiffness and damping constants to CFM and ERP.
            //CFM = (stiffness * dt + damping)^-1
            //ERP = (stiffness * dt) * (stiffness * dt + damping)^-1
            //Or, to rephrase:
            //ERP = (stiffness * dt) * CFM

            //3) Use CFM and ERP to create a softened effective mass matrix and a force mixing term for the solve iterations.
            //Start with a base definition which we won't be deriving, the velocity constraint itself (stated as an equality constraint here):
            //This means 'world space velocity projected into constraint space should equal the velocity bias term combined with the constraint force mixing term'.
            //(The velocity bias term will be computed later- it's the position error scaled by the error reduction parameter, ERP. Position error is used to create a velocity motor goal.)
            //We're pulling back from the implementation of sequential impulses here, so rather than using the term 'accumulated impulse', we'll use 'lambda'
            //(which happens to be consistent with the ODE documentation covering the same topic). Lambda is impulse that satisfies the constraint.
            //wsv * JT = bias - lambda * CFM/dt
            //This can be phrased as:
            //currentVelocity = targetVelocity
            //Or:
            //goalVelocityChange = targetVelocity - currentVelocity
            //lambda = goalVelocityChange * effectiveMass
            //lambda = (targetVelocity - currentVelocity) * effectiveMass
            //lambda = (bias - lambda * CFM/dt - currentVelocity) * effectiveMass
            //Solving for lambda:
            //lambda = (bias - currentVelocity) * effectiveMass - lambda * CFM/dt * effectiveMass
            //lambda + lambda * CFM/dt * effectiveMass = (bias - currentVelocity) * effectiveMass 
            //(lambda + lambda * CFM/dt * effectiveMass) * effectiveMass^-1 = bias - currentVelocity
            //lambda * effectiveMass^-1 + lambda * CFM/dt = bias - currentVelocity
            //lambda * (effectiveMass^-1 + CFM/dt) = bias - currentVelocity
            //lambda = (bias - currentVelocity) * (effectiveMass^-1 + CFM/dt)^-1
            //lambda = (bias - wsv * JT) * (effectiveMass^-1 + CFM/dt)^-1
            //In other words, we transform the velocity change (bias - wsv * JT) into the constraint-satisfying impulse, lambda, using a matrix (effectiveMass^-1 + CFM/dt)^-1.
            //That matrix is the softened effective mass:
            //softenedEffectiveMass = (effectiveMass^-1 + CFM/dt)^-1

            //Here's where some trickiness occurs. (Be mindful of the distinction between the softened and unsoftened effective mass).
            //Start by substituting CFM into the softened effective mass definition:
            //CFM/dt = (stiffness * dt + damping)^-1 / dt = (dt * (stiffness * dt + damping))^-1 = (stiffness * dt^2 + damping*dt)^-1
            //softenedEffectiveMass = (effectiveMass^-1 + (stiffness * dt^2 + damping * dt)^-1)^-1
            //Now substitute the definitions of stiffness and damping, treating the scalar components as uniform scaling matrices of dimension equal to effectiveMass:
            //softenedEffectiveMass = (effectiveMass^-1 + ((effectiveMass * naturalFrequency^2) * dt^2 + (effectiveMass * 2 * dampingRatio * naturalFrequency) * dt)^-1)^-1
            //Combine the inner effectiveMass coefficients, given matrix multiplication distributes over addition:
            //softenedEffectiveMass = (effectiveMass^-1 + (effectiveMass * (naturalFrequency^2 * dt^2) + effectiveMass * (2 * dampingRatio * naturalFrequency * dt))^-1)^-1
            //softenedEffectiveMass = (effectiveMass^-1 + (effectiveMass * (naturalFrequency^2 * dt^2 + 2 * dampingRatio * naturalFrequency * dt))^-1)^-1
            //Apply the inner matrix inverse:
            //softenedEffectiveMass = (effectiveMass^-1 + (naturalFrequency^2 * dt^2 + 2 * dampingRatio * naturalFrequency * dt)^-1 * effectiveMass^-1)^-1
            //Once again, combine coefficients of the inner effectiveMass^-1 terms:
            //softenedEffectiveMass = ((1 + (naturalFrequency^2 * dt^2 + 2 * dampingRatio * naturalFrequency * dt)^-1) * effectiveMass^-1)^-1
            //Apply the inverse again:
            //softenedEffectiveMass = effectiveMass * (1 + (naturalFrequency^2 * dt^2 + 2 * dampingRatio * naturalFrequency * dt)^-1)^-1

            //So, to put it another way- because CFM is based on the effective mass, applying it to the effective mass results in a simple downscale.

            //What has been gained? Consider what happens in the solve iteration.
            //We take the velocity error:
            //velocityError = bias - accumulatedImpulse * CFM/dt - wsv * JT 
            //and convert it to a corrective impulse with the effective mass:
            //impulse = (bias - accumulatedImpulse * CFM/dt - wsv * JT) * softenedEffectiveMass
            //The effective mass distributes over the set:
            //impulse = bias * softenedEffectiveMass - accumulatedImpulse * CFM/dt * softenedEffectiveMass - wsv * JT * softenedEffectiveMass
            //Focus on the CFM term:
            //-accumulatedImpulse * CFM/dt * softenedEffectiveMass
            //What is CFM/dt * softenedEffectiveMass? Substitute.
            //(stiffness * dt^2 + damping * dt)^-1 * softenedEffectiveMass
            //((effectiveMass * naturalFrequency^2) * dt^2 + (effectiveMass * 2 * dampingRatio * naturalFrequency * dt))^-1 * softenedEffectiveMass
            //Combine terms: 
            //(effectiveMass * (naturalFrequency^2 * dt^2 + 2 * dampingRatio * naturalFrequency * dt))^-1 * softenedEffectiveMass
            //Apply inverse:
            //(naturalFrequency^2 * dt^2 + 2 * dampingRatio * naturalFrequency * dt)^-1 * effectiveMass^-1 * softenedEffectiveMass
            //Expand softened effective mass from earlier:
            //(naturalFrequency^2 * dt^2 + 2 * dampingRatio * naturalFrequency * dt)^-1 * effectiveMass^-1 * effectiveMass * (1 + (naturalFrequency^2 * dt^2 + 2 * dampingRatio * naturalFrequency * dt)^-1)^-1
            //Cancel effective masses: (!)
            //(naturalFrequency^2 * dt^2 + 2 * dampingRatio * naturalFrequency * dt)^-1 * (1 + (naturalFrequency^2 * dt^2 + 2 * dampingRatio * naturalFrequency * dt)^-1)^-1
            //Because CFM was created from effectiveMass, the CFM/dt * effectiveMass term is actually independent of the effectiveMass!
            //The remaining expression is still a matrix, but fortunately it is a simple uniform scaling matrix that we can store and apply as a single scalar.

            //4) How do you compute ERP?
            //ERP = (stiffness * dt) * CFM
            //ERP = (stiffness * dt) * (stiffness * dt + damping)^-1
            //ERP = ((effectiveMass * naturalFrequency^2) * dt) * ((effectiveMass * naturalFrequency^2) * dt + (effectiveMass * 2 * dampingRatio * naturalFrequency))^-1
            //Combine denominator terms:
            //ERP = ((effectiveMass * naturalFrequency^2) * dt) * ((effectiveMass * (naturalFrequency^2 * dt + 2 * dampingRatio * naturalFrequency))^-1
            //Apply denominator inverse:
            //ERP = ((effectiveMass * naturalFrequency^2) * dt) * (naturalFrequency^2 * dt + 2 * dampingRatio * naturalFrequency)^-1 * effectiveMass^-1
            //Uniform scaling matrices commute:
            //ERP = (naturalFrequency^2 * dt) * effectiveMass * effectiveMass^-1 * (naturalFrequency^2 * dt + 2 * dampingRatio * naturalFrequency)^-1
            //Cancellation!
            //ERP = (naturalFrequency^2 * dt) * (naturalFrequency^2 * dt + 2 * dampingRatio * naturalFrequency)^-1
            //ERP = (naturalFrequency * dt) * (naturalFrequency * dt + 2 * dampingRatio)^-1
            //ERP is a simple scalar, independent of mass.

            //5) So we can compute CFM, ERP, the softened effective mass matrix, and we have an interesting shortcut on the constraint force mixing term of the solve iterations.
            //Is there anything more that can be done? You bet!
            //Let's look at the post-distribution impulse computation again:
            //impulse = bias * effectiveMass - accumulatedImpulse * CFM/dt * effectiveMass - wsv * JT * effectiveMass
            //During the solve iterations, the only quantities that vary are the accumulated impulse and world space velocities. So the rest can be precomputed.
            //bias * effectiveMass,
            //CFM/dt * effectiveMass,
            //JT * effectiveMass
            //In other words, we bypass the intermediate velocity state and go directly from source velocities to an impulse.
            //Note the sizes of the precomputed types above:
            //bias * effective mass is the same size as bias (vector with dimension equal to constrained DOFs)
            //CFM/dt * effectiveMass is a single scalar regardless of constrained DOFs,
            //JT * effectiveMass is the same size as JT
            //But note that we no longer need to load the effective mass! It is implicit.
            //The resulting computation is:
            //impulse = a - accumulatedImpulse * b - wsv * c
            //two DOF-width adds (add/subtract), one DOF-width multiply, and a 1xDOF * DOFx12 jacobian-sized transform.
            //Compare to;
            //(bias - accumulatedImpulse * CFM/dt - wsv * JT) * effectiveMass
            //two DOF-width adds (add/subtract), one DOF width multiply, a 1xDOF * DOFx12 jacobian-sized transform, and a 1xDOF * DOFxDOF transform.
            //In other words, we shave off a whole 1xDOF * DOFxDOF transform per iteration.
            //So, taken in isolation, this is a strict win both in terms of memory and the amount of computation.

            //Unfortunately, it's not quite so simple- jacobians are ALSO used to transform the impulse into world space so that it can be used to change the body velocities.
            //We still need to have those around. So while we no longer store the effective mass, our jacobian has sort of been duplicated.
            //But wait, there's more!

            //That process looks like:
            //wsv += impulse * J * M^-1
            //So while we need to store something here, we can take advantage of the fact that we aren't using the jacobian anywhere else (it's replaced by the JT * effectiveMass term above).
            //Precompute J*M^-1, too.
            //So you're still loading a jacobian-sized matrix, but you don't need to load M^-1! That saves you 14 scalars. (symmetric 3x3 + 1 + symmetric 3x3 + 1)
            //That saves you the multiplication of (impulse * J) * M^-1, which is 6 multiplies and 6 dot products.

            //Note that this optimization's value depends on the number of constrained DOFs.

            //Net memory change, opt vs no opt, in scalars:
            //1DOF: costs 1x12, saves 1x1 effective mass and the 14 scalar M^-1: -3
            //2DOF: costs 2x12, saves 2x2 symmetric effective mass and the 14 scalar M^-1: 7
            //3DOF: costs 3x12, saves 3x3 symmetric effective mass and the 14 scalar M^-1: 16
            //4DOF: costs 4x12, saves 4x4 symmetric effective mass and the 14 scalar M^-1: 24
            //5DOF: costs 5x12, saves 5x5 symmetric effective mass and the 14 scalar M^-1: 31
            //6DOF: costs 6x12, saves 6x6 symmetric effective mass and the 14 scalar M^-1: 37

            //Net compute savings, opt vs no opt:
            //DOF savings = 1xDOF * DOFxDOF (DOF DOFdot products), 2 1x3 * scalar (6 multiplies), 2 1x3 * 3x3 (6 3dot products)
            //            = (DOF*DOF multiplies + DOF*(DOF-1) adds) + (6 multiplies) + (18 multiplies + 12 adds)
            //            = DOF*DOF + 24 multiplies, DOF*DOF-DOF + 12 adds
            //1DOF: 25 multiplies, 12 adds
            //2DOF: 28 multiplies, 14 adds
            //3DOF: 33 multiplies, 18 adds
            //4DOF: 40 multiplies, 24 adds
            //5DOF: 49 multiplies, 32 adds
            //6DOF: 60 multiplies, 42 adds

            //So does our 'optimization' actually do anything useful? 
            //In 1 DOF constraints, it's often a win with no downsides.
            //2+ are difficult to determine.
            //This depends on heavily on the machine's SIMD width. You do every lane's ALU ops in parallel, but the loads are still fundamentally bound by memory bandwidth.
            //The loads are coherent, at least- no gathers on this stuff. But I wouldn't be surprised if 3DOF+ constraints end up being faster *without* the pretransformations on wide SIMD.
            //This is just something that will require case by case analysis. Constraints can have special structure which change the judgment.

            //(Also, note that large DOF jacobians are often very sparse. Consider the jacobians used by a 6DOF weld joint. You could likely do special case optimizations to reduce the
            //load further. It is unlikely that you could find a way to do the same to JT * effectiveMass. J * M^-1 might have some savings, though. But J*M^-1 isn't *sparser*
            //than J by itself, so the space savings are limited. As long as you precompute, the above load requirement offset will persist.)

            //Good news, though! There are a lot of constraints where this trick is applicable.

            //We'll start with the unsoftened effective mass, constructed from the contributions computed above:
            var effectiveMass = Vector<float>.One / (linearA + linearB + angularA + angularB);

            SpringSettingsWide.ComputeSpringiness(springSettings, dt, out var positionErrorToVelocity, out var effectiveMassCFMScale, out projection.SoftnessImpulseScale);
            var softenedEffectiveMass = effectiveMass * effectiveMassCFMScale;
            
            //Note that we use a bit of a hack when computing the bias velocity- even if our damping ratio/natural frequency implies a strongly springy response
            //that could cause a significant velocity overshoot, we apply an arbitrary clamping value to keep it reasonable.
            //This is useful for a variety of inequality constraints (like contacts) because you don't always want them behaving as true springs.
            var biasVelocity = Vector.Min(positionError * positionErrorToVelocity, maximumRecoveryVelocity);
            projection.BiasImpulse = biasVelocity * softenedEffectiveMass;

            //Precompute the wsv * (JT * softenedEffectiveMass) term.
            //Note that we store it in a Vector3Wide as if it's a row vector, but this is really a column (because JT is a column vector).
            //So we're really storing (JT * softenedEffectiveMass)T = softenedEffectiveMassT * J.
            //Since this constraint is 1DOF, the softenedEffectiveMass is a scalar and the order doesn't matter.
            //In the solve iterations, the WSVtoCSI term will be transposed during transformation,
            //resulting in the proper wsv * (softenedEffectiveMassT * J)T = wsv * (JT * softenedEffectiveMass).
            //You'll see this pattern repeated in higher DOF constraints. We explicitly compute softenedEffectiveMassT * J, and then apply the transpose in the solves.
            //(Why? Because creating a Matrix3x2 and Matrix2x3 and 4x3 and 3x4 and 5x3 and 3x5 and so on just doubles the number of representations with little value.)
            Vector3Wide.Scale(jacobians.LinearA, softenedEffectiveMass, out projection.WSVtoCSILinearA);
            Vector3Wide.Scale(jacobians.AngularA, softenedEffectiveMass, out projection.WSVtoCSIAngularA);
            Vector3Wide.Scale(jacobians.LinearB, softenedEffectiveMass, out projection.WSVtoCSILinearB);
            Vector3Wide.Scale(jacobians.AngularB, softenedEffectiveMass, out projection.WSVtoCSIAngularB);
        }
        //Naming conventions:
        //We transform between two spaces, world and constraint space. We also deal with two quantities- velocities, and impulses. 
        //And we have some number of entities involved in the constraint. So:
        //wsva: world space velocity of body A
        //wsvb: world space velocity of body B
        //csvError: constraint space velocity error- when the body velocities are projected into constraint space and combined with the velocity biases, the result is a single constraint velocity error
        //csva: constraint space velocity of body A; the world space velocities projected onto transpose(jacobianA)
        //csvaLinear: contribution to the constraint space velocity by body A's linear velocity


        /// <summary>
        /// Transforms an impulse from constraint space to world space, uses it to modify the cached world space velocities of the bodies.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ApplyImpulse(ref Projection2Body1DOF data, ref Vector<float> correctiveImpulse,
            ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            //Applying the impulse requires transforming the constraint space impulse into a world space velocity change.
            //The first step is to transform into a world space impulse, which requires transforming by the transposed jacobian
            //(transpose(jacobian) goes from world to constraint space, jacobian goes from constraint to world space).
            //That world space impulse is then converted to a corrective velocity change by scaling the impulse by the inverse mass/inertia.
            //As an optimization for constraints with smaller jacobians, the jacobian * (inertia or mass) transform is precomputed.
            BodyVelocityWide correctiveVelocityA, correctiveVelocityB;
            Vector3Wide.Scale(data.CSIToWSVLinearA, correctiveImpulse, out correctiveVelocityA.Linear);
            Vector3Wide.Scale(data.CSIToWSVAngularA, correctiveImpulse, out correctiveVelocityA.Angular);
            Vector3Wide.Scale(data.CSIToWSVLinearB, correctiveImpulse, out correctiveVelocityB.Linear);
            Vector3Wide.Scale(data.CSIToWSVAngularB, correctiveImpulse, out correctiveVelocityB.Angular);
            Vector3Wide.Add(correctiveVelocityA.Linear, wsvA.Linear, out wsvA.Linear);
            Vector3Wide.Add(correctiveVelocityA.Angular, wsvA.Angular, out wsvA.Angular);
            Vector3Wide.Add(correctiveVelocityB.Linear, wsvB.Linear, out wsvB.Linear);
            Vector3Wide.Add(correctiveVelocityB.Angular, wsvB.Angular, out wsvB.Angular);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void WarmStart(ref Projection2Body1DOF data, ref Vector<float> accumulatedImpulse, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            //TODO: If the previous frame and current frame are associated with different time steps, the previous frame's solution won't be a good solution anymore.
            //To compensate for this, the accumulated impulse should be scaled if dt changes.
            ApplyImpulse(ref data, ref accumulatedImpulse, ref wsvA, ref wsvB);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ComputeCorrectiveImpulse(ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB, ref Projection2Body1DOF projection, ref Vector<float> accumulatedImpulse,
            out Vector<float> correctiveCSI)
        {
            //Take the world space velocity of each body into constraint space by transforming by the transpose(jacobian).
            //(The jacobian is a row vector by convention, while we treat our velocity vectors as a 12x1 row vector for the purposes of constraint space velocity calculation.
            //So we are multiplying v * JT.)
            //Then, transform it into an impulse by applying the effective mass.
            //Here, we combine the projection and impulse conversion into a precomputed value, i.e. v * (JT * softenedEffectiveMass).
            Vector3Wide.Dot(wsvA.Linear, projection.WSVtoCSILinearA, out var csiaLinear);
            Vector3Wide.Dot(wsvA.Angular, projection.WSVtoCSIAngularA, out var csiaAngular);
            Vector3Wide.Dot(wsvB.Linear, projection.WSVtoCSILinearB, out var csibLinear);
            Vector3Wide.Dot(wsvB.Angular, projection.WSVtoCSIAngularB, out var csibAngular);
            //Combine it all together, following:
            //constraint space impulse = (targetVelocity - currentVelocity) * softenedEffectiveMass
            //constraint space impulse = (bias - accumulatedImpulse * softness - wsv * JT) * softenedEffectiveMass
            //constraint space impulse = (bias * softenedEffectiveMass) - accumulatedImpulse * (softness * softenedEffectiveMass) - wsv * (JT * softenedEffectiveMass)
            var csi = projection.BiasImpulse - accumulatedImpulse * projection.SoftnessImpulseScale - (csiaLinear + csiaAngular + csibLinear + csibAngular);

            var previousAccumulated = accumulatedImpulse;
            accumulatedImpulse = Vector.Max(Vector<float>.Zero, accumulatedImpulse + csi);

            correctiveCSI = accumulatedImpulse - previousAccumulated;

        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Solve(ref Projection2Body1DOF projection, ref Vector<float> accumulatedImpulse, ref BodyVelocityWide wsvA, ref BodyVelocityWide wsvB)
        {
            ComputeCorrectiveImpulse(ref wsvA, ref wsvB, ref projection, ref accumulatedImpulse, out var correctiveCSI);
            ApplyImpulse(ref projection, ref correctiveCSI, ref wsvA, ref wsvB);

        }

    }
}
