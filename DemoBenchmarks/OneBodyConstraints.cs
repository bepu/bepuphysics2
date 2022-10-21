using BenchmarkDotNet.Attributes;
using BepuPhysics;
using BepuPhysics.Constraints;
using BepuPhysics.Constraints.Contact;
using BepuUtilities;
using System.Numerics;

namespace DemoBenchmarks;

/// <summary>
/// Evaluates performance of all one body constraints.
/// </summary>
/// <remarks>
/// Note that all constraints operate across <see cref="Vector{}.Count"/> lanes simultaneously where T is of type <see cref="float"/>.
/// <para>The number of bundles being executed does not change if <see cref="Vector{}.Count"/> changes; if larger bundles are allowed, then more lanes end up getting solved.</para>
/// </remarks>
public class OneBodyConstraintBenchmarks
{
    static BodyVelocityWide BenchmarkOneBodyConstraint<TConstraintFunctions, TPrestep, TAccumulatedImpulse>(Vector3Wide positionA, QuaternionWide orientationA, BodyInertiaWide inertiaA, TPrestep prestep)
        where TConstraintFunctions : unmanaged, IOneBodyConstraintFunctions<TPrestep, TAccumulatedImpulse> where TPrestep : unmanaged where TAccumulatedImpulse : unmanaged
    {
        var functions = default(TConstraintFunctions);
        var accumulatedImpulse = default(TAccumulatedImpulse);
        var velocityA = default(BodyVelocityWide);
        //Individual constraint iterations are often extremely cheap, so beef the benchmark up a bit.
        const int iterations = 10000;
        const float inverseDt = 60f;
        const float dt = 1f / inverseDt;
        for (int i = 0; i < iterations; ++i)
        {
            functions.WarmStart(positionA, orientationA, inertiaA, ref prestep, ref accumulatedImpulse, ref velocityA);
            functions.Solve(positionA, orientationA, inertiaA, dt, inverseDt, ref prestep, ref accumulatedImpulse, ref velocityA);
        }
        return velocityA;
    }

    [Benchmark]
    public BodyVelocityWide Contact1OneBody()
    {
        var prestep = new Contact1OneBodyPrestepData
        {
            Contact0 = new() { Depth = Vector<float>.Zero, OffsetA = Vector3Wide.Broadcast(new Vector3(1, 0, 0)) },
            MaterialProperties = new MaterialPropertiesWide
            {
                FrictionCoefficient = new Vector<float>(1f),
                MaximumRecoveryVelocity = new Vector<float>(2f),
                SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
            },
            Normal = Vector3Wide.Broadcast(new Vector3(0, 1, 0))
        };

        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkOneBodyConstraint<Contact1OneBodyFunctions, Contact1OneBodyPrestepData, Contact1AccumulatedImpulses>(new Vector3Wide(), orientation, inertia, prestep);
    }

    [Benchmark]
    public BodyVelocityWide Contact2OneBody()
    {
        var prestep = new Contact2OneBodyPrestepData
        {
            Contact0 = new() { Depth = Vector<float>.Zero, OffsetA = Vector3Wide.Broadcast(new Vector3(1, 0, 0)) },
            Contact1 = new() { Depth = Vector<float>.Zero, OffsetA = Vector3Wide.Broadcast(new Vector3(1, 0, 0)) },
            MaterialProperties = new MaterialPropertiesWide
            {
                FrictionCoefficient = new Vector<float>(1f),
                MaximumRecoveryVelocity = new Vector<float>(2f),
                SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
            },
            Normal = Vector3Wide.Broadcast(new Vector3(0, 1, 0))
        };

        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkOneBodyConstraint<Contact2OneBodyFunctions, Contact2OneBodyPrestepData, Contact2AccumulatedImpulses>(new Vector3Wide(), orientation, inertia, prestep);
    }

    [Benchmark]
    public BodyVelocityWide Contact3OneBody()
    {
        var prestep = new Contact3OneBodyPrestepData
        {
            Contact0 = new() { Depth = Vector<float>.Zero, OffsetA = Vector3Wide.Broadcast(new Vector3(1, 0, 0)) },
            Contact1 = new() { Depth = Vector<float>.Zero, OffsetA = Vector3Wide.Broadcast(new Vector3(1, 0, 0)) },
            Contact2 = new() { Depth = Vector<float>.Zero, OffsetA = Vector3Wide.Broadcast(new Vector3(1, 0, 0)) },
            MaterialProperties = new MaterialPropertiesWide
            {
                FrictionCoefficient = new Vector<float>(1f),
                MaximumRecoveryVelocity = new Vector<float>(2f),
                SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
            },
            Normal = Vector3Wide.Broadcast(new Vector3(0, 1, 0))
        };

        QuaternionWide.Broadcast(Quaternion.Identity, out var orientationA);
        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkOneBodyConstraint<Contact3OneBodyFunctions, Contact3OneBodyPrestepData, Contact3AccumulatedImpulses>(new Vector3Wide(), orientationA, inertia, prestep);
    }
    [Benchmark]
    public BodyVelocityWide Contact4OneBody()
    {
        var prestep = new Contact4OneBodyPrestepData
        {
            Contact0 = new() { Depth = Vector<float>.Zero, OffsetA = Vector3Wide.Broadcast(new Vector3(1, 0, 0)) },
            Contact1 = new() { Depth = Vector<float>.Zero, OffsetA = Vector3Wide.Broadcast(new Vector3(1, 0, 0)) },
            Contact2 = new() { Depth = Vector<float>.Zero, OffsetA = Vector3Wide.Broadcast(new Vector3(1, 0, 0)) },
            Contact3 = new() { Depth = Vector<float>.Zero, OffsetA = Vector3Wide.Broadcast(new Vector3(1, 0, 0)) },
            MaterialProperties = new MaterialPropertiesWide
            {
                FrictionCoefficient = new Vector<float>(1f),
                MaximumRecoveryVelocity = new Vector<float>(2f),
                SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
            },
            Normal = Vector3Wide.Broadcast(new Vector3(0, 1, 0))
        };

        QuaternionWide.Broadcast(Quaternion.Identity, out var orientationA);
        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkOneBodyConstraint<Contact4OneBodyFunctions, Contact4OneBodyPrestepData, Contact4AccumulatedImpulses>(new Vector3Wide(), orientationA, inertia, prestep);
    }

    [Benchmark]
    public BodyVelocityWide Contact2NonconvexOneBody()
    {
        var prestep = new Contact2NonconvexOneBodyPrestepData
        {
            Contact0 = new() { Depth = Vector<float>.Zero, Offset = Vector3Wide.Broadcast(new Vector3(1, 0, 0)), Normal = Vector3Wide.Broadcast(new Vector3(0, 1, 0)) },
            Contact1 = new() { Depth = Vector<float>.Zero, Offset = Vector3Wide.Broadcast(new Vector3(1, 0, 0)), Normal = Vector3Wide.Broadcast(new Vector3(0, 1, 0)) },
            MaterialProperties = new MaterialPropertiesWide
            {
                FrictionCoefficient = new Vector<float>(1f),
                MaximumRecoveryVelocity = new Vector<float>(2f),
                SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
            },
        };

        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkOneBodyConstraint<ContactNonconvexOneBodyFunctions<Contact2NonconvexOneBodyPrestepData, Contact2NonconvexAccumulatedImpulses>,
            Contact2NonconvexOneBodyPrestepData, Contact2NonconvexAccumulatedImpulses>(new Vector3Wide(), orientation, inertia, prestep);
    }

    [Benchmark]
    public BodyVelocityWide Contact3NonconvexOneBody()
    {
        var prestep = new Contact3NonconvexOneBodyPrestepData
        {
            Contact0 = new() { Depth = Vector<float>.Zero, Offset = Vector3Wide.Broadcast(new Vector3(1, 0, 0)), Normal = Vector3Wide.Broadcast(new Vector3(0, 1, 0)) },
            Contact1 = new() { Depth = Vector<float>.Zero, Offset = Vector3Wide.Broadcast(new Vector3(1, 0, 0)), Normal = Vector3Wide.Broadcast(new Vector3(0, 1, 0)) },
            Contact2 = new() { Depth = Vector<float>.Zero, Offset = Vector3Wide.Broadcast(new Vector3(1, 0, 0)), Normal = Vector3Wide.Broadcast(new Vector3(0, 1, 0)) },
            MaterialProperties = new MaterialPropertiesWide
            {
                FrictionCoefficient = new Vector<float>(1f),
                MaximumRecoveryVelocity = new Vector<float>(2f),
                SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
            },
        };

        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkOneBodyConstraint<ContactNonconvexOneBodyFunctions<Contact3NonconvexOneBodyPrestepData, Contact3NonconvexAccumulatedImpulses>,
            Contact3NonconvexOneBodyPrestepData, Contact3NonconvexAccumulatedImpulses>(new Vector3Wide(), orientation, inertia, prestep);
    }

    [Benchmark]
    public BodyVelocityWide Contact4NonconvexOneBody()
    {
        var prestep = new Contact4NonconvexOneBodyPrestepData
        {
            Contact0 = new() { Depth = Vector<float>.Zero, Offset = Vector3Wide.Broadcast(new Vector3(1, 0, 0)), Normal = Vector3Wide.Broadcast(new Vector3(0, 1, 0)) },
            Contact1 = new() { Depth = Vector<float>.Zero, Offset = Vector3Wide.Broadcast(new Vector3(1, 0, 0)), Normal = Vector3Wide.Broadcast(new Vector3(0, 1, 0)) },
            Contact2 = new() { Depth = Vector<float>.Zero, Offset = Vector3Wide.Broadcast(new Vector3(1, 0, 0)), Normal = Vector3Wide.Broadcast(new Vector3(0, 1, 0)) },
            Contact3 = new() { Depth = Vector<float>.Zero, Offset = Vector3Wide.Broadcast(new Vector3(1, 0, 0)), Normal = Vector3Wide.Broadcast(new Vector3(0, 1, 0)) },
            MaterialProperties = new MaterialPropertiesWide
            {
                FrictionCoefficient = new Vector<float>(1f),
                MaximumRecoveryVelocity = new Vector<float>(2f),
                SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
            },
        };

        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkOneBodyConstraint<ContactNonconvexOneBodyFunctions<Contact4NonconvexOneBodyPrestepData, Contact4NonconvexAccumulatedImpulses>,
            Contact4NonconvexOneBodyPrestepData, Contact4NonconvexAccumulatedImpulses>(new Vector3Wide(), orientation, inertia, prestep);
    }

    [Benchmark]
    public BodyVelocityWide OneBodyAngularServo()
    {
        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var prestep = new OneBodyAngularServoPrestepData
        {
            ServoSettings = new() { BaseSpeed = Vector<float>.Zero, MaximumForce = new Vector<float>(float.MaxValue), MaximumSpeed = new Vector<float>(float.MaxValue) },
            SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) },
            TargetOrientation = orientation
        };

        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkOneBodyConstraint<OneBodyAngularServoFunctions, OneBodyAngularServoPrestepData, Vector3Wide>(new Vector3Wide(), orientation, inertia, prestep);
    }

    [Benchmark]
    public BodyVelocityWide OneBodyAngularMotor()
    {
        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var prestep = new OneBodyAngularMotorPrestepData
        {
            Settings = new() { Damping = Vector<float>.One, MaximumForce = Vector<float>.One },
            TargetVelocity = default
        };

        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkOneBodyConstraint<OneBodyAngularMotorFunctions, OneBodyAngularMotorPrestepData, Vector3Wide>(new Vector3Wide(), orientation, inertia, prestep);
    }

    [Benchmark]
    public BodyVelocityWide OneBodyLinearServo()
    {
        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var prestep = new OneBodyLinearServoPrestepData
        {
            ServoSettings = new() { BaseSpeed = Vector<float>.Zero, MaximumForce = new Vector<float>(float.MaxValue), MaximumSpeed = new Vector<float>(float.MaxValue) },
            SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
        };

        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkOneBodyConstraint<OneBodyLinearServoFunctions, OneBodyLinearServoPrestepData, Vector3Wide>(new Vector3Wide(), orientation, inertia, prestep);
    }

    [Benchmark]
    public BodyVelocityWide OneBodyLinearMotor()
    {
        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var prestep = new OneBodyLinearMotorPrestepData
        {
            Settings = new() { Damping = Vector<float>.One, MaximumForce = Vector<float>.One },
        };

        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkOneBodyConstraint<OneBodyLinearMotorFunctions, OneBodyLinearMotorPrestepData, Vector3Wide>(new Vector3Wide(), orientation, inertia, prestep);
    }
}
