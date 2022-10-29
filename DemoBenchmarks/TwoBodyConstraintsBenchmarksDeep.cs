using BenchmarkDotNet.Attributes;
using BepuPhysics;
using BepuPhysics.Constraints;
using BepuPhysics.Constraints.Contact;
using BepuUtilities;
using System.Numerics;
using static DemoBenchmarks.TwoBodyConstraintBenchmarks;

namespace DemoBenchmarks;

/// <summary>
/// Evaluates performance of all two body constraints excluded from <see cref="TwoBodyConstraintBenchmarks"/>.
/// </summary>
/// <remarks>
/// Note that all constraints operate across <see cref="Vector{}.Count"/> lanes simultaneously where T is of type <see cref="float"/>.
/// <para>The number of bundles being executed does not change if <see cref="Vector{}.Count"/> changes; if larger bundles are allowed, then more lanes end up getting solved.</para>
/// </remarks>
public class TwoBodyConstraintBenchmarksDeep
{
    [Benchmark]
    public (BodyVelocityWide, BodyVelocityWide) Contact1()
    {
        var prestep = new Contact1PrestepData
        {
            Contact0 = new ConvexContactWide { Depth = Vector<float>.Zero, OffsetA = Vector3Wide.Broadcast(new Vector3(1, 0, 0)) },
            MaterialProperties = new MaterialPropertiesWide
            {
                FrictionCoefficient = new Vector<float>(1f),
                MaximumRecoveryVelocity = new Vector<float>(2f),
                SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
            },
            Normal = Vector3Wide.Broadcast(new Vector3(0, 1, 0)),
            OffsetB = Vector3Wide.Broadcast(new Vector3(2, 0, 0))
        };

        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<Contact1Functions, Contact1PrestepData, Contact1AccumulatedImpulses>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
    }

    [Benchmark]
    public (BodyVelocityWide, BodyVelocityWide) Contact2()
    {
        var prestep = new Contact2PrestepData
        {
            Contact0 = new ConvexContactWide { Depth = Vector<float>.Zero, OffsetA = Vector3Wide.Broadcast(new Vector3(1, 0, 0)) },
            Contact1 = new ConvexContactWide { Depth = Vector<float>.Zero, OffsetA = Vector3Wide.Broadcast(new Vector3(1, 0, 0)) },
            MaterialProperties = new MaterialPropertiesWide
            {
                FrictionCoefficient = new Vector<float>(1f),
                MaximumRecoveryVelocity = new Vector<float>(2f),
                SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
            },
            Normal = Vector3Wide.Broadcast(new Vector3(0, 1, 0)),
            OffsetB = Vector3Wide.Broadcast(new Vector3(2, 0, 0))
        };

        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<Contact2Functions, Contact2PrestepData, Contact2AccumulatedImpulses>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
    }

    [Benchmark]
    public (BodyVelocityWide, BodyVelocityWide) Contact3()
    {
        var prestep = new Contact3PrestepData
        {
            Contact0 = new ConvexContactWide { Depth = Vector<float>.Zero, OffsetA = Vector3Wide.Broadcast(new Vector3(1, 0, 0)) },
            Contact1 = new ConvexContactWide { Depth = Vector<float>.Zero, OffsetA = Vector3Wide.Broadcast(new Vector3(1, 0, 0)) },
            Contact2 = new ConvexContactWide { Depth = Vector<float>.Zero, OffsetA = Vector3Wide.Broadcast(new Vector3(1, 0, 0)) },
            MaterialProperties = new MaterialPropertiesWide
            {
                FrictionCoefficient = new Vector<float>(1f),
                MaximumRecoveryVelocity = new Vector<float>(2f),
                SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
            },
            Normal = Vector3Wide.Broadcast(new Vector3(0, 1, 0)),
            OffsetB = Vector3Wide.Broadcast(new Vector3(2, 0, 0))
        };

        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<Contact3Functions, Contact3PrestepData, Contact3AccumulatedImpulses>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
    }

    [Benchmark]
    public (BodyVelocityWide, BodyVelocityWide) Contact2Nonconvex()
    {
        var prestep = new Contact2NonconvexPrestepData
        {
            Contact0 = new NonconvexContactPrestepData { Depth = Vector<float>.Zero, Offset = Vector3Wide.Broadcast(new Vector3(1, 0, 0)), Normal = Vector3Wide.Broadcast(new Vector3(0, 1, 0)) },
            Contact1 = new NonconvexContactPrestepData { Depth = Vector<float>.Zero, Offset = Vector3Wide.Broadcast(new Vector3(1, 0, 1)), Normal = Vector3Wide.Broadcast(new Vector3(0, 1, 0)) },
            MaterialProperties = new MaterialPropertiesWide
            {
                FrictionCoefficient = new Vector<float>(1f),
                MaximumRecoveryVelocity = new Vector<float>(2f),
                SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
            },
            OffsetB = Vector3Wide.Broadcast(new Vector3(2, 0, 0))
        };

        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<ContactNonconvexTwoBodyFunctions<Contact2NonconvexPrestepData, Contact2NonconvexAccumulatedImpulses>,
            Contact2NonconvexPrestepData, Contact2NonconvexAccumulatedImpulses>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
    }

    [Benchmark]
    public (BodyVelocityWide, BodyVelocityWide) Contact3Nonconvex()
    {
        var prestep = new Contact3NonconvexPrestepData
        {
            Contact0 = new NonconvexContactPrestepData { Depth = Vector<float>.Zero, Offset = Vector3Wide.Broadcast(new Vector3(1, 0, 0)), Normal = Vector3Wide.Broadcast(new Vector3(0, 1, 0)) },
            Contact1 = new NonconvexContactPrestepData { Depth = Vector<float>.Zero, Offset = Vector3Wide.Broadcast(new Vector3(1, 0, 1)), Normal = Vector3Wide.Broadcast(new Vector3(0, 1, 0)) },
            Contact2 = new NonconvexContactPrestepData { Depth = Vector<float>.Zero, Offset = Vector3Wide.Broadcast(new Vector3(1, 1, 0)), Normal = Vector3Wide.Broadcast(new Vector3(0, 1, 0)) },
            MaterialProperties = new MaterialPropertiesWide
            {
                FrictionCoefficient = new Vector<float>(1f),
                MaximumRecoveryVelocity = new Vector<float>(2f),
                SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
            },
            OffsetB = Vector3Wide.Broadcast(new Vector3(2, 0, 0))
        };

        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<ContactNonconvexTwoBodyFunctions<Contact3NonconvexPrestepData, Contact3NonconvexAccumulatedImpulses>,
            Contact3NonconvexPrestepData, Contact3NonconvexAccumulatedImpulses>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
    }

    [Benchmark]
    public (BodyVelocityWide, BodyVelocityWide) TwistLimit()
    {
        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var prestep = new TwistLimitPrestepData
        {
            LocalBasisA = orientation,
            LocalBasisB = orientation,
            MinimumAngle = Vector<float>.Zero,
            MaximumAngle = Vector<float>.One,
            SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
        };

        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<TwistLimitFunctions, TwistLimitPrestepData, Vector<float>>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
    }

    [Benchmark]
    public (BodyVelocityWide, BodyVelocityWide) TwistMotor()
    {
        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var prestep = new TwistMotorPrestepData
        {
            LocalAxisA = Vector3Wide.Broadcast(new Vector3(0, 1, 0)),
            LocalAxisB = Vector3Wide.Broadcast(new Vector3(0, 1, 0)),
            Settings = new() { Damping = Vector<float>.One, MaximumForce = new Vector<float>(float.MaxValue) }
        };

        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<TwistMotorFunctions, TwistMotorPrestepData, Vector<float>>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
    }

    [Benchmark]
    public (BodyVelocityWide, BodyVelocityWide) AngularMotor()
    {
        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var prestep = new AngularMotorPrestepData
        {
            Settings = new() { Damping = Vector<float>.One, MaximumForce = new Vector<float>(float.MaxValue) }
        };

        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<AngularMotorFunctions, AngularMotorPrestepData, Vector3Wide>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
    }

    [Benchmark]
    public (BodyVelocityWide, BodyVelocityWide) DistanceLimit()
    {
        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var prestep = new DistanceLimitPrestepData
        {
            LocalOffsetA = Vector3Wide.Broadcast(new Vector3(0.5f, 0, 0)),
            LocalOffsetB = Vector3Wide.Broadcast(new Vector3(-0.5f, 0, 0)),
            MinimumDistance = new Vector<float>(0.5f),
            MaximumDistance = new Vector<float>(1.5f),
            SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
        };

        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<DistanceLimitFunctions, DistanceLimitPrestepData, Vector<float>>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
    }

    [Benchmark]
    public (BodyVelocityWide, BodyVelocityWide) LinearAxisMotor()
    {
        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var prestep = new LinearAxisMotorPrestepData
        {
            LocalOffsetA = Vector3Wide.Broadcast(new Vector3(1, 0, 0)),
            LocalOffsetB = Vector3Wide.Broadcast(new Vector3(-1, 0, 0)),
            LocalPlaneNormal = Vector3Wide.Broadcast(new Vector3(0, 1, 0)),
            Settings = new() { Damping = Vector<float>.One, MaximumForce = new Vector<float>(float.MaxValue) }
        };

        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<LinearAxisMotorFunctions, LinearAxisMotorPrestepData, Vector<float>>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
    }

    [Benchmark]
    public (BodyVelocityWide, BodyVelocityWide) LinearAxisLimit()
    {
        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var prestep = new LinearAxisLimitPrestepData
        {
            LocalOffsetA = Vector3Wide.Broadcast(new Vector3(1, 0, 0)),
            LocalOffsetB = Vector3Wide.Broadcast(new Vector3(-1, 0, 0)),
            LocalPlaneNormal = Vector3Wide.Broadcast(new Vector3(0, 1, 0)),
            MaximumOffset = new Vector<float>(3),
            MinimumOffset = new Vector<float>(-3),
            SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
        };

        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<LinearAxisLimitFunctions, LinearAxisLimitPrestepData, Vector<float>>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
    }

    [Benchmark]
    public (BodyVelocityWide, BodyVelocityWide) AngularAxisMotor()
    {
        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var prestep = new AngularAxisMotorPrestepData
        {
            LocalAxisA = Vector3Wide.Broadcast(new Vector3(0, 1, 0)),
            Settings = new() { Damping = Vector<float>.One, MaximumForce = new Vector<float>(float.MaxValue) }
        };

        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<AngularAxisMotorFunctions, AngularAxisMotorPrestepData, Vector<float>>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
    }

    [Benchmark]
    public (BodyVelocityWide, BodyVelocityWide) BallSocketServo()
    {
        var prestep = new BallSocketServoPrestepData
        {
            ServoSettings = new ServoSettingsWide { MaximumForce = new Vector<float>(float.MaxValue), MaximumSpeed = new Vector<float>(float.MaxValue) },
            SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
        };

        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<BallSocketServoFunctions, BallSocketServoPrestepData, Vector3Wide>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
    }

    [Benchmark]
    public (BodyVelocityWide, BodyVelocityWide) BallSocketMotor()
    {
        var prestep = new BallSocketMotorPrestepData
        {
            Settings = new() { Damping = Vector<float>.One, MaximumForce = new Vector<float>(float.MaxValue) }
        };

        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<BallSocketMotorFunctions, BallSocketMotorPrestepData, Vector3Wide>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
    }

    [Benchmark]
    public (BodyVelocityWide, BodyVelocityWide) AngularAxisGearMotor()
    {
        var prestep = new AngularAxisGearMotorPrestepData
        {
            LocalAxisA = Vector3Wide.Broadcast(new Vector3(0, 1, 0)),
            VelocityScale = Vector<float>.One,
            Settings = new() { Damping = Vector<float>.One, MaximumForce = new Vector<float>(float.MaxValue) }
        };

        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<AngularAxisGearMotorFunctions, AngularAxisGearMotorPrestepData, Vector<float>>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
    }

    [Benchmark]
    public (BodyVelocityWide, BodyVelocityWide) CenterDistanceLimit()
    {
        var prestep = new CenterDistanceLimitPrestepData
        {
            MinimumDistance = new Vector<float>(0.2f),
            MaximumDistance = new Vector<float>(3),
            SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
        };

        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<CenterDistanceLimitFunctions, CenterDistanceLimitPrestepData, Vector<float>>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
    }

}
