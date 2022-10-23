using BenchmarkDotNet.Attributes;
using BepuPhysics;
using BepuPhysics.Constraints;
using BepuPhysics.Constraints.Contact;
using BepuUtilities;
using System.Numerics;

namespace DemoBenchmarks;

/// <summary>
/// Evaluates performance of all two body constraints.
/// </summary>
/// <remarks>
/// Note that all constraints operate across <see cref="Vector{}.Count"/> lanes simultaneously where T is of type <see cref="float"/>.
/// <para>The number of bundles being executed does not change if <see cref="Vector{}.Count"/> changes; if larger bundles are allowed, then more lanes end up getting solved.</para>
/// </remarks>
public class TwoBodyConstraintBenchmarks
{
    static (BodyVelocityWide, BodyVelocityWide) BenchmarkTwoBodyConstraint<TConstraintFunctions, TPrestep, TAccumulatedImpulse>(
        Vector3Wide positionA, QuaternionWide orientationA, BodyInertiaWide inertiaA,
        Vector3Wide positionB, QuaternionWide orientationB, BodyInertiaWide inertiaB, TPrestep prestep)
        where TConstraintFunctions : unmanaged, ITwoBodyConstraintFunctions<TPrestep, TAccumulatedImpulse> where TPrestep : unmanaged where TAccumulatedImpulse : unmanaged
    {
        var functions = default(TConstraintFunctions);
        var accumulatedImpulse = default(TAccumulatedImpulse);
        var velocityA = default(BodyVelocityWide);
        var velocityB = default(BodyVelocityWide);
        //Individual constraint iterations are often extremely cheap, so beef the benchmark up a bit.
        const int iterations = 1000;
        const float inverseDt = 60f;
        const float dt = 1f / inverseDt;
        for (int i = 0; i < iterations; ++i)
        {
            functions.WarmStart(positionA, orientationA, inertiaA, positionB, orientationB, inertiaB, ref prestep, ref accumulatedImpulse, ref velocityA, ref velocityB);
            functions.Solve(positionA, orientationA, inertiaA, positionB, orientationB, inertiaB, dt, inverseDt, ref prestep, ref accumulatedImpulse, ref velocityA, ref velocityB);
        }
        return (velocityA, velocityB);
    }

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
    public (BodyVelocityWide, BodyVelocityWide) Contact4()
    {
        var prestep = new Contact4PrestepData
        {
            Contact0 = new ConvexContactWide { Depth = Vector<float>.Zero, OffsetA = Vector3Wide.Broadcast(new Vector3(1, 0, 0)) },
            Contact1 = new ConvexContactWide { Depth = Vector<float>.Zero, OffsetA = Vector3Wide.Broadcast(new Vector3(1, 0, 0)) },
            Contact2 = new ConvexContactWide { Depth = Vector<float>.Zero, OffsetA = Vector3Wide.Broadcast(new Vector3(1, 0, 0)) },
            Contact3 = new ConvexContactWide { Depth = Vector<float>.Zero, OffsetA = Vector3Wide.Broadcast(new Vector3(1, 0, 0)) },
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
        return BenchmarkTwoBodyConstraint<Contact4Functions, Contact4PrestepData, Contact4AccumulatedImpulses>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
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
    public (BodyVelocityWide, BodyVelocityWide) Contact4Nonconvex()
    {
        var prestep = new Contact4NonconvexPrestepData
        {
            Contact0 = new NonconvexContactPrestepData { Depth = Vector<float>.Zero, Offset = Vector3Wide.Broadcast(new Vector3(1, 0, 0)), Normal = Vector3Wide.Broadcast(new Vector3(0, 1, 0)) },
            Contact1 = new NonconvexContactPrestepData { Depth = Vector<float>.Zero, Offset = Vector3Wide.Broadcast(new Vector3(1, 0, 1)), Normal = Vector3Wide.Broadcast(new Vector3(0, 1, 0)) },
            Contact2 = new NonconvexContactPrestepData { Depth = Vector<float>.Zero, Offset = Vector3Wide.Broadcast(new Vector3(1, 1, 0)), Normal = Vector3Wide.Broadcast(new Vector3(0, 1, 0)) },
            Contact3 = new NonconvexContactPrestepData { Depth = Vector<float>.Zero, Offset = Vector3Wide.Broadcast(new Vector3(1, 1, 1)), Normal = Vector3Wide.Broadcast(new Vector3(0, 1, 0)) },
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
        return BenchmarkTwoBodyConstraint<ContactNonconvexTwoBodyFunctions<Contact4NonconvexPrestepData, Contact4NonconvexAccumulatedImpulses>,
            Contact4NonconvexPrestepData, Contact4NonconvexAccumulatedImpulses>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
    }

    [Benchmark]
    public (BodyVelocityWide, BodyVelocityWide) BallSocket()
    {
        var prestep = new BallSocketPrestepData
        {
            LocalOffsetA = Vector3Wide.Broadcast(new Vector3(1, 0, 0)),
            LocalOffsetB = Vector3Wide.Broadcast(new Vector3(-1, 0, 0)),
            SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
        };

        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<BallSocketFunctions, BallSocketPrestepData, Vector3Wide>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
    }

    [Benchmark]
    public (BodyVelocityWide, BodyVelocityWide) AngularHinge()
    {
        var prestep = new AngularHingePrestepData
        {
            LocalHingeAxisA = Vector3Wide.Broadcast(new Vector3(0, 1, 0)),
            LocalHingeAxisB = Vector3Wide.Broadcast(new Vector3(0, 1, 0)),
            SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
        };

        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<AngularHingeFunctions, AngularHingePrestepData, Vector2Wide>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
    }

    [Benchmark]
    public (BodyVelocityWide, BodyVelocityWide) AngularSwivelHinge()
    {
        var prestep = new AngularSwivelHingePrestepData
        {
            LocalSwivelAxisA = Vector3Wide.Broadcast(new Vector3(0, 1, 0)),
            LocalHingeAxisB = Vector3Wide.Broadcast(new Vector3(1, 0, 0)),
            SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
        };

        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<AngularSwivelHingeFunctions, AngularSwivelHingePrestepData, Vector<float>>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
    }

    [Benchmark]
    public (BodyVelocityWide, BodyVelocityWide) SwingLimit()
    {
        var prestep = new SwingLimitPrestepData
        {
            AxisLocalA = Vector3Wide.Broadcast(new Vector3(0, 1, 0)),
            AxisLocalB = Vector3Wide.Broadcast(new Vector3(0, 1, 0)),
            MinimumDot = new Vector<float>(0.9f),
            SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
        };

        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<SwingLimitFunctions, SwingLimitPrestepData, Vector<float>>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
    }

    [Benchmark]
    public (BodyVelocityWide, BodyVelocityWide) TwistServo()
    {
        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var prestep = new TwistServoPrestepData
        {
            LocalBasisA = orientation,
            LocalBasisB = orientation,
            ServoSettings = new ServoSettingsWide { MaximumForce = new Vector<float>(float.MaxValue), MaximumSpeed = new Vector<float>(float.MaxValue) },
            SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
        };

        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<TwistServoFunctions, TwistServoPrestepData, Vector<float>>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
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
    public (BodyVelocityWide, BodyVelocityWide) AngularServo()
    {
        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var prestep = new AngularServoPrestepData
        {
            TargetRelativeRotationLocalA = orientation,
            ServoSettings = new ServoSettingsWide { MaximumForce = new Vector<float>(float.MaxValue), MaximumSpeed = new Vector<float>(float.MaxValue) },
            SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
        };

        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<AngularServoFunctions, AngularServoPrestepData, Vector3Wide>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
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
    public (BodyVelocityWide, BodyVelocityWide) Weld()
    {
        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var prestep = new WeldPrestepData
        {
            LocalOffset = Vector3Wide.Broadcast(new Vector3(1, 0, 0)),
            LocalOrientation = orientation,
            SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
        };

        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<WeldFunctions, WeldPrestepData, WeldAccumulatedImpulses>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
    }

    [Benchmark]
    public (BodyVelocityWide, BodyVelocityWide) DistanceServo()
    {
        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var prestep = new DistanceServoPrestepData
        {
            LocalOffsetA = Vector3Wide.Broadcast(new Vector3(0.5f, 0, 0)),
            LocalOffsetB = Vector3Wide.Broadcast(new Vector3(-0.5f, 0, 0)),
            TargetDistance = Vector<float>.One,
            ServoSettings = new ServoSettingsWide { MaximumForce = new Vector<float>(float.MaxValue), MaximumSpeed = new Vector<float>(float.MaxValue) },
            SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
        };

        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<DistanceServoFunctions, DistanceServoPrestepData, Vector<float>>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
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
    public (BodyVelocityWide, BodyVelocityWide) CenterDistanceConstraint()
    {
        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var prestep = new CenterDistancePrestepData
        {
            TargetDistance = new Vector<float>(0.5f),
            SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
        };

        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<CenterDistanceConstraintFunctions, CenterDistancePrestepData, Vector<float>>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
    }

    [Benchmark]
    public (BodyVelocityWide, BodyVelocityWide) PointOnLineServo()
    {
        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var prestep = new PointOnLineServoPrestepData
        {
            LocalDirection = Vector3Wide.Broadcast(new Vector3(1, 0, 0)),
            LocalOffsetA = Vector3Wide.Broadcast(new Vector3(1, 0, 0)),
            LocalOffsetB = Vector3Wide.Broadcast(new Vector3(-1, 0, 0)),
            ServoSettings = new ServoSettingsWide { MaximumForce = new Vector<float>(float.MaxValue), MaximumSpeed = new Vector<float>(float.MaxValue) },
            SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
        };

        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<PointOnLineServoFunctions, PointOnLineServoPrestepData, Vector2Wide>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
    }

    [Benchmark]
    public (BodyVelocityWide, BodyVelocityWide) LinearAxisServo()
    {
        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var prestep = new LinearAxisServoPrestepData
        {
            LocalOffsetA = Vector3Wide.Broadcast(new Vector3(1, 0, 0)),
            LocalOffsetB = Vector3Wide.Broadcast(new Vector3(-1, 0, 0)),
            LocalPlaneNormal = Vector3Wide.Broadcast(new Vector3(0, 1, 0)),
            ServoSettings = new ServoSettingsWide { MaximumForce = new Vector<float>(float.MaxValue), MaximumSpeed = new Vector<float>(float.MaxValue) },
            SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
        };

        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<LinearAxisServoFunctions, LinearAxisServoPrestepData, Vector<float>>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
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
    public (BodyVelocityWide, BodyVelocityWide) SwivelHinge()
    {
        var prestep = new SwivelHingePrestepData
        {
            LocalSwivelAxisA = Vector3Wide.Broadcast(new Vector3(0, 1, 0)),
            LocalHingeAxisB = Vector3Wide.Broadcast(new Vector3(1, 0, 0)),
            SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
        };

        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<SwivelHingeFunctions, SwivelHingePrestepData, Vector4Wide>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
    }

    [Benchmark]
    public (BodyVelocityWide, BodyVelocityWide) Hinge()
    {
        var prestep = new HingePrestepData
        {
            LocalHingeAxisA = Vector3Wide.Broadcast(new Vector3(0, 1, 0)),
            LocalHingeAxisB = Vector3Wide.Broadcast(new Vector3(0, 1, 0)),
            SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
        };

        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkTwoBodyConstraint<HingeFunctions, HingePrestepData, HingeAccumulatedImpulses>(new Vector3Wide(), orientation, inertia, Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia, prestep);
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
