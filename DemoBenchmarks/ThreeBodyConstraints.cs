using BenchmarkDotNet.Attributes;
using BepuPhysics;
using BepuPhysics.Constraints;
using BepuPhysics.Constraints.Contact;
using BepuUtilities;
using System.Numerics;

namespace DemoBenchmarks;

/// <summary>
/// Evaluates performance of all three body constraints.
/// </summary>
/// <remarks>
/// Note that all constraints operate across <see cref="Vector{}.Count"/> lanes simultaneously where T is of type <see cref="float"/>.
/// <para>The number of bundles being executed does not change if <see cref="Vector{}.Count"/> changes; if larger bundles are allowed, then more lanes end up getting solved.</para>
/// </remarks>
public class ThreeBodyConstraintBenchmarks
{
    static (BodyVelocityWide, BodyVelocityWide, BodyVelocityWide) BenchmarkThreeBodyConstraint<TConstraintFunctions, TPrestep, TAccumulatedImpulse>(
        Vector3Wide positionA, QuaternionWide orientationA, BodyInertiaWide inertiaA,
        Vector3Wide positionB, QuaternionWide orientationB, BodyInertiaWide inertiaB,
        Vector3Wide positionC, QuaternionWide orientationC, BodyInertiaWide inertiaC, TPrestep prestep)
        where TConstraintFunctions : unmanaged, IThreeBodyConstraintFunctions<TPrestep, TAccumulatedImpulse> where TPrestep : unmanaged where TAccumulatedImpulse : unmanaged
    {
        var functions = default(TConstraintFunctions);
        var accumulatedImpulse = default(TAccumulatedImpulse);
        var velocityA = default(BodyVelocityWide);
        var velocityB = default(BodyVelocityWide);
        var velocityC = default(BodyVelocityWide);
        //Individual constraint iterations are often extremely cheap, so beef the benchmark up a bit.
        const int iterations = 10000;
        const float inverseDt = 60f;
        const float dt = 1f / inverseDt;
        for (int i = 0; i < iterations; ++i)
        {
            functions.WarmStart(positionA, orientationA, inertiaA, positionB, orientationB, inertiaB, positionC, orientationC, inertiaC, ref prestep, ref accumulatedImpulse, ref velocityA, ref velocityB, ref velocityC);
            functions.Solve(positionA, orientationA, inertiaA, positionB, orientationB, inertiaB, positionC, orientationC, inertiaC, dt, inverseDt, ref prestep, ref accumulatedImpulse, ref velocityA, ref velocityB, ref velocityC);
        }
        return (velocityA, velocityB, velocityC);
    }

    //Not a lot of these yet!
    [Benchmark]
    public (BodyVelocityWide, BodyVelocityWide, BodyVelocityWide) AreaConstraint()
    {
        var prestep = new AreaConstraintPrestepData
        {
            TargetScaledArea = Vector<float>.One,
            SpringSettings = new() { TwiceDampingRatio = new Vector<float>(2), AngularFrequency = new Vector<float>(20 * MathF.PI) }
        };

        QuaternionWide.Broadcast(Quaternion.Identity, out var orientation);
        var inertia = new BodyInertiaWide { InverseInertiaTensor = new Symmetric3x3Wide { XX = Vector<float>.One, YY = Vector<float>.One, ZZ = Vector<float>.One }, InverseMass = Vector<float>.One };
        return BenchmarkThreeBodyConstraint<AreaConstraintFunctions, AreaConstraintPrestepData, Vector<float>>(
            new Vector3Wide(), orientation, inertia,
            Vector3Wide.Broadcast(new Vector3(2, 0, 0)), orientation, inertia,
            Vector3Wide.Broadcast(new Vector3(0, 2, 0)), orientation, inertia, prestep);
    }
}
