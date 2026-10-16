using BepuPhysics;
using BepuUtilities;
using System.Numerics;

namespace Demos.Broken;

/// <summary>
/// An implementation of INarrowPhaseCallbacks (a BePu interface) needed to get simulation running with BePu.
/// </summary>
/// <remarks> Basically a verbatim copy from the example given here: https://github.com/bepu/bepuphysics2/blob/master/Demos/Demos/SimpleSelfContainedDemo.cs </remarks>
public struct PoseIntegratorCallbacks : IPoseIntegratorCallbacks
{
    public void Initialize(Simulation simulation) { }

    public readonly AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.Nonconserving;

    public readonly bool AllowSubstepsForUnconstrainedBodies => false;

    public readonly bool IntegrateVelocityForKinematics => false;

    private readonly Vector3Wide _gravityWideDt;
    public PoseIntegratorCallbacks(Vector3 gravity, float timeStepSeconds) : this()
    {
        _gravityWideDt = Vector3Wide.Broadcast(gravity * timeStepSeconds);
    }

    public void IntegrateVelocity(Vector<int> bodyIndices, Vector3Wide position, QuaternionWide orientation, BodyInertiaWide localInertia, Vector<int> integrationMask, int workerIndex, Vector<float> dt, ref BodyVelocityWide velocity)
    {
        velocity.Linear += _gravityWideDt;
    }

    public void PrepareForIntegration(float dt) { }
}
