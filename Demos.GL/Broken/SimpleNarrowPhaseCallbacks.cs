using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;

namespace Demos.Broken;

/// <summary>
/// An implementation of INarrowPhaseCallbacks (a BePu interface) needed to get simulation running with BePu.
/// </summary>
/// <remarks>Roughly a verbatim copy of the example given here: https://github.com/bepu/bepuphysics2/blob/master/Demos/Demos/SimpleSelfContainedDemo.cs</remarks>
public struct SimpleNarrowPhaseCallbacks : INarrowPhaseCallbacks
{
    private readonly float _frictionCoefficient;
    private readonly float _maximumRecoveryVelocity;
    private readonly float _impactFrequency;
    private readonly float _impactDampingRatio;

    //TODO (Nigel): verify that these populate in the online docs without the summary tag

    /// <param name="frictionCoefficient"> Coefficient of friction to apply for the constraint. Maximum friction force will be equal to the normal force times the friction coefficient.</param>
    /// <param name="maximumRecoveryVelocity">Maximum relative velocity along the contact normal at which the collision constraint will recover from penetration. Clamps the velocity goal created from the spring settings.</param>
    /// <param name="impactFrequency">Target number of undamped oscillations per unit of time.</param>
    /// <param name="impactDampingRatio"> Ratio of the spring's actual damping to its critical damping. 0 is undamped, 1 is critically damped, and higher values are overdamped.</param>
    public SimpleNarrowPhaseCallbacks(float frictionCoefficient, float maximumRecoveryVelocity, float impactFrequency, float impactDampingRatio)
    {
        _frictionCoefficient = frictionCoefficient;
        _maximumRecoveryVelocity = maximumRecoveryVelocity;
        _impactFrequency = impactFrequency;
        _impactDampingRatio = impactDampingRatio;
    }
    public void Initialize(Simulation simulation) { }

    public void Dispose() { }

    public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB) => true;
    public bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial) where TManifold : unmanaged, IContactManifold<TManifold>
    {
        pairMaterial.FrictionCoefficient = _frictionCoefficient;
        pairMaterial.MaximumRecoveryVelocity = _maximumRecoveryVelocity;
        pairMaterial.SpringSettings = new SpringSettings(_impactFrequency, _impactDampingRatio);
        //For the purposes of the demo, contact constraints are always generated.
        return true;
    }
    public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold) => true;

    public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin)
    {

        return a.Mobility == CollidableMobility.Dynamic || b.Mobility == CollidableMobility.Dynamic;
    }
}
