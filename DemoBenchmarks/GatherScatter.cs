using BenchmarkDotNet.Attributes;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.CollisionDetection.CollisionTasks;
using BepuPhysics.Constraints;
using BepuPhysics.Trees;
using BepuUtilities;
using BepuUtilities.Memory;
using System.Diagnostics;
using System.Numerics;
using static DemoBenchmarks.BenchmarkHelper;

namespace DemoBenchmarks;

/// <summary>
/// Evaluates performance of scatter/gather operations used by constraints to pull body data.
/// </summary>
public class GatherScatter
{
    unsafe struct NarrowPhaseCallbacks : INarrowPhaseCallbacks
    {
        public void Initialize(Simulation simulation) { }
        public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin) => a.Mobility == CollidableMobility.Dynamic || b.Mobility == CollidableMobility.Dynamic;
        public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB) => true;
        public unsafe bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial) where TManifold : unmanaged, IContactManifold<TManifold>
        {
            pairMaterial = default;
            return true;
        }
        public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold) { return true; }
        public void Dispose() { }
    }

    public struct PoseIntegratorCallbacks : IPoseIntegratorCallbacks
    {
        public void Initialize(Simulation simulation) { }
        public readonly AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.Nonconserving;
        public readonly bool AllowSubstepsForUnconstrainedBodies => false;
        public readonly bool IntegrateVelocityForKinematics => false;
        public void PrepareForIntegration(float dt) { }
        public void IntegrateVelocity(Vector<int> bodyIndices, Vector3Wide position, QuaternionWide orientation, BodyInertiaWide localInertia, Vector<int> integrationMask, int workerIndex, Vector<float> dt, ref BodyVelocityWide velocity) { }

    }


    const int iterationCount = 1000;
    const int bodyCount = 1000;
    BufferPool pool;


    Buffer<Vector<int>> bodyIndices;
    Buffer<RigidPoseWide> poses;
    Buffer<BodyVelocityWide> velocities;
    Buffer<BodyInertiaWide> inertias;
    Simulation simulation;


    [GlobalSetup]
    public unsafe void Setup()
    {
        pool = new BufferPool();
        pool.Take(iterationCount, out bodyIndices);
        pool.Take(iterationCount, out poses);
        pool.Take(iterationCount, out velocities);
        pool.Take(iterationCount, out inertias);
        simulation = Simulation.Create(pool, new NarrowPhaseCallbacks(), new PoseIntegratorCallbacks(), new SolveDescription(1, 1));

        Random random = new(5);

        //Fill random values for pair tests.
        BoundingBox bounds = new() { Min = new Vector3(0, 0, 0), Max = new Vector3(50, 50, 50) };
        for (int i = 0; i < bodyCount; ++i)
        {
            simulation.Bodies.Add(BodyDescription.CreateDynamic(
                CreateRandomPose(random, bounds),
                new BodyInertia { InverseMass = 1, InverseInertiaTensor = new Symmetric3x3 { XX = 1, YY = 1, ZZ = 1 } },
                default, new BodyActivityDescription(-0.01f)));
        }

        Span<int> bodyIndicesBundle = stackalloc int[Vector<int>.Count];
        for (int i = 0; i < iterationCount; ++i)
        {
            for (int j = 0; j < Vector<int>.Count; ++j)
            {
                bodyIndicesBundle[j] = random.Next(0, bodyCount);
            }
            bodyIndices[i] = new Vector<int>(bodyIndicesBundle);

            simulation.Bodies.GatherState<AccessAll>(bodyIndices[i], true, out poses[i].Position, out poses[i].Orientation, out velocities[i], out inertias[i]);
        }
    }

    [GlobalCleanup]
    public void Cleanup()
    {
        //All outstanding allocations poof when the pool is cleared.
        pool.Clear();
    }



    [Benchmark]
    public unsafe Vector<float> GatherState()
    {
        Vector<float> sum = default;
        for (int i = 0; i < iterationCount; ++i)
        {
            simulation.Bodies.GatherState<AccessAll>(bodyIndices[i], true, out var position, out var orientation, out var velocity, out var inertia);
            sum += position.X + position.Y + position.Z +
                velocity.Linear.X + velocity.Linear.Y + velocity.Linear.Z +
                velocity.Angular.X + velocity.Angular.Y + velocity.Angular.Z +
                orientation.X + orientation.Y + orientation.Z + orientation.Z +
                inertia.InverseInertiaTensor.XX + inertia.InverseInertiaTensor.YX + inertia.InverseInertiaTensor.YY +
                inertia.InverseInertiaTensor.ZX + inertia.InverseInertiaTensor.ZY + inertia.InverseInertiaTensor.ZZ + inertia.InverseMass;
        }
        return sum;
    }


    [Benchmark]
    public unsafe void ScatterPose()
    {
        var mask = new Vector<int>(-1);
        for (int i = 0; i < iterationCount; ++i)
        {
            ref var pose = ref poses[i];
            simulation.Bodies.ScatterPose(ref pose.Position, ref pose.Orientation, bodyIndices[i], mask);
        }
    }

    [Benchmark]
    public unsafe void ScatterInertia()
    {
        var mask = new Vector<int>(-1);
        for (int i = 0; i < iterationCount; ++i)
        {
            simulation.Bodies.ScatterInertia(ref inertias[i], bodyIndices[i], mask);
        }
    }
    [Benchmark]
    public unsafe void ScatterVelocities()
    {
        for (int i = 0; i < iterationCount; ++i)
        {
            simulation.Bodies.ScatterVelocities<AccessAll>(ref velocities[i], ref bodyIndices[i]);
        }
    }


}
