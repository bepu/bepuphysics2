using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;
using System.Numerics;
using Xunit;

namespace DemoTests
{
    public static class FallbackBatchTests
    {
        struct NarrowPhaseCallbacks : INarrowPhaseCallbacks
        {
            public void Initialize(Simulation simulation) { }
            public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin)
                => a.Mobility == CollidableMobility.Dynamic || b.Mobility == CollidableMobility.Dynamic;
            public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB) => true;
            public bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial) where TManifold : unmanaged, IContactManifold<TManifold>
            {
                pairMaterial = new PairMaterialProperties(1, 2, new SpringSettings(30, 1));
                return true;
            }
            public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold) => true;
            public void Dispose() { }
        }

        struct PoseIntegratorCallbacks : IPoseIntegratorCallbacks
        {
            public AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.Nonconserving;
            public bool AllowSubstepsForUnconstrainedBodies => false;
            public bool IntegrateVelocityForKinematics => false;
            public void Initialize(Simulation simulation) { }
            public void PrepareForIntegration(float dt) { }
            public void IntegrateVelocity(System.Numerics.Vector<int> bodyIndices, Vector3Wide position, QuaternionWide orientation, BodyInertiaWide localInertia, System.Numerics.Vector<int> integrationMask, int workerIndex, System.Numerics.Vector<float> dt, ref BodyVelocityWide velocity) { }
        }

        /// <summary>
        /// Removing fallback batch constraints between a dynamic body and several kinematics used to dereference a released lookup table
        /// when the last dynamic reference was removed before a kinematic reference in the same removal list.
        /// </summary>
        [Fact]
        public static void RemovingFallbackConstraintsWithKinematicsAfterLastDynamicDoesNotThrow()
        {
            var pool = new BufferPool();
            var simulation = Simulation.Create(pool, new NarrowPhaseCallbacks(), new PoseIntegratorCallbacks(), new SolveDescription(4, 1, fallbackBatchThreshold: 1));
            var activity = new BodyActivityDescription(0.01f);
            var sphere = new Sphere(1f);
            var ball = simulation.Bodies.Add(BodyDescription.CreateDynamic(RigidPose.Identity, sphere.ComputeInertia(1f), simulation.Shapes.Add(sphere), activity));
            var box = simulation.Shapes.Add(new Box(1, 1, 1));
            foreach (var direction in new[] { Vector3.UnitX, -Vector3.UnitX, Vector3.UnitY, -Vector3.UnitY, Vector3.UnitZ, -Vector3.UnitZ })
                simulation.Bodies.Add(BodyDescription.CreateKinematic(new RigidPose(direction * 1.3f), box, activity));

            simulation.Timestep(1 / 60f);

            var reference = simulation.Bodies[ball];
            reference.Pose.Position = new Vector3(1000, 0, 0);
            reference.Velocity = default;
            reference.UpdateBounds();
            simulation.Timestep(1 / 60f);

            simulation.Dispose();
            pool.Clear();
        }
    }
}
