using BenchmarkDotNet.Attributes;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace DemoBenchmarks;

/// <summary>
/// Evaluates performance of a simulation similar to the first timesteps of the ShapePileTestDemo.
/// </summary>
public class ShapePileBenchmark
{
    public struct DemoPoseIntegratorCallbacks : IPoseIntegratorCallbacks
    {
        public Vector3 Gravity;
        public float LinearDamping;
        public float AngularDamping;
        public readonly AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.Nonconserving;
        public readonly bool AllowSubstepsForUnconstrainedBodies => false;
        public readonly bool IntegrateVelocityForKinematics => false;
        public void Initialize(Simulation simulation) { }
        public DemoPoseIntegratorCallbacks(Vector3 gravity, float linearDamping = .03f, float angularDamping = .03f) : this()
        {
            Gravity = gravity;
            LinearDamping = linearDamping;
            AngularDamping = angularDamping;
        }
        Vector3Wide gravityWideDt;
        Vector<float> linearDampingDt;
        Vector<float> angularDampingDt;
        public void PrepareForIntegration(float dt)
        {
            linearDampingDt = new Vector<float>(MathF.Pow(MathHelper.Clamp(1 - LinearDamping, 0, 1), dt));
            angularDampingDt = new Vector<float>(MathF.Pow(MathHelper.Clamp(1 - AngularDamping, 0, 1), dt));
            gravityWideDt = Vector3Wide.Broadcast(Gravity * dt);
        }
        public void IntegrateVelocity(Vector<int> bodyIndices, Vector3Wide position, QuaternionWide orientation, BodyInertiaWide localInertia, Vector<int> integrationMask, int workerIndex, Vector<float> dt, ref BodyVelocityWide velocity)
        {
            velocity.Linear = (velocity.Linear + gravityWideDt) * linearDampingDt;
            velocity.Angular = velocity.Angular * angularDampingDt;
        }
    }
    public unsafe struct DemoNarrowPhaseCallbacks : INarrowPhaseCallbacks
    {
        public SpringSettings ContactSpringiness;
        public float MaximumRecoveryVelocity;
        public float FrictionCoefficient;
        public DemoNarrowPhaseCallbacks(SpringSettings contactSpringiness, float maximumRecoveryVelocity = 2f, float frictionCoefficient = 1f)
        {
            ContactSpringiness = contactSpringiness;
            MaximumRecoveryVelocity = maximumRecoveryVelocity;
            FrictionCoefficient = frictionCoefficient;
        }
        public void Initialize(Simulation simulation)
        {
            if (ContactSpringiness.AngularFrequency == 0 && ContactSpringiness.TwiceDampingRatio == 0)
            {
                ContactSpringiness = new(30, 1);
                MaximumRecoveryVelocity = 2f;
                FrictionCoefficient = 1f;
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin)
        {
            return a.Mobility == CollidableMobility.Dynamic || b.Mobility == CollidableMobility.Dynamic;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
        {
            return true;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial) where TManifold : unmanaged, IContactManifold<TManifold>
        {
            pairMaterial.FrictionCoefficient = FrictionCoefficient;
            pairMaterial.MaximumRecoveryVelocity = MaximumRecoveryVelocity;
            pairMaterial.SpringSettings = ContactSpringiness;
            return true;
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold)
        {
            return true;
        }
        public void Dispose()
        {
        }
    }


    const int timestepCount = 512;
    BufferPool BufferPool;
    Simulation Simulation;

    [IterationSetup]
    public unsafe void Initialize()
    {
        BufferPool = new BufferPool();
        Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(4, 1));
        Simulation.Deterministic = true;

        var sphere = new Sphere(1.5f);
        var capsule = new Capsule(1f, 1f);
        var box = new Box(1f, 3f, 2f);
        var cylinder = new Cylinder(1.5f, 0.3f);
        var points = new QuickList<Vector3>(32, BufferPool);
        //Boxlike point cloud.
        //points.Allocate(BufferPool) = new Vector3(0, 0, 0);
        //points.Allocate(BufferPool) = new Vector3(0, 0, 1);
        //points.Allocate(BufferPool) = new Vector3(0, 1, 0);
        //points.Allocate(BufferPool) = new Vector3(0, 1, 1);
        //points.Allocate(BufferPool) = new Vector3(1, 0, 0);
        //points.Allocate(BufferPool) = new Vector3(1, 0, 1);
        //points.Allocate(BufferPool) = new Vector3(1, 1, 0);
        //points.Allocate(BufferPool) = new Vector3(1, 1, 1);

        //Rando pointcloud.
        //var random = new Random(5);
        //for (int i = 0; i < 32; ++i)
        //{
        //    points.Allocate(BufferPool) = new Vector3(3 * random.NextSingle(), 1 * random.NextSingle(), 3 * random.NextSingle());
        //}

        //Dodecahedron pointcloud.
        points.Allocate(BufferPool) = new Vector3(-1, -1, -1);
        points.Allocate(BufferPool) = new Vector3(-1, -1, 1);
        points.Allocate(BufferPool) = new Vector3(-1, 1, -1);
        points.Allocate(BufferPool) = new Vector3(-1, 1, 1);
        points.Allocate(BufferPool) = new Vector3(1, -1, -1);
        points.Allocate(BufferPool) = new Vector3(1, -1, 1);
        points.Allocate(BufferPool) = new Vector3(1, 1, -1);
        points.Allocate(BufferPool) = new Vector3(1, 1, 1);

        const float goldenRatio = 1.618033988749f;
        const float oogr = 1f / goldenRatio;

        points.Allocate(BufferPool) = new Vector3(0, goldenRatio, oogr);
        points.Allocate(BufferPool) = new Vector3(0, -goldenRatio, oogr);
        points.Allocate(BufferPool) = new Vector3(0, goldenRatio, -oogr);
        points.Allocate(BufferPool) = new Vector3(0, -goldenRatio, -oogr);

        points.Allocate(BufferPool) = new Vector3(oogr, 0, goldenRatio);
        points.Allocate(BufferPool) = new Vector3(oogr, 0, -goldenRatio);
        points.Allocate(BufferPool) = new Vector3(-oogr, 0, goldenRatio);
        points.Allocate(BufferPool) = new Vector3(-oogr, 0, -goldenRatio);

        points.Allocate(BufferPool) = new Vector3(goldenRatio, oogr, 0);
        points.Allocate(BufferPool) = new Vector3(goldenRatio, -oogr, 0);
        points.Allocate(BufferPool) = new Vector3(-goldenRatio, oogr, 0);
        points.Allocate(BufferPool) = new Vector3(-goldenRatio, -oogr, 0);

        var convexHull = new ConvexHull(points.Span.Slice(points.Count), BufferPool, out _);
        var boxInertia = box.ComputeInertia(1);
        var capsuleInertia = capsule.ComputeInertia(1);
        var sphereInertia = sphere.ComputeInertia(1);
        var cylinderInertia = cylinder.ComputeInertia(1);
        var hullInertia = convexHull.ComputeInertia(1);
        var boxIndex = Simulation.Shapes.Add(box);
        var capsuleIndex = Simulation.Shapes.Add(capsule);
        var sphereIndex = Simulation.Shapes.Add(sphere);
        var cylinderIndex = Simulation.Shapes.Add(cylinder);
        var hullIndex = Simulation.Shapes.Add(convexHull);
        const int width = 8;
        const int height = 8;
        const int length = 8;
        var shapeCount = 0;
        for (int i = 0; i < width; ++i)
        {
            for (int j = 0; j < height; ++j)
            {
                for (int k = 0; k < length; ++k)
                {
                    var location = new Vector3(6, 3, 6) * new Vector3(i, j, k) + new Vector3(-width * 1.5f, 5.5f, -length * 1.5f);
                    var bodyDescription = BodyDescription.CreateKinematic(location, new(default, ContinuousDetection.Passive), 0.01f);
                    var index = shapeCount++;
                    switch (index % 5)
                    {
                        case 0:
                            bodyDescription.Collidable.Shape = sphereIndex;
                            bodyDescription.LocalInertia = sphereInertia;
                            break;
                        case 1:
                            bodyDescription.Collidable.Shape = capsuleIndex;
                            bodyDescription.LocalInertia = capsuleInertia;
                            break;
                        case 2:
                            bodyDescription.Collidable.Shape = boxIndex;
                            bodyDescription.LocalInertia = boxInertia;
                            break;
                        case 3:
                            bodyDescription.Collidable.Shape = cylinderIndex;
                            bodyDescription.LocalInertia = cylinderInertia;
                            break;
                        case 4:
                            bodyDescription.Collidable.Shape = hullIndex;
                            bodyDescription.LocalInertia = hullInertia;
                            break;
                    }
                    Simulation.Bodies.Add(bodyDescription);
                }
            }
        }

        //Simulation.Statics.Add(new StaticDescription(new Vector3(), Simulation.Shapes.Add(new Box(500, 1, 500))));
        BenchmarkHelper.CreateDeformedPlane(128, 128, (x, y) => new Vector3(x - 64, 2f * (float)(Math.Sin(x * 0.5f) * Math.Sin(y * 0.5f)), y - 64), new Vector3(4, 1, 4), BufferPool, out var mesh);
        Simulation.Statics.Add(new StaticDescription(new Vector3(), Simulation.Shapes.Add(mesh)));
    }

    [IterationCleanup]
    public void CleanUp()
    {
        BufferPool.Clear();
        BufferPool = null;
    }

    [Benchmark]
    public unsafe void ShapePileBenchmarks()
    {
        for (int i = 0; i < timestepCount; ++i)
        {
            Simulation.Timestep(1 / 60f);
        }
    }
}
