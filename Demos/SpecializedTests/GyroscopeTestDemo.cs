using System.Numerics;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;

namespace Demos.Demos
{
    struct GyroscopicIntegratorCallbacks : IPoseIntegratorCallbacks
    {
        //We'll use all the usual demo integration stuff, but use ConserveMomentumWithGyroscopicForce instead of the DemoPoseIntegratorCallbacks Nonconserving mode.
        //Pose integration isn't very expensive so using the higher quality option isn't that much of an issue, but it's also pretty subtle.
        //Unless your simulation requires the extra fidelity, there's not much reason to spend the extra time on it.
        DemoPoseIntegratorCallbacks innerCallbacks;
        public readonly AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.ConserveMomentum;
        //For this demo, we'll allow substepping for unconstrained bodies.
        public readonly bool AllowSubstepsForUnconstrainedBodies => true;

        public readonly bool IntegrateVelocityForKinematics => false;

        public void Initialize(Simulation simulation)
        {
            innerCallbacks.Initialize(simulation);
        }

        public GyroscopicIntegratorCallbacks(Vector3 gravity, float linearDamping, float angularDamping)
        {
            innerCallbacks = new DemoPoseIntegratorCallbacks(gravity, linearDamping, angularDamping);
        }

        public void PrepareForIntegration(float dt)
        {
            innerCallbacks.PrepareForIntegration(dt);
        }

        public void IntegrateVelocity(Vector<int> bodyIndices, Vector3Wide position, QuaternionWide orientation, BodyInertiaWide localInertia, Vector<int> integrationMask, int workerIndex, Vector<float> dt, ref BodyVelocityWide velocity)
        {
            innerCallbacks.IntegrateVelocity(bodyIndices, position, orientation, localInertia, integrationMask, workerIndex, dt, ref velocity);
        }

    }

    public class GyroscopeTestDemo : Demo
    {
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 2, -5);
            camera.Yaw = MathHelper.Pi;
            camera.Pitch = 0;

            //Note the lack of damping- we want the gyroscope to keep spinning.
            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new GyroscopicIntegratorCallbacks(new Vector3(0, -10, 0), 0f, 0f), new SolveDescription(1, 8));

            Simulation.Statics.Add(new StaticDescription(new Vector3(), Simulation.Shapes.Add(new Box(100, 1, 100))));

            var gyroBaseBody = Simulation.Bodies.Add(BodyDescription.CreateConvexKinematic(new Vector3(0, 2, 0), Simulation.Shapes, new Box(.1f, 4, .1f)));
            var gyroSpinnerBody = Simulation.Bodies.Add(BodyDescription.CreateConvexDynamic(new Vector3(2, 4, 0), (default, new Vector3(300, 0, 0)), 1, Simulation.Shapes, new Box(0.1f, 1f, 1f)));
            Simulation.Solver.Add(gyroBaseBody, gyroSpinnerBody, new BallSocket { LocalOffsetA = new Vector3(0, 2, 0), LocalOffsetB = new Vector3(-2, 0, 0), SpringSettings = new SpringSettings(30, 1) });


            var builder = new CompoundBuilder(BufferPool, Simulation.Shapes, 2);
            builder.Add(new Box(1, 0.3f, 0.3f), new Vector3(-0.5f, 0, 0), 1);
            builder.Add(new Box(0.3f, 1.5f, 0.3f), new Vector3(0.15f, 0, 0), 3);
            builder.BuildDynamicCompound(out var children, out var inertia, out _);
            builder.Dispose();
            var dzhanibekovShape = Simulation.Shapes.Add(new Compound(children));
            var dzhanibekovSpinnerBody = Simulation.Bodies.Add(
                BodyDescription.CreateDynamic(new Vector3(6, 4, 0), (new Vector3(0, 0, 1), new Vector3(3, 1e-5f, 0)), inertia, dzhanibekovShape, 0.01f));
            var dzhanibekovBaseBody = Simulation.Bodies.Add(BodyDescription.CreateConvexKinematic(new Vector3(6, 1, 0), Simulation.Shapes, new Box(.1f, 2, .1f)));
            Simulation.Solver.Add(dzhanibekovBaseBody, dzhanibekovSpinnerBody, new BallSocket { LocalOffsetA = new Vector3(0, 3, 0), LocalOffsetB = new Vector3(0, 0, 0), SpringSettings = new SpringSettings(30, 1) });
        }
    }
}
