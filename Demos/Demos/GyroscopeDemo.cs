using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using BepuPhysics;
using BepuPhysics.Collidables;
using System.Runtime.CompilerServices;
using BepuPhysics.Constraints;

namespace Demos.Demos
{
    struct GyroscopicIntegratorCallbacks : IPoseIntegratorCallbacks
    {
        //We'll use all the usual demo integration stuff, but use ConserveMomentumWithGyroscopicForce instead of the DemoPoseIntegratorCallbacks Nonconserving mode.
        //Pose integration isn't very expensive so using the higher quality option isn't that much of an issue, but it's also pretty subtle.
        //Unless your simulation requires the extra fidelity, there's not much reason to spend the extra time on it.
        DemoPoseIntegratorCallbacks innerCallbacks;
        public AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.ConserveMomentumWithGyroscopicForce;

        public GyroscopicIntegratorCallbacks(Vector3 gravity, float linearDamping, float angularDamping)
        {
            innerCallbacks = new DemoPoseIntegratorCallbacks(gravity);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void PrepareForIntegration(float dt)
        {
            innerCallbacks.PrepareForIntegration(dt);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void IntegrateVelocity(int bodyIndex, in RigidPose pose, in BodyInertia localInertia, int workerIndex, ref BodyVelocity velocity)
        {
            innerCallbacks.IntegrateVelocity(bodyIndex, pose, localInertia, workerIndex, ref velocity);
        }

    }

    public class GyroscopeDemo : Demo
    {
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 2, -5);
            camera.Yaw = MathHelper.Pi;
            camera.Pitch = 0;

            //Note the lack of damping- we want the gyroscope to keep spinning.
            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new GyroscopicIntegratorCallbacks(new Vector3(0, -10, 0), 0f, 0f), new SubsteppingTimestepper(32), 2);

            Simulation.Statics.Add(new StaticDescription(new Vector3(), new CollidableDescription(Simulation.Shapes.Add(new Box(100, 1, 100)), 0.1f)));

            var baseBody = Simulation.Bodies.Add(BodyDescription.CreateConvexKinematic(new Vector3(0, 2, 0), Simulation.Shapes, new Box(.1f, 4, .1f)));
            var spinnerBody = Simulation.Bodies.Add(BodyDescription.CreateConvexDynamic(new Vector3(2, 4, 0), new BodyVelocity(new Vector3(0, 0, 1), new Vector3(200, 0, 0)), 1, Simulation.Shapes, new Box(0.1f, 1f, 1f)));
            Simulation.Solver.Add(baseBody, spinnerBody, new BallSocket { LocalOffsetA = new Vector3(0, 2, 0), LocalOffsetB = new Vector3(-2, 0, 0), SpringSettings = new SpringSettings(30, 1) });
        }
    }
}
