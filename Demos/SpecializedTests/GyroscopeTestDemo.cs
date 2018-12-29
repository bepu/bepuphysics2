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
        public AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.ConserveMomentumWithGyroscopicTorque;

        public GyroscopicIntegratorCallbacks(Vector3 gravity, float linearDamping, float angularDamping)
        {
            innerCallbacks = new DemoPoseIntegratorCallbacks(gravity, linearDamping, angularDamping);
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

    public class GyroscopeTestDemo : Demo
    {
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 2, -5);
            camera.Yaw = MathHelper.Pi;
            camera.Pitch = 0;

            //Note the lack of damping- we want the gyroscope to keep spinning.
            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new GyroscopicIntegratorCallbacks(new Vector3(0, -10, 0), 0f, 0f), new SubsteppingTimestepper(4), 2);

            Simulation.Statics.Add(new StaticDescription(new Vector3(), new CollidableDescription(Simulation.Shapes.Add(new Box(100, 1, 100)), 0.1f)));

            var gyroBaseBody = Simulation.Bodies.Add(BodyDescription.CreateConvexKinematic(new Vector3(0, 2, 0), Simulation.Shapes, new Box(.1f, 4, .1f)));
            var gyroSpinnerBody = Simulation.Bodies.Add(BodyDescription.CreateConvexDynamic(new Vector3(2, 4, 0), new BodyVelocity(default, new Vector3(300, 0, 0)), 1, Simulation.Shapes, new Box(0.1f, 1f, 1f)));
            Simulation.Solver.Add(gyroBaseBody, gyroSpinnerBody, new BallSocket { LocalOffsetA = new Vector3(0, 2, 0), LocalOffsetB = new Vector3(-2, 0, 0), SpringSettings = new SpringSettings(30, 1) });


            CompoundBuilder builder = new CompoundBuilder(BufferPool, Simulation.Shapes, 2);
            builder.Add(new Box(1, 0.3f, 0.3f), new RigidPose(new Vector3(-0.5f, 0, 0)), 1);
            builder.Add(new Box(0.3f, 2f, 0.3f), new RigidPose(new Vector3(0.15f, 0, 0)), 2);
            builder.BuildDynamicCompound(out var children, out var inertia, out _);
            builder.Dispose();
            var dzhanibekovShape = Simulation.Shapes.Add(new Compound(children));
            var dzhanibekovSpinnerBody = Simulation.Bodies.Add(
                BodyDescription.CreateDynamic(new Vector3(6, 4, 0), new BodyVelocity(new Vector3(0, 0, 1), new Vector3(5, .001f, .001f)), inertia, new CollidableDescription(dzhanibekovShape, 0.1f), new BodyActivityDescription(0.01f)));
            var dzhanibekovBaseBody = Simulation.Bodies.Add(BodyDescription.CreateConvexKinematic(new Vector3(6, 1, 0), Simulation.Shapes, new Box(.1f, 2, .1f)));
            Simulation.Solver.Add(dzhanibekovBaseBody, dzhanibekovSpinnerBody, new BallSocket { LocalOffsetA = new Vector3(0, 3, 0), LocalOffsetB = new Vector3(0, 0, 0), SpringSettings = new SpringSettings(30, 1) });
        }
    }
}
