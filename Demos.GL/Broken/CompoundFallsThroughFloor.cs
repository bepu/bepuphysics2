
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using DemoUtilities;
using System.Numerics;

namespace Demos.Broken;

internal class CompoundFallsThroughFloor : Demo
{
    private BodyActivityDescription NeverSleep() => new BodyActivityDescription(sleepThreshold: float.MinValue, minimumTimestepCountUnderThreshold: byte.MaxValue);

    public override void Initialize(ContentArchive content, Camera camera)
    {
        camera.Position = new Vector3(-1000f, 400, -1000f);
        camera.Yaw = MathHelper.Pi * 3f / 4;
        camera.Pitch = MathHelper.Pi * 0.1f;

        var narrowPhaseCallbacks = new SimpleNarrowPhaseCallbacks(
            frictionCoefficient: 0.3f,
            maximumRecoveryVelocity: 500,
            impactFrequency: 20,
            impactDampingRatio: 0.5f
            );

        var gravity = new Vector3(0, -9807f, 0); // mm/s
        var timeStepSeconds = .12f;

        Simulation = Simulation.Create(BufferPool, narrowPhaseCallbacks, new PoseIntegratorCallbacks(gravity: gravity, timeStepSeconds: timeStepSeconds), new SolveDescription(8, 1));
        var floor = new Box(1000, 30, 1000);

        Simulation.Statics.Add(new StaticDescription(RigidPose.Identity, Simulation.Shapes.Add(floor)));

        // Item as a standalone (not compound)
        {
            var rawCollidableBox = new Box(100, 130, 100);
            var boxPose = new RigidPose(new Vector3(-300, rawCollidableBox.HalfHeight + floor.HalfHeight + 18, 300), Quaternion.Identity);
            var boxInertia = rawCollidableBox.ComputeInertia(1);
            var boxIndex = Simulation.Shapes.Add(rawCollidableBox);
            var collisionDetection = new ContinuousDetection();
            var boxCollidable = new CollidableDescription(boxIndex, collisionDetection);
            var boxDescription = BodyDescription.CreateDynamic(boxPose, boxInertia, boxCollidable, NeverSleep());
            Simulation.Bodies.Add(boxDescription);
        }

    }

    public override void Update(Window window, Camera camera, Input input, float dt)
    {
        base.Update(window, camera, input, dt);
    }
}
