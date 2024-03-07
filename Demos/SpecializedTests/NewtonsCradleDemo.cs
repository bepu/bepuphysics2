using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using DemoContentLoader;
using DemoRenderer;
using DemoUtilities;
using System;
using System.Numerics;

namespace Demos.SpecializedTests;

/// <summary>
/// Shows a newton's cradle, primarily for behavioral experimentation (in case an alternative solver is ever implemented).
/// The type of solver currently used does not handle the conservation of momentum over the constraint graph in the expected way.
/// The bounce gets distributed fuzzily.
/// </summary>
public class NewtonsCradleDemo : Demo
{
    public override void Initialize(ContentArchive content, Camera camera)
    {
        camera.Yaw = 0;
        camera.Pitch = 0;
        Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(20, 0), float.MaxValue, 0), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0), 0, 0), new SolveDescription(1, 1));

        const int ballCount = 50;
        const float ballRadius = 0.5f;
        const float ballSpacing = 0.08f;
        const float ballHangHeight = 12f;
        const float barSpacing = 3f;

        var barShape = new Box(ballCount * ballRadius * 2 + (ballCount - 1) * ballSpacing, 0.2f, 0.2f);
        var barShapeIndex = Simulation.Shapes.Add(barShape);
        var bar0 = Simulation.Bodies.Add(BodyDescription.CreateKinematic(new Vector3(barShape.HalfWidth, ballHangHeight, barSpacing * -0.5f), barShapeIndex, 0f));
        var bar1 = Simulation.Bodies.Add(BodyDescription.CreateKinematic(new Vector3(barShape.HalfWidth, ballHangHeight, barSpacing * 0.5f), barShapeIndex, 0f));

        camera.Position = new Vector3(barShape.HalfWidth, ballHangHeight * 0.5f, 2 + Math.Max(ballHangHeight, barShape.HalfWidth));

        var ballShape = new Sphere(ballRadius);
        var ballShapeIndex = Simulation.Shapes.Add(ballShape);
        var ballInertia = ballShape.ComputeInertia(1);
        var ballConstraintSpringSettings = new SpringSettings(300, 1);
        for (int i = 0; i < ballCount; ++i)
        {
            var ballPosition = new Vector3(ballRadius + i * (ballSpacing + ballRadius * 2), 0, 0);
            var ball = Simulation.Bodies.Add(BodyDescription.CreateDynamic(ballPosition, ballInertia, new CollidableDescription(ballShapeIndex, 0), 0.0f));
            Simulation.Solver.Add(ball, bar0, new BallSocket { LocalOffsetA = new Vector3(0, ballHangHeight, -barSpacing * 0.5f), LocalOffsetB = new Vector3(ballPosition.X - barShape.HalfWidth, 0, 0), SpringSettings = ballConstraintSpringSettings });
            Simulation.Solver.Add(ball, bar1, new BallSocket { LocalOffsetA = new Vector3(0, ballHangHeight, barSpacing * 0.5f), LocalOffsetB = new Vector3(ballPosition.X - barShape.HalfWidth, 0, 0), SpringSettings = ballConstraintSpringSettings });
        }

        Simulation.Statics.Add(new StaticDescription(new Vector3(0, -ballHangHeight - ballRadius - 1 - 0.5f, 0), Simulation.Shapes.Add(new Box(2500, 1, 2500))));
    }

    public override void Update(Window window, Camera camera, Input input, float dt)
    {
        const int substeps = 100;
        for (int i = 0; i < substeps; ++i)
            Simulation.Timestep(1f / (60f * substeps));
    }

}
