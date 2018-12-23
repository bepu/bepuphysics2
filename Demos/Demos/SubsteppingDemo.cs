using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using Quaternion = BepuUtilities.Quaternion;

namespace Demos.Demos
{
    /// <summary>
    /// Shows how to use substepping to solve extremely difficult simulations at a not-completely-absurd cost.
    /// </summary>
    public class SubsteppingDemo : Demo
    {
        RolloverInfo rolloverInfo;

        SubsteppingTimestepper timestepper;
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 25, 80);
            camera.Yaw = 0;
            camera.Pitch = 0;
            timestepper = new SubsteppingTimestepper(8);
            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks() { ContactSpringiness = new SpringSettings(120, 120) }, new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), timestepper, 8);

            rolloverInfo = new RolloverInfo();
            {
                //We'll create a 0 level arm rope like the one from the RopeStabilityDemo. No skip constraints, though- and the mass ratio will be 1000:1 instead of 100:1!
                var startLocation = new Vector3(15, 40, 0);
                const float bodySpacing = 0.3f;
                const float bodyRadius = 0.5f;
                var springSettings = new SpringSettings(240, 480);
                var bodyHandles = RopeStabilityDemo.BuildRope(Simulation, startLocation, 12, bodyRadius, bodySpacing, 0, 1, 0, springSettings);

                var bigWreckingBall = new Sphere(5);
                bigWreckingBall.ComputeInertia(1000, out var bigWreckingBallInertia);

                RopeStabilityDemo.AttachWreckingBall(Simulation, bodyHandles, bodyRadius, bodySpacing, 0, bigWreckingBall.Radius, bigWreckingBallInertia, Simulation.Shapes.Add(bigWreckingBall), springSettings);
                rolloverInfo.Add(startLocation + new Vector3(0, 2, 0), "1000:1 mass ratio");
            }

            {
                //Stack with a heavy block on top. Note that the contact springiness we chose in the DemoNarrowPhaseCallbacks above is important to making the stack resist the weight of the top block.
                //It's also the reason why we need higher substeps- 120hz frequency is too high for 60hz solving! Watch what happens when you drop the substep count to 3.
                //(Note that the demos timestep frequency is 60hz, so 4 substeps is a 240hz solve rate- twice the 120hz contact frequency.)
                var boxShape = new Box(4, 0.5f, 6f);
                boxShape.ComputeInertia(1, out var boxInertia);
                //Note that sleeping is disabled with a negative velocity threshold. We want to watch the stack as we change simulation settings; if it's inactive, it won't respond!
                var boxDescription = BodyDescription.CreateDynamic(new Vector3(), boxInertia, new CollidableDescription(Simulation.Shapes.Add(boxShape), 0.1f), new BodyActivityDescription(-1f));
                for (int i = 0; i < 20; ++i)
                {
                    boxDescription.Pose = new RigidPose(new Vector3(0, 0.5f + boxShape.Height * (i + 0.5f), 0), Quaternion.CreateFromAxisAngle(Vector3.UnitY, MathF.PI * 0.05f * i));
                    Simulation.Bodies.Add(boxDescription);
                }
                var topBlockShape = new Box(8, 2, 8);
                topBlockShape.ComputeInertia(200, out var topBlockInertia);
                Simulation.Bodies.Add(
                    BodyDescription.CreateDynamic(boxDescription.Pose.Position + new Vector3(0, boxShape.HalfHeight + 1f, 0), topBlockInertia,
                    new CollidableDescription(Simulation.Shapes.Add(topBlockShape), 0.1f), new BodyActivityDescription(-1f)));

                rolloverInfo.Add(boxDescription.Pose.Position + new Vector3(0, 4, 0), "200:1 mass ratio");
            }

            {
                //Now a weird rotating multi-arm thing. Long constraint sequences with high leverages under stress are a really tough problem for iterative velocity solvers.
                //(Fortunately, all 5 degrees of freedom of each hinge constraint are solved analytically, so the convergence issues aren't quite as bad as they could be.)
                var basePosition = new Vector3(-20, 20, 0);
                var boxShape = new Box(0.5f, 0.5f, 3f);
                var boxCollidable = new CollidableDescription(Simulation.Shapes.Add(boxShape), 0.1f);
                boxShape.ComputeInertia(1, out var boxInertia);
                var linkDescription = BodyDescription.CreateDynamic(new Vector3(), boxInertia, boxCollidable, new BodyActivityDescription(0.01f));
   

                for (int chainIndex = 0; chainIndex < 4; ++chainIndex)
                {
                    linkDescription.Pose.Position = basePosition + new Vector3(0, 0, chainIndex * 15);
                    var previousLinkHandle = Simulation.Bodies.Add(BodyDescription.CreateKinematic(linkDescription.Pose.Position, boxCollidable, new BodyActivityDescription(0.01f)));
                    for (int linkIndex = 0; linkIndex < 8; ++linkIndex)
                    {
                        var previousPosition = linkDescription.Pose.Position;
                        var offset = new Vector3(boxShape.Width * 1.05f, 0, boxShape.Length - boxShape.Width);
                        linkDescription.Pose.Position += offset;
                        var linkHandle = Simulation.Bodies.Add(linkDescription);
                        Simulation.Solver.Add(previousLinkHandle, linkHandle, new Hinge
                        {
                            LocalHingeAxisA = Vector3.UnitX,
                            LocalHingeAxisB = Vector3.UnitX,
                            LocalOffsetA = offset * 0.5f,
                            LocalOffsetB = offset * -0.5f,
                            //Once again, the choice of high stiffness makes this potentially unstable without substepping.
                            SpringSettings = new SpringSettings(120, 1)
                        });
                        Simulation.Solver.Add(previousLinkHandle, linkHandle, new AngularAxisMotor
                        {
                            LocalAxisA = Vector3.UnitX,
                            TargetVelocity = .25f,
                            Settings = new MotorSettings(float.MaxValue, 0.0001f)
                        });
                        previousLinkHandle = linkHandle;
                    }
                }
                rolloverInfo.Add(basePosition + new Vector3(0, 4, 0), "High stiffness, long lever arm motorized chain");
            }


            Simulation.Statics.Add(new StaticDescription(new Vector3(0, 0, 0), new CollidableDescription(Simulation.Shapes.Add(new Box(200, 1, 200)), 0.1f)));

        }
        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            var substepCountChange = (int)MathF.Max(1f, timestepper.SubstepCount * 0.25f);
            var iterationCountChange = (int)MathF.Max(1f, Simulation.Solver.IterationCount * 0.25f);
            if (input.WasPushed(OpenTK.Input.Key.Z))
            {
                timestepper.SubstepCount = Math.Max(1, timestepper.SubstepCount - substepCountChange);
            }
            if (input.WasPushed(OpenTK.Input.Key.X))
            {
                timestepper.SubstepCount = Math.Min(8192, timestepper.SubstepCount + substepCountChange);
            }
            if (input.WasPushed(OpenTK.Input.Key.C))
            {
                Simulation.Solver.IterationCount = Math.Max(1, Simulation.Solver.IterationCount - iterationCountChange);
            }
            if (input.WasPushed(OpenTK.Input.Key.V))
            {
                Simulation.Solver.IterationCount = Math.Min(8192, Simulation.Solver.IterationCount + iterationCountChange);
            }
            base.Update(window, camera, input, dt);
        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            renderer.TextBatcher.Write(text.Clear().Append("Substep count: ").Append(timestepper.SubstepCount), new Vector2(16, renderer.Surface.Resolution.Y - 64), 16, new Vector3(1), font);
            renderer.TextBatcher.Write(text.Clear().Append("Solver iteration count: ").Append(Simulation.Solver.IterationCount), new Vector2(16, renderer.Surface.Resolution.Y - 48), 16, new Vector3(1), font);
            renderer.TextBatcher.Write(text.Clear().Append("Press Z/X to change substep count, C/V to change solver iteration count."), new Vector2(16, renderer.Surface.Resolution.Y - 32), 16, new Vector3(1), font);
            rolloverInfo.Render(renderer, camera, input, text, font);
            base.Render(renderer, camera, input, text, font);
        }

    }
}
