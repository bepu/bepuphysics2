using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Collections;
using BepuUtilities.Memory;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using System;
using System.Numerics;
namespace Demos.Demos
{
    /// <summary>
    /// Shows how to use substepping to solve extremely difficult simulations at a not-completely-absurd cost.
    /// </summary>
    public class SubsteppingDemo : Demo
    {
        RolloverInfo rolloverInfo;

        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 25, 80);
            camera.Yaw = 0;
            camera.Pitch = 0;
            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(640, 480), float.MaxValue, 1), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(2, 48));

            rolloverInfo = new RolloverInfo();
            {
                //We'll create a 0 level arm rope like the one from the RopeStabilityDemo. No skip constraints, though- and the mass ratio will be 10000:1 instead of 100:1!
                var startLocation = new Vector3(15, 40, 0);
                const float bodySpacing = 0.3f;
                const float bodyRadius = 0.5f;
                var springSettings = new SpringSettings(480, 480);
                var bodyHandles = RopeStabilityDemo.BuildRope(Simulation, startLocation, 12, bodyRadius, bodySpacing, 0, 1, 0, springSettings);

                var bigWreckingBall = new Sphere(5);
                const float mass = 10000;
                var bigWreckingBallInertia = bigWreckingBall.ComputeInertia(mass);

                RopeStabilityDemo.AttachWreckingBall(Simulation, bodyHandles, bodyRadius, bodySpacing, 0, bigWreckingBall.Radius, bigWreckingBallInertia, Simulation.Shapes.Add(bigWreckingBall), springSettings);
                rolloverInfo.Add(startLocation + new Vector3(0, 2, 0), $"{mass}:1 mass ratio");
            }

            {
                //Stack with a heavy block on top. Note that the contact springiness we chose in the DemoNarrowPhaseCallbacks above is important to making the stack resist the weight of the top block.
                //It's also the reason why we need higher substeps- 120hz frequency is too high for 60hz solving! Watch what happens when you drop the substep count to 3.
                //(Note that the demos timestep frequency is 60hz, so 4 substeps is a 240hz solve rate- twice the 120hz contact frequency.)
                var boxShape = new Box(4, 0.5f, 6f);
                var boxInertia = boxShape.ComputeInertia(1);
                var boxDescription = BodyDescription.CreateDynamic(new Vector3(), boxInertia, Simulation.Shapes.Add(boxShape), 0.01f);
                for (int i = 0; i < 20; ++i)
                {
                    boxDescription.Pose = (new Vector3(0, 0.5f + boxShape.Height * (i + 0.5f), 0), QuaternionEx.CreateFromAxisAngle(Vector3.UnitY, MathF.PI * 0.05f * i));
                    Simulation.Bodies.Add(boxDescription);
                }
                var topBlockShape = new Box(8, 2, 8);
                const float mass = 10000;
                Simulation.Bodies.Add(
                    BodyDescription.CreateDynamic(boxDescription.Pose.Position + new Vector3(0, boxShape.HalfHeight + 1f, 0), topBlockShape.ComputeInertia(mass),
                    Simulation.Shapes.Add(topBlockShape), .01f));

                rolloverInfo.Add(boxDescription.Pose.Position + new Vector3(0, 4, 0), $"{mass}:1 mass ratio");
            }

            {
                //Now a weird rotating multi-arm thing. Long constraint sequences with high leverages under stress are a really tough problem for iterative velocity solvers.
                //(Fortunately, all 5 degrees of freedom of each hinge constraint are solved analytically, so the convergence issues aren't quite as bad as they could be.)
                var basePosition = new Vector3(-20, 20, 0);
                var boxShape = new Box(0.5f, 0.5f, 3f);
                var boxShapeIndex = Simulation.Shapes.Add(boxShape);
                var boxInertia = boxShape.ComputeInertia(1);
                var linkDescription = BodyDescription.CreateDynamic(new Vector3(), boxInertia, boxShapeIndex, 0.01f);

                for (int chainIndex = 0; chainIndex < 4; ++chainIndex)
                {
                    linkDescription.Pose.Position = basePosition + new Vector3(0, 0, chainIndex * 15);
                    var previousLinkHandle = Simulation.Bodies.Add(BodyDescription.CreateKinematic(linkDescription.Pose.Position, boxShapeIndex, 0.01f));
                    for (int linkIndex = 0; linkIndex < 8; ++linkIndex)
                    {
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
                            SpringSettings = new SpringSettings(480, 1)
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


            Simulation.Statics.Add(new StaticDescription(new Vector3(0, 0, 0), Simulation.Shapes.Add(new Box(200, 1, 200))));

        }

        unsafe void AwakenAllBodies()
        {
            //Any time the simulation configuration changes, it could change behavior.
            //For example, reducing substep/iteration count to very low values will cause severe instability.
            //Sleeping objects don't move, though, so wake them up so the result of the change can be seen!
            var sleepingSetsMemory = stackalloc int[Simulation.Bodies.Sets.Length - 1];
            var sleepingSets = new QuickList<int>(new Buffer<int>(sleepingSetsMemory, Simulation.Bodies.Sets.Length - 1));
            for (int i = 1; i < Simulation.Bodies.Sets.Length; ++i)
            {
                if (Simulation.Bodies.Sets[i].Allocated)
                {
                    sleepingSets.AllocateUnsafely() = i;
                }
            }
            Simulation.Awakener.AwakenSets(ref sleepingSets);
        }

        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            var substepCountChange = (int)MathF.Max(1f, Simulation.Solver.SubstepCount * 0.25f);
            var iterationCountChange = (int)MathF.Max(1f, Simulation.Solver.VelocityIterationCount * 0.25f);
            if (input.WasPushed(OpenTK.Input.Key.Z))
            {
                Simulation.Solver.SubstepCount = Math.Max(1, Simulation.Solver.SubstepCount - substepCountChange);
                AwakenAllBodies();
            }
            if (input.WasPushed(OpenTK.Input.Key.X))
            {
                Simulation.Solver.SubstepCount = Math.Min(8192, Simulation.Solver.SubstepCount + substepCountChange);
                AwakenAllBodies();
            }
            if (input.WasPushed(OpenTK.Input.Key.C))
            {
                Simulation.Solver.VelocityIterationCount = Math.Max(1, Simulation.Solver.VelocityIterationCount - iterationCountChange);
                AwakenAllBodies();
            }
            if (input.WasPushed(OpenTK.Input.Key.V))
            {
                Simulation.Solver.VelocityIterationCount = Math.Min(8192, Simulation.Solver.VelocityIterationCount + iterationCountChange);
                AwakenAllBodies();
            }
            base.Update(window, camera, input, dt);
        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            var resolution = renderer.Surface.Resolution;
            renderer.TextBatcher.Write(text.Clear().Append("Substepping makes the solver run multiple mini timesteps for each call to Simulation.Timestep."), new Vector2(16, resolution.Y - 160), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Substeps can make extreme mass ratios and difficult constraint articulations stable at low costs."), new Vector2(16, resolution.Y - 144), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Simulations with substepping can use fewer velocity iterations per substep while remaining stable."), new Vector2(16, resolution.Y - 128), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Some difficult simulations will be vastly cheaper when using substepping than just increasing velocity iterations."), new Vector2(16, resolution.Y - 112), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Try modifying the substep/iteration counts to observe the effect on simulation stability."), new Vector2(16, resolution.Y - 96), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Check the Substepping documentation for more information."), new Vector2(16, resolution.Y - 80), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Substep count: ").Append(Simulation.Solver.SubstepCount), new Vector2(16, renderer.Surface.Resolution.Y - 48), 16, new Vector3(1), font);
            renderer.TextBatcher.Write(text.Clear().Append("Solver iteration count: ").Append(Simulation.Solver.VelocityIterationCount), new Vector2(16, renderer.Surface.Resolution.Y - 32), 16, new Vector3(1), font);
            renderer.TextBatcher.Write(text.Clear().Append("Press Z/X to change substep count, C/V to change solver iteration count."), new Vector2(16, renderer.Surface.Resolution.Y - 16), 16, new Vector3(1), font);
            rolloverInfo.Render(renderer, camera, input, text, font);
            base.Render(renderer, camera, input, text, font);
        }

    }
}
