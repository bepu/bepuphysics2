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
using System.Numerics;

namespace Demos.Demos
{
    /// <summary>
    /// A string of beads launches itself out of a container.
    /// </summary>
    public class ChainFountainDemo : Demo
    {
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-30, 40, -30);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = MathHelper.Pi * 0.2f;

            var filters = new CollidableProperty<RopeFilter>();
            Simulation = Simulation.Create(BufferPool, new RopeNarrowPhaseCallbacks(filters, new PairMaterialProperties(0.1f, float.MaxValue, new SpringSettings(480, 1)), 3), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(1, 32));

            var beadSpacing = 0.3f;
            var beadShape = new Capsule(0.05f, beadSpacing);
            var beadDescription = BodyDescription.CreateDynamic(new Vector3(), beadShape.ComputeInertia(1), Simulation.Shapes.Add(beadShape), 0.01f);

            const int beadCount = 4096;
            var handles = new BodyHandle[beadCount];
            for (int i = 0; i < beadCount; ++i)
            {
                beadDescription.Pose.Position = new Vector3(0, i * beadSpacing + 4, 0);
                //beadDescription.Velocity.Linear.X = i * 0.00013f;
                handles[i] = Simulation.Bodies.Add(beadDescription);
                filters.Allocate(handles[i]) = new RopeFilter { RopeIndex = 1, IndexInRope = (short)i };
            }
            for (int i = 1; i < beadCount; ++i)
            {
                Simulation.Solver.Add(handles[i - 1], handles[i], new BallSocket { LocalOffsetA = new Vector3(0, beadSpacing * 0.5f, 0), LocalOffsetB = new Vector3(0, beadSpacing * -0.5f, 0), SpringSettings = new SpringSettings(480, 1) });
                Simulation.Solver.Add(handles[i - 1], handles[i], new SwingLimit { AxisLocalA = Vector3.UnitY, AxisLocalB = Vector3.UnitY, SpringSettings = new SpringSettings(480, 1), MaximumSwingAngle = MathF.PI * 0.15f });
            }
            var radius = 2f;
            var anglePerIteration = 2 * MathF.Asin(beadSpacing / (2 * radius));
            var heightPerIteration = beadShape.Radius * 2 / (MathF.PI * 2 / anglePerIteration);
            for (int i = 0; i < beadCount; ++i)
            {
                var bead = Simulation.Bodies[handles[i]];
                var angle = i * anglePerIteration;
                var nextAngle = (i + 1) * anglePerIteration;

                var currentPosition = new Vector3(3 + MathF.Sin(angle) * radius, 0.5f + heightPerIteration * i, MathF.Cos(angle) * radius);
                var nextPosition = new Vector3(3 + MathF.Sin(nextAngle) * radius, 0.5f + heightPerIteration * (i + 1), MathF.Cos(nextAngle) * radius);

                //The constraints were built along the local Y axis, so get the shortest rotation from Y to the current orientation.
                var offset = currentPosition - nextPosition;
                var cross = Vector3.Cross(Vector3.Normalize(offset), new Vector3(0, 1, 0));
                var crossLength = cross.Length();
                var orientation = crossLength > 1e-8f ? QuaternionEx.CreateFromAxisAngle(cross / crossLength, (float)Math.Asin(crossLength)) : Quaternion.Identity;

                bead.Pose = new RigidPose(currentPosition, orientation);

            }

            for (int i = beadCount - 128; i < beadCount; ++i)
            {
                Simulation.Bodies[handles[i]].Velocity.Linear = new Vector3(20, 5, 0);
            }


            Simulation.Statics.Add(new StaticDescription(new Vector3(0, 0f, 0), Simulation.Shapes.Add(new Box(11, .2f, 40))));
            var wall = Simulation.Shapes.Add(new Box(.5f, 3, 40));
            Simulation.Statics.Add(new StaticDescription(new Vector3(5.75f, 2.4f - 1.5f, 0), wall));
            Simulation.Statics.Add(new StaticDescription(new Vector3(-5.75f, 2.4f - 1.5f, 0), wall));
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -500f, 0), Simulation.Shapes.Add(new Box(500, 1, 500))));

        }
        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            //const float outerLoopSubsteps = 16;
            //for (int i = 0; i < outerLoopSubsteps; ++i)
            //    Simulation.Timestep(TimestepDuration / outerLoopSubsteps, ThreadDispatcher);
            base.Update(window, camera, input, dt);
        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            var resolution = renderer.Surface.Resolution;
            renderer.TextBatcher.Write(text.Clear().Append("The chain fountain is sometimes called Newton's beads or the Mould Effect."), new Vector2(16, resolution.Y - 48), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("A segmented rope gets yanked upwards and over the lip by falling segments."), new Vector2(16, resolution.Y - 32), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Peculiarly, the rope sometimes climbs to heights far over the container as bits of the rope 'kick' off the floor on their way out."), new Vector2(16, resolution.Y - 16), 16, Vector3.One, font);
            base.Render(renderer, camera, input, text, font);
        }
    }
}
