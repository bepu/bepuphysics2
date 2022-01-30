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
    /// A segmented rope chain thing launches itself out of a container. See also: https://en.wikipedia.org/wiki/Chain_fountain
    /// </summary>
    public class ChainFountainDemo : Demo
    {
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(5.65f, 3, -23);
            camera.Yaw = MathF.PI;
            camera.Pitch = 0;

            var filters = new CollidableProperty<RopeFilter>();
            Simulation = Simulation.Create(BufferPool, new RopeNarrowPhaseCallbacks(filters, new PairMaterialProperties(0.1f, float.MaxValue, new SpringSettings(240, 0)), 3), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(1, 12));

            var beadSpacing = 0.3f;
            var beadShape = new Capsule(0.05f, beadSpacing);
            var beadDescription = BodyDescription.CreateDynamic(new Vector3(), beadShape.ComputeInertia(1), Simulation.Shapes.Add(beadShape), 0.01f);

            const int beadCount = 4096;
            var handles = new BodyHandle[beadCount];
            var radius = 2.5f;
            var anglePerIteration = 2 * MathF.Asin(beadSpacing / (2 * radius));
            var heightPerIteration = beadShape.Radius * 2 / (MathF.PI * 2 / anglePerIteration);
            for (int i = 0; i < beadCount; ++i)
            {
                var angle = MathF.PI + i * anglePerIteration;
                var nextAngle = MathF.PI + (i + 1) * anglePerIteration;

                var currentPosition = new Vector3(2.8f + MathF.Sin(angle) * radius, 0.5f + heightPerIteration * i, -15 + MathF.Cos(angle) * radius);
                var nextPosition = new Vector3(2.8f + MathF.Sin(nextAngle) * radius, 0.5f + heightPerIteration * (i + 1), -15 + MathF.Cos(nextAngle) * radius);
                //The constraints were built along the local Y axis, so get the shortest rotation from Y to the current orientation.
                var offset = currentPosition - nextPosition;
                var cross = Vector3.Cross(Vector3.Normalize(offset), new Vector3(0, 1, 0));
                var crossLength = cross.Length();
                var orientation = crossLength > 1e-8f ? QuaternionEx.CreateFromAxisAngle(cross / crossLength, (float)Math.Asin(crossLength)) : Quaternion.Identity;

                //Include a little nudge. This is going to create constraint error, but that's fine. It distributes the rope over the platform to avoid tangles.
                beadDescription.Pose = new RigidPose(currentPosition + new Vector3(0, 0, i * 0.006f), orientation);
                //Throw the tip of the rope off the edge.
                if (i > beadCount - 32)
                    beadDescription.Velocity.Linear = new(20, 0, 0);
                handles[i] = Simulation.Bodies.Add(beadDescription);
                filters.Allocate(handles[i]) = new RopeFilter { RopeIndex = 1, IndexInRope = (short)i };

                if (i > 0)
                {
                    Simulation.Solver.Add(handles[i - 1], handles[i], new BallSocket { LocalOffsetA = new Vector3(0, beadSpacing * 0.5f, 0), LocalOffsetB = new Vector3(0, beadSpacing * -0.5f, 0), SpringSettings = new SpringSettings(120, 1) });
                    Simulation.Solver.Add(handles[i - 1], handles[i], new SwingLimit { AxisLocalA = Vector3.UnitY, AxisLocalB = Vector3.UnitY, SpringSettings = new SpringSettings(120, 1), MaximumSwingAngle = MathF.PI * 0.05f });
                }
            }

            Simulation.Statics.Add(new StaticDescription(new Vector3(0, 0f, 0), Simulation.Shapes.Add(new Box(11.6f, .2f, 40))));
            var wall = Simulation.Shapes.Add(new Box(.4f, 1, 40));
            Simulation.Statics.Add(new StaticDescription(new Vector3(5.65f, 2.4f - 2f, 0), wall));
            Simulation.Statics.Add(new StaticDescription(new Vector3(-5.65f, 2.4f - 2f, 0), wall));
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -500f, 0), Simulation.Shapes.Add(new Box(500, 1, 500))));

        }
        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            //If you want smooth slow mo (instead of discrete slow mo by holding middle mouse), try uncommenting this (and commenting the base.Update).
            //Simulation.Timestep(TimestepDuration / 16, ThreadDispatcher);
            base.Update(window, camera, input, dt);
        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            ////If you'd like to see forces visualized, you can uncomment this section. It reuses the SolverContactEnumerationDemo extractor to pull contact forces.
            ////Technically, you could be a little pickier about what data you collect- checking the impulses before trying to load offsets/normals.
            //var extractor = new SolverContactDataExtractor(BufferPool, 4);
            //var impulseToForce = Simulation.Solver.SubstepCount / TimestepDuration;
            //for (int i = 0; i < Simulation.Bodies.ActiveSet.Count; ++i)
            //{
            //    var constraintsForBody = Simulation.Bodies.ActiveSet.Constraints[i];
            //    for (int j = 0; j < constraintsForBody.Count; ++j)
            //    {
            //        if (Simulation.NarrowPhase.TryExtractSolverContactData(constraintsForBody[j].ConnectingConstraintHandle, ref extractor))
            //        {
            //            for (int k = 0; k < extractor.Constraints.Count; ++k)
            //            {
            //                ref var manifold = ref extractor.Constraints[k];
            //                //This is a collision against a static, so we can consider visualizing it.
            //                var contacts = manifold.Contacts;
            //                var bodyA = manifold.BodyA;
            //                for (int l = 0; l < contacts.Count; ++l)
            //                {
            //                    ref var contact = ref contacts[l];
            //                    var force = contact.PenetrationImpulse * impulseToForce;
            //                    if (force > 100)
            //                    {
            //                        var cylinder = new Cylinder(0.15f, force * 0.0005f);
            //                        renderer.Shapes.AddShape(cylinder, Simulation.Shapes, new RigidPose(contact.OffsetA + Simulation.Bodies[bodyA].Pose.Position + new Vector3(0, cylinder.HalfLength, 0), QuaternionEx.Identity), new Vector3(0, 1, 1));
            //                    }

            //                }
            //            }

            //        }
            //    }
            //    extractor.Reset();
            //}
            var resolution = renderer.Surface.Resolution;
            renderer.TextBatcher.Write(text.Clear().Append("The chain fountain is sometimes called Newton's beads or the Mould Effect."), new Vector2(16, resolution.Y - 64), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("A stiff segmented rope gets yanked upwards and over the lip by falling segments."), new Vector2(16, resolution.Y - 48), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Peculiarly, the rope sometimes climbs to heights far higher than the container edge as bits of the rope 'kick' off the floor on their way out."), new Vector2(16, resolution.Y - 32), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("The effect tends to be more visible with chains that resist bending more."), new Vector2(16, resolution.Y - 16), 16, Vector3.One, font);
            base.Render(renderer, camera, input, text, font);
        }
    }
}
