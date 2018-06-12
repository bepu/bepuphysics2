using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace Demos.Demos
{
    /// <summary>
    /// A pyramid of boxes, because you can't have a physics engine without pyramids of boxes.
    /// </summary>
    public class PyramidDemo : Demo
    {
        public unsafe override void Initialize(Camera camera)
        {
            camera.Position = new Vector3(-30, 8, -60);
            camera.Yaw = MathHelper.Pi * 3f / 4;

            Simulation = Simulation.Create(BufferPool, new TestCallbacks());
            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);

            var boxShape = new Box(1, 1, 1);
            boxShape.ComputeInertia(1, out var boxInertia);
            var boxIndex = Simulation.Shapes.Add(boxShape);
            const int pyramidCount = 20;
            for (int pyramidIndex = 0; pyramidIndex < pyramidCount; ++pyramidIndex)
            {
                const int rowCount = 20;
                for (int rowIndex = 0; rowIndex < rowCount; ++rowIndex)
                {
                    int columnCount = rowCount - rowIndex;
                    for (int columnIndex = 0; columnIndex < columnCount; ++columnIndex)
                    {
                        var bodyDescription = new BodyDescription
                        {
                            LocalInertia = boxInertia,
                            Pose = new RigidPose
                            {
                                Position = new Vector3(
                                    (-columnCount * 0.5f + columnIndex) * boxShape.Width,
                                    (rowIndex + 0.5f) * boxShape.Height,
                                    (pyramidIndex - pyramidCount * 0.5f) * (boxShape.Length + 4)),
                                Orientation = BepuUtilities.Quaternion.Identity
                            },
                            Activity = new BodyActivityDescription { MinimumTimestepCountUnderThreshold = 32, SleepThreshold = .01f },
                            Collidable = new CollidableDescription { Shape = boxIndex, SpeculativeMargin = .1f }
                        };
                        Simulation.Bodies.Add(bodyDescription);
                    }
                }
            }

            var staticShape = new Box(200, 1, 200);
            var staticShapeIndex = Simulation.Shapes.Add(staticShape);

            var staticDescription = new StaticDescription
            {
                Collidable = new CollidableDescription
                {
                    Continuity = new ContinuousDetectionSettings { Mode = ContinuousDetectionMode.Discrete },
                    Shape = staticShapeIndex,
                    SpeculativeMargin = 0.1f
                },
                Pose = new RigidPose
                {
                    Position = new Vector3(1, -0.5f, 1),
                    Orientation = BepuUtilities.Quaternion.Identity
                }
            };
            Simulation.Statics.Add(staticDescription);

        }

        //We'll randomize the size of bullets.
        Random random = new Random(5);
        public override void Update(Input input, float dt)
        {
            if (input.WasPushed(OpenTK.Input.Key.Q))
            {
                //Create the shape that we'll launch at the pyramids when the user presses a button.
                var bulletShape = new Sphere(0.5f + 5 * (float)random.NextDouble());
                //Note that this can produce some pretty serious mass ratios. Observe what happens when a large ball sits on top of a few boxes with a fraction of the mass-
                //the collision appears much squishier and less stable. For most games, if you want to maintain rigidity, you'll want to use some combination of:
                //1) Limit the ratio of heavy object masses to light object masses when those heavy objects depend on the light objects.
                //2) Use a greater number of solver iterations.
                //3) Use a shorter timestep duration and update more frequently.
                //#2 and #3 can become very expensive. In pathological cases, it can end up slower than using a quality-focused solver for the same simulation.
                //Unfortunately, at the moment, bepuphysics v2 does not contain any alternative solvers, so if you can't afford to brute force the the problem away,
                //the best solution is to cheat as much as possible to avoid the corner cases.
                bulletShape.ComputeInertia(bulletShape.Radius * bulletShape.Radius * bulletShape.Radius, out var bulletInertia);
                var bulletShapeIndex = Simulation.Shapes.Add(bulletShape);
                var bodyDescription = new BodyDescription
                {
                    LocalInertia = bulletInertia,
                    Pose = new RigidPose
                    {
                        Position = new Vector3(0, 8, -100),
                        Orientation = BepuUtilities.Quaternion.Identity
                    },
                    Activity = new BodyActivityDescription { MinimumTimestepCountUnderThreshold = 32, SleepThreshold = .01f },
                    Collidable = new CollidableDescription { Shape = bulletShapeIndex, SpeculativeMargin = .1f },
                    Velocity = new BodyVelocity { Linear = new Vector3(0, 0, 150) }
                };
                Simulation.Bodies.Add(bodyDescription);
            }
            base.Update(input, dt);
        }

        public override void Render(Renderer renderer, TextBuilder text, Font font)
        {
            text.Clear().Append("Press Q to launch a ball!");
            renderer.TextBatcher.Write(text, new Vector2(20, renderer.Surface.Resolution.Y - 20), 16, new Vector3(1, 1, 1), font);
            base.Render(renderer, text, font);
        }

    }
}
