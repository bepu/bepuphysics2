﻿using BepuPhysics;
using BepuPhysics.Collidables;
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
    /// A pyramid of boxes, because you can't have a physics engine without pyramids of boxes.
    /// </summary>
    public class PyramidDemo : Demo
    {
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-30, 8, -110);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = 0;
            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));

            var boxShape = new Box(1, 1, 1);
            var boxInertia = boxShape.ComputeInertia(1);
            var boxIndex = Simulation.Shapes.Add(boxShape);
            const int pyramidCount = 40;
            for (int pyramidIndex = 0; pyramidIndex < pyramidCount; ++pyramidIndex)
            {
                const int rowCount = 20;
                for (int rowIndex = 0; rowIndex < rowCount; ++rowIndex)
                {
                    int columnCount = rowCount - rowIndex;
                    for (int columnIndex = 0; columnIndex < columnCount; ++columnIndex)
                    {
                        Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(
                            (-columnCount * 0.5f + columnIndex) * boxShape.Width,
                            (rowIndex + 0.5f) * boxShape.Height,
                            (pyramidIndex - pyramidCount * 0.5f) * (boxShape.Length + 4)),
                            boxInertia, boxIndex, 0.01f));
                    }
                }
            }

            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -0.5f, 0), Simulation.Shapes.Add(new Box(2500, 1, 2500))));
        }

        //We'll randomize the size of bullets.
        Random random = new(5);
        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            if (input != null && input.WasPushed(OpenTK.Input.Key.Z))
            {
                //Create the shape that we'll launch at the pyramids when the user presses a button.
                var bulletShape = new Sphere(0.5f + 5 * random.NextSingle());
                //Note that the use of radius^3 for mass can produce some pretty serious mass ratios. 
                //Observe what happens when a large ball sits on top of a few boxes with a fraction of the mass-
                //the collision appears much squishier and less stable. For most games, if you want to maintain rigidity, you'll want to use some combination of:
                //1) Limit the ratio of heavy object masses to light object masses when those heavy objects depend on the light objects.
                //2) Use a shorter timestep duration and update more frequently.
                //3) Use a greater number of solver iterations.
                //#2 and #3 can become very expensive. In pathological cases, it can end up slower than using a quality-focused solver for the same simulation.
                //Unfortunately, at the moment, bepuphysics v2 does not contain any alternative solvers, so if you can't afford to brute force the the problem away,
                //the best solution is to cheat as much as possible to avoid the corner cases.
                var bodyDescription = BodyDescription.CreateConvexDynamic(
                    new Vector3(0, 8, -130), new Vector3(0, 0, 150), bulletShape.Radius * bulletShape.Radius * bulletShape.Radius, Simulation.Shapes, bulletShape);
                Simulation.Bodies.Add(bodyDescription);
            }
            base.Update(window, camera, input, dt);
        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            text.Clear().Append("Press Z to launch a ball!");
            renderer.TextBatcher.Write(text, new Vector2(20, renderer.Surface.Resolution.Y - 20), 16, new Vector3(1, 1, 1), font);
            base.Render(renderer, camera, input, text, font);
        }

    }
}
