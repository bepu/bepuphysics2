using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using DemoUtilities;
using System;
using System.Numerics;

namespace Demos.SpecializedTests.Media
{
    /// <summary>
    /// A pyramid of boxes, because you can't have a physics engine without pyramids of boxes.
    /// </summary>
    public class ExcessivePyramidVideoDemo : Demo
    {
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-120, 32, 1045);
            camera.Yaw = MathHelper.Pi * 1f / 4;
            camera.Pitch = 0;
            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1), frictionCoefficient: 2), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));

            var boxShape = new Box(1, 1, 1);
            var boxInertia = boxShape.ComputeInertia(1);
            var boxIndex = Simulation.Shapes.Add(boxShape);
            const int pyramidCount = 420;
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
                            boxInertia, new CollidableDescription(boxIndex, 0.1f), 0.01f));
                    }
                }
            }
            Console.WriteLine($"bodies count: {Simulation.Bodies.ActiveSet.Count}");

            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -0.5f, 0), Simulation.Shapes.Add(new Box(2500, 1, 2500))));
        }

        int frameCount;
        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            ++frameCount;
            if (frameCount == 128 || (input != null && input.WasPushed(OpenTK.Input.Key.Z)))
            {
                var bulletShape = new Sphere(6);
                var bodyDescription = BodyDescription.CreateDynamic(
                    new Vector3(0, 8, -1200), new Vector3(0, 0, 230), bulletShape.ComputeInertia(5000000), new(Simulation.Shapes.Add(bulletShape), 0.1f), 0.01f);
                Simulation.Bodies.Add(bodyDescription);
            }
            base.Update(window, camera, input, dt);
        }
    }
}
