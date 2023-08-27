using BepuPhysics;
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
    /// Shows how to use custom velocity integration to implement planetary gravity.
    /// </summary>
    public class PlanetDemo : Demo
    {
        struct PlanetaryGravityCallbacks : IPoseIntegratorCallbacks
        {
            public Vector3 PlanetCenter;
            public float Gravity;

            public readonly AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.Nonconserving;

            public readonly bool AllowSubstepsForUnconstrainedBodies => false;

            public readonly bool IntegrateVelocityForKinematics => false;

            public void Initialize(Simulation simulation)
            {
            }

            float gravityDt;
            public void PrepareForIntegration(float dt)
            {
                //No point in repeating this for every body; cache it.
                gravityDt = dt * Gravity;
            }

            public void IntegrateVelocity(Vector<int> bodyIndices, Vector3Wide position, QuaternionWide orientation, BodyInertiaWide localInertia, Vector<int> integrationMask, int workerIndex, Vector<float> dt, ref BodyVelocityWide velocity)
            {
                var offset = position - Vector3Wide.Broadcast(PlanetCenter);
                var distance = offset.Length();
                velocity.Linear -= new Vector<float>(gravityDt) * offset / Vector.Max(Vector<float>.One, distance * distance * distance);
            }
        }


        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(110, -80, 12);
            camera.Yaw = 0;
            camera.Pitch = MathF.PI * -0.5f;

            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new PlanetaryGravityCallbacks() { PlanetCenter = new Vector3(), Gravity = 100000 }, new SolveDescription(4, 1));

            Simulation.Statics.Add(new StaticDescription(new Vector3(), Simulation.Shapes.Add(new Sphere(50))));

            var orbiter = new Sphere(1f);
            var inertia = orbiter.ComputeInertia(1);
            var orbiterShapeIndex = Simulation.Shapes.Add(orbiter);
            var spacing = new Vector3(5);
            const int length = 40;
            for (int i = 0; i < length; ++i)
            {
                for (int j = 0; j < 20; ++j)
                {
                    const int width = 40;
                    var origin = new Vector3(-50, 95, 0) + spacing * new Vector3(length * -0.5f, 0, width * -0.5f);
                    for (int k = 0; k < width; ++k)
                    {
                        Simulation.Bodies.Add(BodyDescription.CreateDynamic(
                            origin + new Vector3(i, j, k) * spacing, new Vector3(30, 0, 0), inertia, orbiterShapeIndex, 0.01f));
                    }
                }
            }

        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            var bottomY = renderer.Surface.Resolution.Y;
            renderer.TextBatcher.Write(text.Clear().Append("The library does not prescribe any particular kind of gravity."), new Vector2(16, bottomY - 48), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("The IPoseIntegratorCallbacks provided to the simulation is responsible for telling the simulation how to integrate."), new Vector2(16, bottomY - 32), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("In this demo, all bodies are pulled towards the center of the planet."), new Vector2(16, bottomY - 16), 16, Vector3.One, font);
            base.Render(renderer, camera, input, text, font);
        }
    }
}
