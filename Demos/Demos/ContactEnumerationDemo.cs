using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using BepuUtilities;
using BepuPhysics;
using DemoContentLoader;
using DemoRenderer;
using BepuPhysics.Collidables;
using DemoUtilities;
using DemoRenderer.UI;

namespace Demos.Demos
{
    /// <summary>
    /// Shows how to enumerate over contacts that exist in the solver.
    /// </summary>
    public class ContactEnumerationDemo : Demo
    {
        int bodyHandle;
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 8, -20);
            camera.Yaw = MathHelper.Pi;

            Simulation = Simulation.Create(BufferPool, new TestCallbacks());
            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);

            bodyHandle = Simulation.Bodies.Add(BodyDescription.CreateConvexDynamic(new Vector3(0, 5, 0), 1, Simulation.Shapes, new Box(1, 2, 3)));

            Simulation.Statics.Add(new StaticDescription(new Vector3(1, -0.5f, 0), new CollidableDescription(Simulation.Shapes.Add(new Box(30, 1, 30)), 0.1f)));
        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            var body = new BodyReference(bodyHandle, Simulation.Bodies);
            ref var pose = ref body.Pose;
            ref var constraints = ref body.Constraints;
            for (int i = 0; i < constraints.Count; ++i)
            {
                ref var constraint = ref constraints[i];
                

            }
            base.Render(renderer, camera, input, text, font);
        }
    }
}
