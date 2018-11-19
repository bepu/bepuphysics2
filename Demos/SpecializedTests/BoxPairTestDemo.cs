using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using BepuPhysics;
using BepuPhysics.Collidables;

namespace Demos.SpecializedTests
{
    public class BoxPairTestDemo : Demo
    {
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 4, -6);
            camera.Yaw = MathHelper.Pi;

            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, 0, 0)));

            Simulation.Bodies.Add(BodyDescription.CreateConvexKinematic(new Vector3(), Simulation.Shapes, new Box(3, 1, 4)));
            Simulation.Bodies.Add(BodyDescription.CreateConvexDynamic(new Vector3(0, 1.1f, 0), 1, Simulation.Shapes, new Box(2, 1, 3)));
        }
    }
}
