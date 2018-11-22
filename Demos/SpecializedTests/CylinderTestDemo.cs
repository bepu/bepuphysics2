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
    public class CylinderTestDemo : Demo
    {
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 4, -6);
            camera.Yaw = MathHelper.Pi;

            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, 0, 0)));

            //Simulation.Bodies.Add(BodyDescription.CreateConvexKinematic(new Vector3(), Simulation.Shapes, new Cylinder(3, 4)));
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 1; ++j)
                    Simulation.Bodies.Add(BodyDescription.CreateConvexDynamic(new Vector3(i * 11, (i + j) / 100f, j * 11), 1, Simulation.Shapes, new Cylinder(5, 1)));


            //Simulation.Statics.Add(new StaticDescription(new Vector3(0, -2, 0), new CollidableDescription(Simulation.Shapes.Add(new Cylinder(5, 1)), 0.1f)));
        }
    }
}
