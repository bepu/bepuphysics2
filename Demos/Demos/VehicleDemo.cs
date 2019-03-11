using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using DemoContentLoader;
using DemoRenderer;

namespace Demos.Demos
{
    public class VehicleDemo : Demo
    {
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 25, 100);
            camera.Yaw = 0;
            camera.Pitch = 0;

            var filters = new BodyProperty<SubgroupCollisionFilter>();
            Simulation = Simulation.Create(BufferPool, new SubgroupFilteredCallbacks() { CollisionFilters = filters }, new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));

        }
    }
}
