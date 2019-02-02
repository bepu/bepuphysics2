using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.SweepTasks;
using BepuUtilities.Memory;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.Constraints;
using DemoRenderer.UI;
using DemoUtilities;
using Quaternion = BepuUtilities.Quaternion;

namespace Demos.SpecializedTests
{
    public class MPRTestDemo : Demo
    {
        Buffer<LineInstance> shapeLines;

        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-13f, 6, -13f);
            camera.Yaw = MathF.PI * 3f / 4;
            camera.Pitch = MathF.PI * 0.05f;
            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));

            var shapeA = new Cylinder(0.5f, 0.5f);
            var poseA = new RigidPose(new Vector3(0, 0, 0));
            var shapeb = new Cylinder(0.5f, 0.5f);
            var poseB = new RigidPose(new Vector3(0.5f, 0.5f, 0), Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), MathF.PI * 0.5f));

            shapeLines = MinkowskiShapeVisualizer.CreateLines<Cylinder, CylinderWide, CylinderSupportFinder, Cylinder, CylinderWide, CylinderSupportFinder>(
                shapeA, shapeb, poseA, poseB, 65536, 
                0.01f, new Vector3(0.4f, 0.4f, 0),
                0.1f, new Vector3(0, 1, 0), default, default, BufferPool);
        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            MinkowskiShapeVisualizer.Draw(shapeLines, renderer);
            base.Render(renderer, camera, input, text, font);
        }
    }
}
