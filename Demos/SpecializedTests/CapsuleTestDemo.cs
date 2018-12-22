using BepuUtilities;
using DemoRenderer;
using DemoUtilities;
using BepuPhysics;
using BepuPhysics.Collidables;
using System;
using System.Numerics;
using System.Diagnostics;
using BepuUtilities.Memory;
using BepuUtilities.Collections;
using DemoContentLoader;

namespace Demos.SpecializedTests
{
    public class CapsuleTestDemo : Demo
    {
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-10, 5, -10);
            //camera.Yaw = MathHelper.Pi ; 
            camera.Yaw = MathHelper.Pi * 3f / 4;
            //camera.Pitch = MathHelper.Pi * 0.1f;
            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));

            var shape = new Capsule(.5f, .5f);
            shape.ComputeInertia(1, out var localInertia);
            var shapeIndex = Simulation.Shapes.Add(shape);
            const int width = 1;
            const int height = 1;
            const int length = 1;
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    for (int k = 0; k < length; ++k)
                    {
                        var location = new Vector3(1.5f, 1.5f, 4.4f) * new Vector3(i, j, k) + new Vector3(-width * 0.5f, 0.5f, -length * 0.5f);
                        var bodyDescription = new BodyDescription
                        {
                            Activity = new BodyActivityDescription { MinimumTimestepCountUnderThreshold = 32, SleepThreshold = 0.01f },
                            LocalInertia = localInertia,
                            Pose = new RigidPose
                            {
                                Orientation = BepuUtilities.Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), MathF.PI / 2),
                                Position = location
                            },
                            Collidable = new CollidableDescription { SpeculativeMargin = 50.1f, Shape = shapeIndex }
                        };
                        Simulation.Bodies.Add(bodyDescription);

                    }
                }
            }
            var boxShape = new Box(0.5f, 0.5f, 2.5f);
            boxShape.ComputeInertia(1, out var boxLocalInertia);
            var boxDescription = new BodyDescription
            {
                Activity = new BodyActivityDescription { MinimumTimestepCountUnderThreshold = 32, SleepThreshold = -0.01f },
                LocalInertia = boxLocalInertia,
                Pose = new RigidPose
                {
                    Orientation = BepuUtilities.Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), 0),
                    Position = new Vector3(1, -0.5f, 0)
                },
                Collidable = new CollidableDescription { SpeculativeMargin = 50.1f, Shape = Simulation.Shapes.Add(boxShape) }
            };
            Simulation.Bodies.Add(boxDescription);

            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -3, 0), new CollidableDescription { SpeculativeMargin = 0.1f, Shape = Simulation.Shapes.Add(new Box(4, 1, 4)) }));

        }

        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            if (input.WasDown(OpenTK.Input.Key.P))
                Console.WriteLine("$");
            base.Update(window, camera, input, dt);
        }
    }
}
