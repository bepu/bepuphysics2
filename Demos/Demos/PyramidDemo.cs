using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using DemoRenderer;
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
            camera.Position = new Vector3(-20, 10, -20);
            //camera.Yaw = MathHelper.Pi ; 
            camera.Yaw = MathHelper.Pi * 3f / 4;
            //camera.Pitch = MathHelper.PiOver2 * 0.999f;
            Simulation = Simulation.Create(BufferPool, new TestCallbacks());

            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);
            Simulation.Deterministic = false;

            var boxShape = new Box(1, 1, 1);
            BodyInertia boxInertia;
            boxInertia.InverseMass = 1;
            boxShape.ComputeLocalInverseInertia(boxInertia.InverseMass, out boxInertia.InverseInertiaTensor);
            var boxIndex = Simulation.Shapes.Add(ref boxShape);
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
                                    (pyramidIndex - pyramidCount * 0.5f) * (boxShape.Length + 2)),
                                Orientation = BepuUtilities.Quaternion.Identity
                            },
                            Activity = new BodyActivityDescription { MinimumTimestepCountUnderThreshold = 32, SleepThreshold = .00f },
                            Collidable = new CollidableDescription { Shape = boxIndex, SpeculativeMargin = .1f }
                        };
                        Simulation.Bodies.Add(ref bodyDescription);
                    }
                }
            }

            var staticShape = new Box(100, 1, 100);
            var staticShapeIndex = Simulation.Shapes.Add(ref staticShape);

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
            Simulation.Statics.Add(ref staticDescription);


        }

    }
}
