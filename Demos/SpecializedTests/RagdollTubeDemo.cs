using BepuUtilities;
using DemoRenderer;
using BepuPhysics;
using BepuPhysics.Collidables;
using System.Numerics;
using Quaternion = BepuUtilities.Quaternion;
using System;
using BepuPhysics.CollisionDetection;
using System.Runtime.CompilerServices;
using System.Diagnostics;
using BepuPhysics.Constraints;
using DemoContentLoader;
using DemoUtilities;
using BepuUtilities.Memory;
using Demos.Demos;

namespace Demos.SpecializedTests
{
    /// <summary>
    /// Subjects a bunch of unfortunate ragdolls to a tumble dry cycle.
    /// </summary>
    public class RagdollTubeDemo : Demo
    {
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 9, -40);
            camera.Yaw = MathHelper.Pi;
            camera.Pitch = 0;
            var filters = new BodyProperty<SubgroupCollisionFilter>();
            Simulation = Simulation.Create(BufferPool, new SubgroupFilteredCallbacks { CollisionFilters = filters }, new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));

            int ragdollIndex = 0;
            var spacing = new Vector3(1.7f, 1.8f, 0.5f);
            int width = 4;
            int height = 4;
            int length = 44;
            var origin = -0.5f * spacing * new Vector3(width - 1, 0, length - 1) + new Vector3(0, 5f, 0);
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    for (int k = 0; k < length; ++k)
                    {
                        RagdollDemo.AddRagdoll(origin + spacing * new Vector3(i, j, k), Quaternion.CreateFromAxisAngle(new Vector3(0, 1, 0), MathHelper.Pi * 0.05f), ragdollIndex++, filters, Simulation);
                    }
                }
            }

            var tubeCenter = new Vector3(0, 8, 0);
            const int panelCount = 20;
            const float tubeRadius = 6;
            var panelShape = new Box(MathF.PI * 2 * tubeRadius / panelCount, 1, 80);
            var panelShapeIndex = Simulation.Shapes.Add(panelShape);
            var builder = new CompoundBuilder(BufferPool, Simulation.Shapes, panelCount + 1);
            for (int i = 0; i < panelCount; ++i)
            {
                var rotation = Quaternion.CreateFromAxisAngle(Vector3.UnitZ, i * MathHelper.TwoPi / panelCount);
                Quaternion.TransformUnitY(rotation, out var localUp);
                var position = localUp * tubeRadius;
                builder.AddForKinematic(panelShapeIndex, new RigidPose(position, rotation), 1);
            }
            builder.AddForKinematic(Simulation.Shapes.Add(new Box(1, 2, panelShape.Length)), new RigidPose(new Vector3(0, tubeRadius - 1, 0)), 0);
            builder.BuildKinematicCompound(out var children);
            var compound = new BigCompound(children, Simulation.Shapes, BufferPool);
            Simulation.Bodies.Add(BodyDescription.CreateKinematic(tubeCenter, new BodyVelocity(default, new Vector3(0, 0, .25f)), new CollidableDescription(Simulation.Shapes.Add(compound), 0.1f), new BodyActivityDescription()));
            builder.Dispose();

            var staticShape = new Box(300, 1, 300);
            var staticShapeIndex = Simulation.Shapes.Add(staticShape);
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
                    Position = new Vector3(0, -0.5f, 0),
                    Orientation = Quaternion.Identity
                }
            };
            Simulation.Statics.Add(staticDescription);
        }

    }
}


