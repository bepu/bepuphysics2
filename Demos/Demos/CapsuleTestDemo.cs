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

namespace Demos
{
    public class CapsuleTestDemo : Demo
    {
        public unsafe override void Initialize(Camera camera)
        {
            camera.Position = new Vector3(-10, 5, -10);
            //camera.Yaw = MathHelper.Pi ; 
            camera.Yaw = MathHelper.Pi * 3f / 4;
            //camera.Pitch = MathHelper.Pi * 0.1f;
            Simulation = Simulation.Create(BufferPool, new TestCallbacks());

            var shape = new Capsule(.5f, .5f);
            BodyInertia localInertia;
            localInertia.InverseMass = 1f;
            shape.ComputeLocalInverseInertia(localInertia.InverseMass, out localInertia.InverseInertiaTensor);
            //capsuleInertia.InverseInertiaTensor = new Triangular3x3();
            var shapeIndex = Simulation.Shapes.Add(ref shape);
            const int width = 32;
            const int height = 32;
            const int length = 32;
            var latticeSpacing = 1.1f;
            var latticeOffset = 0;// -0.5f * width * latticeSpacing;
            SimulationSetup.BuildLattice(
                new RegularGridBuilder(new Vector3(latticeSpacing, 2.1f, latticeSpacing), new Vector3(latticeOffset, 7, latticeOffset), localInertia, shapeIndex),
                new ConstraintlessLatticeBuilder(),
                width, height, length, Simulation, out var bodyHandles, out var constraintHandles);
            Simulation.PoseIntegrator.Gravity = new Vector3(0, -1, 0);
            Simulation.Deterministic = false;

            //var bodyDescription = new BodyDescription
            //{
            //    Pose = new RigidPose
            //    {
            //        Position = new Vector3(0, 5, -0.5f),
            //        //Orientation = BepuUtilities.Quaternion.Identity
            //        Orientation = BepuUtilities.Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), MathHelper.PiOver2)
            //    },
            //    LocalInertia = new BodyInertia(),
            //    Collidable = new CollidableDescription
            //    {
            //        Continuity = new ContinuousDetectionSettings(),
            //        SpeculativeMargin = 0.1f,
            //        Shape = shapeIndex
            //    },
            //    Activity = new BodyActivityDescription
            //    {
            //        SleepThreshold = -.1f,
            //        MinimumTimestepCountUnderThreshold = 32
            //    },
            //    //Velocity = new BodyVelocity { Angular = new Vector3(0, (rowIndex % 2 - 0.5f) * 20, 0) }
            //};
            //Simulation.Bodies.Add(ref bodyDescription);

            var staticShape = new Sphere(4f);
            var staticShapeIndex = Simulation.Shapes.Add(ref staticShape);
            const int staticGridWidth = 64;
            const float staticSpacing = 6;
            var gridOffset = -0.5f * staticGridWidth * staticSpacing;
            for (int i = 0; i < staticGridWidth; ++i)
            {
                for (int j = 0; j < staticGridWidth; ++j)
                {
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
                            Position = new Vector3(
                            gridOffset + i * staticSpacing,
                            -4,
                            gridOffset + j * staticSpacing),
                            Orientation = BepuUtilities.Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 0, 1)), MathHelper.PiOver4)
                        }
                    };
                    Simulation.Statics.Add(ref staticDescription);
                }
            }

        }


    }
}
