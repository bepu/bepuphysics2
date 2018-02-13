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
    public class BoxTestDemo : Demo
    {
        public unsafe override void Initialize(Camera camera)
        {
            camera.Position = new Vector3(-10, 5, -10);
            //camera.Yaw = MathHelper.Pi ; 
            camera.Yaw = MathHelper.Pi * 3f / 4;
            //camera.Pitch = MathHelper.Pi * 0.1f;
            Simulation = Simulation.Create(BufferPool, new TestCallbacks());

            var shape = new Capsule(.25f, 2.5f);
            BodyInertia localInertia;
            localInertia.InverseMass = 1f;
            shape.ComputeLocalInverseInertia(localInertia.InverseMass, out localInertia.InverseInertiaTensor);
            //capsuleInertia.InverseInertiaTensor = new Triangular3x3();
            var shapeIndex = Simulation.Shapes.Add(ref shape);
            const int width = 8;
            const int height = 8;
            const int length = 8;
            var latticeSpacing = 1.1f;
            var latticeOffset = 0;// -0.5f * width * latticeSpacing;
            SimulationSetup.BuildLattice(
                new RegularGridBuilder(new Vector3(latticeSpacing, 2.1f, 2 * latticeSpacing), new Vector3(latticeOffset, 5, latticeOffset), localInertia, shapeIndex),
                new ConstraintlessLatticeBuilder(),
                width, height, length, Simulation, out var bodyHandles, out var constraintHandles);
            Simulation.PoseIntegrator.Gravity = new Vector3(0, -1, 0);
            Simulation.Deterministic = false;


            var staticShape = new Box(32, 4, 32);
            var staticShapeIndex = Simulation.Shapes.Add(ref staticShape);
            const int staticGridWidth = 1;
            const float staticSpacing = 12;
            var gridOffset = 0;// -0.5f * staticGridWidth * staticSpacing;
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
                                0 + gridOffset + i * staticSpacing,
                                -4,
                                2 + gridOffset + j * staticSpacing),
                            //Orientation = BepuUtilities.Quaternion.Identity
                            Orientation = BepuUtilities.Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 0, 1)), MathHelper.PiOver4 * 1e-3f)
                        }
                    };
                    Simulation.Statics.Add(ref staticDescription);
                }
            }

        }


    }
}
