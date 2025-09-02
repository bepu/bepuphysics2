using System;
using System.Linq;
using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using Demos.Demos;
using DemoUtilities;

namespace Demos.SpecializedTests;

/// <summary>
/// Stress tests contact manifold reduction heuristics at a variety of scales using box-box tests.
/// </summary>
public class ManifoldReductionScaleTestDemo : Demo
{
    public override void Initialize(ContentArchive content, Camera camera)
    {
        camera.Position = new Vector3(0, 0.5f, -3);
        camera.Yaw = MathF.PI;
        camera.Pitch = -0.3f;

        var bodyGravities = new CollidableProperty<float>(BufferPool);
        Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(10, 1), float.MaxValue, 0.01f),
            new PerBodyGravityDemo.PerBodyGravityDemoCallbacks(bodyGravities), new SolveDescription(8, 1));

        var scales = new[] { 0.001f, 0.01f, 0.1f, 1f, 10f, 100f, 1000f };
        var offsets = new[] { new Vector3(-0.5f, 0, -0.5f), new Vector3(-0.25f, 0, -0.25f), new Vector3(-0.125f, 0, -0.125f), new Vector3(-0.5f, 0, 0), new Vector3(-0.25f, 0, 0), new Vector3(-0.125f, 0, 0), };

        const int rotationSteps = 256;
        const float maxRotationTop = MathF.PI * 2f;
        const float maxRotationBottom = MathF.PI * 3f;

        const float pairSpacing = 4f;

        for (int scaleIndex = 0; scaleIndex < scales.Length; scaleIndex++)
        {
            var scale = scales[scaleIndex];
            var scaleGroupY = scaleIndex * pairSpacing * scale;
            var size = new Vector3(2f, 1f, 2f) * scale;
            var box = new Box(size.X, size.Y, size.Z);
            var shapeIndex = Simulation.Shapes.Add(box);
            var boxInertia = box.ComputeInertia(1f);
            for (int offsetIndex = 0; offsetIndex < offsets.Length; ++offsetIndex)
            {
                var offsetGroupZ = offsetIndex * pairSpacing * scale;
                var offset = offsets[offsetIndex] * scale;

                for (int rotationIndex = 0; rotationIndex < rotationSteps; rotationIndex++)
                {
                    var rotationAngleTop = (float)rotationIndex / (rotationSteps - 1) * maxRotationTop;
                    var rotationAngleBottom = (float)rotationIndex / (rotationSteps - 1) * maxRotationBottom;
                    var pairX = (rotationIndex - rotationSteps / 2) * pairSpacing * scale;

                    Simulation.Statics.Add(new StaticDescription(
                        new RigidPose(new Vector3(pairX, scaleGroupY, offsetGroupZ), Quaternion.CreateFromAxisAngle(Vector3.UnitY, rotationAngleBottom)), shapeIndex));

                    // Create top box (dynamic) - will be rotated around Y axis
                    var bodyHandle = Simulation.Bodies.Add(BodyDescription.CreateDynamic(
                        new RigidPose(new Vector3(pairX, size.Y + scaleGroupY, offsetGroupZ) + offset, Quaternion.CreateFromAxisAngle(Vector3.UnitY, rotationAngleTop)),
                        boxInertia, shapeIndex, -0.01f));
                    bodyGravities.Allocate(bodyHandle) = -10 * scale;
                }
            }
        }

        var groundBox = new Box(100000, 0.1f, 100000);
        Simulation.Statics.Add(new StaticDescription(
            new RigidPose(new Vector3(0, -1, 0), Quaternion.Identity),
            Simulation.Shapes.Add(groundBox)));
    }
}