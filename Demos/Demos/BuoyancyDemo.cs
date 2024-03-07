using System.Collections.Generic;
using System.Numerics;
using BepuPhysics.Constraints;
using BepuPhysics;
using DemoContentLoader;
using DemoRenderer;
using BepuPhysics.Collidables;
using System;
using DemoRenderer.UI;
using DemoUtilities;

namespace Demos.Demos;

public class BuoyancyDemo : Demo
{
    public List<BuoyantBody> BuoyantBodies;
    public float BuoyancyConstant;
    public float WaterDensity = 1000f;

    public override void Initialize(ContentArchive content, Camera camera)
    {
        camera.Position = new Vector3(0, 30, 100);

        var gravity = new Vector3(0, -10f, 0);
        BuoyancyConstant = WaterDensity * gravity.Y;
        Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), 
            new DemoPoseIntegratorCallbacks(gravity), new SolveDescription(8, 1));

        BuoyantBodies = new List<BuoyantBody>();

        var numBoxes = 300;
        for (var i = 0; i < numBoxes; i++)
        {
            var pos = new Vector3(Random.Shared.Next(-200, 200), Random.Shared.Next(5, 50), Random.Shared.Next(-200, 200));
            var orientation = GenerateRandomQuaternion();
            var size = new Vector3(Random.Shared.Next(1, 4), Random.Shared.Next(1, 4), Random.Shared.Next(1, 4));
            var volume = size.X * size.Y * size.Z;
            var density = Random.Shared.Next(300,600);
            var mass = volume * density;
            CreateBuoyantBox(pos, orientation, size, mass);
        }

        var numCylinders = 300;
        for (var i = 0; i < numCylinders; i++)
        {
            var pos = new Vector3(Random.Shared.Next(-200, 200), Random.Shared.Next(5, 50), Random.Shared.Next(-200, 200));
            var orientation = GenerateRandomQuaternion();
            var radius = Random.Shared.Next(1, 4);
            var length = Random.Shared.Next(1, 10);
            var volume = MathF.PI * radius * radius * length;
            var density = Random.Shared.Next(300, 600);
            var mass = volume * density;
            CreateBuoyantCylinder(pos, orientation, radius, length, mass);
        }
    }

    private void CreateBuoyantBox(Vector3 pos, Quaternion orientation, Vector3 size, float mass)
    {
        var box = new Box(size.X, size.Y, size.Z);
        var boxInertia = box.ComputeInertia(mass);
        var boxHandle = Simulation.Bodies.Add(BodyDescription.CreateDynamic(
            new RigidPose(pos, orientation), boxInertia,
            Simulation.Shapes.Add(box), 0.01f));
        var gridElementSize = MathF.Min(MathF.Min(size.X, size.Y), size.Z) / 2;

        BuoyantBodies.Add(new BuoyantBody
        {
            GridElementSize = gridElementSize,
            Handle = boxHandle,
            Shape = box
        });
    }

    private void CreateBuoyantCylinder(Vector3 pos, Quaternion orientation, float radius, float length, float mass)
    {
        var cylinder = new Cylinder(radius, length);
        var boxInertia = cylinder.ComputeInertia(mass);
        var boxHandle = Simulation.Bodies.Add(BodyDescription.CreateDynamic(
            new RigidPose(pos, orientation), boxInertia,
            Simulation.Shapes.Add(cylinder), 0.01f));
        var gridElementSize = MathF.Min(radius * 2, length) / 2;

        BuoyantBodies.Add(new BuoyantBody
        {
            GridElementSize = gridElementSize,
            Handle = boxHandle,
            Shape = cylinder
        });
    }

    private Quaternion GenerateRandomQuaternion()
    {
        // Generate random values for quaternion components
        var x = (float)Random.Shared.NextDouble();
        var y = (float)Random.Shared.NextDouble();
        var z = (float)Random.Shared.NextDouble();
        var w = (float)Random.Shared.NextDouble();

        // Normalize the quaternion to ensure it represents a valid rotation
        var magnitude = (float)Math.Sqrt(x * x + y * y + z * z + w * w);
        x /= magnitude;
        y /= magnitude;
        z /= magnitude;
        w /= magnitude;

        return new Quaternion(x, y, z, w);
    }

    private float ComputeWaveHeight(float xPosition, float zPosition)
    {
        return 0; // TODO: Hook this into an actual wave mesh. Currently just treating the water surface as a flat plane at y = 0
    }

    public override void Update(Window window, Camera camera, Input input, float dt)
    {
        foreach (var buoyantBody in BuoyantBodies)
        {
            var body = Simulation.Bodies[buoyantBody.Handle];
            if (TryGetSubmergedVolumeData(body, buoyantBody, out var submergedVolumeData))
            {
                CalculateBuoyancy(body, submergedVolumeData);
            }
        }

        base.Update(window, camera, input, dt);
    }

    /// <summary>
    /// Computes a rough estimate of submerged volume by raycasting the boundary box and approximating volumes as rectangular prisms
    /// </summary>
    /// <returns>False if totally unsubmerged</returns>
    private bool TryGetSubmergedVolumeData(BodyReference body, BuoyantBody buoyantBody, out SubmergedVolumeData submergedVolumeData)
    {
        submergedVolumeData = default;
        
        var bodyBoundingBox = body.BoundingBox;
        var shape = buoyantBody.Shape;

        var xLength = bodyBoundingBox.Max.X - bodyBoundingBox.Min.X;
        var zLength = bodyBoundingBox.Max.Z - bodyBoundingBox.Min.Z;

        var gridElementSize = buoyantBody.GridElementSize;

        var xElements = (int)MathF.Floor(xLength / gridElementSize);
        var zElements = (int)MathF.Floor(zLength / gridElementSize);

        var gridElementSurfaceArea = gridElementSize * gridElementSize;

        var totalSubmergedVolume = 0f;
        var totalUnsubmergedVolume = 0f;
        var sumX = 0f;
        var sumY = 0f;
        var sumZ = 0f;
        
        for (var i = 0; i < xElements; i++)
        {
            for (var j = 0; j < zElements; j++)
            {
                // Add half the grid element size so that we apply the ray test at the middle of the grid cell
                var xPoint = bodyBoundingBox.Min.X + gridElementSize / 2 + i * gridElementSize;
                var zPoint = bodyBoundingBox.Min.Z + gridElementSize / 2 + j * gridElementSize;

                var xzPointBelow = new Vector3(xPoint, -10000, zPoint);
                var xzPointAbove = new Vector3(xPoint, 10000, zPoint);

                // Ray test from below first
                if (!shape.RayTest(body.Pose, xzPointBelow, Vector3.UnitY, out var tBelow,
                        out var normalBelow)) continue;

                // Calculate the projection factor based on the normal. This helps us approximate the rectangular prism volume when it's not aligned with the raycast collision normal
                // That said this can be done better than a simple scaling factor
                var projectionFactorFromBelow = Math.Abs(Vector3.Dot(normalBelow, Vector3.UnitY));

                var belowHitY = xzPointBelow.Y + tBelow;

                var waveHeight = ComputeWaveHeight(xPoint, zPoint);

                var isElementFullyAboveWater = belowHitY > waveHeight;

                // Ray test from above
                if (!shape.RayTest(body.Pose, xzPointAbove, -Vector3.UnitY, out var tAbove,
                        out var normalAbove)) continue;

                var projectionFactorFromAbove = Math.Abs(Vector3.Dot(normalAbove, -Vector3.UnitY));

                var aboveHitY = xzPointAbove.Y - tAbove;
                if (aboveHitY > waveHeight)
                {
                    // First let's calculate the unsubmerged volume
                    var unsubmergedDistance =
                        isElementFullyAboveWater ? aboveHitY - belowHitY : aboveHitY - waveHeight;
                    var unsubmergedElementVolume = unsubmergedDistance * gridElementSurfaceArea * projectionFactorFromAbove;
                    totalUnsubmergedVolume += unsubmergedElementVolume;

                    // If it is above water height, we clamp the value to the water height
                    aboveHitY = waveHeight;
                }

                if (isElementFullyAboveWater)
                    continue;

                // Now we can calculate submerged distance
                var submergedDistance = aboveHitY - belowHitY;

                // Calculate submerged volume
                var submergedElementVolume = submergedDistance * gridElementSurfaceArea * projectionFactorFromBelow;
                sumX += xPoint * submergedElementVolume;
                sumY += (belowHitY + submergedDistance / 2) * submergedElementVolume;
                sumZ += zPoint * submergedElementVolume;
                totalSubmergedVolume += submergedElementVolume;
            }
        }

        // Need to check again, otherwise NaNs are possible. This is because boundingbox.min.y doesn't catch everything due to grid element size not always catching those points
        if (totalSubmergedVolume == 0) return false;

        var centerOfVolume = new Vector3(sumX / totalSubmergedVolume, sumY / totalSubmergedVolume,
            sumZ / totalSubmergedVolume);

        submergedVolumeData.CenterOfVolume = centerOfVolume;
        submergedVolumeData.SubmergedVolume = totalSubmergedVolume;
        submergedVolumeData.SubmergedVolumeRatio = totalSubmergedVolume / (totalSubmergedVolume + totalUnsubmergedVolume);

        return true;
    }

    public void CalculateBuoyancy(BodyReference body, SubmergedVolumeData submergedVolumeData)
    {
        var deltaTime = TimestepDuration;
        var buoyancyForce = BuoyancyConstant * submergedVolumeData.SubmergedVolume * deltaTime;

        var centerOfVolumeOffsetFromPosition = submergedVolumeData.CenterOfVolume - body.Pose.Position;

        var bodyBoundingBox = body.BoundingBox;
        var width = MathF.Min(bodyBoundingBox.Max.Z - bodyBoundingBox.Min.Z,
            bodyBoundingBox.Max.X - bodyBoundingBox.Min.X); //TODO: Should calculate actual submerged width for better accuracy
        var submergedCrossSectionalArea = submergedVolumeData.SubmergedVolume / width;
        var dragCoefficient = 1f;

        // Approximate linear drag force using the drag equation
        var linearDragForce = -Vector3.Normalize(body.Velocity.Linear) * 0.5f * dragCoefficient * submergedCrossSectionalArea *
                               WaterDensity * body.Velocity.Linear * body.Velocity.Linear * deltaTime;

        body.ApplyImpulse(new Vector3(0, buoyancyForce, 0) + linearDragForce, centerOfVolumeOffsetFromPosition);

        // Angular drag is just reduced by a factor. This can be done better
        var dragTorque = -2f * body.Velocity.Angular * deltaTime;
        dragTorque.Y = -0.1f * body.Velocity.Angular.Y * deltaTime;

        body.Velocity.Angular += dragTorque;
    }
    public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
    {
        renderer.Shapes.AddShape(new Box(1000, 0.1f, 1000), Simulation.Shapes, Vector3.Zero, new Vector3(0, 0.2f, 1));
        var resolution = renderer.Surface.Resolution;
        renderer.TextBatcher.Write(text.Clear().Append("This demo shows buoyancy on a flat plane. Because it approximates volume as rectangular prisms there can be strange behaviours depending on the rotation of the buoyant body."), new Vector2(16, resolution.Y - 16), 16, Vector3.One, font);
        base.Render(renderer, camera, input, text, font);
    }

    public struct SubmergedVolumeData
    {
        public Vector3 CenterOfVolume;
        public float SubmergedVolume;
        public float SubmergedVolumeRatio;
    }

    public struct BuoyantBody
    {
        public BodyHandle Handle;
        public IConvexShape Shape;
        public float GridElementSize;
    }
}