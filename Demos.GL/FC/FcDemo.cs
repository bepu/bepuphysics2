using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using Fulfil.Libs.Machines.Mars.Dispense.SideDispenseDropTargeting.DTO;
using Fulfil.Libs.Machines.Mars.Dispense.SideDispenseDropTargeting.Physics;
using Fulfil.Tests.Libraries.Machines.Mars.Dispense.SideDispenseDropTargeting.Mocks;
using Fulfil.Tests.TestObjects;
using System.Numerics;

namespace Demos.FC;

internal class FcDemo : Demo
{
    public override void Initialize(ContentArchive content, Camera camera)
    {
        camera.Position = new Vector3(100, 150, 300);
        camera.Yaw = 0;
        camera.Pitch = MathHelper.Pi * 0.1f;

        var startingHeight = 30;
        const double foamThickness = 9.5;
        var itemDims = new RectangularPrism(100, 130, 100);
        var itemDrop = MockItemGenerator.Build(itemDimensions: itemDims);
        var dropContext = MockDropContextBuilder.Build(item: itemDrop);
        var bagDims = dropContext.PhysicalDropConditions.SystemPhysicalProperties.BagCavityDimensions;
        var itemLocation = new Transform(Vector3Helpers.FromDoubles(x: bagDims.XLengthMm / 2, y: startingHeight + foamThickness, z: -bagDims.ZLengthMm / 2));
        var physicsEvaluator = new BePuPhysicsSimulator(new IgnoreTheReleasePoint(itemLocation))
        {
            TimeStepMs = 120,
            EvaluationTimeMs = 2500,
            FrictionCoefficient = 0.3f,
            MaximumRecoveryVelocity = 50,
            ImpactFrequency = 20,
            ImpactDampingRatio = 0.5f,
            DispenseArmRetractionSpeedMmPerSecond = 500,
            DispenseArmConveyorSpeedMMs = 0,
            DispenseArmRemovalTimeMs = 2500,
            StartingDistanceBehindTippingPointMm = 0,
            StoppedDistanceMm = 0.5,
            StoppedRotaionDegrees = 0.5,
            StoppedFrames = 3,
        };
        var simSetup = physicsEvaluator.InitializeSimulation(new Vector3(0, 0, 0), dropContext, BufferPool);
        Simulation = simSetup.Simulation;
    }
}
