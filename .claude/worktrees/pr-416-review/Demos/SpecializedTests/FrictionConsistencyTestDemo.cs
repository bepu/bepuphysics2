using System;
using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using Demos.Demos;

namespace Demos.SpecializedTests;

/// <summary>
/// Launches identical rows of boxes across two surfaces with matched materials: a single static box (convex manifolds, one Contact4OneBody per pair)
/// and a densely tessellated flat mesh (MeshReduction merges several convex triangle manifolds into one nonconvex manifold, Contact4NonconvexOneBody).
/// If the convex and nonconvex constraint formulations treat the friction coefficient consistently, each row should stop at the same distance on both
/// surfaces, and both should land near the analytic v^2 / (2 * mu * g).
/// </summary>
public class FrictionConsistencyTestDemo : Demo
{
    const int rowCount = 10;
    const float launchSpeed = 10;
    const float spinSpeed = 10;
    const float gravity = 10;

    public BodyHandle[] BoxSurfaceBoxes;
    public BodyHandle[] MeshSurfaceBoxes;
    public BodyHandle[] BoxSurfaceSpinners;
    public BodyHandle[] MeshSurfaceSpinners;
    public BodyHandle[] BoxSurfaceSpinSliders;
    public BodyHandle[] MeshSurfaceSpinSliders;
    public float[] BoxSurfaceSpinAngles;
    public float[] MeshSurfaceSpinAngles;
    public float[] BoxSurfaceSpinSliderAngles;
    public float[] MeshSurfaceSpinSliderAngles;
    public float[] SpinSliderStartZ;
    public float[] FrictionCoefficients;
    float startX;

    public override void Initialize(ContentArchive content, Camera camera)
    {
        camera.Position = new Vector3(10, 5, 120);
        camera.Yaw = 0;
        camera.Pitch = 0.4f;

        var collidableMaterials = new CollidableProperty<FrictionDemo.SimpleMaterial>();
        Simulation = Simulation.Create(BufferPool, new FrictionDemo.FrictionCallbacks { CollidableMaterials = collidableMaterials }, new DemoPoseIntegratorCallbacks(new Vector3(0, -gravity, 0)), new SolveDescription(4, 1));

        var surfaceMaterial = new FrictionDemo.SimpleMaterial { FrictionCoefficient = 1, MaximumRecoveryVelocity = 2, SpringSettings = new SpringSettings(30, 1) };

        //Convex surface: one big box.
        collidableMaterials.Allocate(Simulation.Statics.Add(new StaticDescription(new Vector3(50, -0.5f, 14), Simulation.Shapes.Add(new Box(140, 1, 64))))) = surfaceMaterial;

        //Nonconvex surface: flat mesh with 0.5 unit cells, so a 2x2 box footprint spans several triangles and MeshReduction has multiple manifolds to merge.
        var mesh = DemoMeshHelper.CreateDeformedPlane(281, 129, (x, y) => new Vector3(x, 0, y), new Vector3(0.5f, 1, 0.5f), BufferPool);
        collidableMaterials.Allocate(Simulation.Statics.Add(new StaticDescription(new Vector3(-20, 0, 50), Simulation.Shapes.Add(mesh)))) = surfaceMaterial;

        var shape = new Box(2, 0.5f, 2);
        var shapeIndex = Simulation.Shapes.Add(shape);
        var inertia = shape.ComputeInertia(1);

        BoxSurfaceBoxes = new BodyHandle[rowCount];
        MeshSurfaceBoxes = new BodyHandle[rowCount];
        BoxSurfaceSpinners = new BodyHandle[rowCount];
        MeshSurfaceSpinners = new BodyHandle[rowCount];
        BoxSurfaceSpinSliders = new BodyHandle[rowCount];
        MeshSurfaceSpinSliders = new BodyHandle[rowCount];
        BoxSurfaceSpinAngles = new float[rowCount];
        MeshSurfaceSpinAngles = new float[rowCount];
        BoxSurfaceSpinSliderAngles = new float[rowCount];
        MeshSurfaceSpinSliderAngles = new float[rowCount];
        SpinSliderStartZ = new float[rowCount];
        FrictionCoefficients = new float[rowCount];
        startX = 0;
        for (int i = 0; i < rowCount; ++i)
        {
            //The surface material's coefficient is 1 and the callbacks blend multiplicatively, so the pair coefficient equals the sliding box's coefficient.
            FrictionCoefficients[i] = 0.25f * (i + 1);
            var material = new FrictionDemo.SimpleMaterial { FrictionCoefficient = FrictionCoefficients[i], MaximumRecoveryVelocity = 2, SpringSettings = new SpringSettings(30, 1) };

            var description = BodyDescription.CreateDynamic(new Vector3(startX, 0.251f, -15 + i * 3), new Vector3(launchSpeed, 0, 0), inertia, shapeIndex, -1);
            BoxSurfaceBoxes[i] = Simulation.Bodies.Add(description);
            collidableMaterials.Allocate(BoxSurfaceBoxes[i]) = material;

            description.Pose.Position.Z = 53 + i * 3;
            MeshSurfaceBoxes[i] = Simulation.Bodies.Add(description);
            collidableMaterials.Allocate(MeshSurfaceBoxes[i]) = material;

            //Twist friction comparison: boxes spinning in place about the vertical axis behind the launch line.
            //The convex surface resists through the manifold's central twist constraint; the mesh's nonconvex manifold has no twist constraint,
            //so its resistance emerges from the per-contact tangent friction constraints.
            var spinnerDescription = BodyDescription.CreateDynamic(new Vector3(-12, 0.251f, -15 + i * 3), new BodyVelocity(default, new Vector3(0, spinSpeed, 0)), inertia, shapeIndex, -1);
            BoxSurfaceSpinners[i] = Simulation.Bodies.Add(spinnerDescription);
            collidableMaterials.Allocate(BoxSurfaceSpinners[i]) = material;

            spinnerDescription.Pose.Position.Z = 53 + i * 3;
            MeshSurfaceSpinners[i] = Simulation.Bodies.Add(spinnerDescription);
            collidableMaterials.Allocate(MeshSurfaceSpinners[i]) = material;

            //Simultaneous slide + spin couples the two friction modes: every contact's friction direction has to serve both the linear and angular motion,
            //so matching trajectories require the tangent/twist budget split to agree between the convex and nonconvex formulations.
            SpinSliderStartZ[i] = 16 + i * 3;
            var spinSliderDescription = BodyDescription.CreateDynamic(new Vector3(startX, 0.251f, SpinSliderStartZ[i]), new BodyVelocity(new Vector3(launchSpeed, 0, 0), new Vector3(0, spinSpeed, 0)), inertia, shapeIndex, -1);
            BoxSurfaceSpinSliders[i] = Simulation.Bodies.Add(spinSliderDescription);
            collidableMaterials.Allocate(BoxSurfaceSpinSliders[i]) = material;

            spinSliderDescription.Pose.Position.Z = SpinSliderStartZ[i] + 68;
            MeshSurfaceSpinSliders[i] = Simulation.Bodies.Add(spinSliderDescription);
            collidableMaterials.Allocate(MeshSurfaceSpinSliders[i]) = material;
        }
    }

    public override void Update(Window window, Camera camera, Input input, float dt)
    {
        base.Update(window, camera, input, dt);
        //Integrate spun angle so multi-revolution spins can be reported; orientation alone wraps.
        for (int i = 0; i < rowCount; ++i)
        {
            BoxSurfaceSpinAngles[i] += Simulation.Bodies[BoxSurfaceSpinners[i]].Velocity.Angular.Y * TimestepDuration;
            MeshSurfaceSpinAngles[i] += Simulation.Bodies[MeshSurfaceSpinners[i]].Velocity.Angular.Y * TimestepDuration;
            BoxSurfaceSpinSliderAngles[i] += Simulation.Bodies[BoxSurfaceSpinSliders[i]].Velocity.Angular.Y * TimestepDuration;
            MeshSurfaceSpinSliderAngles[i] += Simulation.Bodies[MeshSurfaceSpinSliders[i]].Velocity.Angular.Y * TimestepDuration;
        }
    }

    float GetDistance(BodyHandle handle)
    {
        return Simulation.Bodies[handle].Pose.Position.X - startX;
    }

    public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
    {
        var resolution = renderer.Surface.Resolution;
        var lineY = resolution.Y - 32 - 16 * (rowCount * 2 + 5);
        renderer.TextBatcher.Write(text.Clear().Append("Identical boxes slide on a static box (convex manifolds, near strip) and a dense flat mesh (nonconvex manifolds, far strip)."), new Vector2(16, lineY), 16, Vector3.One, font);
        lineY += 16;
        renderer.TextBatcher.Write(text.Clear().Append("Expected stop distance is v^2 / (2 * mu * g); consistent friction handling would make all three slide columns match per row."), new Vector2(16, lineY), 16, Vector3.One, font);
        lineY += 16;
        renderer.TextBatcher.Write(text.Clear().Append("The spinner columns report revolutions spun in place; the convex twist constraint and the mesh's emergent per-contact resistance should roughly agree."), new Vector2(16, lineY), 16, Vector3.One, font);
        lineY += 24;
        for (int i = 0; i < rowCount; ++i)
        {
            text.Clear().Append("mu ").Append(FrictionCoefficients[i], 2)
                .Append(":   slide box ").Append(GetDistance(BoxSurfaceBoxes[i]), 2)
                .Append(", mesh ").Append(GetDistance(MeshSurfaceBoxes[i]), 2)
                .Append(", expected ").Append(launchSpeed * launchSpeed / (2 * FrictionCoefficients[i] * gravity), 2)
                .Append("   |   spin box ").Append(BoxSurfaceSpinAngles[i] / (2 * MathF.PI), 2)
                .Append(" rev, mesh ").Append(MeshSurfaceSpinAngles[i] / (2 * MathF.PI), 2).Append(" rev");
            renderer.TextBatcher.Write(text, new Vector2(16, lineY), 16, Vector3.One, font);
            lineY += 16;
        }
        lineY += 8;
        renderer.TextBatcher.Write(text.Clear().Append("Spin-sliders launch with both velocities at once; consistent friction would give matching endpoints and drift on both surfaces."), new Vector2(16, lineY), 16, Vector3.One, font);
        lineY += 24;
        for (int i = 0; i < rowCount; ++i)
        {
            text.Clear().Append("mu ").Append(FrictionCoefficients[i], 2)
                .Append(":   spin-slide box (x ").Append(GetDistance(BoxSurfaceSpinSliders[i]), 2)
                .Append(", z ").Append(Simulation.Bodies[BoxSurfaceSpinSliders[i]].Pose.Position.Z - SpinSliderStartZ[i], 2)
                .Append(", ").Append(BoxSurfaceSpinSliderAngles[i] / (2 * MathF.PI), 2)
                .Append(" rev),   mesh (x ").Append(GetDistance(MeshSurfaceSpinSliders[i]), 2)
                .Append(", z ").Append(Simulation.Bodies[MeshSurfaceSpinSliders[i]].Pose.Position.Z - (SpinSliderStartZ[i] + 68), 2)
                .Append(", ").Append(MeshSurfaceSpinSliderAngles[i] / (2 * MathF.PI), 2).Append(" rev)");
            renderer.TextBatcher.Write(text, new Vector2(16, lineY), 16, Vector3.One, font);
            lineY += 16;
        }

        //Show which contact constraint types actually exist so there's no guessing about whether the mesh produced nonconvex manifolds.
        text.Clear().Append("Constraints: ");
        Span<int> typeCounts = stackalloc int[PairCache.CollisionConstraintTypeCount];
        typeCounts.Clear();
        ref var activeSet = ref Simulation.Solver.ActiveSet;
        for (int batchIndex = 0; batchIndex < activeSet.Batches.Count; ++batchIndex)
        {
            ref var batch = ref activeSet.Batches[batchIndex];
            for (int typeBatchIndex = 0; typeBatchIndex < batch.TypeBatches.Count; ++typeBatchIndex)
            {
                ref var typeBatch = ref batch.TypeBatches[typeBatchIndex];
                if (typeBatch.TypeId < typeCounts.Length)
                    typeCounts[typeBatch.TypeId] += typeBatch.ConstraintCount;
            }
        }
        bool first = true;
        for (int typeId = 0; typeId < typeCounts.Length; ++typeId)
        {
            if (typeCounts[typeId] > 0)
            {
                if (!first)
                    text.Append(",  ");
                first = false;
                var name = Simulation.Solver.TypeProcessors[typeId].GetType().Name;
                text.Append(name, 0, name.Length - "TypeProcessor".Length).Append(" x").Append(typeCounts[typeId]);
            }
        }
        renderer.TextBatcher.Write(text, new Vector2(16, lineY + 8), 16, Vector3.One, font);
        base.Render(renderer, camera, input, text, font);
    }
}
