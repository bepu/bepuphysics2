using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using Demos.Demos;
using Demos.Demos.Dancers;
using DemoUtilities;
using System;
using System.Diagnostics;
using System.Numerics;

namespace Demos.SpecializedTests.Media;

/// <summary>
/// A bunch of somewhat overweight background dancers struggle to keep up with the masterful purple prancer.
/// Combined with the <see cref="DemoDancers"/> implementation, this shows an example of how cosmetic deformable physics could be applied to characters.
/// </summary>
public class VideoPlumpDancerDemo : Demo
{
    //This demo relies on the DemoDancers to manage all the ragdolls and their simulations. 
    //All this demo needs to do is make the fatsuits.
    DemoDancers dancers;

    //While creating the fatsuits, we'll precompute some stuff to make testing a little quicker.
    //Could make this significantly faster still by being trickier with vectorization, but the demo launch time is already reasonable.
    struct TestCapsule
    {
        public Vector3 Start;
        public Vector3 Direction;
        public float Length;
        public float Radius;
    }
    static TestCapsule CreateTestCapsule(Simulation simulation, BodyHandle handle)
    {
        var body = simulation.Bodies[handle];
        Debug.Assert(body.Collidable.Shape.Type == Capsule.Id, "For the purposes of this demo, we assume that all of the bodies that are being tested are capsules.");
        ref var shape = ref simulation.Shapes.GetShape<Capsule>(body.Collidable.Shape.Index);
        var pose = body.Pose;
        TestCapsule toReturn;
        QuaternionEx.TransformUnitY(pose.Orientation, out toReturn.Direction);
        toReturn.Start = pose.Position - toReturn.Direction * shape.HalfLength;
        toReturn.Radius = shape.Radius;
        toReturn.Length = shape.HalfLength * 2;
        return toReturn;

    }
    unsafe static void CreateBodyGrid(DancerBodyHandles bodyHandles, Int3 axisSizeInBodies, Vector3 gridMinimum, Vector3 gridMaximum, float bodyRadius, float massPerBody,
        int instanceId, Simulation simulation, CollidableProperty<DeformableCollisionFilter> filters)
    {
        var shape = new Sphere(bodyRadius);
        var shapeIndex = simulation.Shapes.Add(shape);
        //Note that, unlike the DancerDemo where cloth nodes cannot rotate, the deformable sub-bodies can rotate.
        //That's because this demo is going to connect bodies together using Weld constraints, which control all six degrees of freedom.
        //You could also use a CenterDistanceConstraint/Limit with VolumeConstraints to maintain shape, but a bunch of Weld constraints is a little simpler.
        var description = BodyDescription.CreateDynamic(QuaternionEx.Identity, shape.ComputeInertia(massPerBody), shapeIndex, 0.01f);
        BodyHandle[,,] handles = new BodyHandle[axisSizeInBodies.X, axisSizeInBodies.Y, axisSizeInBodies.Z];
        BodyHandle[,,] nearestHandles = new BodyHandle[axisSizeInBodies.X, axisSizeInBodies.Y, axisSizeInBodies.Z];
        var gridSpan = gridMaximum - gridMinimum;
        var gridSpacing = gridSpan / new Vector3(axisSizeInBodies.X - 1, axisSizeInBodies.Y - 1, axisSizeInBodies.Z - 1);
        Span<TestCapsule> testCapsules = stackalloc TestCapsule[11];

        //DancerBodyHandles stores the head last, so we can just check the first 11 bodies that are all capsules. The head isn't going to be covered in the fatsuit, so it doesn't need to be checked anyway.
        var handlesBuffer = DancerBodyHandles.AsBuffer(&bodyHandles);
        for (int i = 0; i < 11; ++i)
        {
            testCapsules[i] = CreateTestCapsule(simulation, handlesBuffer[i]);
        }
        var center = (gridMinimum + gridMaximum) * 0.5f;

        for (int x = 0; x < axisSizeInBodies.X; ++x)
        {
            for (int y = 0; y < axisSizeInBodies.Y; ++y)
            {
                for (int z = 0; z < axisSizeInBodies.Z; ++z)
                {
                    var position = gridMinimum + gridSpacing * new Vector3(x, y, z);
                    float minimumDistance = float.MaxValue;
                    int minimumIndex = 0;
                    for (int i = 0; i < testCapsules.Length; ++i)
                    {
                        var testCapsule = testCapsules[i];
                        var distance = Vector3.Distance(position, testCapsule.Start + MathF.Max(0, MathF.Min(testCapsule.Length, Vector3.Dot(position - testCapsule.Start, testCapsule.Direction))) * testCapsule.Direction) - testCapsule.Radius;
                        if (distance < minimumDistance)
                        {
                            minimumDistance = distance;
                            minimumIndex = i;
                        }
                    }
                    nearestHandles[x, y, z] = handlesBuffer[minimumIndex];

                    var maximumDistanceForCreatingNodes = MathF.Max(0.1f, 0.8f - 1.5f * Vector3.Distance(position, center));
                    if (minimumDistance < bodyRadius)
                    {
                        //Intersecting; don't create a body. -2 for this demo marks the body as intersecting, so we can disambiguate it from slots that are just empty due to being too far away.
                        handles[x, y, z] = new BodyHandle { Value = -2 };
                    }
                    else if (minimumDistance > maximumDistanceForCreatingNodes)
                    {
                        //-1 means too far.
                        handles[x, y, z] = new BodyHandle { Value = -1 };
                    }
                    else
                    {
                        //Nearby. Create and attach it to the nearest body part.
                        description.Pose.Position = position;
                        var handle = simulation.Bodies.Add(description);
                        handles[x, y, z] = handle;
                        if (filters != null)
                            filters.Allocate(handle) = new DeformableCollisionFilter(x, y, z, instanceId);

                        var nearestHandle = handlesBuffer[minimumIndex];
                        var nearestPose = simulation.Bodies[nearestHandle].Pose;
                        var conjugate = Quaternion.Conjugate(nearestPose.Orientation);

                    }
                }
            }
        }
        for (int x = 0; x < axisSizeInBodies.X; ++x)
        {
            for (int y = 0; y < axisSizeInBodies.Y; ++y)
            {
                for (int z = 0; z < axisSizeInBodies.Z; ++z)
                {
                    //Kind of hacky, but simple: for every node that is exposed to the air (a neighbor has a body handle flagged as -1), make sure it has a collidable.
                    //Anything inside doesn't need a collidable.
                    var handle = handles[x, y, z];
                    if (handle.Value >= 0)
                    {
                        var needsAnchor =
                            (x != 0 && handles[x - 1, y, z].Value == -2) ||
                            (x != handles.GetLength(0) - 1 && handles[x + 1, y, z].Value == -2) ||
                            (y != 0 && handles[x, y - 1, z].Value == -2) ||
                            (y != handles.GetLength(1) - 1 && handles[x, y + 1, z].Value == -2) ||
                            (z != 0 && handles[x, y, z - 1].Value == -2) ||
                            (z != handles.GetLength(2) - 1 && handles[x, y, z + 1].Value == -2);
                        var source = simulation.Bodies[handle];
                        if (needsAnchor)
                        {
                            var nearestHandle = nearestHandles[x, y, z];
                            var nearestPose = simulation.Bodies[nearestHandle].Pose;
                            var conjugate = Quaternion.Conjugate(nearestPose.Orientation);
                            simulation.Solver.Add(nearestHandle, handle, new Weld
                            {
                                LocalOffset = QuaternionEx.Transform(source.Pose.Position - nearestPose.Position, conjugate),
                                LocalOrientation = conjugate,
                                SpringSettings = new SpringSettings(6, 0.4f)
                            });
                        }
                        var needsCollidable =
                            (x == 0 || handles[x - 1, y, z].Value == -1) || (x == handles.GetLength(0) - 1 || handles[x + 1, y, z].Value == -1) ||
                            (y == 0 || handles[x, y - 1, z].Value == -1) || (y == handles.GetLength(1) - 1 || handles[x, y + 1, z].Value == -1) ||
                            (z == 0 || handles[x, y, z - 1].Value == -1) || (z == handles.GetLength(2) - 1 || handles[x, y, z + 1].Value == -1);
                        if (!needsCollidable)
                        {
                            source.SetShape(default);
                        }

                        static void TryAdd(Simulation simulation, BodyReference source, BodyHandle targetHandle)
                        {
                            if (targetHandle.Value >= 0)
                            {
                                var target = simulation.Bodies[targetHandle];
                                simulation.Solver.Add(source.Handle, targetHandle, new Weld { LocalOffset = target.Pose.Position - source.Pose.Position, LocalOrientation = Quaternion.Identity, SpringSettings = new SpringSettings(6, 0.4f) });
                            }
                        }
                        if (x < handles.GetLength(0) - 1)
                        {
                            TryAdd(simulation, source, handles[x + 1, y, z]);
                        }
                        if (y < handles.GetLength(1) - 1)
                        {
                            TryAdd(simulation, source, handles[x, y + 1, z]);
                        }
                        if (z < handles.GetLength(2) - 1)
                        {
                            TryAdd(simulation, source, handles[x, y, z + 1]);
                        }
                    }
                }
            }
        }

    }


    static void CreateFatSuit(Simulation simulation, CollidableProperty<DeformableCollisionFilter> filters, DancerBodyHandles bodyHandles, int dancerIndex, int dancerGridWidth, float levelOfDetail)
    {
        //The demo uses lower resolution grids on dancers further away from the main dancer.
        //This is a sorta-example of level of detail. In a 'real' use case, you'd probably want to transition between levels of detail dynamically as the camera moved around.
        //That's a little trickier, but doable. Going low to high, for example, requires creating bodies at interpolated positions between existing bodies, while going to a lower level of detail removes them.
        levelOfDetail = MathF.Max(0f, MathF.Min(0.8f, levelOfDetail));
        var suitSize = new Vector3(1, 1f, 1);
        var fullDetailAxisBodyCounts = new Int3 { X = 23, Y = 23, Z = 23 };
        var scale = MathF.Pow(2, levelOfDetail);
        var axisBodyCounts = new Int3 { X = (int)MathF.Ceiling(fullDetailAxisBodyCounts.X / scale), Y = (int)MathF.Ceiling(fullDetailAxisBodyCounts.Y / scale), Z = (int)MathF.Ceiling(fullDetailAxisBodyCounts.Z / scale) };
        var bodyRadius = MathF.Min(suitSize.X / axisBodyCounts.X, MathF.Min(suitSize.Y / axisBodyCounts.Y, suitSize.Z / axisBodyCounts.Z));

        var chest = simulation.Bodies[bodyHandles.Chest];
        ref var chestShape = ref simulation.Shapes.GetShape<Capsule>(chest.Collidable.Shape.Index);
        var topOfChestHeight = chest.Pose.Position.Y + chestShape.Radius;
        var topOfChestPosition = new Vector3(0, topOfChestHeight, 0) + DemoDancers.GetOffsetForDancer(dancerIndex, dancerGridWidth);
        var suitMinimum = topOfChestPosition - suitSize * new Vector3(0.5f, 1f, 0.5f);
        var suitMaximum = suitMinimum + suitSize;
        CreateBodyGrid(bodyHandles, axisBodyCounts, suitMinimum, suitMaximum, bodyRadius, 0.01f, dancerIndex, simulation, filters);
    }


    public override void Initialize(ContentArchive content, Camera camera)
    {
        camera.Position = new Vector3(0, 2, 10);
        camera.Yaw = 0;
        camera.Pitch = 0;

        var collisionFilters = new CollidableProperty<SubgroupCollisionFilter>();
        Simulation = Simulation.Create(BufferPool, new SubgroupFilteredCallbacks(collisionFilters), new DemoPoseIntegratorCallbacks(new Vector3(0, 0, 0)), new SolveDescription(8, 1));

        //Note that, because the constraints in the fat suit are quite soft, we can get away with extremely minimal solving time. There's one substep with one velocity iteration.
        dancers = new DemoDancers().Initialize<DeformableCallbacks, DeformableCollisionFilter>(32, 32, Simulation, collisionFilters, ThreadDispatcher, BufferPool, new SolveDescription(1, 1), CreateFatSuit, new DeformableCollisionFilter(0, 0, 0, -1));

    }
    public override void Update(Window window, Camera camera, Input input, float dt)
    {
        dancers.UpdateTargets(Simulation);
        base.Update(window, camera, input, dt);
    }

    public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
    {
        renderer.Shapes.AddInstances(dancers.Simulations, ThreadDispatcher);
        renderer.Lines.Extract(dancers.Simulations, ThreadDispatcher);

        var resolution = renderer.Surface.Resolution;
        //renderer.TextBatcher.Write(text.Clear().Append("Cosmetic simulations, like character blubber, often don't need to be in a game's main simulation."), new Vector2(16, resolution.Y - 144), 16, Vector3.One, font);
        //renderer.TextBatcher.Write(text.Clear().Append("Every background dancer in this demo has its own simulation. All dancers can be easily updated in parallel."), new Vector2(16, resolution.Y - 128), 16, Vector3.One, font);
        //renderer.TextBatcher.Write(text.Clear().Append("Dancers further from the main dancer use sparser body grids and disable self collision for extra performance."), new Vector2(16, resolution.Y - 112), 16, Vector3.One, font);
        renderer.TextBatcher.Write(text.Clear().Append("Dancer count: ").Append(dancers.Handles.Length), new Vector2(16, resolution.Y - 80), 16, Vector3.One, font);
        renderer.TextBatcher.Write(text.Clear().Append("Total deformable body count: ").Append(dancers.BodyCount), new Vector2(16, resolution.Y - 64), 16, Vector3.One, font);
        renderer.TextBatcher.Write(text.Clear().Append("Total deformable constraint count: ").Append(dancers.ConstraintCount), new Vector2(16, resolution.Y - 48), 16, Vector3.One, font);
        renderer.TextBatcher.Write(text.Clear().Append("Total dancer execution time (ms): ").Append(dancers.ExecutionTime * 1000, 2), new Vector2(16, resolution.Y - 32), 16, Vector3.One, font);
        renderer.TextBatcher.Write(text.Clear().Append("Amortized execution time per dancer (us): ").Append(dancers.ExecutionTime * 1e6 / dancers.Handles.Length, 1), new Vector2(16, resolution.Y - 16), 16, Vector3.One, font);

        base.Render(renderer, camera, input, text, font);
    }
    protected override void OnDispose()
    {
        dancers.Dispose(BufferPool);

    }
}
