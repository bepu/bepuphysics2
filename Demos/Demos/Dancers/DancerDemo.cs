using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using System;
using System.Numerics;
using System.Runtime.InteropServices;

namespace Demos.Demos.Dancers
{
    /// <summary>
    /// A bunch of background dancers struggle to keep up with the masterful purple prancer while wearing dresses made of out of balls connected by constraints.
    /// Combined with the <see cref="DemoDancers"/> implementation, this provides a starting point for cosmetic cloth attached to characters. 
    /// </summary>
    public class DancerDemo : Demo
    {
        //This demo relies on the DemoDancers to manage all the ragdolls and their simulations. 
        //All this demo needs to do is make a dress out of balls and drape it onto them.
        DemoDancers dancers;
        static BodyHandle[,] CreateDressBodyGrid(Vector3 position, int widthInNodes, float spacing, float bodyRadius, float massPerBody,
            int instanceId, Simulation simulation, CollidableProperty<ClothCollisionFilter> filters)
        {
            var description = BodyDescription.CreateDynamic(QuaternionEx.Identity, new BodyInertia { InverseMass = 1f / massPerBody }, simulation.Shapes.Add(new Sphere(bodyRadius)), 0.01f);
            BodyHandle[,] handles = new BodyHandle[widthInNodes, widthInNodes];
            var armHoleCenter = new Vector2(DemoDancers.ArmOffsetX + 0.065f, 0);
            var armHoleRadius = 0.095f;
            var armHoleRadiusSquared = armHoleRadius * armHoleRadius;
            var halfWidth = widthInNodes * spacing / 2;
            var halfWidthSquared = halfWidth * halfWidth;
            var halfWidthOffset = new Vector2(halfWidth);
            for (int rowIndex = 0; rowIndex < widthInNodes; ++rowIndex)
            {
                for (int columnIndex = 0; columnIndex < widthInNodes; ++columnIndex)
                {
                    var horizontalPosition = new Vector2(columnIndex, rowIndex) * spacing - halfWidthOffset;
                    var distanceSquared0 = Vector2.DistanceSquared(horizontalPosition, armHoleCenter);
                    var distanceSquared1 = Vector2.DistanceSquared(horizontalPosition, -armHoleCenter);
                    var centerDistanceSquared = horizontalPosition.LengthSquared();
                    if (distanceSquared0 < armHoleRadiusSquared || distanceSquared1 < armHoleRadiusSquared || centerDistanceSquared > halfWidthSquared)
                    {
                        //Too close to an arm or too far from the center, don't create any bodies here.
                        handles[rowIndex, columnIndex] = new BodyHandle { Value = -1 };
                    }
                    else
                    {
                        description.Pose.Position = new Vector3(horizontalPosition.X, 0, horizontalPosition.Y) + position;
                        var handle = simulation.Bodies.Add(description);
                        handles[rowIndex, columnIndex] = handle;
                        if (filters != null)
                            filters.Allocate(handle) = new ClothCollisionFilter(rowIndex, columnIndex, instanceId);
                    }
                }
            }
            return handles;
        }
        static void CreateDistanceConstraints(BodyHandle[,] bodyHandles, SpringSettings springSettings, Simulation simulation)
        {
            void CreateConstraintBetweenBodies(BodyHandle aHandle, BodyHandle bHandle)
            {
                //Only create a constraint if bodies on both sides of the pair actually exist.
                //In this demo, we use -1 in the body handle slot to represent 'no body'.
                if (aHandle.Value >= 0 && bHandle.Value >= 0)
                {
                    var a = simulation.Bodies[aHandle];
                    var b = simulation.Bodies[bHandle];
                    //Note the use of a limit; the distance is allowed to go smaller.
                    //This helps stop the cloth from having unnatural rigidity.
                    var distance = Vector3.Distance(a.Pose.Position, b.Pose.Position);
                    simulation.Solver.Add(aHandle, bHandle, new CenterDistanceLimit(distance * 0.15f, distance, springSettings));
                }
            }
            for (int rowIndex = 0; rowIndex < bodyHandles.GetLength(0); ++rowIndex)
            {
                for (int columnIndex = 0; columnIndex < bodyHandles.GetLength(1) - 1; ++columnIndex)
                {
                    CreateConstraintBetweenBodies(bodyHandles[rowIndex, columnIndex], bodyHandles[rowIndex, columnIndex + 1]);
                }
            }
            for (int rowIndex = 0; rowIndex < bodyHandles.GetLength(0) - 1; ++rowIndex)
            {
                for (int columnIndex = 0; columnIndex < bodyHandles.GetLength(1); ++columnIndex)
                {
                    CreateConstraintBetweenBodies(bodyHandles[rowIndex, columnIndex], bodyHandles[rowIndex + 1, columnIndex]);
                }
            }
            for (int rowIndex = 0; rowIndex < bodyHandles.GetLength(0) - 1; ++rowIndex)
            {
                for (int columnIndex = 0; columnIndex < bodyHandles.GetLength(1) - 1; ++columnIndex)
                {
                    CreateConstraintBetweenBodies(bodyHandles[rowIndex, columnIndex], bodyHandles[rowIndex + 1, columnIndex + 1]);
                    CreateConstraintBetweenBodies(bodyHandles[rowIndex, columnIndex + 1], bodyHandles[rowIndex + 1, columnIndex]);
                }
            }
        }


        static void TailorDress(Simulation simulation, CollidableProperty<ClothCollisionFilter> filters, DancerBodyHandles bodyHandles, int dancerIndex, int dancerGridWidth, float levelOfDetail)
        {
            //The demo uses lower resolution grids on dancers further away from the main dancer.
            //This is a sorta-example of level of detail. In a 'real' use case, you'd probably want to transition between levels of detail dynamically as the camera moved around.
            //That's a little trickier, but doable. Going low to high, for example, requires creating bodies at interpolated positions between existing bodies, while going to a lower level of detail removes them.
            levelOfDetail = MathF.Max(0f, MathF.Min(1.5f, levelOfDetail));
            var targetDressDiameter = 2.6f;
            var fullDetailWidthInBodies = 29;
            float spacingAtFullDetail = targetDressDiameter / fullDetailWidthInBodies;
            float bodyRadius = spacingAtFullDetail / 1.75f;
            var scale = MathF.Pow(2, levelOfDetail);
            var widthInBodies = (int)MathF.Ceiling(fullDetailWidthInBodies / scale);
            var spacing = spacingAtFullDetail * scale;
            var chest = simulation.Bodies[bodyHandles.Chest];
            ref var chestShape = ref simulation.Shapes.GetShape<Capsule>(chest.Collidable.Shape.Index);
            var topOfChestHeight = chest.Pose.Position.Y + chestShape.Radius + bodyRadius;
            var bodies = CreateDressBodyGrid(new Vector3(0, topOfChestHeight, 0) + DemoDancers.GetOffsetForDancer(dancerIndex, dancerGridWidth), widthInBodies, spacing, bodyRadius, 0.01f, dancerIndex, simulation, filters);
            //Create constraints that bind the cloth bodies closest to the chest, to the chest. This keeps the dress from sliding around.
            //In the higher resolution simulations, the arm holes and cloth bodies can actually handle it with no help, but for lower levels of detail it can be useful.
            //Also, it's very common to want to control how cloth sticks to a character. You could extend this approach to, for example, keep cloth near the body at the waist like a belt.
            //This demo uses constraints to attach a subset of the cloth bodies to the chest.
            //You could also either treat the bodies as kinematic and have them follow the body, or attach any constraints that would have involved the cloth body to the body instead.
            //Using constraints gives you more options in configuration- the attachment doesn't have to be perfectly rigid.
            //For the purposes of this demo, it's also simpler to just use some more constraints.
            var midpoint = (widthInBodies * 0.5f - 0.5f);
            var zRange = (chestShape.Radius * 0.65f) / spacing;
            var xRange = (chestShape.Radius * 0.5f + chestShape.HalfLength) / spacing;
            var minX = (int)MathF.Ceiling(midpoint - xRange);
            var maxX = (int)(midpoint + xRange);
            var minZ = (int)MathF.Ceiling(midpoint - zRange);
            var maxZ = (int)(midpoint + zRange);
            for (int z = minZ; z <= maxZ; ++z)
            {
                for (int x = minX; x <= maxX; ++x)
                {
                    var clothNodeHandle = bodies[z, x];
                    //When creating bodies, we set handles for bodies that don't exist to -1.
                    if (clothNodeHandle.Value >= 0)
                    {
                        var clothNodeBody = simulation.Bodies[clothNodeHandle];
                        simulation.Solver.Add(chest.Handle, clothNodeBody.Handle,
                            new BallSocket
                            {
                                LocalOffsetA = QuaternionEx.Transform(clothNodeBody.Pose.Position - chest.Pose.Position, Quaternion.Conjugate(chest.Pose.Orientation)),
                                SpringSettings = new SpringSettings(30, 1)
                            });
                    }
                }
            }
            CreateDistanceConstraints(bodies, new SpringSettings(60, 1), simulation);
        }


        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 2, 10);
            camera.Yaw = 0;
            camera.Pitch = 0;

            var collisionFilters = new CollidableProperty<SubgroupCollisionFilter>();
            Simulation = Simulation.Create(BufferPool, new SubgroupFilteredCallbacks { CollisionFilters = collisionFilters }, new DemoPoseIntegratorCallbacks(new Vector3(0, 0, 0)), new SolveDescription(8, 1));

            dancers = new DemoDancers().Initialize<ClothCallbacks, ClothCollisionFilter>(16, 16, Simulation, collisionFilters, ThreadDispatcher, BufferPool, new SolveDescription(1, 4), TailorDress, new ClothCollisionFilter(0, 0, -1));

        }
        public unsafe override void Update(Window window, Camera camera, Input input, float dt)
        {
            dancers.UpdateTargets(Simulation);
            base.Update(window, camera, input, dt);
        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            renderer.Shapes.AddInstances(dancers.Simulations, ThreadDispatcher);
            renderer.Lines.Extract(dancers.Simulations, ThreadDispatcher);

            var resolution = renderer.Surface.Resolution;
            renderer.TextBatcher.Write(text.Clear().Append("Cosmetic simulations, like cloth, often don't need to be in a game's main simulation."), new Vector2(16, resolution.Y - 144), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Every background dancer in this demo has its own simulation. All dancers can be easily updated in parallel."), new Vector2(16, resolution.Y - 128), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Dancers further from the main dancer use sparser cloth and disable self collision for extra performance."), new Vector2(16, resolution.Y - 112), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Dancer count: ").Append(dancers.Handles.Length), new Vector2(16, resolution.Y - 80), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Total cloth body count: ").Append(dancers.BodyCount), new Vector2(16, resolution.Y - 64), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Total cloth constraint count: ").Append(dancers.ConstraintCount), new Vector2(16, resolution.Y - 48), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Total dancer execution time (ms): ").Append(dancers.ExecutionTime * 1000, 2), new Vector2(16, resolution.Y - 32), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Amortized execution time per dancer (us): ").Append(dancers.ExecutionTime * 1e6 / dancers.Handles.Length, 1), new Vector2(16, resolution.Y - 16), 16, Vector3.One, font);

            base.Render(renderer, camera, input, text, font);
        }
        protected override void OnDispose()
        {
            dancers.Dispose(BufferPool);

        }
    }
}
