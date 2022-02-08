using DemoContentLoader;
using DemoRenderer;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using BepuPhysics;
using DemoRenderer.UI;
using System.IO;
using DemoUtilities;
using System.Diagnostics;
using BepuUtilities.Collections;
using BepuPhysics.Collidables;
using Demos.Demos.Characters;

namespace Demos.Demos.Sponsors
{
    public class NewtTyrannyDemo : Demo
    {
        QuickList<SponsorNewt> newts;

        Vector2 newtArenaMin, newtArenaMax;
        Random random;
        CharacterControllers characterControllers;
        QuickList<SponsorCharacterAI> characterAIs;
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(130, 50, 130);
            camera.Yaw = -MathF.PI * 0.25f;
            camera.Pitch = 0.4f;

            characterControllers = new CharacterControllers(BufferPool);
            Simulation = Simulation.Create(BufferPool, new CharacterNarrowphaseCallbacks(characterControllers), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));
            Simulation.Deterministic = true;

            DemoMeshHelper.LoadModel(content, BufferPool, @"Content\newt.obj", new Vector3(-10, 10, -10), out var newtMesh);
            var newtShape = Simulation.Shapes.Add(newtMesh);
            var newtCount = 10;
            newts = new QuickList<SponsorNewt>(newtCount, BufferPool);
            newtArenaMin = new Vector2(-250);
            newtArenaMax = new Vector2(250);
            random = new Random(8);
            for (int i = 0; i < newtCount; ++i)
            {
                ref var newt = ref newts.AllocateUnsafely();
                newt = new SponsorNewt(Simulation, newtShape, 0, newtArenaMin, newtArenaMax, random, i);
            }

            const float floorSize = 520;
            const float wallThickness = 200;
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -10f, 0), Simulation.Shapes.Add(new Box(floorSize, 20, floorSize))));
            Simulation.Statics.Add(new StaticDescription(new Vector3(floorSize * -0.5f - wallThickness * 0.5f, -5, 0), Simulation.Shapes.Add(new Box(wallThickness, 30, floorSize + wallThickness * 2))));
            Simulation.Statics.Add(new StaticDescription(new Vector3(floorSize * 0.5f + wallThickness * 0.5f, -5, 0), Simulation.Shapes.Add(new Box(wallThickness, 30, floorSize + wallThickness * 2))));
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -5, floorSize * -0.5f - wallThickness * 0.5f), Simulation.Shapes.Add(new Box(floorSize, 30, wallThickness))));
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -5, floorSize * 0.5f + wallThickness * 0.5f), Simulation.Shapes.Add(new Box(floorSize, 30, wallThickness))));

            const int characterCount = 2000;
            characterAIs = new QuickList<SponsorCharacterAI>(characterCount, BufferPool);
            var characterCollidable = Simulation.Shapes.Add(new Capsule(0.5f, 1f));
            for (int i = 0; i < characterCount; ++i)
            {
                var position2D = newtArenaMin + (newtArenaMax - newtArenaMin) * new Vector2(random.NextSingle(), random.NextSingle());
                var targetPosition = 0.5f * (newtArenaMin + (newtArenaMax - newtArenaMin) * new Vector2(random.NextSingle(), random.NextSingle()));
                characterAIs.AllocateUnsafely() = new SponsorCharacterAI(characterControllers, characterCollidable, new Vector3(position2D.X, 5, position2D.Y), targetPosition);
            }

            const int hutCount = 120;
            var hutBoxShape = new Box(0.4f, 2, 3);
            var obstacleDescription = BodyDescription.CreateDynamic(new Vector3(), hutBoxShape.ComputeInertia(20), new CollidableDescription(Simulation.Shapes.Add(hutBoxShape), 0.1f), 1e-2f);

            for (int i = 0; i < hutCount; ++i)
            {
                var position2D = newtArenaMin + (newtArenaMax - newtArenaMin) * new Vector2(random.NextSingle(), random.NextSingle());
                ColosseumDemo.CreateRing(Simulation, new Vector3(position2D.X, 0, position2D.Y), hutBoxShape, obstacleDescription, 4 + random.NextSingle() * 8, 2, random.Next(1, 10));

            }

            var overlordNewtShape = newtMesh;
            overlordNewtShape.Scale = new Vector3(60, 60, 60);
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, 10, -floorSize * 0.5f - 70), Simulation.Shapes.Add(overlordNewtShape)));


            character = new CharacterInput(characterControllers, new Vector3(-108.89504f, 28.403418f, 38.27505f), new Capsule(0.5f, 1), 0.1f, .1f, 20, 100, 6, 4, MathF.PI * 0.4f);

            Console.WriteLine($"body count: {Simulation.Bodies.ActiveSet.Count}");
        }

        CharacterInput character;



        double simulationTime;
        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            character.UpdateCharacterGoals(input, camera, TimestepDuration);
            Simulation.Timestep(TimestepDuration, ThreadDispatcher);
            character.UpdateCameraPosition(camera, -0.3f);
            for (int i = 0; i < newts.Count; ++i)
            {
                newts[i].Update(Simulation, simulationTime, 0, newtArenaMin, newtArenaMax, random, 1f / TimestepDuration);
            }
            for (int i = 0; i < characterAIs.Count; ++i)
            {
                characterAIs[i].Update(characterControllers, Simulation, ref newts, newtArenaMin, newtArenaMax, random);
            }
            simulationTime += TimestepDuration;


            if(input.WasPushed(OpenTK.Input.Key.P))
            {
                Console.WriteLine($"camera position: {camera.Position}");
            }
        }

    }
}
