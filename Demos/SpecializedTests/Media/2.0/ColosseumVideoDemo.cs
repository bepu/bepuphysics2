﻿using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using Demos.Demos.Characters;
using DemoUtilities;
using OpenTK.Input;
using System;
using System.Numerics;

namespace Demos.SpecializedTests.Media
{
    /// <summary>
    /// Version of the colosseum demo for video purposes.
    /// </summary>
    public class ColosseumVideoDemo : Demo
    {
        void CreateRingWall(Vector3 position, Box ringBoxShape, BodyDescription bodyDescription, int height, float radius)
        {
            var circumference = MathF.PI * 2 * radius;
            var boxCountPerRing = (int)(0.9f * circumference / ringBoxShape.Length);
            float increment = MathHelper.TwoPi / boxCountPerRing;
            for (int ringIndex = 0; ringIndex < height; ringIndex++)
            {
                for (int i = 0; i < boxCountPerRing; i++)
                {
                    var angle = ((ringIndex & 1) == 0 ? i + 0.5f : i) * increment;
                    bodyDescription.Pose = (position + new Vector3(-MathF.Cos(angle) * radius, (ringIndex + 0.5f) * ringBoxShape.Height, MathF.Sin(angle) * radius), QuaternionEx.CreateFromAxisAngle(Vector3.UnitY, angle));
                    Simulation.Bodies.Add(bodyDescription);
                }
            }
        }

        void CreateRingPlatform(Vector3 position, Box ringBoxShape, BodyDescription bodyDescription, float radius)
        {
            var innerCircumference = MathF.PI * 2 * (radius - ringBoxShape.HalfLength);
            var boxCount = (int)(0.95f * innerCircumference / ringBoxShape.Height);
            float increment = MathHelper.TwoPi / boxCount;
            for (int i = 0; i < boxCount; i++)
            {
                var angle = i * increment;
                bodyDescription.Pose = (position + new Vector3(-MathF.Cos(angle) * radius, ringBoxShape.HalfWidth, MathF.Sin(angle) * radius),
                    QuaternionEx.Concatenate(QuaternionEx.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI * 0.5f), QuaternionEx.CreateFromAxisAngle(Vector3.UnitY, angle + MathF.PI * 0.5f)));
                Simulation.Bodies.Add(bodyDescription);
            }
        }

        Vector3 CreateRing(Vector3 position, Box ringBoxShape, BodyDescription bodyDescription, float radius, int heightPerPlatformLevel, int platformLevels)
        {
            for (int platformIndex = 0; platformIndex < platformLevels; ++platformIndex)
            {
                var wallOffset = ringBoxShape.HalfLength - ringBoxShape.HalfWidth;
                CreateRingWall(position, ringBoxShape, bodyDescription, heightPerPlatformLevel, radius + wallOffset);
                CreateRingWall(position, ringBoxShape, bodyDescription, heightPerPlatformLevel, radius - wallOffset);
                CreateRingPlatform(position + new Vector3(0, heightPerPlatformLevel * ringBoxShape.Height, 0), ringBoxShape, bodyDescription, radius);
                position.Y += heightPerPlatformLevel * ringBoxShape.Height + ringBoxShape.Width;
            }
            return position;
        }

        CharacterControllers characters;
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-30, 40, -30);
            camera.Yaw = MathHelper.Pi * 3f / 4;
            camera.Pitch = MathHelper.Pi * 0.2f;

            characters = new CharacterControllers(BufferPool);
            Simulation = Simulation.Create(BufferPool, new CharacterNarrowphaseCallbacks(characters), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));

            var ringBoxShape = new Box(0.5f, 1.5f, 3);
            var boxDescription = BodyDescription.CreateDynamic(new Vector3(), ringBoxShape.ComputeInertia(1), Simulation.Shapes.Add(ringBoxShape), 0.01f);

            var layerPosition = new Vector3();
            const int layerCount = 10;
            var innerRadius = 5f;
            var heightPerPlatform = 2;
            var platformsPerLayer = 1;
            var ringSpacing = 0.5f;
            for (int layerIndex = 0; layerIndex < layerCount; ++layerIndex)
            {
                var ringCount = layerCount - layerIndex;
                for (int ringIndex = 0; ringIndex < ringCount; ++ringIndex)
                {
                    CreateRing(layerPosition, ringBoxShape, boxDescription, innerRadius + ringIndex * (ringBoxShape.Length + ringSpacing) + layerIndex * (ringBoxShape.Length - ringBoxShape.Width), heightPerPlatform, platformsPerLayer);
                }
                layerPosition.Y += platformsPerLayer * (ringBoxShape.Height * heightPerPlatform + ringBoxShape.Width);
            }

            Console.WriteLine($"box count: {Simulation.Bodies.ActiveSet.Count}");
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -0.5f, 0), Simulation.Shapes.Add(new Box(500, 1, 500))));

            var bulletShape = new Sphere(0.5f);
            bulletDescription = BodyDescription.CreateDynamic(new Vector3(), bulletShape.ComputeInertia(.1f), Simulation.Shapes.Add(bulletShape), 0.01f);

            var shootiePatootieShape = new Sphere(3f);
            shootiePatootieDescription = BodyDescription.CreateDynamic(new Vector3(), shootiePatootieShape.ComputeInertia(1000), new (Simulation.Shapes.Add(shootiePatootieShape), 0.1f), 0.01f);
        }

        bool characterActive;
        CharacterInput character;
        void CreateCharacter(Vector3 position)
        {
            characterActive = true;
            character = new CharacterInput(characters, position, new Capsule(0.5f, 1), 0.1f, 1, 20, 100, 6, 4, MathF.PI * 0.4f);
        }


        BodyDescription bulletDescription;
        BodyDescription shootiePatootieDescription;
        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            if (input != null)
            {
                if (input.WasPushed(Key.C))
                {
                    if (characterActive)
                    {
                        character.Dispose();
                        characterActive = false;
                    }
                    else
                    {
                        CreateCharacter(camera.Position);
                    }
                }
                if (characterActive)
                {
                    character.UpdateCharacterGoals(input, camera, Demo.TimestepDuration);
                }

                if (input.WasPushed(Key.Z))
                {
                    bulletDescription.Pose.Position = camera.Position;
                    bulletDescription.Velocity.Linear = camera.GetRayDirection(input.MouseLocked, window.GetNormalizedMousePosition(input.MousePosition)) * 400;
                    Simulation.Bodies.Add(bulletDescription);
                }
                else if (input.WasPushed(Key.X))
                {
                    shootiePatootieDescription.Pose.Position = camera.Position;
                    shootiePatootieDescription.Velocity.Linear = camera.GetRayDirection(input.MouseLocked, window.GetNormalizedMousePosition(input.MousePosition)) * 100;
                    Simulation.Bodies.Add(shootiePatootieDescription);
                }
            }
            base.Update(window, camera, input, dt);
        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            if (characterActive)
            {
                character.UpdateCameraPosition(camera, 0);
            }
            //text.Clear().Append("Press Z to shoot a bullet, press X to super shootie patootie!");
            //renderer.TextBatcher.Write(text, new Vector2(20, renderer.Surface.Resolution.Y - 20), 16, new Vector3(1, 1, 1), font);
            base.Render(renderer, camera, input, text, font);
        }
    }
}
