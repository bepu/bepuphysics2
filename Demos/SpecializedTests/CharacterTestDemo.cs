using BepuUtilities;
using DemoRenderer;
using BepuPhysics;
using BepuPhysics.Collidables;
using System.Numerics;
using Quaternion = BepuUtilities.Quaternion;
using System;
using BepuPhysics.CollisionDetection;
using System.Runtime.CompilerServices;
using System.Diagnostics;
using BepuPhysics.Constraints;
using DemoContentLoader;
using DemoUtilities;
using BepuUtilities.Memory;
using static BepuUtilities.GatherScatter;
using Demos.Demos.Characters;
using BepuUtilities.Collections;
using Demos.Demos;

namespace Demos.SpecializedTests
{
    public class CharacterTestDemo : Demo
    {
        CharacterControllers characters;
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(20, 10, 20);
            camera.Yaw = MathHelper.Pi * -1f / 4;
            camera.Pitch = MathHelper.Pi * 0.05f;
            var masks = new BodyProperty<ulong>();
            characters = new CharacterControllers(BufferPool);
            Simulation = Simulation.Create(BufferPool, new CharacterNarrowphaseCallbacks(characters), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));

            var random = new Random(5);
            for (int i = 0; i < 8192; ++i)
            {
                ref var character = ref characters.AllocateCharacter(
                    Simulation.Bodies.Add(
                        BodyDescription.CreateDynamic(
                            new Vector3(250 * (float)random.NextDouble() - 125, 2, 250 * (float)random.NextDouble() - 125), new BodyInertia { InverseMass = 1 },
                            new CollidableDescription(Simulation.Shapes.Add(new Capsule(0.5f, 1f)), 0.1f),
                            new BodyActivityDescription(-1))));

                character.CosMaximumSlope = .707f;
                character.LocalUp = Vector3.UnitY;
                character.MaximumHorizontalForce = 10;
                character.MaximumVerticalForce = 10;
                character.MinimumSupportContinuationDepth = -0.1f;
                character.MinimumSupportDepth = -0.01f;
                character.TargetVelocity = new Vector2(4, 0);
                character.ViewDirection = new Vector3(0, 0, -1);
                character.JumpVelocity = 4;
            }

            var origin = new Vector3(-3f, 0, 0);
            var spacing = new Vector3(0.5f, 0, -0.5f);
            //for (int i = 0; i < 12; ++i)
            //{
            //    for (int j = 0; j < 100; ++j)
            //    {
            //        var position = origin + new Vector3(i, 0, j) * spacing;
            //        var orientation = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(0.0001f) + new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble())), 10 * (float)random.NextDouble());
            //        var shape = new Box(0.1f + 0.3f * (float)random.NextDouble(), 0.1f + 0.3f * (float)random.NextDouble(), 0.1f + 0.3f * (float)random.NextDouble());
            //        var collidable = new CollidableDescription(Simulation.Shapes.Add(shape), 0.1f);
            //        shape.ComputeInertia(1, out var inertia);
            //        var choice = (i + j) % 3;
            //        switch (choice)
            //        {
            //            case 0:
            //                Simulation.Bodies.Add(BodyDescription.CreateDynamic(new RigidPose(position, orientation), inertia, collidable, new BodyActivityDescription(0.01f)));
            //                break;
            //            case 1:
            //                Simulation.Bodies.Add(BodyDescription.CreateKinematic(new RigidPose(position, orientation), collidable, new BodyActivityDescription(0.01f)));
            //                break;
            //            case 2:
            //                Simulation.Statics.Add(new StaticDescription(position, orientation, collidable));
            //                break;

            //        }
            //    }
            //}


            //Simulation.Statics.Add(new StaticDescription(
            //    new Vector3(0, -0.5f, 0), Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1, 0, 1)), MathF.PI * 0.00f), new CollidableDescription(Simulation.Shapes.Add(new Box(3000, 1, 3000)), 0.1f)));

            const int planeWidth = 256;
            const int planeHeight = 256;
            DemoMeshHelper.CreateDeformedPlane(planeWidth, planeHeight,
                (int x, int y) =>
                {
                    Vector2 offsetFromCenter = new Vector2(x - planeWidth / 2, y - planeHeight / 2);
                    return new Vector3(offsetFromCenter.X, MathF.Cos(x / 2f) + MathF.Sin(y / 2f), offsetFromCenter.Y);
                }, new Vector3(2, 1, 2), BufferPool, out var planeMesh);
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -2, 0), Quaternion.CreateFromAxisAngle(new Vector3(0, 1, 0), MathF.PI / 2),
                new CollidableDescription(Simulation.Shapes.Add(planeMesh), 0.1f)));

            removedCharacters = new QuickQueue<CharacterController>(characters.CharacterCount, BufferPool);
        }

        QuickQueue<CharacterController> removedCharacters;
        int frameIndex;
        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            var rotation = Matrix3x3.CreateFromAxisAngle(new Vector3(0, 1, 0), 0.5f * dt);
            for (int i = 0; i < characters.CharacterCount; ++i)
            {
                ref var character = ref characters.GetCharacterByIndex(i);
                if (frameIndex % 128 == 0)
                    character.TryJump = true;
                var tangent = Vector3.Cross(new BodyReference(character.BodyHandle, Simulation.Bodies).Pose.Position, Vector3.UnitY);
                var tangentLengthSquared = tangent.LengthSquared();
                if (tangentLengthSquared > 1e-12f)
                    tangent = tangent / MathF.Sqrt(tangentLengthSquared);
                else
                    tangent = Vector3.UnitX;
                tangent *= 4;
                character.TargetVelocity.X = -tangent.X;
                character.TargetVelocity.Y = tangent.Z;
                //Matrix3x3.Transform(new Vector3(character.TargetVelocity.X, 0, character.TargetVelocity.Y), rotation, out var rotatedVelocity);
                //character.TargetVelocity.X = rotatedVelocity.X;
                //character.TargetVelocity.Y = rotatedVelocity.Z;          
            }
            //{
            //    if (characters.CharacterCount > 0)
            //    {
            //        var indexToRemove = frameIndex % characters.CharacterCount;
            //        removedCharacters.EnqueueUnsafely(characters.GetCharacterByIndex(indexToRemove));
            //        characters.RemoveCharacterByIndex(indexToRemove);
            //    }

            //    var readdCount = (int)(removedCharacters.Count * 0.05f);
            //    for (int i = 0; i < readdCount; ++i)
            //    {
            //        var toAdd = removedCharacters.Dequeue();
            //        ref var character = ref characters.AllocateCharacter(toAdd.BodyHandle, out var characterIndex);
            //        character = toAdd;
            //    }

            //}
            frameIndex++;
            base.Update(window, camera, input, dt);
        }
    }
}


