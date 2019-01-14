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
using DemoRenderer.UI;
using OpenTK.Input;

namespace Demos.Demos
{
    /// <summary>
    /// Convenience structure that wraps a CharacterController reference and its associated body.
    /// </summary>
    /// <remarks>
    /// This should be treated as an example- nothing here is intended to suggest how you *must* handle characters. 
    /// On the contrary, this does some fairly inefficient stuff if you're dealing with hundreds of characters in a predictable way.
    /// It's just a fairly convenient interface for demos usage.
    /// </remarks>
    public struct CharacterInput
    {
        int bodyHandle;
        CharacterControllers characters;
        float speed;
        Capsule shape;

        public CharacterInput(CharacterControllers characters, Vector3 initialPosition, Capsule shape,
            float speculativeMargin, float mass, float maximumHorizontalForce, float maximumVerticalGlueForce,
            float jumpVelocity, float speed, float maximumSlope = MathF.PI * 0.25f)
        {
            this.characters = characters;
            var shapeIndex = characters.Simulation.Shapes.Add(shape);

            bodyHandle = characters.Simulation.Bodies.Add(BodyDescription.CreateDynamic(initialPosition, new BodyInertia { InverseMass = 1f / mass }, new CollidableDescription(shapeIndex, speculativeMargin), new BodyActivityDescription(shape.Radius * 0.02f)));
            ref var character = ref characters.AllocateCharacter(bodyHandle, out var characterIndex);
            character.LocalUp = new Vector3(0, 1, 0);
            character.CosMaximumSlope = MathF.Cos(maximumSlope);
            character.JumpVelocity = jumpVelocity;
            character.MaximumVerticalForce = maximumVerticalGlueForce;
            character.MaximumHorizontalForce = maximumHorizontalForce;
            character.MinimumSupportDepth = shape.Radius * -0.01f;
            character.MinimumSupportContinuationDepth = -speculativeMargin;
            this.speed = speed;
            this.shape = shape;
        }

        static Key MoveForward = Key.W;
        static Key MoveBackward = Key.S;
        static Key MoveRight = Key.D;
        static Key MoveLeft = Key.A;
        static Key Sprint = Key.LShift;
        static Key Jump = Key.Space;
        static Key JumpAlternate = Key.BackSpace; //I have a weird keyboard.

        public void UpdateCharacterGoals(Input input, Camera camera)
        {
            Vector2 movementDirection = default;
            if (input.IsDown(MoveForward))
            {
                movementDirection = new Vector2(0, 1);
            }
            if (input.IsDown(MoveBackward))
            {
                movementDirection += new Vector2(0, -1);
            }
            if (input.IsDown(MoveLeft))
            {
                movementDirection += new Vector2(-1, 0);
            }
            if (input.IsDown(MoveRight))
            {
                movementDirection += new Vector2(1, 0);
            }
            var lengthSquared = movementDirection.LengthSquared();
            if (lengthSquared > 0)
            {
                movementDirection /= MathF.Sqrt(lengthSquared);
            }

            ref var character = ref characters.GetCharacterByBodyHandle(bodyHandle);
            character.TryJump = input.WasPushed(Jump) || input.WasPushed(JumpAlternate);
            var characterBody = new BodyReference(bodyHandle, characters.Simulation.Bodies);
            var newTargetVelocity = movementDirection * (input.IsDown(Sprint) ? speed * 1.75f : speed);
            var viewDirection = camera.Forward;
            //Modifying the character's raw data does not automatically wake the character up, so we do so explicitly if necessary.
            //If you don't explicitly wake the character up, it won't respond to the changed motion goals.
            //(You can also specify a negative deactivation threshold in the BodyActivityDescription to prevent the character from sleeping at all.)
            if (!characterBody.IsActive &&
                ((character.TryJump && character.Supported) ||
                newTargetVelocity != character.TargetVelocity ||
                (newTargetVelocity != Vector2.Zero && character.ViewDirection != viewDirection)))
            {
                characters.Simulation.Awakener.AwakenBody(character.BodyHandle);
            }
            character.TargetVelocity = newTargetVelocity;
            character.ViewDirection = viewDirection;
        }

        public void UpdateCameraPosition(Camera camera)
        {
            //We'll override the demo harness's camera control by attaching the camera to the character controller body.
            ref var character = ref characters.GetCharacterByBodyHandle(bodyHandle);
            var characterBody = new BodyReference(bodyHandle, characters.Simulation.Bodies);
            //Use a simple sorta-neck model so that when the camera looks down, the center of the screen sees past the character.
            //Makes mouselocked ray picking easier.
            camera.Position = characterBody.Pose.Position + new Vector3(0, shape.HalfLength, 0) +
                camera.Up * (shape.Radius * 1.2f) -
                camera.Forward * (shape.HalfLength + shape.Radius) * 4;
        }

        void RenderControl(ref Vector2 position, float textHeight, string controlName, string controlValue, TextBuilder text, TextBatcher textBatcher, Font font)
        {
            text.Clear().Append(controlName).Append(": ").Append(controlValue);
            textBatcher.Write(text, position, textHeight, new Vector3(1), font);
            position.Y += textHeight * 1.1f;
        }
        public void RenderControls(Vector2 position, float textHeight, TextBatcher textBatcher, TextBuilder text, Font font)
        {
            RenderControl(ref position, textHeight, nameof(MoveForward), MoveForward.ToString(), text, textBatcher, font);
            RenderControl(ref position, textHeight, nameof(MoveBackward), MoveBackward.ToString(), text, textBatcher, font);
            RenderControl(ref position, textHeight, nameof(MoveRight), MoveRight.ToString(), text, textBatcher, font);
            RenderControl(ref position, textHeight, nameof(MoveLeft), MoveLeft.ToString(), text, textBatcher, font);
            RenderControl(ref position, textHeight, nameof(Sprint), Sprint.ToString(), text, textBatcher, font);
            RenderControl(ref position, textHeight, nameof(Jump), Jump.ToString(), text, textBatcher, font);
        }


        /// <summary>
        /// Removes the character's body from the simulation and the character from the associated characters set.
        /// </summary>
        public void Dispose()
        {
            characters.Simulation.Shapes.Remove(new BodyReference(bodyHandle, characters.Simulation.Bodies).Collidable.Shape);
            characters.Simulation.Bodies.Remove(bodyHandle);
            characters.RemoveCharacterByBodyHandle(bodyHandle);
        }
    }

    /// <summary>
    /// Implements simple callbacks to inform the CharacterControllers system of created contacts.
    /// </summary>
    struct CharacterNarrowphaseCallbacks : INarrowPhaseCallbacks
    {
        public CharacterControllers Characters;

        public CharacterNarrowphaseCallbacks(CharacterControllers characters)
        {
            Characters = characters;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b)
        {
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
        {
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void GetMaterial(out PairMaterialProperties pairMaterial)
        {
            pairMaterial = new PairMaterialProperties { FrictionCoefficient = 1, MaximumRecoveryVelocity = 2, SpringSettings = new SpringSettings(30, 1) };
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, ConvexContactManifold* manifold, out PairMaterialProperties pairMaterial)
        {
            GetMaterial(out pairMaterial);
            Characters.TryReportContacts(pair, ref *manifold, workerIndex, ref pairMaterial);
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, NonconvexContactManifold* manifold, out PairMaterialProperties pairMaterial)
        {
            GetMaterial(out pairMaterial);
            Characters.TryReportContacts(pair, ref *manifold, workerIndex, ref pairMaterial);
            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ConvexContactManifold* manifold)
        {
            return true;
        }

        public void Dispose()
        {
            Characters.Dispose();
        }

        public void Initialize(Simulation simulation)
        {
            Characters.Initialize(simulation);
        }
    }

    public class CharacterDemo : Demo
    {
        CharacterControllers characters;
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(20, 10, 20);
            camera.Yaw = MathF.PI;
            camera.Pitch = 0;
            var masks = new BodyProperty<ulong>();
            characters = new CharacterControllers(BufferPool);
            Simulation = Simulation.Create(BufferPool, new CharacterNarrowphaseCallbacks(characters), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));

            CreateCharacter(new Vector3(0, 2, -4));

            //Create a bunch of legos to hurt your feet on.
            var random = new Random(5);
            var origin = new Vector3(-3f, 0.5f, 0);
            var spacing = new Vector3(0.5f, 0, -0.5f);
            for (int i = 0; i < 12; ++i)
            {
                for (int j = 0; j < 12; ++j)
                {
                    var position = origin + new Vector3(i, 0, j) * spacing;
                    var orientation = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(0.0001f) + new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble())), 10 * (float)random.NextDouble());
                    var shape = new Box(0.1f + 0.3f * (float)random.NextDouble(), 0.1f + 0.3f * (float)random.NextDouble(), 0.1f + 0.3f * (float)random.NextDouble());
                    var collidable = new CollidableDescription(Simulation.Shapes.Add(shape), 0.1f);
                    shape.ComputeInertia(1, out var inertia);
                    var choice = (i + j) % 3;
                    switch (choice)
                    {
                        case 0:
                            Simulation.Bodies.Add(BodyDescription.CreateDynamic(new RigidPose(position, orientation), inertia, collidable, new BodyActivityDescription(0.01f)));
                            break;
                        case 1:
                            Simulation.Bodies.Add(BodyDescription.CreateKinematic(new RigidPose(position, orientation), collidable, new BodyActivityDescription(0.01f)));
                            break;
                        case 2:
                            Simulation.Statics.Add(new StaticDescription(position, orientation, collidable));
                            break;

                    }
                }
            }

            //Add some spinning fans to get slapped by.
            var bladeDescription = BodyDescription.CreateConvexDynamic(new Vector3(), 3, Simulation.Shapes, new Box(10, 0.2f, 2));
            var bladeBaseDescription = BodyDescription.CreateConvexKinematic(new Vector3(), Simulation.Shapes, new Box(0.2f, 1, 0.2f));
            for (int i = 0; i < 3; ++i)
            {
                bladeBaseDescription.Pose.Position = new Vector3(-22, 1, i * 11);
                bladeDescription.Pose.Position = new Vector3(-22, 1.7f, i * 11);
                var baseHandle = Simulation.Bodies.Add(bladeBaseDescription);
                var bladeHandle = Simulation.Bodies.Add(bladeDescription);
                Simulation.Solver.Add(baseHandle, bladeHandle,
                    new Hinge
                    {
                        LocalHingeAxisA = Vector3.UnitY,
                        LocalHingeAxisB = Vector3.UnitY,
                        LocalOffsetA = new Vector3(0, 0.7f, 0),
                        LocalOffsetB = new Vector3(0, 0, 0),
                        SpringSettings = new SpringSettings(30, 1)
                    });
                Simulation.Solver.Add(baseHandle, bladeHandle,
                    new AngularAxisMotor
                    {
                        LocalAxisA = Vector3.UnitY,
                        TargetVelocity = (i + 1) * (i + 1) * (i + 1) * (i + 1) * 0.2f,
                        Settings = new MotorSettings(5 * (i + 1), 0.0001f)
                    });
            }

            //Include a giant newt to test character-newt behavior and to ensure thematic consistency.
            MeshDemo.LoadModel(content, BufferPool, @"Content\newt.obj", new Vector3(15, 15, 15), out var newtMesh);
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, 0.5f, 0), new CollidableDescription(Simulation.Shapes.Add(newtMesh), 0.1f)));

            //Give the newt a tongue, I guess.
            var tongueBase = Simulation.Bodies.Add(BodyDescription.CreateKinematic(new Vector3(0, 8.4f, 24), default, default));
            var tongue = Simulation.Bodies.Add(BodyDescription.CreateConvexDynamic(new Vector3(0, 8.4f, 27.5f), 1, Simulation.Shapes, new Box(1, 0.1f, 6f)));
            Simulation.Solver.Add(tongueBase, tongue, new Hinge
            {
                LocalHingeAxisA = Vector3.UnitX,
                LocalHingeAxisB = Vector3.UnitX,
                LocalOffsetB = new Vector3(0, 0, -3f),
                SpringSettings = new SpringSettings(30, 1)
            });
            Simulation.Solver.Add(tongueBase, tongue, new AngularServo
            {
                TargetRelativeRotationLocalA = Quaternion.Identity,
                ServoSettings = ServoSettings.Default,
                SpringSettings = new SpringSettings(2, 0)
            });

            //And a seesaw thing?
            var seesawBase = Simulation.Bodies.Add(BodyDescription.CreateKinematic(new Vector3(0, 1f, 34f), new CollidableDescription(Simulation.Shapes.Add(new Box(0.2f, 1, 0.2f)), 0.1f), new BodyActivityDescription(0.01f)));
            var seesaw = Simulation.Bodies.Add(BodyDescription.CreateConvexDynamic(new Vector3(0, 1.7f, 34f), 1, Simulation.Shapes, new Box(1, 0.1f, 6f)));
            Simulation.Solver.Add(seesawBase, seesaw, new Hinge
            {
                LocalHingeAxisA = Vector3.UnitX,
                LocalHingeAxisB = Vector3.UnitX,
                LocalOffsetA = new Vector3(0, 0.7f, 0),
                LocalOffsetB = new Vector3(0, 0, 0),
                SpringSettings = new SpringSettings(30, 1)
            });

            Simulation.Bodies.Add(BodyDescription.CreateConvexDynamic(new Vector3(0, 2.25f, 35.5f), 0.5f, Simulation.Shapes, new Box(1f, 1f, 1f)));

            //Prevent the character from falling into the void.
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, 0, 0), new CollidableDescription(Simulation.Shapes.Add(new Box(200, 1, 200)), 0.1f)));
        }

        bool characterActive;
        CharacterInput character;
        void CreateCharacter(Vector3 position)
        {
            characterActive = true;
            character = new CharacterInput(characters, position, new Capsule(0.5f, 1), 0.1f, 1, 20, 100, 6, 4, MathF.PI * 0.4f);
        }

        public override void Update(Window window, Camera camera, Input input, float dt)
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
                character.UpdateCharacterGoals(input, camera);
            }
            base.Update(window, camera, input, dt);
        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            float textHeight = 16;
            var position = new Vector2(32, renderer.Surface.Resolution.Y - textHeight * 9);
            renderer.TextBatcher.Write(text.Clear().Append("Toggle character: C"), position, textHeight, new Vector3(1), font);
            position.Y += textHeight * 1.2f;
            character.RenderControls(position, textHeight, renderer.TextBatcher, text, font);
            if (characterActive)
            {
                character.UpdateCameraPosition(camera);
            }
            base.Render(renderer, camera, input, text, font);
        }
    }
}


