using BepuUtilities;
using DemoRenderer;
using BepuPhysics;
using BepuPhysics.Collidables;
using System.Numerics;
using System;
using BepuPhysics.Constraints;
using DemoContentLoader;
using DemoUtilities;
using DemoRenderer.UI;
using OpenTK.Input;

namespace Demos.Demos.Characters
{
    /// <summary>
    /// Shows one way of using the dynamic character controller in the context of a giant newt and levitating pads.
    /// </summary>
    public class CharacterDemo : Demo
    {
        CharacterControllers characters;
        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(20, 10, 20);
            camera.Yaw = MathF.PI;
            camera.Pitch = 0;
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
                    var orientation = QuaternionEx.CreateFromAxisAngle(Vector3.Normalize(new Vector3(0.0001f) + new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble())), 10 * (float)random.NextDouble());
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
            DemoMeshHelper.LoadModel(content, BufferPool, @"Content\newt.obj", new Vector3(15, 15, 15), out var newtMesh);
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

            //Create some moving platforms to jump on.
            movingPlatforms = new MovingPlatform[16];
            Func<double, RigidPose> poseCreator = time =>
            {
                RigidPose pose;
                var horizontalScale = (float)(45 + 10 * Math.Sin(time * 0.015));
                //Float in a noisy ellipse around the newt.
                pose.Position = new Vector3(0.7f * horizontalScale * (float)Math.Sin(time * 0.1), 10 + 4 * (float)Math.Sin((time + Math.PI * 0.5f) * 0.25), horizontalScale * (float)Math.Cos(time * 0.1));
                //As the platform goes behind the newt, dip toward the ground. Use smoothstep for a less jerky ride.
                var x = MathF.Max(0f, MathF.Min(1f, 1f - (pose.Position.Z + 20f) / -20f));
                var smoothStepped = 3 * x * x - 2 * x * x * x;
                pose.Position.Y = smoothStepped * (pose.Position.Y - 0.025f) + 0.025f;
                pose.Orientation = Quaternion.Identity;
                return pose;
            };
            var platformCollidable = new CollidableDescription(Simulation.Shapes.Add(new Box(5, 1, 5)), 0.1f);
            for (int i = 0; i < movingPlatforms.Length; ++i)
            {
                movingPlatforms[i] = new MovingPlatform(platformCollidable, i * 3559, 1f / 60f, Simulation, poseCreator);
            }

            //Prevent the character from falling into the void.
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, 0, 0), new CollidableDescription(Simulation.Shapes.Add(new Box(200, 1, 200)), 0.1f)));
        }


        struct MovingPlatform
        {
            public int BodyHandle;
            public float InverseGoalSatisfactionTime;
            public double TimeOffset;
            public Func<double, RigidPose> PoseCreator;

            public MovingPlatform(CollidableDescription collidable, double timeOffset, float goalSatisfactionTime, Simulation simulation, Func<double, RigidPose> poseCreator)
            {
                PoseCreator = poseCreator;
                BodyHandle = simulation.Bodies.Add(BodyDescription.CreateKinematic(poseCreator(timeOffset), collidable, new BodyActivityDescription(-1)));
                InverseGoalSatisfactionTime = 1f / goalSatisfactionTime;
                TimeOffset = timeOffset;
            }

            public void Update(Simulation simulation, double time)
            {
                var body = new BodyReference(BodyHandle, simulation.Bodies);
                ref var pose = ref body.Pose;
                ref var velocity = ref body.Velocity;
                var targetPose = PoseCreator(time + TimeOffset);
                velocity.Linear = (targetPose.Position - pose.Position) * InverseGoalSatisfactionTime;
                QuaternionEx.GetRelativeRotationWithoutOverlap(pose.Orientation, targetPose.Orientation, out var rotation);
                QuaternionEx.GetAxisAngleFromQuaternion(rotation, out var axis, out var angle);
                velocity.Angular = axis * (angle * InverseGoalSatisfactionTime);
            }
        }
        MovingPlatform[] movingPlatforms;

        bool characterActive;
        CharacterInput character;
        double time;
        void CreateCharacter(Vector3 position)
        {
            characterActive = true;
            character = new CharacterInput(characters, position, new Capsule(0.5f, 1), 0.1f, 1, 20, 100, 6, 4, MathF.PI * 0.4f);
        }

        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            const float simulationDt = 1 / 60f;
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
                character.UpdateCharacterGoals(input, camera, simulationDt);
            }
            //Using a fixed time per update to match the demos simulation update rate.
            time += 1 / 60f;
            for (int i = 0; i < movingPlatforms.Length; ++i)
            {
                movingPlatforms[i].Update(Simulation, time);
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


