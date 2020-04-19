using System;
using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using OpenTK.Input;

namespace Demos.Demos.Cars
{
    public class CarDemo : Demo
    {
        SimpleCarController playerController;

        struct AIController
        {
            public SimpleCarController Controller;
            public float LaneOffset;
        }

        Buffer<AIController> aiControllers;
        RaceTrack raceTrack;

        static Key Forward = Key.W;
        static Key Backward = Key.S;
        static Key Right = Key.D;
        static Key Left = Key.A;
        static Key Zoom = Key.LShift;
        static Key Brake = Key.Space;
        static Key BrakeAlternate = Key.BackSpace; //I have a weird keyboard.
        static Key ToggleCar = Key.C;
        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 5, 10);
            camera.Yaw = 0;
            camera.Pitch = 0;

            var properties = new BodyProperty<CarBodyProperties>();
            Simulation = Simulation.Create(BufferPool, new CarCallbacks() { Properties = properties }, new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));

            var builder = new CompoundBuilder(BufferPool, Simulation.Shapes, 2);
            builder.Add(new Box(1.85f, 0.7f, 4.73f), RigidPose.Identity, 10);
            builder.Add(new Box(1.85f, 0.6f, 2.5f), new RigidPose(new Vector3(0, 0.65f, -0.35f)), 0.5f);
            builder.BuildDynamicCompound(out var children, out var bodyInertia, out _);
            builder.Dispose();
            var bodyShape = new Compound(children);
            var bodyShapeIndex = Simulation.Shapes.Add(bodyShape);
            var wheelShape = new Cylinder(0.4f, .18f);
            wheelShape.ComputeInertia(0.25f, out var wheelInertia);
            var wheelShapeIndex = Simulation.Shapes.Add(wheelShape);

            const float x = 0.9f;
            const float y = -0.1f;
            const float frontZ = 1.7f;
            const float backZ = -1.7f;
            playerController = new SimpleCarController(SimpleCar.Create(Simulation, properties, new RigidPose(new Vector3(0, 10, 0), Quaternion.Identity), bodyShapeIndex, bodyInertia, 0.5f, wheelShapeIndex, wheelInertia, 2f,
                new Vector3(-x, y, frontZ), new Vector3(x, y, frontZ), new Vector3(-x, y, backZ), new Vector3(x, y, backZ), new Vector3(0, -1, 0), 0.25f,
                new SpringSettings(5f, 0.7f), QuaternionEx.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI * 0.5f)),
                forwardSpeed: 75, forwardForce: 6, zoomMultiplier: 2, backwardSpeed: 30, backwardForce: 4, idleForce: 0.25f, brakeForce: 7, steeringSpeed: 1.5f, maximumSteeringAngle: MathF.PI * 0.23f);

            //Create a bunch of AI cars to race against.
            const int aiCount = 384;
            BufferPool.Take(aiCount, out aiControllers);


            const int planeWidth = 257;
            const float scale = 3;
            Vector2 terrainPosition = new Vector2(1 - planeWidth, 1 - planeWidth) * scale * 0.5f;
            raceTrack = new RaceTrack { QuadrantRadius = (planeWidth - 32) * scale * 0.25f, Center = default };
            var random = new Random(5);

            //Add some building-ish landmarks in the middle of each of the four racetrack quadrants.
            for (int i = 0; i < 4; ++i)
            {
                var landmarkCenter = new Vector3((i & 1) * raceTrack.QuadrantRadius * 2 - raceTrack.QuadrantRadius, -20, (i & 2) * raceTrack.QuadrantRadius - raceTrack.QuadrantRadius);
                var landmarkMin = landmarkCenter - new Vector3(raceTrack.QuadrantRadius * 0.5f, 0, raceTrack.QuadrantRadius * 0.5f);
                var landmarkSpan = new Vector3(raceTrack.QuadrantRadius, 0, raceTrack.QuadrantRadius);
                for (int j = 0; j < 25; ++j)
                {
                    var buildingShape = new Box(10 + (float)random.NextDouble() * 10, 20 + (float)random.NextDouble() * 20, 10 + (float)random.NextDouble() * 10);
                    Simulation.Statics.Add(new StaticDescription(
                        new Vector3(0, buildingShape.HalfHeight, 0) + landmarkMin + landmarkSpan * new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()),
                        QuaternionEx.CreateFromAxisAngle(Vector3.UnitY, (float)random.NextDouble() * MathF.PI),
                        new CollidableDescription(Simulation.Shapes.Add(buildingShape), 0.1f)));
                }
            }

            Vector3 min = new Vector3(-planeWidth * scale * 0.45f, 10, -planeWidth * scale * 0.45f);
            Vector3 span = new Vector3(planeWidth * scale * 0.9f, 15, planeWidth * scale * 0.9f);

            for (int i = 0; i < aiCount; ++i)
            {
                //The AI cars are very similar, except... we handicap them a little to make the player good about themselves.
                var position = min + span * new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble());
                var orientation = QuaternionEx.CreateFromAxisAngle(new Vector3(0, 1, 0), (float)random.NextDouble() * MathF.PI * 2);
                aiControllers[i].Controller = new SimpleCarController(SimpleCar.Create(Simulation, properties, new RigidPose(position, orientation), bodyShapeIndex, bodyInertia, 0.5f, wheelShapeIndex, wheelInertia, 2f,
                    new Vector3(-x, y, frontZ), new Vector3(x, y, frontZ), new Vector3(-x, y, backZ), new Vector3(x, y, backZ), new Vector3(0, -1, 0), 0.25f,
                    new SpringSettings(5, 0.7f), QuaternionEx.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI * 0.5f)),
                    forwardSpeed: 50, forwardForce: 5, zoomMultiplier: 2, backwardSpeed: 10, backwardForce: 4, idleForce: 0.25f, brakeForce: 7, steeringSpeed: 1.5f, maximumSteeringAngle: MathF.PI * 0.23f);
                aiControllers[i].LaneOffset = (float)random.NextDouble() * 20 - 10;
            }


            DemoMeshHelper.CreateDeformedPlane(planeWidth, planeWidth,
                (int vX, int vY) =>
                {
                    var octave0 = (MathF.Sin((vX + 5f) * 0.05f) + MathF.Sin((vY + 11) * 0.05f)) * 1.8f;
                    var octave1 = (MathF.Sin((vX + 17) * 0.15f) + MathF.Sin((vY + 19) * 0.15f)) * 0.9f;
                    var octave2 = (MathF.Sin((vX + 37) * 0.35f) + MathF.Sin((vY + 93) * 0.35f)) * 0.4f;
                    var octave3 = (MathF.Sin((vX + 53) * 0.65f) + MathF.Sin((vY + 47) * 0.65f)) * 0.2f;
                    var octave4 = (MathF.Sin((vX + 67) * 1.50f) + MathF.Sin((vY + 13) * 1.5f)) * 0.125f;
                    var distanceToEdge = planeWidth / 2 - Math.Max(Math.Abs(vX - planeWidth / 2), Math.Abs(vY - planeWidth / 2));
                    var edgeRamp = 25f / (distanceToEdge + 1);
                    var terrainHeight = octave0 + octave1 + octave2 + octave3 + octave4;
                    var vertexPosition = new Vector2(vX * scale, vY * scale) + terrainPosition;
                    var distanceToTrack = raceTrack.GetDistance(vertexPosition);
                    var trackWeight = MathF.Min(1f, 3f / (distanceToTrack * 0.1f + 1f));
                    var height = trackWeight * -10f + terrainHeight * (1 - trackWeight);
                    return new Vector3(vertexPosition.X, height + edgeRamp, vertexPosition.Y);

                }, new Vector3(1, 1, 1), BufferPool, out var planeMesh);
            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -15, 0), QuaternionEx.CreateFromAxisAngle(new Vector3(0, 1, 0), MathF.PI / 2),
                new CollidableDescription(Simulation.Shapes.Add(planeMesh), 0.1f)));


        }

        bool playerControlActive = true;
        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            if (input.WasPushed(ToggleCar))
                playerControlActive = !playerControlActive;
            //For control purposes, we'll match the fixed update rate of the simulation. Could decouple it- this dt isn't
            //vulnerable to the same instabilities as the simulation itself with variable durations.
            const float controlDt = 1 / 60f;
            if (playerControlActive)
            {
                float steeringSum = 0;
                if (input.IsDown(Left))
                {
                    steeringSum += 1;
                }
                if (input.IsDown(Right))
                {
                    steeringSum -= 1;
                }
                var targetSpeedFraction = input.IsDown(Forward) ? 1f : input.IsDown(Backward) ? -1f : 0;
                var zoom = input.IsDown(Zoom);
                playerController.Update(Simulation, controlDt, steeringSum, targetSpeedFraction, zoom, input.IsDown(Brake) || input.IsDown(BrakeAlternate));
            }

            for (int i = 0; i < aiControllers.Length; ++i)
            {
                ref var ai = ref aiControllers[i];
                var body = Simulation.Bodies.GetBodyReference(ai.Controller.Car.Body);
                ref var pose = ref body.Pose;
                Matrix3x3.CreateFromQuaternion(pose.Orientation, out var orientation);
                var forwardVelocity = Vector3.Dot(orientation.Z, body.Velocity.Linear);
                var predictedLocation = new Vector2(pose.Position.X, pose.Position.Z) + new Vector2(orientation.Z.X, orientation.Z.Z) * (5 + forwardVelocity * 2);
                raceTrack.GetClosestPoint(predictedLocation, ai.LaneOffset, out var closestPoint, out var flowDirection);
                float steeringAngle;
                if (flowDirection.X * orientation.Z.X + flowDirection.Y * orientation.Z.Z < 0)
                {
                    //Don't drive against traffic!
                    steeringAngle = ai.Controller.MaximumSteeringAngle;
                }
                else
                {
                    var toClosestPoint = closestPoint - new Vector2(pose.Position.X, pose.Position.Z);
                    var horizontalOffset = orientation.X.X * toClosestPoint.X + orientation.X.Z * toClosestPoint.Y;
                    var forwardOffset = orientation.Z.X * toClosestPoint.X + orientation.Z.Z * toClosestPoint.Y;
                    steeringAngle = MathF.Atan2(horizontalOffset, forwardOffset);
                }
                var speedFraction = 0.25f + MathF.Min(0.75f, MathF.Max(0, 0.75f * (MathF.Abs(steeringAngle) - 0.2f) / -0.4f));
                if (orientation.Y.Y < 0.4f)
                    speedFraction = 0;
                ai.Controller.Update(Simulation, controlDt, steeringAngle, speedFraction, steeringAngle < 0.05f, steeringAngle > MathF.PI * 0.2f && forwardVelocity > ai.Controller.ForwardSpeed * 0.6f);
            }

            base.Update(window, camera, input, dt);
        }

        void RenderControl(ref Vector2 position, float textHeight, string controlName, string controlValue, TextBuilder text, TextBatcher textBatcher, Font font)
        {
            text.Clear().Append(controlName).Append(": ").Append(controlValue);
            textBatcher.Write(text, position, textHeight, new Vector3(1), font);
            position.Y += textHeight * 1.1f;
        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            if (playerControlActive)
            {
                var carBody = new BodyReference(playerController.Car.Body, Simulation.Bodies);
                QuaternionEx.TransformUnitY(carBody.Pose.Orientation, out var carUp);
                camera.Position = carBody.Pose.Position + carUp * 1.3f + camera.Backward * 8;
            }

            var textHeight = 16;
            var position = new Vector2(32, renderer.Surface.Resolution.Y - 128);
            RenderControl(ref position, textHeight, nameof(Forward), Forward.ToString(), text, renderer.TextBatcher, font);
            RenderControl(ref position, textHeight, nameof(Backward), Backward.ToString(), text, renderer.TextBatcher, font);
            RenderControl(ref position, textHeight, nameof(Right), Right.ToString(), text, renderer.TextBatcher, font);
            RenderControl(ref position, textHeight, nameof(Left), Left.ToString(), text, renderer.TextBatcher, font);
            RenderControl(ref position, textHeight, nameof(Zoom), Zoom.ToString(), text, renderer.TextBatcher, font);
            RenderControl(ref position, textHeight, nameof(Brake), Brake.ToString(), text, renderer.TextBatcher, font);
            RenderControl(ref position, textHeight, nameof(ToggleCar), ToggleCar.ToString(), text, renderer.TextBatcher, font);
            base.Render(renderer, camera, input, text, font);
        }
    }
}