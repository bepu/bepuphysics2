using System;
using System.Numerics;
using DemoContentLoader;
using DemoRenderer;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using DemoUtilities;
using DemoRenderer.UI;

namespace Demos.Demos
{
    public class ContinuousCollisionDetectionDemo : Demo
    {
        ConstraintHandle spinnerMotorDefaultA, spinnerMotorDefaultB, spinnerMotorSweepA, spinnerMotorSweepB;
        RolloverInfo rolloverInfo;

        ConstraintHandle BuildSpinner(Vector3 initialPosition, float rotationSpeed, ContinuousDetection continuousDetection)
        {
            var spinnerBase = Simulation.Bodies.Add(BodyDescription.CreateDynamic(initialPosition, new BodyInertia { InverseMass = 1e-2f }, Simulation.Shapes.Add(new Box(2, 2, 2)), 0.01f));
            var bladeShape = new Box(5, 0.01f, 1);
            var bladeInertia = bladeShape.ComputeInertia(1);
            var shapeIndex = Simulation.Shapes.Add(bladeShape);
            //Note that both the minimum progression duration and the sweep convergence duration are both very small at 1e-4. 
            //That will detect collisions with a precision equal to an update rate of 10,000hz.
            //The blades are extremely thin and spinning very quickly, so that kind of precision is helpful.
            //Note that you can likely get away with a larger sweep convergence duration. 
            //The sweep convergence duration is the maximum size of the 'time of first impact' region that the sweep is allowed to terminate with; 
            //using a time of impact which is a little bit off won't usually cause much of a problem.
            //Minimum progression duration is far more important to keep small for this type of use case, since collisions with a duration below the minimum progression duration may be missed entirely.

            //Note that it's possible for the blades to still go through each other in certain corner cases- the CCD sweep only detects time of *first* impact.
            //It's possible for the contacts associated with the first impact to be insufficient for later collisions within the same frame.
            //It's pretty rare, though- if you have a situation where that sort of failure is common, consider increasing the collidable's speculative margin or using a higher update rate.
            //(The reason why we don't always just rely on large speculative margins is ghost collisions- the speculative contacts might not represent collisions
            //that would have actually happened, but get included in the constraint solution anyway. They're fairly rare, but it's something to watch out for.)

            //Using a restricted speculative margin by setting the maximumSpeculativeMargin to 0.2 means that collision detection won't accept distant contacts.
            //This pretty much eliminates ghost collisions, while the continuous sweep helps avoid missed collisions.
            var spinnerBlade = Simulation.Bodies.Add(BodyDescription.CreateDynamic(initialPosition, bladeInertia, new(shapeIndex, continuousDetection), 0.01f));
            Simulation.Solver.Add(spinnerBase, spinnerBlade, new Hinge { LocalHingeAxisA = new Vector3(0, 0, 1), LocalHingeAxisB = new Vector3(0, 0, 1), LocalOffsetB = new Vector3(0, 0, -3), SpringSettings = new SpringSettings(30, 1) });
            Simulation.Solver.Add(spinnerBase, spinnerBlade, new AngularAxisMotor { LocalAxisA = new Vector3(0, 0, 1), Settings = new MotorSettings(10, 1e-4f), TargetVelocity = rotationSpeed });
            return Simulation.Solver.Add(spinnerBase, new OneBodyLinearServo { ServoSettings = ServoSettings.Default, SpringSettings = new SpringSettings(30, 1) });
        }

        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 10, 30);
            camera.Yaw = 0;
            camera.Pitch = 0;
            //Note the higher stiffness on contacts for this demo. That's not ideal for general stability at the demo timestep duration default of 60hz, but
            //this demo doesn't have any significant solver complexity and we want to see the CCD in action more clearly- which means more rigid contact.
            //Having objects bounce (or even squish through each other if they're thin enough!) makes things harder to see.
            //Note that this demo only uses 1 substep. For high impact velocities, more velocity iterations can avoid inducing rotations due to incomplete contact solves in any given substep.
            //That's handy for keeping the impact more controlled and visualizing the difference between discrete and continuous modes.
            Simulation = Simulation.Create(BufferPool,
                new DemoNarrowPhaseCallbacks(new SpringSettings(120, 1), maximumRecoveryVelocity: 1f),
                new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));

            var shape = new Box(1, 1, 1);
            var inertia = shape.ComputeInertia(1);
            var shapeIndex = Simulation.Shapes.Add(shape);
            for (int i = 0; i < 10; ++i)
            {
                for (int j = 0; j < 10; ++j)
                {
                    //The first set of boxes are forced to use very small speculative margins. They're going to tunnel into the ground, since no contacts will be created to stop it.
                    Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(-37 + 2 * j, 100 + (i + j) * 2, -30 + i * 2), new Vector3(0, -150, 0), inertia,
                        new(shapeIndex, ContinuousDetection.Discrete(maximumSpeculativeMargin: 0.01f)), 0.01f));

                    //The second set of boxes are not using explicit continuous collision sweeps, but have unlimited speculative margins.
                    //This configuration is the most common one you're likely to see in the demos (and use in your own applications).
                    //"ContinuousDetection.Passive" here just sets the maximum speculative margin to float.MaxValue in passive mode.
                    //Discrete vs Passive mode just controls how the bounding box is expanded:
                    //in Discrete mode, the bounding box can only be expanded by velocity up to the speculative margin.
                    //In Passive mode, the bounding box will expand to encompass the whole velocity. 
                    //If Discrete is given a maximum speculative margin of float.MaxValue, they're functionally equivalent.
                    //(The difference exists because bounding box expansion is sometimes required to catch collisions from *other* continuous bodies.
                    //Without expanding the bounding box on a discrete body with low margin, another continuous body might not know the unexpanded body even exists and fly right by.
                    //If you don't care about that corner case for a given body, then using discrete is fine.)
                    Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(-9 + 2 * j, 100 + (i + j) * 2, -30 + i * 2), new Vector3(0, -150, 0), inertia,
                        new(shapeIndex, ContinuousDetection.Passive), 0.01f));

                    //The third set of boxes uses explicit continuous sweeps. Because of the sweeps, the speculative margin can be kept very small.
                    //The minimum progression duration parameter at 1e-3 means the CCD sweep won't miss any collisions that last at least 1e-3 units of time- so, if time is measured in seconds,
                    //then this will capture any collision that an update rate of 1000hz would.
                    //Note also that the sweep convergence threshold is actually pretty loose at 100hz. Despite that, it can still lead to reasonably good speculative contacts with solid impact behavior.
                    //That's because the sweep does not directly generate contacts- it generates a time of impact estimate, and then the discrete contact generation
                    //runs to create the actual contact manifold. That provides high quality contact positions and speculative depths.
                    //If the ground that these boxes were smashing into was something like a mesh- which is infinitely thin- you may want to increase the sweep accuracy.
                    Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(17 + 2 * j, 100 + (i + j) * 2, -30 + i * 2), new Vector3(0, -150, 0), inertia,
                        new(shapeIndex, ContinuousDetection.Continuous(1e-3f, 1e-2f, maximumSpeculativeMargin: 0.01f)), 0.01f));
                }
            }
            rolloverInfo = new RolloverInfo();
            rolloverInfo.Add(new Vector3(-25, 2, -30), "Small speculative margin");
            rolloverInfo.Add(new Vector3(0, 2, -30), "Unlimited speculative margin");
            rolloverInfo.Add(new Vector3(25, 2, -30), "Small margin, continuous sweep");

            //Build a couple of spinners to ram into each other to showcase angular CCD. Note that the spin speeds are slightly different- that helps avoid 
            //synchronization that makes the blades frequently miss each other, which sorta ruins a CCD demo.
            var onlySpeculativeMargin = ContinuousDetection.Passive;
            spinnerMotorDefaultA = BuildSpinner(new Vector3(-20, 14, 0), 53, onlySpeculativeMargin);
            spinnerMotorDefaultB = BuildSpinner(new Vector3(-10, 14, 0), 59, onlySpeculativeMargin);
            rolloverInfo.Add(new Vector3(-15, 14, -5), "Unlimited speculative margin");

            var continuous = ContinuousDetection.Continuous(1e-4f, 1e-4f, maximumSpeculativeMargin: 0.2f);
            spinnerMotorSweepA = BuildSpinner(new Vector3(10, 14, 0), 53, continuous);
            spinnerMotorSweepB = BuildSpinner(new Vector3(20, 14, 0), 59, continuous);
            rolloverInfo.Add(new Vector3(15, 14, -5), "Small margin, continuous sweep");

            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -5f, 0), Simulation.Shapes.Add(new Box(300, 10, 300))));
        }

        double time;
        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            //Scoot the spinners around.
            var servo = new OneBodyLinearServo { ServoSettings = ServoSettings.Default, SpringSettings = new SpringSettings(30, 1) };
            var leftServoTarget = new Vector3(-3.5f * (float)Math.Sin(time), 10, -5);
            var rightServoTarget = new Vector3(3.5f * (float)Math.Sin(time), 10, -5);
            servo.Target = new Vector3(-20, 0, 0) + leftServoTarget;
            Simulation.Solver.ApplyDescription(spinnerMotorDefaultA, servo);
            servo.Target = new Vector3(-10, 0, 0) + rightServoTarget;
            Simulation.Solver.ApplyDescription(spinnerMotorDefaultB, servo);
            servo.Target = new Vector3(10, 0, 0) + leftServoTarget;
            Simulation.Solver.ApplyDescription(spinnerMotorSweepA, servo);
            servo.Target = new Vector3(20, 0, 0) + rightServoTarget;
            Simulation.Solver.ApplyDescription(spinnerMotorSweepB, servo);
            time += dt;
            base.Update(window, camera, input, dt);
        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            rolloverInfo.Render(renderer, camera, input, text, font);
            var bottomY = renderer.Surface.Resolution.Y;
            renderer.TextBatcher.Write(text.Clear().Append("The library uses speculative contacts for collision detection. That means their penetration depth can be negative."), new Vector2(16, bottomY - 240), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Continuous collision detection can be handled through solving these negative depth constraints in a nice unified way."), new Vector2(16, bottomY - 224), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("You can observe default speculative margin behavior in the middle set of boxes that fell in the background, or the left spinners."), new Vector2(16, bottomY - 208), 16, Vector3.One, font);

            renderer.TextBatcher.Write(text.Clear().Append("You can limit speculative margins so contacts won't be created for more distant pairs."), new Vector2(16, bottomY - 176), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("The left set of boxes shows this. When moving quickly, low-margin bodies will tend to tunnel into other bodies."), new Vector2(16, bottomY - 160), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("However, unlimited speculative contacts can produce 'ghost collisions'- contacts generated and solved for collisions that wouldn't have actually occurred."), new Vector2(16, bottomY - 144), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("This is often a subtle effect. Try turning on contact visualization (K) and slowing down timestepping (middle mouse) and watch the left spinners."), new Vector2(16, bottomY - 128), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Sometimes, the spinners will seem to interact and slow down even when they wouldn't have touched. That's a ghost collision."), new Vector2(16, bottomY - 112), 16, Vector3.One, font);

            renderer.TextBatcher.Write(text.Clear().Append("If you need to avoid this for a particular body, you can enable swept continuous collision detection."), new Vector2(16, bottomY - 80), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Instead of using the current body poses, a sweep test identifies a time of future impact and creates contacts at that point."), new Vector2(16, bottomY - 64), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Speculative contacts generated in this way are far less likely to cause ghost collisions."), new Vector2(16, bottomY - 48), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Sweeps do cost a bit more, but they are only used when necessary."), new Vector2(16, bottomY - 32), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("The right spinners and the rightmost set of boxes use swept collision detection."), new Vector2(16, bottomY - 16), 16, Vector3.One, font);
            base.Render(renderer, camera, input, text, font);
        }
    }
}
