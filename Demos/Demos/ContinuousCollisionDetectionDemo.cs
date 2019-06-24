using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using DemoUtilities;
using DemoRenderer.UI;
using Quaternion = BepuUtilities.Quaternion;

namespace Demos.Demos
{
    public class ContinuousCollisionDetectionDemo : Demo
    {
        int spinnerMotorA;
        int spinnerMotorB;
        RolloverInfo rolloverInfo;

        int BuildSpinner(Vector3 initialPosition, float rotationSpeed)
        {
            var spinnerBase = Simulation.Bodies.Add(BodyDescription.CreateDynamic(initialPosition, new BodyInertia { InverseMass = 1e-2f }, new CollidableDescription(Simulation.Shapes.Add(new Box(2, 2, 2)), 0.1f), new BodyActivityDescription(0.01f)));
            var bladeShape = new Box(5, 0.01f, 1);
            bladeShape.ComputeInertia(1, out var bladeInertia);
            var shapeIndex = Simulation.Shapes.Add(bladeShape);
            //Note that both the minimum progression duration and the sweep convergence duration are both very small at 1e-4. 
            //That will detect collisions with a precision equal to an update rate of 10,000hz.
            //The blades are extremely thin and spinning very quickly, so that kind of precision is helpful.
            //Note that you can likely get away with a larger sweep convergence duration. 
            //The sweep convergence duration is the maximum size of the 'time of first impact' region that the sweep is allowed to terminate with; 
            //using a time of impact which is a little bit off won't usually cause much of a problem.
            //Minimum progression duration is far more important to keep small, since collisions with a duration below the minimum progression duration may be missed entirely.

            //Note that it's possible for the blades to still go through each other in certain corner cases- the CCD sweep only detects time of *first* impact.
            //It's possible for the contacts associated with the first impact to be insufficient for later collisions within the same frame.
            //It's pretty rare, though- if you have a situation where that sort of failure is common, consider increasing the collidable's speculative margin or using a higher update rate.
            //(The reason why we don't always just rely on large speculative margins is ghost collisions- the speculative contacts might not represent collisions
            //that would have actually happened, but get included in the constraint solution anyway. They're fairly rare, but it's something to watch out for.)
            var spinnerBlade = Simulation.Bodies.Add(BodyDescription.CreateDynamic(initialPosition, bladeInertia, new CollidableDescription(shapeIndex, 0.2f, ContinuousDetectionSettings.Continuous(1e-4f, 1e-4f)), new BodyActivityDescription(0.01f)));
            Simulation.Solver.Add(spinnerBase, spinnerBlade, new Hinge { LocalHingeAxisA = new Vector3(0, 0, 1), LocalHingeAxisB = new Vector3(0, 0, 1), LocalOffsetB = new Vector3(0, 0, -3), SpringSettings = new SpringSettings(30, 1) });
            Simulation.Solver.Add(spinnerBase, spinnerBlade, new AngularAxisMotor { LocalAxisA = new Vector3(0, 0, 1), Settings = new MotorSettings(10, 1e-4f), TargetVelocity = rotationSpeed });
            return Simulation.Solver.Add(spinnerBase, new OneBodyLinearServo { ServoSettings = ServoSettings.Default, SpringSettings = new SpringSettings(30, 1) });
        }

        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 10, 40);
            camera.Yaw = 0;
            camera.Pitch = 0;
            //Note the higher stiffness on contacts for this demo. That's not ideal for general stability at the demo timestep duration default of 60hz, but
            //this demo doesn't have any significant solver complexity and we want to see the CCD in action more clearly- which means more rigid contact.
            //Having objects bounce a bunch on impact makes it harder to see.
            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks() { ContactSpringiness = new SpringSettings(120, 1) }, new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)));

            var shape = new Box(1, 1, 1);
            shape.ComputeInertia(1, out var inertia);
            var shapeIndex = Simulation.Shapes.Add(shape);
            for (int i = 0; i < 10; ++i)
            {
                for (int j = 0; j < 10; ++j)
                {
                    //These two falling dynamics have pretty small speculative margins. The second one uses continuous collision detection sweeps to generate speculative contacts.
                    Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(-4 - 2 * j, 100 + (i + j) * 2, i * 2), new BodyVelocity { Linear = new Vector3(0, -150, 0) }, inertia,
                        new CollidableDescription(shapeIndex, 0.01f), new BodyActivityDescription(0.01f)));
                    //The minimum progression duration parameter at 1e-3 means the CCD sweep won't miss any collisions that last at least 1e-3 units of time- so, if time is measured in seconds,
                    //then this will capture any collision that an update rate of 1000hz would.
                    //Note also that the sweep convergence threshold is actually pretty loose at 100hz. Despite that, it can still lead to reasonably good speculative contacts with solid impact behavior.
                    //That's because the sweep does not directly generate contacts- it generates a time of impact estimate, and then the discrete contact generation
                    //runs to create the actual contact manifold. That provides high quality contact positions and speculative depths.
                    //If the ground that these boxes were smashing into was something like a mesh- which is infinitely thin- you may want to increase the sweep accuracy.
                    Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(4 + 2 * j, 100 + (i + j) * 2, i * 2), new BodyVelocity { Linear = new Vector3(0, -150, 0) }, inertia,
                        new CollidableDescription(shapeIndex, 0.01f, ContinuousDetectionSettings.Continuous(1e-3f, 1e-2f)), new BodyActivityDescription(0.01f)));
                }
            }
            rolloverInfo = new RolloverInfo();
            rolloverInfo.Add(new Vector3(-12, 2, 0), "Discrete");
            rolloverInfo.Add(new Vector3(12, 2, 0), "Continuous");

            //Build a couple of spinners to ram into each other to showcase angular CCD. Note that the spin speeds are slightly different- that helps avoid 
            //synchronization that makes the blades frequently miss each other, which sorta ruins a CCD demo.
            spinnerMotorA = BuildSpinner(new Vector3(-5, 10, -5), 53);
            spinnerMotorB = BuildSpinner(new Vector3(5, 10, -5), 59);
            rolloverInfo.Add(new Vector3(0, 12, -5), "High angular velocity continuous detection");

            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -5f, 0), new CollidableDescription(Simulation.Shapes.Add(new Box(300, 10, 300)), 0.1f)));
        }

        double time;
        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            //Scoot the spinners around.
            var servo = new OneBodyLinearServo { ServoSettings = ServoSettings.Default, SpringSettings = new SpringSettings(30, 1) };
            servo.Target = new Vector3(-5 - 3.5f * (float)Math.Sin(time), 10, -5);
            Simulation.Solver.ApplyDescription(spinnerMotorA, servo);
            servo.Target = new Vector3(5 + 3.5f * (float)Math.Sin(time), 10, -5);
            Simulation.Solver.ApplyDescription(spinnerMotorB, servo);
            time += dt;
            base.Update(window, camera, input, dt);
        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            rolloverInfo.Render(renderer, camera, input, text, font);
            base.Render(renderer, camera, input, text, font);
        }
    }
}
