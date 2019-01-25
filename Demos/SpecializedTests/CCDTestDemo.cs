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

namespace Demos.SpecializedTests
{
    public class CCDTestDemo : Demo
    {
        int spinnerBaseA;
        int spinnerBaseB;
        RolloverInfo rolloverInfo;

        int BuildSpinner(Vector3 position)
        {
            var spinnerBase = Simulation.Bodies.Add(BodyDescription.CreateKinematic(position, new CollidableDescription(Simulation.Shapes.Add(new Box(2, 2, 2)), 0.1f), new BodyActivityDescription(0.01f)));
            var bladeShape = new Box(5, 0.01f, 1);
            bladeShape.ComputeInertia(1, out var bladeInertia);
            var shapeIndex = Simulation.Shapes.Add(bladeShape);
            //Note that the minimum progression parameter is very small, at 1e-4, while the convergence threshold is quite loose at 1e-2.
            //The minimum progression parameter is the duration of collisions that we're willing to skip. Given the extreme thinness of the spinners in this test, we don't want to skip them at all.
            var spinnerBlade = Simulation.Bodies.Add(BodyDescription.CreateDynamic(position, bladeInertia, new CollidableDescription(shapeIndex, 0.1f, ContinuousDetectionSettings.Continuous(1e-4f, 1e-2f)), new BodyActivityDescription(0.01f)));
            Simulation.Solver.Add(spinnerBase, spinnerBlade, new Hinge { LocalHingeAxisA = new Vector3(0, 0, 1), LocalHingeAxisB = new Vector3(0, 0, 1), LocalOffsetB = new Vector3(0, 0, -3), SpringSettings = new SpringSettings(30, 1) });
            Simulation.Solver.Add(spinnerBase, spinnerBlade, new AngularAxisMotor { LocalAxisA = new Vector3(0, 0, 1), Settings = new MotorSettings(10, 1e-4f), TargetVelocity = 40 });
            return spinnerBase;
        }

        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 10, 35);
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
                Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(-4, 100 + i * 10, i * 2), new BodyVelocity { Linear = new Vector3(0, -150, 0) }, inertia,
                    new CollidableDescription(shapeIndex, 0.01f), new BodyActivityDescription(0.01f)));
                //These two falling dynamics have pretty small speculative margins. The second one uses continuous collision detection sweeps to generate speculative contacts.
                //Note that the sweep accuracy (defined by the parameters to the continuous detection settings) is actually pretty loose.
                //Despite that, it can still lead to reasonably good speculative contacts with solid impact behavior.
                //(That's because the sweep does not directly generate contacts- it generates a time of impact estimate, and then the discrete contact generation
                //runs to create the actual contact manifold. That provides high quality contact positions and speculative depths.)
                Simulation.Bodies.Add(BodyDescription.CreateDynamic(new Vector3(4, 100 + i * 1, i * 2), new BodyVelocity { Linear = new Vector3(0, -150, 0) }, inertia,
                    new CollidableDescription(shapeIndex, 0.01f, ContinuousDetectionSettings.Continuous(1e-5f, 1e-5f)), new BodyActivityDescription(0.01f)));
            }
            rolloverInfo = new RolloverInfo();
            rolloverInfo.Add(new Vector3(-5, 2, 0), "Discrete");
            rolloverInfo.Add(new Vector3(5, 2, 0), "Continuous");

            spinnerBaseA = BuildSpinner(new Vector3(-2.5f, 10, -5));
            spinnerBaseB = BuildSpinner(new Vector3(2.5f, 10, -5));
            rolloverInfo.Add(new Vector3(0, 12, -5), "High angular velocity continuous detection");

            Simulation.Statics.Add(new StaticDescription(new Vector3(0, -5f, 0), new CollidableDescription(Simulation.Shapes.Add(new Box(300, 10, 300)), 0.1f)));
        }

        double time;
        public override void Update(Window window, Camera camera, Input input, float dt)
        {
            //This will drift over time, but that's fine.
            var velocity = (float)Math.Cos(time);
            new BodyReference(spinnerBaseA, Simulation.Bodies).Velocity.Linear.X = velocity;
            new BodyReference(spinnerBaseB, Simulation.Bodies).Velocity.Linear.X = -velocity;
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
