using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using System;
using System.Numerics;

namespace Demos.Demos
{

    /// <summary>
    /// Shows how to use custom velocity integration to implement per-body gravity.
    /// </summary>
    public class PerBodyGravityDemo : Demo
    {
        struct PerBodyGravityDemoCallbacks : IPoseIntegratorCallbacks
        {
            /// <summary>
            /// Maps body handles to per-body gravity values.
            /// CollidableProperty stores data aligned with the integer value of a body handle which doesn't change over the lifespan of a body in the simulation.
            /// Unlike active set body indices, body handles don't move around when other bodies are removed or slept.
            /// There's nothing special about the CollidableProperty; it's just a helper, feel free to use any approach that works.
            /// </summary>
            public CollidableProperty<float> BodyGravities;
            /// <summary>
            /// Used to look up body handles using the callback-provided body indices.
            /// </summary>
            private Bodies bodies;

            public readonly AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.Nonconserving;

            public readonly bool AllowSubstepsForUnconstrainedBodies => false;

            public readonly bool IntegrateVelocityForKinematics => false;

            public void Initialize(Simulation simulation)
            {
                //Bit awkward, but the CollidableProperty wants to know about the Simulation so it can resize things intelligently,
                //and the Simulation doesn't exist when the CollidableProperty instance is created... so we let the callbacks initialize it.
                BodyGravities.Initialize(simulation);
                bodies = simulation.Bodies;
            }

            public void PrepareForIntegration(float dt)
            {
            }

            public void IntegrateVelocity(Vector<int> bodyIndices, Vector3Wide position, QuaternionWide orientation, BodyInertiaWide localInertia, Vector<int> integrationMask, int workerIndex, Vector<float> dt, ref BodyVelocityWide velocity)
            {
                //Velocity integration runs over bundles of bodies, not just one at a time.
                //The reason is that all calling contexts of this function are vectorized and transitioning from AOSOA to the more familiar AOS layout is not free.
                //The provided representation is the simplest and fastest one given the way integration works internally.
                //Each lane in the SIMD vectors corresponds to a different body.
                //In other words, velocity.Linear.Y[2] corresponds to the Y component of the linear velocity of the third body in the bundle.

                //Applications which require per-body information to be gathered dynamically are difficult to vectorize.
                //Here, we write each slot in the bundle with a value gathered through a handle lookup in the BodyGravities.
                //This is *not* the fastest way to gather data for a vectorized representation, but it is relatively simple. 
                //If you have a bunch of data that you want to gather per body, you could consider a more complex vectorized transposition, like how Bodies.GatherState works.

                //While this is more expensive than not looking up per-body data, it's a good idea to keep things in perspective:
                //On a 3970x running this simulation, the full frame cost difference between this per-body lookup
                //(versus something like velocity.Linear.Y += new Vector<float>(-10) * dt) is about 50-100 microseconds.
                Span<float> gravityValues = stackalloc float[Vector<float>.Count];
                for (int bundleSlotIndex = 0; bundleSlotIndex < Vector<int>.Count; ++bundleSlotIndex)
                {
                    var bodyIndex = bodyIndices[bundleSlotIndex];
                    //Not every slot in the SIMD vector is guaranteed to be filled.
                    //The integration mask tells us which ones are active in a way that's convenient for vectorized operations, but the bodyIndex for empty lanes will also be -1.
                    if (bodyIndex >= 0)
                    {
                        var bodyHandle = bodies.ActiveSet.IndexToHandle[bodyIndex];
                        gravityValues[bundleSlotIndex] = BodyGravities[bodyHandle];
                    }
                }
                //Note that the callback doesn't have to filter writes based on the integration mask, even though not every slot might be active.
                //The caller is responsible for only using the active slots.
                velocity.Linear.Y += new Vector<float>(gravityValues) * dt;
            }
        }


        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 20, 80);
            camera.Yaw = 0;
            camera.Pitch = 0;

            //The CollidableProperty is a helper that associates body handles to whatever data you'd like to store. You don't have to use it, but it's fairly convenient.
            var bodyGravities = new CollidableProperty<float>(BufferPool);
            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new PerBodyGravityDemoCallbacks() { BodyGravities = bodyGravities }, new SolveDescription(4, 1));

            Simulation.Statics.Add(new StaticDescription(new Vector3(), Simulation.Shapes.Add(new Box(1000, 10, 1000))));

            //Make bodies with different shapes, and give each shape type its own gravity so that it's visually comprehensible.
            var sphereShape = new Sphere(1f);
            var sphereInertia = sphereShape.ComputeInertia(1);
            var sphereShapeIndex = Simulation.Shapes.Add(sphereShape);
            var capsuleShape = new Capsule(1f, 1f);
            var capsuleInertia = capsuleShape.ComputeInertia(1);
            var capsuleShapeIndex = Simulation.Shapes.Add(capsuleShape);
            var boxShape = new Box(1f, 1f, 1f);
            var boxInertia = boxShape.ComputeInertia(1);
            var boxShapeIndex = Simulation.Shapes.Add(boxShape);
            var spacing = new Vector3(4);
            const int length = 1;
            const int width = 1;
            const int height = 1;
            for (int i = 0; i < length; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    var origin = new Vector3(0, 40, 0) + spacing * new Vector3(length * -0.5f, 0, width * -0.5f);
                    for (int k = 0; k < width; ++k)
                    {
                        BodyInertia inertia;
                        TypedIndex shapeIndex;
                        float gravity;
                        switch ((i + k) % 3)
                        {
                            case 0:
                                inertia = sphereInertia;
                                shapeIndex = sphereShapeIndex;
                                gravity = -.1f;
                                break;
                            case 1:
                                inertia = capsuleInertia;
                                shapeIndex = capsuleShapeIndex;
                                gravity = -3f;
                                break;
                            default:
                                inertia = boxInertia;
                                shapeIndex = boxShapeIndex;
                                gravity = -10f;
                                break;
                        }

                        var bodyHandle = Simulation.Bodies.Add(BodyDescription.CreateDynamic(
                            origin + new Vector3(i, j, k) * spacing, new Vector3(0, 0, 0), inertia, shapeIndex, 0.001f));
                        bodyGravities.Allocate(bodyHandle) = gravity;
                    }
                }
            }

        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            var bottomY = renderer.Surface.Resolution.Y;
            renderer.TextBatcher.Write(text.Clear().Append("The user-supplied IPoseIntegratorCallbacks.IntegrateVelocity implementation defines how body velocities change over time."), new Vector2(16, bottomY - 48), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("It's commonly used for gravity. Here, each body's gravity is defined independently."), new Vector2(16, bottomY - 32), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Spheres fall slowly, boxes quickly, and capsules are in between."), new Vector2(16, bottomY - 16), 16, Vector3.One, font);
            base.Render(renderer, camera, input, text, font);
        }
    }
}
