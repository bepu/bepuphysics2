using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace Demos.Demos
{
    /// <summary>
    /// Shows how to configure things to slide at different rates.
    /// </summary>
    /// <remarks>
    /// For a similar example of bounciness, see the <see cref="BouncinessDemo"/>.
    /// </remarks>
    public class FrictionDemo : Demo
    {
        public struct SimpleMaterial
        {
            public SpringSettings SpringSettings;
            public float FrictionCoefficient;
            public float MaximumRecoveryVelocity;
        }
        public unsafe struct FrictionCallbacks : INarrowPhaseCallbacks
        {
            /// <summary>
            /// Maps <see cref="CollidableReference"/> entries to their <see cref="SimpleMaterial"/>.
            /// </summary>
            /// <remarks>
            /// The narrow phase callbacks need some way to get the material data for this demo, but there's no requirement that you use the <see cref="CollidableProperty{T}"/> type.
            /// It's just a fairly convenient and simple option.
            /// </remarks>
            public CollidableProperty<SimpleMaterial> CollidableMaterials;

            public void Initialize(Simulation simulation)
            {
                //The callbacks get created before the simulation so that they can be given to the simulation. The property needs a simulation reference, so we hand it over in the initialize.
                CollidableMaterials.Initialize(simulation);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowContactGeneration(int workerIndex, CollidableReference a, CollidableReference b, ref float speculativeMargin)
            {
                //While the engine won't even try creating pairs between statics at all, it will ask about kinematic-kinematic pairs.
                //Those pairs cannot emit constraints since both involved bodies have infinite inertia. Since most of the demos don't need
                //to collect information about kinematic-kinematic pairs, we'll require that at least one of the bodies needs to be dynamic.
                return a.Mobility == CollidableMobility.Dynamic || b.Mobility == CollidableMobility.Dynamic;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowContactGeneration(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB)
            {
                return true;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public unsafe bool ConfigureContactManifold<TManifold>(int workerIndex, CollidablePair pair, ref TManifold manifold, out PairMaterialProperties pairMaterial) where TManifold : unmanaged, IContactManifold<TManifold>
            {
                //For the purposes of this demo, we'll use multiplicative blending for the friction and choose spring properties according to which collidable has a higher maximum recovery velocity.
                var a = CollidableMaterials[pair.A];
                var b = CollidableMaterials[pair.B];
                pairMaterial.FrictionCoefficient = a.FrictionCoefficient * b.FrictionCoefficient;
                pairMaterial.MaximumRecoveryVelocity = MathF.Max(a.MaximumRecoveryVelocity, b.MaximumRecoveryVelocity);
                pairMaterial.SpringSettings = pairMaterial.MaximumRecoveryVelocity == a.MaximumRecoveryVelocity ? a.SpringSettings : b.SpringSettings;
                return true;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool ConfigureContactManifold(int workerIndex, CollidablePair pair, int childIndexA, int childIndexB, ref ConvexContactManifold manifold)
            {
                return true;
            }

            public void Dispose()
            {
            }
        }

        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(0, 20, 200);
            camera.Yaw = 0;
            camera.Pitch = 0;
            //Unlike the bounciness demo, we don't have to worry about including strong substepping to help with stiff contact constraint bounces. Friction is easier. 
            var collidableMaterials = new CollidableProperty<SimpleMaterial>();
            Simulation = Simulation.Create(BufferPool, new FrictionCallbacks() { CollidableMaterials = collidableMaterials }, new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0), 0, 0), new SolveDescription(4, 1));

            //Note that the box description includes a significant sideways velocity to make the box go weee.
            var shape = new Box(1, 1, 1);
            var boxDescription = BodyDescription.CreateDynamic(RigidPose.Identity, new Vector3(20, 0, 0), shape.ComputeInertia(1), Simulation.Shapes.Add(shape), 1e-2f);

            int boxCount = 100;
            float maximumFriction = 3;
            for (int i = 0; i < 100; ++i)
            {
                //Drop the boxes in a line with varying friction.
                boxDescription.Pose.Position = new Vector3(-80, 0.5f, i * 1.2f);
                collidableMaterials.Allocate(Simulation.Bodies.Add(boxDescription)) = new SimpleMaterial
                {
                    FrictionCoefficient = maximumFriction * i / (boxCount - 1f),
                    MaximumRecoveryVelocity = 2,
                    SpringSettings = new SpringSettings(30, 1)
                };
            }

            collidableMaterials.Allocate(Simulation.Statics.Add(new StaticDescription(new Vector3(0, -15f, 0), Simulation.Shapes.Add(new Box(2500, 30, 2500))))) =
                new SimpleMaterial { FrictionCoefficient = 1, MaximumRecoveryVelocity = 2, SpringSettings = new SpringSettings(30, 1) };
        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            var resolution = renderer.Surface.Resolution;
            renderer.TextBatcher.Write(text.Clear().Append("Every contact constraint can be configured with its own material property by the INarrowPhaseCallbacks."), new Vector2(16, resolution.Y - 80), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("In this demo, a pair's coefficient of friction is the coefficients of the involved collidables multiplied together."), new Vector2(16, resolution.Y - 64), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Material state is configured ahead of time and stored in per-collidable SimpleMaterial definitions."), new Vector2(16, resolution.Y - 48), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Friction coefficients range from 0 to 3 from far to near."), new Vector2(16, resolution.Y - 32), 16, Vector3.One, font);
            renderer.TextBatcher.Write(text.Clear().Append("Notably, you don't have to do it this way; materials can change over time, or be procedural, or anything else you can express in the callback."), new Vector2(16, resolution.Y - 16), 16, Vector3.One, font);
            base.Render(renderer, camera, input, text, font);
        }
    }
}
