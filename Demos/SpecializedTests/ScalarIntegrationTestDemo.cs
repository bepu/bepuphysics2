using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuPhysics;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer.UI;
using DemoRenderer;
using DemoUtilities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using BepuUtilities.Collections;

namespace Demos.SpecializedTests
{
    public unsafe class ScalarIntegrationTestDemo : Demo
    {
        struct ScalarIntegrationCallbacks : IPoseIntegratorCallbacks
        {
            public delegate*<int, Vector3, Quaternion, BodyInertia, int, float, BodyVelocity*, void> IntegrateVelocityFunction;

            public readonly AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.Nonconserving;

            public readonly bool AllowSubstepsForUnconstrainedBodies => false;

            public readonly bool IntegrateVelocityForKinematics => false;

            public void Initialize(Simulation simulation)
            {
            }

            public void PrepareForIntegration(float dt)
            {
            }

            public void IntegrateVelocity(Vector<int> bodyIndices, Vector3Wide position, QuaternionWide orientation, BodyInertiaWide localInertia, Vector<int> integrationMask, int workerIndex, Vector<float> dt, ref BodyVelocityWide velocity)
            {
                //TODO: This is going to be a very bad implementation for now. Vectorized transposition would speed this up.
                for (int i = 0; i < Vector<float>.Count; ++i)
                {
                    if (integrationMask[i] != 0)
                    {
                        Vector3Wide.ReadSlot(ref position, i, out var scalarPosition);
                        QuaternionWide.ReadSlot(ref orientation, i, out var scalarOrientation);
                        BodyInertia scalarInertia;
                        scalarInertia.InverseInertiaTensor.XX = localInertia.InverseInertiaTensor.XX[i];
                        scalarInertia.InverseInertiaTensor.YX = localInertia.InverseInertiaTensor.YX[i];
                        scalarInertia.InverseInertiaTensor.YY = localInertia.InverseInertiaTensor.YY[i];
                        scalarInertia.InverseInertiaTensor.ZX = localInertia.InverseInertiaTensor.ZX[i];
                        scalarInertia.InverseInertiaTensor.ZY = localInertia.InverseInertiaTensor.ZY[i];
                        scalarInertia.InverseInertiaTensor.ZZ = localInertia.InverseInertiaTensor.ZZ[i];
                        scalarInertia.InverseMass = localInertia.InverseMass[i];
                        BodyVelocity scalarVelocity;
                        Vector3Wide.ReadSlot(ref velocity.Linear, i, out scalarVelocity.Linear);
                        Vector3Wide.ReadSlot(ref velocity.Angular, i, out scalarVelocity.Angular);

                        IntegrateVelocityFunction(bodyIndices[i], scalarPosition, scalarOrientation, scalarInertia, workerIndex, dt[i], &scalarVelocity);

                        Vector3Wide.WriteSlot(scalarVelocity.Linear, i, ref velocity.Linear);
                        Vector3Wide.WriteSlot(scalarVelocity.Angular, i, ref velocity.Angular);
                    }
                }
            }
        }

        static void IntegrateVelocity(int bodyIndex, Vector3 position, Quaternion orientation, BodyInertia inertia, int workerIndex, float dt, BodyVelocity* velocity)
        {
            velocity->Linear += new Vector3(0, -10 / 60f, 0);
        }

        public unsafe override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-30, 10, -30);
            //camera.Yaw = MathHelper.Pi ; 
            camera.Yaw = MathHelper.Pi * 3f / 4;
            //camera.Pitch = MathHelper.PiOver2 * 0.999f;
            ScalarIntegrationCallbacks callbacks = new() { IntegrateVelocityFunction = &IntegrateVelocity };
            //DemoPoseIntegratorCallbacks callbacks = new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0));

            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), callbacks, new SolveDescription(1, 4));

            var sphere = new Sphere(1.5f);
            var capsule = new Capsule(1f, 1f);
            var box = new Box(1f, 3f, 2f);
            var cylinder = new Cylinder(1.5f, 0.3f);
            var points = new QuickList<Vector3>(32, BufferPool);
            //Boxlike point cloud.
            //points.Allocate(BufferPool) = new Vector3(0, 0, 0);
            //points.Allocate(BufferPool) = new Vector3(0, 0, 1);
            //points.Allocate(BufferPool) = new Vector3(0, 1, 0);
            //points.Allocate(BufferPool) = new Vector3(0, 1, 1);
            //points.Allocate(BufferPool) = new Vector3(1, 0, 0);
            //points.Allocate(BufferPool) = new Vector3(1, 0, 1);
            //points.Allocate(BufferPool) = new Vector3(1, 1, 0);
            //points.Allocate(BufferPool) = new Vector3(1, 1, 1);

            //Rando pointcloud.
            //var random = new Random(5);
            //for (int i = 0; i < 32; ++i)
            //{
            //    points.Allocate(BufferPool) = new Vector3(3 * random.NextSingle(), 1 * random.NextSingle(), 3 * random.NextSingle());
            //}

            //Dodecahedron pointcloud.
            points.Allocate(BufferPool) = new Vector3(-1, -1, -1);
            points.Allocate(BufferPool) = new Vector3(-1, -1, 1);
            points.Allocate(BufferPool) = new Vector3(-1, 1, -1);
            points.Allocate(BufferPool) = new Vector3(-1, 1, 1);
            points.Allocate(BufferPool) = new Vector3(1, -1, -1);
            points.Allocate(BufferPool) = new Vector3(1, -1, 1);
            points.Allocate(BufferPool) = new Vector3(1, 1, -1);
            points.Allocate(BufferPool) = new Vector3(1, 1, 1);

            const float goldenRatio = 1.618033988749f;
            const float oogr = 1f / goldenRatio;

            points.Allocate(BufferPool) = new Vector3(0, goldenRatio, oogr);
            points.Allocate(BufferPool) = new Vector3(0, -goldenRatio, oogr);
            points.Allocate(BufferPool) = new Vector3(0, goldenRatio, -oogr);
            points.Allocate(BufferPool) = new Vector3(0, -goldenRatio, -oogr);

            points.Allocate(BufferPool) = new Vector3(oogr, 0, goldenRatio);
            points.Allocate(BufferPool) = new Vector3(oogr, 0, -goldenRatio);
            points.Allocate(BufferPool) = new Vector3(-oogr, 0, goldenRatio);
            points.Allocate(BufferPool) = new Vector3(-oogr, 0, -goldenRatio);

            points.Allocate(BufferPool) = new Vector3(goldenRatio, oogr, 0);
            points.Allocate(BufferPool) = new Vector3(goldenRatio, -oogr, 0);
            points.Allocate(BufferPool) = new Vector3(-goldenRatio, oogr, 0);
            points.Allocate(BufferPool) = new Vector3(-goldenRatio, -oogr, 0);

            var convexHull = new ConvexHull(points.Span.Slice(points.Count), BufferPool, out _);
            var boxInertia = box.ComputeInertia(1);
            var capsuleInertia = capsule.ComputeInertia(1);
            var sphereInertia = sphere.ComputeInertia(1);
            var cylinderInertia = cylinder.ComputeInertia(1);
            var hullInertia = convexHull.ComputeInertia(1);
            var boxIndex = Simulation.Shapes.Add(box);
            var capsuleIndex = Simulation.Shapes.Add(capsule);
            var sphereIndex = Simulation.Shapes.Add(sphere);
            var cylinderIndex = Simulation.Shapes.Add(cylinder);
            var hullIndex = Simulation.Shapes.Add(convexHull);
            const int width = 32;
            const int height = 32;
            const int length = 32;
            var shapeCount = 0;
            for (int i = 0; i < width; ++i)
            {
                for (int j = 0; j < height; ++j)
                {
                    for (int k = 0; k < length; ++k)
                    {
                        var location = new Vector3(6, 3, 6) * new Vector3(i, j, k) + new Vector3(-width * 1.5f, 5.5f, -length * 1.5f);
                        var bodyDescription = BodyDescription.CreateKinematic(location, new(default, ContinuousDetection.Passive), -0.01f);
                        var index = shapeCount++;
                        switch (index % 5)
                        {
                            case 0:
                                bodyDescription.Collidable.Shape = sphereIndex;
                                bodyDescription.LocalInertia = sphereInertia;
                                break;
                            case 1:
                                bodyDescription.Collidable.Shape = capsuleIndex;
                                bodyDescription.LocalInertia = capsuleInertia;
                                break;
                            case 2:
                                bodyDescription.Collidable.Shape = boxIndex;
                                bodyDescription.LocalInertia = boxInertia;
                                break;
                            case 3:
                                bodyDescription.Collidable.Shape = cylinderIndex;
                                bodyDescription.LocalInertia = cylinderInertia;
                                break;
                            case 4:
                                bodyDescription.Collidable.Shape = hullIndex;
                                bodyDescription.LocalInertia = hullInertia;
                                break;
                        }
                        Simulation.Bodies.Add(bodyDescription);

                    }
                }
            }

            //Simulation.Statics.Add(new StaticDescription(new Vector3(), Simulation.Shapes.Add(new Box(500, 1, 500))));
            var mesh = DemoMeshHelper.CreateDeformedPlane(128, 128, (x, y) => new Vector3(x - 64, 2f * (float)(Math.Sin(x * 0.5f) * Math.Sin(y * 0.5f)), y - 64), new Vector3(4, 1, 4), BufferPool);
            Simulation.Statics.Add(new StaticDescription(new Vector3(), Simulation.Shapes.Add(mesh)));

        }
    }
}
