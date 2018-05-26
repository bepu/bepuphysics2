using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using BepuUtilities;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Text;

namespace Demos.SpecializedTests
{
    public class TriangleTestDemo : Demo
    {
        void GetPointOnTriangle(Random random, in Triangle triangle, in RigidPose pose, out Vector3 pointOnTriangle)
        {
            float total;
            float a, b, c;
            do
            {
                a = (float)random.NextDouble();
                b = (float)random.NextDouble();
                c = (float)random.NextDouble();
                total = a + b + c;
            } while (total < 1e-7f);
            var inverseTotal = 1f / total;
            a *= inverseTotal;
            b *= inverseTotal;
            c *= inverseTotal;

            var localP = triangle.A * a + triangle.B * b + triangle.C * c;
            BepuUtilities.Quaternion.TransformWithoutOverlap(localP, pose.Orientation, out pointOnTriangle);
            pointOnTriangle += pose.Position;
        }

        void TestRay(in Triangle triangle, in RigidPose pose, in Vector3 rayOrigin, in Vector3 rayDirection, bool expectedImpact)
        {
            var hit = triangle.RayTest(pose, rayOrigin, rayDirection, out var t, out var normal);

            TriangleWide wide;
            wide.Broadcast(triangle);
            RigidPoses.Broadcast(pose, out var poses);
            RayWide rayWide;
            Vector3Wide.Broadcast(rayOrigin, out rayWide.Origin);
            Vector3Wide.Broadcast(rayDirection, out rayWide.Direction);
            wide.RayTest(ref poses, ref rayWide, out var intersectedWide, out var tWide, out var normalWide);

            Debug.Assert(hit == intersectedWide[0] < 0);
            if (hit)
            {
                Debug.Assert(expectedImpact);
                Debug.Assert(Math.Abs(t - tWide[0]) < 1e-7f);
                Vector3Wide.GetLane(ref normalWide, 0, out var normalWideLane0);
                var normalDot = Vector3.Dot(normalWideLane0, normal);
                Debug.Assert(normalDot > 0.9999f && normalDot < 1.00001f);

            }
        }

        public unsafe override void Initialize(Camera camera)
        {
            camera.Position = new Vector3(-30, 8, -60);
            camera.Yaw = MathHelper.Pi * 3f / 4;

            Simulation = Simulation.Create(BufferPool, new TestCallbacks());
            Simulation.PoseIntegrator.Gravity = new Vector3(0, -10, 0);

            Triangle triangle;
            triangle.A = new Vector3(0, 0, 0);
            triangle.B = new Vector3(1, 0, 0);
            triangle.C = new Vector3(0, 0, 1);
            var pose = new RigidPose
            {
                Orientation = BepuUtilities.Quaternion.Identity,
                Position = new Vector3(0)
            };
            var rayOrigin = new Vector3(1 / 3f, 5, 1 / 3f);
            var rayDirection = new Vector3(0, -1, 0);

            //The other convex ray tester doesn't quite map well to infinitely thin triangles, so we have our own little tester here.
            TestRay(triangle, pose, rayOrigin, rayDirection, true);

            Random random = new Random(5);
            const float shapeMin = -50;
            const float shapeSpan = 100;
            for (int i = 0; i < 100; ++i)
            {
                triangle.A = new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) * shapeSpan + new Vector3(shapeMin);
                triangle.B = new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) * shapeSpan + new Vector3(shapeMin);
                triangle.C = new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) * shapeSpan + new Vector3(shapeMin);

                rayOrigin = new Vector3((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()) * shapeSpan + new Vector3(shapeMin);
                var triangleCenter = triangle.A + triangle.B + triangle.C;
                pose.Position = triangleCenter;
                triangle.A -= triangleCenter;
                triangle.B -= triangleCenter;
                triangle.C -= triangleCenter;
                do
                {
                    GetPointOnTriangle(random, triangle, pose, out var pointOnTriangle);
                    rayDirection = pointOnTriangle - rayOrigin;
                } while (rayDirection.LengthSquared() < 1e-9f);
                float orientationLengthSquared;
                while (true)
                {
                    pose.Orientation.X = (float)random.NextDouble() * 2 - 1;
                    pose.Orientation.Y = (float)random.NextDouble() * 2 - 1;
                    pose.Orientation.Z = (float)random.NextDouble() * 2 - 1;
                    pose.Orientation.W = (float)random.NextDouble() * 2 - 1;
                    orientationLengthSquared = pose.Orientation.LengthSquared();
                    if (orientationLengthSquared > 1e-7f)
                        break;
                }
                BepuUtilities.Quaternion.Scale(ref pose.Orientation, 1f / (float)Math.Sqrt(orientationLengthSquared), out pose.Orientation);
                
                TestRay(triangle, pose, rayOrigin, rayDirection, true);
            }

        }


    }
}
