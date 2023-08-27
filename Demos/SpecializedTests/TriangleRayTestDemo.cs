﻿using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using System;
using System.Diagnostics;
using System.Numerics;

namespace Demos.SpecializedTests
{
    public class TriangleRayTestDemo : Demo
    {
        void GetPointOnTriangle(Random random, in Triangle triangle, in RigidPose pose, out Vector3 pointOnTriangle)
        {
            float total;
            float a, b, c;
            do
            {
                a = random.NextSingle();
                b = random.NextSingle();
                c = random.NextSingle();
                total = a + b + c;
            } while (total < 1e-7f);
            var inverseTotal = 1f / total;
            a *= inverseTotal;
            b *= inverseTotal;
            c *= inverseTotal;

            var localP = triangle.A * a + triangle.B * b + triangle.C * c;
            BepuUtilities.QuaternionEx.TransformWithoutOverlap(localP, pose.Orientation, out pointOnTriangle);
            pointOnTriangle += pose.Position;
        }

        void GetPointOutsideTriangle(Random random, in Triangle triangle, in RigidPose pose, out Vector3 pointOutsideTriangle)
        {
            //This is a pretty biased random generator, but that's fine.
            var edgeIndex = random.Next(3);
            Vector3 borderPoint;
            switch (edgeIndex)
            {
                case 0:
                    borderPoint = triangle.A + (triangle.B - triangle.A) * random.NextSingle();
                    break;
                case 1:
                    borderPoint = triangle.A + (triangle.C - triangle.A) * random.NextSingle();
                    break;
                default:
                    borderPoint = triangle.B + (triangle.C - triangle.B) * random.NextSingle();
                    break;
            }
            var center = (triangle.A + triangle.B + triangle.C) / 3f;
            var offsetToBorder = borderPoint - center;
            var localP = center + offsetToBorder * (1.01f + 4 * random.NextSingle());

            BepuUtilities.QuaternionEx.TransformWithoutOverlap(localP, pose.Orientation, out pointOutsideTriangle);
            pointOutsideTriangle += pose.Position;
        }

        void TestRay(in Triangle triangle, in RigidPose pose, Vector3 rayOrigin, Vector3 rayDirection, bool expectedImpact, Vector3 pointOnTrianglePlane)
        {
            var hit = triangle.RayTest(pose, rayOrigin, rayDirection, out var t, out var normal);

            TriangleWide wide = default;
            wide.Broadcast(triangle);
            RigidPoseWide.Broadcast(pose, out var poses);
            RayWide rayWide;
            Vector3Wide.Broadcast(rayOrigin, out rayWide.Origin);
            Vector3Wide.Broadcast(rayDirection, out rayWide.Direction);
            wide.RayTest(ref poses, ref rayWide, out var intersectedWide, out var tWide, out var normalWide);

            Debug.Assert(expectedImpact == hit);
            Debug.Assert(hit == intersectedWide[0] < 0);
            if (hit)
            {
                Debug.Assert(Math.Abs(t - tWide[0]) < 1e-7f);
                Vector3Wide.ReadSlot(ref normalWide, 0, out var normalWideLane0);
                var normalDot = Vector3.Dot(normalWideLane0, normal);
                Debug.Assert(normalDot > 0.9999f && normalDot < 1.00001f);
                var hitLocationError = rayDirection * t + (rayOrigin - pointOnTrianglePlane);
                Debug.Assert(hitLocationError.Length() < 1e-2f * MathF.Max(pointOnTrianglePlane.Length(), rayDirection.Length()));

            }
        }

        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(-30, 8, -60);
            camera.Yaw = MathHelper.Pi * 3f / 4;

            Simulation = Simulation.Create(BufferPool, new DemoNarrowPhaseCallbacks(new SpringSettings(30, 1)), new DemoPoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));

            Triangle triangle;
            triangle.A = new Vector3(0, 0, 0);
            triangle.B = new Vector3(1, 0, 0);
            triangle.C = new Vector3(0, 0, 1);
            var pose = new RigidPose
            {
                Orientation = Quaternion.Identity,
                Position = new Vector3(0)
            };
            var rayOrigin = new Vector3(1f / 3f, 5, 1 / 3f);
            var rayDirection = new Vector3(0, -1, 0);

            //The other convex ray tester doesn't quite map well to infinitely thin triangles, so we have our own little tester here.
            TestRay(triangle, pose, rayOrigin, rayDirection, true, new Vector3(rayOrigin.X, 0, rayOrigin.Z));

            Random random = new Random(5);
            const float shapeMin = -50;
            const float shapeSpan = 100;
            for (int i = 0; i < 10000; ++i)
            {
                triangle.A = new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle()) * shapeSpan + new Vector3(shapeMin);
                triangle.B = new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle()) * shapeSpan + new Vector3(shapeMin);
                triangle.C = new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle()) * shapeSpan + new Vector3(shapeMin);
                
                var localTriangleCenter = (triangle.A + triangle.B + triangle.C) / 3f;
                triangle.A -= localTriangleCenter;
                triangle.B -= localTriangleCenter;
                triangle.C -= localTriangleCenter;

                rayOrigin = new Vector3(random.NextSingle(), random.NextSingle(), random.NextSingle()) * shapeSpan + new Vector3(shapeMin);

                float orientationLengthSquared;
                while (true)
                {
                    pose.Orientation.X = random.NextSingle() * 2 - 1;
                    pose.Orientation.Y = random.NextSingle() * 2 - 1;
                    pose.Orientation.Z = random.NextSingle() * 2 - 1;
                    pose.Orientation.W = random.NextSingle() * 2 - 1;
                    orientationLengthSquared = pose.Orientation.LengthSquared();
                    if (orientationLengthSquared > 1e-7f)
                        break;
                }
                BepuUtilities.QuaternionEx.Scale(pose.Orientation, 1f / (float)Math.Sqrt(orientationLengthSquared), out pose.Orientation);
                pose.Position = BepuUtilities.QuaternionEx.Transform(localTriangleCenter, pose.Orientation);

                var normal = Vector3.Cross(triangle.C - triangle.A, triangle.B - triangle.A);
                var normalLength = normal.Length();
                if (normalLength < 1e-7f)
                    continue;
                normal /= normalLength;


                BepuUtilities.QuaternionEx.Transform(normal, pose.Orientation, out normal);
                Vector3 pointOnTriangle;
                do
                {
                    GetPointOnTriangle(random, triangle, pose, out pointOnTriangle);
                    rayDirection = pointOnTriangle - rayOrigin;
                } while (rayDirection.LengthSquared() < 1e-9f);
                rayDirection *= (0.5f + 10 * random.NextSingle()) / rayDirection.Length();
                var shouldHit = Vector3.Dot(rayDirection, normal) < 0;
                TestRay(triangle, pose, rayOrigin, rayDirection, shouldHit, pointOnTriangle);

                Vector3 pointOutsideTriangle;
                do
                {
                    GetPointOutsideTriangle(random, triangle, pose, out pointOutsideTriangle);
                    rayDirection = pointOutsideTriangle - rayOrigin;
                } while (rayDirection.LengthSquared() < 1e-9f);
                rayDirection *= (0.5f + 10 * random.NextSingle()) / rayDirection.Length();
                TestRay(triangle, pose, rayOrigin, rayDirection, false, pointOutsideTriangle);
            }

        }


    }
}
