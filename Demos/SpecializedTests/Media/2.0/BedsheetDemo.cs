﻿using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using System;
using System.Numerics;

namespace Demos.Demos.Media
{

    /// <summary>
    /// Shows a few different examples of cloth-ish constraint lattices.
    /// </summary>
    public class BedsheetDemo : Demo
    {
        delegate bool KinematicDecider(int rowIndex, int columnIndex, int width, int height);

        BodyHandle[,] CreateBodyGrid(Vector3 position, Quaternion orientation, int width, int height, float spacing, float bodyRadius, float massPerBody,
            int instanceId, CollidableProperty<ClothCollisionFilter> filters, KinematicDecider isKinematic)
        {
            var description = BodyDescription.CreateKinematic(orientation, Simulation.Shapes.Add(new Sphere(bodyRadius)), 0.01f);
            var inverseMass = 1f / massPerBody;
            BodyHandle[,] handles = new BodyHandle[height, width];
            for (int rowIndex = 0; rowIndex < height; ++rowIndex)
            {
                for (int columnIndex = 0; columnIndex < width; ++columnIndex)
                {
                    description.LocalInertia.InverseMass = isKinematic(rowIndex, columnIndex, width, height) ? 0 : inverseMass;
                    var localPosition = new Vector3(columnIndex * spacing, rowIndex * -spacing, 0);
                    QuaternionEx.TransformWithoutOverlap(localPosition, orientation, out var rotatedPosition);
                    description.Pose.Position = rotatedPosition + position;
                    var handle = Simulation.Bodies.Add(description);
                    handles[rowIndex, columnIndex] = handle;
                    filters.Allocate(handle) = new ClothCollisionFilter(rowIndex, columnIndex, instanceId);
                }
            }
            return handles;
        }

        void CreateAreaConstraints(BodyHandle[,] bodyHandles, SpringSettings springSettings)
        {
            for (int rowIndex = 0; rowIndex < bodyHandles.GetLength(0) - 1; ++rowIndex)
            {
                for (int columnIndex = 0; columnIndex < bodyHandles.GetLength(1) - 1; ++columnIndex)
                {
                    var aHandle = bodyHandles[rowIndex, columnIndex];
                    var bHandle = bodyHandles[rowIndex + 1, columnIndex];
                    var cHandle = bodyHandles[rowIndex, columnIndex + 1];
                    var dHandle = bodyHandles[rowIndex + 1, columnIndex + 1];
                    var a = new BodyReference(aHandle, Simulation.Bodies);
                    var b = new BodyReference(bHandle, Simulation.Bodies);
                    var c = new BodyReference(cHandle, Simulation.Bodies);
                    var d = new BodyReference(dHandle, Simulation.Bodies);
                    //Not worried about kinematics here- we create at most one row of kinematics in this demo. These are three body constraints that operate in a local quad, so 
                    //there's no way for them to all be kinematic.
                    Simulation.Solver.Add(aHandle, bHandle, cHandle, new AreaConstraint(a.Pose.Position, b.Pose.Position, c.Pose.Position, springSettings));
                    Simulation.Solver.Add(bHandle, cHandle, dHandle, new AreaConstraint(b.Pose.Position, c.Pose.Position, d.Pose.Position, springSettings));
                }
            }
        }
        void CreateDistanceConstraints(BodyHandle[,] bodyHandles, SpringSettings springSettings)
        {
            void CreateConstraintBetweenBodies(BodyHandle aHandle, BodyHandle bHandle)
            {
                var a = new BodyReference(aHandle, Simulation.Bodies);
                var b = new BodyReference(bHandle, Simulation.Bodies);
                //Don't create constraints between two kinematic bodies.
                if (a.LocalInertia.InverseMass > 0 || b.LocalInertia.InverseMass > 0)
                {
                    //Note the use of a limit; the distance is allowed to go smaller.
                    //This helps stop the cloth from having unnatural rigidity.
                    var distance = Vector3.Distance(a.Pose.Position, b.Pose.Position);
                    Simulation.Solver.Add(aHandle, bHandle, new CenterDistanceLimit(distance * 0.15f, distance, springSettings));
                }
            }
            for (int rowIndex = 0; rowIndex < bodyHandles.GetLength(0); ++rowIndex)
            {
                for (int columnIndex = 0; columnIndex < bodyHandles.GetLength(1) - 1; ++columnIndex)
                {
                    CreateConstraintBetweenBodies(bodyHandles[rowIndex, columnIndex], bodyHandles[rowIndex, columnIndex + 1]);
                }
            }
            for (int rowIndex = 0; rowIndex < bodyHandles.GetLength(0) - 1; ++rowIndex)
            {
                for (int columnIndex = 0; columnIndex < bodyHandles.GetLength(1); ++columnIndex)
                {
                    CreateConstraintBetweenBodies(bodyHandles[rowIndex, columnIndex], bodyHandles[rowIndex + 1, columnIndex]);
                }
            }
            for (int rowIndex = 0; rowIndex < bodyHandles.GetLength(0) - 1; ++rowIndex)
            {
                for (int columnIndex = 0; columnIndex < bodyHandles.GetLength(1) - 1; ++columnIndex)
                {
                    CreateConstraintBetweenBodies(bodyHandles[rowIndex, columnIndex], bodyHandles[rowIndex + 1, columnIndex + 1]);
                    CreateConstraintBetweenBodies(bodyHandles[rowIndex, columnIndex + 1], bodyHandles[rowIndex + 1, columnIndex]);
                }
            }
        }

        RolloverInfo rolloverInfo;

        public override void Initialize(ContentArchive content, Camera camera)
        {
            camera.Position = new Vector3(70, 40, -80);
            camera.Yaw = -MathF.PI * 0.8f;
            camera.Pitch = MathF.PI * 0.1f;

            var filters = new CollidableProperty<ClothCollisionFilter>();
            Simulation = Simulation.Create(BufferPool, new ClothCallbacks(filters), new DemoPoseIntegratorCallbacks(new Vector3(0, -50, 0)), new SolveDescription(8, 1));
            rolloverInfo = new RolloverInfo();

            bool FullyDynamic(int rowIndex, int columnIndex, int width, int height)
            {
                return false;
            }

            int clothInstanceId = 0;
            var initialRotation = QuaternionEx.CreateFromAxisAngle(new Vector3(1, 0, 0), MathF.PI * -0.5f);




            Simulation.Statics.Add(new StaticDescription(new Vector3(0, 10, 0), Simulation.Shapes.Add(new Box(80, 20, 80))));
            Simulation.Statics.Add(new StaticDescription(new Vector3(-20, 22, 30), Simulation.Shapes.Add(new Box(34, 4, 14))));
            Simulation.Statics.Add(new StaticDescription(new Vector3(20, 22, 30), Simulation.Shapes.Add(new Box(34, 4, 14))));


            Simulation.Statics.Add(new StaticDescription(new Vector3(65.5f, 8f, 20), Simulation.Shapes.Add(new Cylinder(15, 15))));


            {
                var position = new Vector3(96 * 1.15f * -0.5f, 30, 86 * 1.15f * -0.5f);
                var handles = CreateBodyGrid(position, initialRotation, 96, 86, 1.15f, 1f, 1, clothInstanceId++, filters, FullyDynamic);
                CreateDistanceConstraints(handles, new SpringSettings(20, 1));
                CreateAreaConstraints(handles, new SpringSettings(30, 1));
            }

            {
                var position = new Vector3(65.5f + 56 * 0.8f * -0.5f, 25, 20 + 56 * 0.8f * -0.5f);
                var handles = CreateBodyGrid(position, initialRotation, 56, 56, 0.8f, 0.65f, 1, clothInstanceId++, filters, FullyDynamic);
                CreateDistanceConstraints(handles, new SpringSettings(20, 1));
                CreateAreaConstraints(handles, new SpringSettings(30, 1));
            }

            Simulation.Statics.Add(new StaticDescription(new Vector3(0, 0, 0), Simulation.Shapes.Add(new Box(400, 1, 400))));

        }

        public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
        {
            rolloverInfo.Render(renderer, camera, input, text, font);
            base.Render(renderer, camera, input, text, font);
        }

    }
}
