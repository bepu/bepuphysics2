using BepuUtilities.Memory;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuPhysics.Constraints;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using BepuUtilities;
using BepuPhysics.Constraints.Contact;

namespace Demos.SpecializedTests
{

    public unsafe class ConstraintDescriptionMappingTests
    {

        static void FillWithRandomBytes<T>(ref T item, Random random) where T : struct
        {
            ref var bytes = ref Unsafe.As<T, byte>(ref item);
            for (int i = 0; i < Unsafe.SizeOf<T>(); ++i)
            {
                Unsafe.Add(ref bytes, i) = (byte)random.Next(256);
            }
        }
        public static void Test<T>(BufferPool pool, Random random, int constraintTypeBodyCount) where T : struct, IConstraintDescription<T>
        {
            var simulation = Simulation.Create(pool, new DemoNarrowPhaseCallbacks(), new DemoPoseIntegratorCallbacks());


            const int bodyCount = 2048;
            for (int i = 0; i < bodyCount; ++i)
            {
                var bodyDescription = new BodyDescription { LocalInertia = new BodyInertia { InverseMass = 1 }, Pose = new RigidPose { Orientation = BepuUtilities.Quaternion.Identity } };
                simulation.Bodies.Add(bodyDescription);
            }

            int constraintTestCount = Vector<float>.Count * 128;

            int[] constraintBodyHandles = new int[constraintTypeBodyCount];
            int[] constraintHandles = new int[constraintTestCount];
            T[] sources = new T[constraintTestCount];


            var size = Unsafe.SizeOf<T>();
            //Generate a set of constraints whose descriptions are filled with fuzzed data, referencing random bodies.
            for (int constraintTestIndex = 0; constraintTestIndex < constraintTestCount; ++constraintTestIndex)
            {
                for (int indexInConstraint = 0; indexInConstraint < constraintTypeBodyCount; ++indexInConstraint)
                {
                    int bodyHandleForConstraint;
                    do
                    {
                        bodyHandleForConstraint = simulation.Bodies.ActiveSet.IndexToHandle[random.Next(simulation.Bodies.ActiveSet.Count)];
                    } while (Array.IndexOf(constraintBodyHandles, bodyHandleForConstraint, 0, indexInConstraint) >= 0);

                    constraintBodyHandles[indexInConstraint] = bodyHandleForConstraint;
                }

                ref var source = ref sources[constraintTestIndex];
                FillWithRandomBytes(ref source, random);
                constraintHandles[constraintTestIndex] = simulation.Solver.Add(ref constraintBodyHandles[0], constraintTypeBodyCount, ref source);

            }
            for (int constraintTestIndex = 0; constraintTestIndex < constraintTestCount; ++constraintTestIndex)
            {
                simulation.Solver.GetDescription(constraintHandles[constraintTestIndex], out T description);
                var aValue = (ValueType)description;
                var bValue = (ValueType)sources[constraintTestIndex];
                CheckEquality(typeof(T).Name, typeof(T), aValue, bValue);
            }
            simulation.Dispose();
        }

        static bool CheckEquality(string parentString, Type type, ValueType a, ValueType b)
        {
            if (type.IsPrimitive)
            {
                if (!a.Equals(b))
                {
                    Console.WriteLine($"{parentString} doesn't match: {a} vs {b}");
                    return false;
                }
                return true;
            }
            bool equals = true;
            foreach (var field in type.GetFields(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic))
            {
                var aValue = (ValueType)field.GetValue(a);
                var bValue = (ValueType)field.GetValue(b);
                var nameString = $"{parentString}.{field.Name}";
                if (!CheckEquality(nameString, field.FieldType, aValue, bValue))
                    equals = false;
            }
            return equals;
        }

        public static void Test()
        {
            var pool = new BufferPool();            
            var random = new Random(5);
            Test<Contact1OneBody>(pool, random, 1);
            Test<Contact2OneBody>(pool, random, 1);
            Test<Contact3OneBody>(pool, random, 1);
            Test<Contact4OneBody>(pool, random, 1);
            Test<Contact1>(pool, random, 2);
            Test<Contact2>(pool, random, 2);
            Test<Contact3>(pool, random, 2);
            Test<Contact4>(pool, random, 2);
            Test<Contact2NonconvexOneBody>(pool, random, 1);
            Test<Contact3NonconvexOneBody>(pool, random, 1);
            Test<Contact4NonconvexOneBody>(pool, random, 1);
            //Test<Contact5NonconvexOneBody>(pool, random, 1);
            //Test<Contact6NonconvexOneBody>(pool, random, 1);
            //Test<Contact7NonconvexOneBody>(pool, random, 1);
            //Test<Contact8NonconvexOneBody>(pool, random, 1);
            Test<Contact2Nonconvex>(pool, random, 2);
            Test<Contact3Nonconvex>(pool, random, 2);
            Test<Contact4Nonconvex>(pool, random, 2);
            //Test<Contact5Nonconvex>(pool, random, 2);
            //Test<Contact6Nonconvex>(pool, random, 2);
            //Test<Contact7Nonconvex>(pool, random, 2);
            //Test<Contact8Nonconvex>(pool, random, 2);

            Test<BallSocket>(pool, random, 2);
            Test<AngularHinge>(pool, random, 2);
            Test<AngularSwivelHinge>(pool, random, 2);
            Test<SwingLimit>(pool, random, 2);
            Test<TwistServo>(pool, random, 2);
            Test<TwistLimit>(pool, random, 2);
            Test<TwistMotor>(pool, random, 2);
            Test<AngularServo>(pool, random, 2);
            Test<AngularMotor>(pool, random, 2);
            Test<Weld>(pool, random, 2);
            Test<VolumeConstraint>(pool, random, 4);
            Test<DistanceServo>(pool, random, 2);
            Test<DistanceLimit>(pool, random, 2);
            Test<CenterDistanceConstraint>(pool, random, 2);
            Test<AreaConstraint>(pool, random, 3);
            Test<PointOnLineServo>(pool, random, 2);
            Test<LinearAxisServo>(pool, random, 2);
            Test<LinearAxisMotor>(pool, random, 2);
            Test<LinearAxisLimit>(pool, random, 2);
            Test<AngularAxisMotor>(pool, random, 2);
            Test<OneBodyAngularServo>(pool, random, 1);
            Test<OneBodyAngularMotor>(pool, random, 1);
            Test<OneBodyLinearServo>(pool, random, 1);
            Test<OneBodyLinearMotor>(pool, random, 1);

            pool.Clear();
        }
    }
}
