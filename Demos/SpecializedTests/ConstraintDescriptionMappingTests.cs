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
        public static void Test<T>(Simulation simulation, Random random, int constraintTypeBodyCount) where T : struct, IConstraintDescription<T>
        {
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
                        bodyHandleForConstraint = simulation.Bodies.IndexToHandle[random.Next(simulation.Bodies.Count)];
                    } while (Array.IndexOf(constraintBodyHandles, bodyHandleForConstraint, 0, indexInConstraint) >= 0);

                    constraintBodyHandles[indexInConstraint] = bodyHandleForConstraint;
                }

                ref var source = ref sources[constraintTestIndex];
                FillWithRandomBytes(ref source, random);
                constraintHandles[constraintTestIndex] = simulation.Add(ref constraintBodyHandles[0], constraintTypeBodyCount, ref source);

            }
            for (int constraintTestIndex = 0; constraintTestIndex < constraintTestCount; ++constraintTestIndex)
            {
                simulation.Solver.GetDescription(constraintHandles[constraintTestIndex], out T description);
                var aValue = (ValueType)description;
                var bValue = (ValueType)sources[constraintTestIndex];
                if (!Equals(typeof(T).Name, typeof(T), aValue, bValue))
                    break;
                simulation.RemoveConstraint(constraintHandles[constraintTestIndex]);
            }
        }

        static bool Equals(string parentString, Type type, ValueType a, ValueType b)
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
                if (!Equals(nameString, field.FieldType, aValue, bValue))
                    equals = false;
            }
            return equals;
        }

        public static void Test()
        {
            var pool = new BufferPool();
            var simulation = Simulation.Create(pool, new TestCallbacks());


            const int bodyCount = 2048;
            for (int i = 0; i < bodyCount; ++i)
            {
                var bodyDescription = new BodyDescription { LocalInertia = new BodyInertia { InverseMass = 1 }, Pose = new RigidPose { Orientation = BepuUtilities.Quaternion.Identity } };
                simulation.Add(ref bodyDescription);
            }
            var random = new Random(5);
            Test<Contact1>(simulation, random, 2);
            Test<Contact4>(simulation, random, 2);
            Test<BallSocket>(simulation, random, 2);

            pool.Clear();
        }
    }
}
