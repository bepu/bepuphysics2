using BepuUtilities;
using System;
using System.Numerics;

namespace BEPUutilitiesTests
{
    public static class SymmetricTests
    {
        public static float TestAddition(int iterationCount)
        {
            Symmetric3x3 m0 = new() { XX = 4, YY = 1, ZZ = 3, YX = 5, ZX = 6, ZY = 4 };
            Symmetric3x3 m1 = new() { XX = 1, YY = 2, ZZ = 1, YX = -1, ZX = 14, ZY = 8 };
            for (int i = 0; i < iterationCount; ++i)
            {
                Symmetric3x3.Add(m0, m0, out m0);
                Symmetric3x3.Add(m1, m1, out m1);
                Symmetric3x3.Add(m0, m1, out m0);
                Symmetric3x3.Add(m0, m0, out m0);
                Symmetric3x3.Add(m1, m1, out m1);
                Symmetric3x3.Add(m0, m1, out m0);
                Symmetric3x3.Add(m0, m0, out m0);
                Symmetric3x3.Add(m1, m1, out m1);
                Symmetric3x3.Add(m0, m1, out m0);
            }
            return m0.XX;
        }
        public static float TestOperatorAddition(int iterationCount)
        {
            Symmetric3x3 m0 = new() { XX = 4, YY = 1, ZZ = 3, YX = 5, ZX = 6, ZY = 4 };
            Symmetric3x3 m1 = new() { XX = 1, YY = 2, ZZ = 1, YX = -1, ZX = 14, ZY = 8 };
            for (int i = 0; i < iterationCount; ++i)
            {
                m0 = m0 + m0;
                m1 = m1 + m1;
                m0 = m0 + m1;
                m0 = m0 + m0;
                m1 = m1 + m1;
                m0 = m0 + m1;
                m0 = m0 + m0;
                m1 = m1 + m1;
                m0 = m0 + m1;
            }
            return m0.XX;
        }

        public static float TestMultiplication(int iterationCount)
        {
            Symmetric3x3 m0 = new() { XX = .5f, YY = 0.75f, ZZ = 0.25f, YX = 1.1f, ZX = 1.2f, ZY = 0.3f };
            Symmetric3x3 m1;
            Symmetric3x3 m2 = new() { XX = .025f, YY = 0.0575f, ZZ = 0.0425f, YX = .1f, ZX = .2f, ZY = 0.53f };
            for (int i = 0; i < iterationCount; ++i)
            {
                Symmetric3x3.MultiplyWithoutOverlap(m0, m2, out m1);
                Symmetric3x3.MultiplyWithoutOverlap(m1, m2, out m0);
                Symmetric3x3.MultiplyWithoutOverlap(m0, m1, out m2);
                Symmetric3x3.MultiplyWithoutOverlap(m0, m2, out m1);
                Symmetric3x3.MultiplyWithoutOverlap(m1, m2, out m0);
                Symmetric3x3.MultiplyWithoutOverlap(m0, m1, out m2);
                Symmetric3x3.MultiplyWithoutOverlap(m0, m2, out m1);
                Symmetric3x3.MultiplyWithoutOverlap(m1, m2, out m0);
                Symmetric3x3.MultiplyWithoutOverlap(m0, m1, out m2);
            }
            return m0.XX;
        }
        public static float TestOperatorMultiplication(int iterationCount)
        {
            Symmetric3x3 m0 = new() { XX = .5f, YY = 0.75f, ZZ = 0.25f, YX = 1.1f, ZX = 1.2f, ZY = 0.3f };
            Symmetric3x3 m1;
            Symmetric3x3 m2 = new() { XX = .025f, YY = 0.0575f, ZZ = 0.0425f, YX = .1f, ZX = .2f, ZY = 0.53f };
            for (int i = 0; i < iterationCount; ++i)
            {
                m1 = m0 * m2;
                m0 = m1 * m2;
                m2 = m0 * m1;
                m1 = m0 * m2;
                m0 = m1 * m2;
                m2 = m0 * m1;
                m1 = m0 * m2;
                m0 = m1 * m2;
                m2 = m0 * m1;
            }
            return m0.XX;
        }


        public unsafe static void Test()
        {
            Console.WriteLine("Symmetric3x3 RESULTS:");
            Console.WriteLine($"Size: {sizeof(Symmetric3x3)}");
            const int iterationCount = 10000000;

            var a = new Matrix3x3 { X = new Vector3(1, 2, 3), Y = new Vector3(2, 3, 4), Z = new Vector3(-3, -4, -5) };
            var b = new Symmetric3x3 { XX = 1, YX = 2, ZX = 3, YY = 4, ZY = 5, ZZ = 6 };
            var c = a * b;
            Symmetric3x3.Multiply(a, b, out var c2);
            var d = b * a;
            var b2 = new Matrix3x3 { X = new Vector3(b.XX, b.YX, b.ZX), Y = new Vector3(b.YX, b.YY, b.ZY), Z = new Vector3(b.ZX, b.ZY, b.ZZ) };
            var d2 = b2 * a;

            Helper.Test("Symmetric3x3.Add", TestAddition, iterationCount);
            Helper.Test("Symmetric3x3.+", TestOperatorAddition, iterationCount);

            Helper.Test("Symmetric3x3.MultiplyWithoutOverlap", TestMultiplication, iterationCount);
            Helper.Test("Symmetric3x3.*", TestOperatorMultiplication, iterationCount);

        }
    }
}
