using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection.SweepTasks;
using BepuUtilities;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics.CollisionDetection.CollisionTasks
{
    public static class GradientRefine<TShapeA, TShapeWideA, TSupportFinderA, TShapeB, TShapeWideB, TSupportFinderB>
        where TShapeA : IConvexShape
        where TShapeWideA : IShapeWide<TShapeA>
        where TSupportFinderA : ISupportFinder<TShapeA, TShapeWideA>
        where TShapeB : IConvexShape
        where TShapeWideB : IShapeWide<TShapeB>
        where TSupportFinderB : ISupportFinder<TShapeB, TShapeWideB>
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void ComputeGradient(in TShapeWideA a, in TShapeWideB b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB, ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB, in Vector3Wide localNormal, out Vector3Wide gradient)
        {
            //Normals are always calibrated to point from B to A.
            supportFinderB.ComputeSupport(b, localOrientationB, localNormal, out var extremeB);
            Vector3Wide.Add(extremeB, localOffsetB, out extremeB);
            Vector3Wide.Negate(localNormal, out var negatedNormal);
            supportFinderA.ComputeLocalSupport(a, negatedNormal, out var extremeA);

            Vector3Wide.Subtract(extremeB, extremeA, out gradient);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void NarrowSearchSpace(in Vector3Wide localNormal, in Vector3Wide gradient, ref Vector3Wide min, ref Vector3Wide max)
        {
            var xNegative = Vector.LessThan(gradient.X, Vector<float>.Zero);
            var yNegative = Vector.LessThan(gradient.Y, Vector<float>.Zero);
            var zNegative = Vector.LessThan(gradient.Z, Vector<float>.Zero);
            min.X = Vector.ConditionalSelect(xNegative, localNormal.X, min.X);
            min.Y = Vector.ConditionalSelect(yNegative, localNormal.Y, min.Y);
            min.Z = Vector.ConditionalSelect(zNegative, localNormal.Z, min.Z);
            max.X = Vector.ConditionalSelect(xNegative, max.X, localNormal.X);
            max.Y = Vector.ConditionalSelect(yNegative, max.Y, localNormal.Y);
            max.Z = Vector.ConditionalSelect(zNegative, max.Z, localNormal.Z);
        }

        public static void Refine(
            in TShapeWideA a, in TShapeWideB b, in Vector3Wide localOffsetB, in Matrix3x3Wide localOrientationB,
            ref TSupportFinderA supportFinderA, ref TSupportFinderB supportFinderB, in Vector3Wide initialGuess, in Vector<float> searchSize, out Vector3Wide localNormal)
        {
            localNormal = initialGuess;
            var rawNormal = initialGuess;
            Vector3Wide min, max;
            min.X = localNormal.X - searchSize;
            min.Y = localNormal.Y - searchSize;
            min.Z = localNormal.Z - searchSize;
            max.X = localNormal.X + searchSize;
            max.Y = localNormal.Y + searchSize;
            max.Z = localNormal.Z + searchSize;

            for (int i = 0; i < 10; ++i)
            {
                ComputeGradient(a, b, localOffsetB, localOrientationB, ref supportFinderA, ref supportFinderB, localNormal, out var gradient);        

                NarrowSearchSpace(rawNormal, gradient, ref min, ref max);
                Vector3Wide.Add(min, max, out rawNormal);
                Vector3Wide.Scale(rawNormal, new Vector<float>(0.5f), out rawNormal);
                //TODO: may not require normalization depending on support finders.
                Vector3Wide.Normalize(rawNormal, out localNormal);
            }


        }
    }
}
