using BepuUtilities.Memory;
using System;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuPhysics.Constraints;
using BepuPhysics.Collidables;
using BepuPhysics.CollisionDetection;
using BepuUtilities;
using static BepuUtilities.GatherScatter;
using System.Runtime.Intrinsics.X86;
using System.Runtime.Intrinsics;

namespace BepuPhysics
{
    public partial class Bodies
    {

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void WriteGatherInertia(int index, int bodyIndexInBundle, ref Buffer<SolverState> states, ref BodyInertiaWide gatheredInertias)
        {
            ref var source = ref states[index].Inertia.World;
            ref var targetSlot = ref GetOffsetInstance(ref gatheredInertias, bodyIndexInBundle);
            GetFirst(ref targetSlot.InverseInertiaTensor.XX) = source.InverseInertiaTensor.XX;
            GetFirst(ref targetSlot.InverseInertiaTensor.YX) = source.InverseInertiaTensor.YX;
            GetFirst(ref targetSlot.InverseInertiaTensor.YY) = source.InverseInertiaTensor.YY;
            GetFirst(ref targetSlot.InverseInertiaTensor.ZX) = source.InverseInertiaTensor.ZX;
            GetFirst(ref targetSlot.InverseInertiaTensor.ZY) = source.InverseInertiaTensor.ZY;
            GetFirst(ref targetSlot.InverseInertiaTensor.ZZ) = source.InverseInertiaTensor.ZZ;
            GetFirst(ref targetSlot.InverseMass) = source.InverseMass;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void WriteGatherMotionState(int index, int bodyIndexInBundle, ref Buffer<SolverState> states,
            ref Vector3Wide position, ref QuaternionWide orientation, ref BodyVelocityWide velocity)
        {
            ref var state = ref states[index].Motion;
            Vector3Wide.WriteFirst(state.Pose.Position, ref GetOffsetInstance(ref position, bodyIndexInBundle));
            QuaternionWide.WriteFirst(state.Pose.Orientation, ref GetOffsetInstance(ref orientation, bodyIndexInBundle));
            Vector3Wide.WriteFirst(state.Velocity.Linear, ref GetOffsetInstance(ref velocity.Linear, bodyIndexInBundle));
            Vector3Wide.WriteFirst(state.Velocity.Angular, ref GetOffsetInstance(ref velocity.Angular, bodyIndexInBundle));
        }

        /// <summary>
        /// Gathers motion state information for a body bundle into an AOSOA bundle.
        /// </summary>
        /// <param name="references">Active body indices being gathered.</param>
        /// <param name="position">Gathered absolute position of the body.</param>
        /// <param name="orientation">Gathered orientation of the body.</param>
        /// <param name="velocity">Gathered velocity of the body.</param>
        /// <param name="inertia">Gathered inertia of the body.</param>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GatherState(ref Vector<int> references,
            out Vector3Wide position, out QuaternionWide orientation, out BodyVelocityWide velocity, out BodyInertiaWide inertia)
        {
            Unsafe.SkipInit(out position);
            Unsafe.SkipInit(out orientation);
            Unsafe.SkipInit(out velocity);
            Unsafe.SkipInit(out inertia);
            //TODO: This function and its users (which should be relatively few) is a problem for large world position precision.
            //It directly reports the position, thereby infecting vectorized logic with the high precision representation.
            //You might be able to redesign the users of this function to not need it, but that comes with its own difficulties
            //(for example, making the grab motor rely on having its goal offset updated every frame by the user).
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndex = ref Unsafe.As<Vector<int>, int>(ref references);

            for (int i = 0; i < Vector<int>.Count; ++i)
            {
                var index = Unsafe.Add(ref baseIndex, i);
                if (index < 0)
                    continue;
                WriteGatherMotionState(index, i, ref ActiveSet.SolverStates, ref position, ref orientation, ref velocity);
                WriteGatherInertia(index, i, ref ActiveSet.SolverStates, ref inertia);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe static void FallbackGatherMotionState(SolverState* states, Vector<int> baseIndex, ref Vector3Wide position, ref QuaternionWide orientation, ref BodyVelocityWide velocity)
        {
            var indices = (int*)Unsafe.AsPointer(ref baseIndex);
            var pPositionX = (float*)Unsafe.AsPointer(ref position.X);
            var pPositionY = (float*)Unsafe.AsPointer(ref position.Y);
            var pPositionZ = (float*)Unsafe.AsPointer(ref position.Z);
            var pOrientationX = (float*)Unsafe.AsPointer(ref orientation.X);
            var pOrientationY = (float*)Unsafe.AsPointer(ref orientation.Y);
            var pOrientationZ = (float*)Unsafe.AsPointer(ref orientation.Z);
            var pOrientationW = (float*)Unsafe.AsPointer(ref orientation.W);
            var pLinearX = (float*)Unsafe.AsPointer(ref velocity.Linear.X);
            var pLinearY = (float*)Unsafe.AsPointer(ref velocity.Linear.Y);
            var pLinearZ = (float*)Unsafe.AsPointer(ref velocity.Linear.Z);
            var pAngularX = (float*)Unsafe.AsPointer(ref velocity.Angular.X);
            var pAngularY = (float*)Unsafe.AsPointer(ref velocity.Angular.Y);
            var pAngularZ = (float*)Unsafe.AsPointer(ref velocity.Angular.Z);

            for (int i = 0; i < Vector<int>.Count; ++i)
            {
                var index = indices[i];
                if (index < 0)
                    continue;
                var stateValues = (float*)(states + index);
                pPositionX[i] = stateValues[MotionState.OffsetToPositionX];
                pPositionY[i] = stateValues[MotionState.OffsetToPositionY];
                pPositionZ[i] = stateValues[MotionState.OffsetToPositionZ];
                pOrientationX[i] = stateValues[MotionState.OffsetToOrientationX];
                pOrientationY[i] = stateValues[MotionState.OffsetToOrientationY];
                pOrientationZ[i] = stateValues[MotionState.OffsetToOrientationZ];
                pOrientationW[i] = stateValues[MotionState.OffsetToOrientationW];
                pLinearX[i] = stateValues[MotionState.OffsetToLinearX];
                pLinearY[i] = stateValues[MotionState.OffsetToLinearY];
                pLinearZ[i] = stateValues[MotionState.OffsetToLinearZ];
                pAngularX[i] = stateValues[MotionState.OffsetToAngularX];
                pAngularY[i] = stateValues[MotionState.OffsetToAngularY];
                pAngularZ[i] = stateValues[MotionState.OffsetToAngularZ];
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe static void FallbackGatherInertia(SolverState* states, Vector<int> baseIndex, ref BodyInertiaWide inertia, int offsetInFloats)
        {
            var indices = (int*)Unsafe.AsPointer(ref baseIndex);
            var pMass = (float*)Unsafe.AsPointer(ref inertia.InverseMass);
            var pInertiaXX = (float*)Unsafe.AsPointer(ref inertia.InverseInertiaTensor.XX);
            var pInertiaYX = (float*)Unsafe.AsPointer(ref inertia.InverseInertiaTensor.YX);
            var pInertiaYY = (float*)Unsafe.AsPointer(ref inertia.InverseInertiaTensor.YY);
            var pInertiaZX = (float*)Unsafe.AsPointer(ref inertia.InverseInertiaTensor.ZX);
            var pInertiaZY = (float*)Unsafe.AsPointer(ref inertia.InverseInertiaTensor.ZY);
            var pInertiaZZ = (float*)Unsafe.AsPointer(ref inertia.InverseInertiaTensor.ZZ);

            for (int i = 0; i < Vector<int>.Count; ++i)
            {
                var index = indices[i];
                if (index < 0)
                    continue;
                var inertiaValues = (float*)(states + index) + offsetInFloats;
                pInertiaXX[i] = inertiaValues[0];
                pInertiaYX[i] = inertiaValues[1];
                pInertiaYY[i] = inertiaValues[2];
                pInertiaZX[i] = inertiaValues[3];
                pInertiaZY[i] = inertiaValues[4];
                pInertiaZZ[i] = inertiaValues[5];
                pMass[i] = inertiaValues[6];
            }
        }

        public const int DoesntExistFlagIndex = 31;
        public const int KinematicFlagIndex = 30;
        public const uint KinematicMask = 1u << KinematicFlagIndex;
        /// <summary>
        /// Constraint body index references greater than a given unsigned value are either kinematic (1<<30 set) or correspond to an empty lane (1<<31 set).
        /// </summary>
        public const uint DynamicLimit = KinematicMask;
        public const uint BodyIndexMetadataMask = (1u << DoesntExistFlagIndex) | KinematicMask;
        public const int BodyIndexMask = (int)~BodyIndexMetadataMask;

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void GatherState<TAccessFilter>(Vector<int> encodedBodyIndices, bool worldInertia, out Vector3Wide position, out QuaternionWide orientation, out BodyVelocityWide velocity, out BodyInertiaWide inertia)
            where TAccessFilter : unmanaged, IBodyAccessFilter
        {
            var solverStates = ActiveSet.SolverStates.Memory;
            Unsafe.SkipInit(out TAccessFilter filter);
            if (Avx.IsSupported && Vector<float>.Count == 8)
            {
                var bodyIndices0 = encodedBodyIndices[0];
                var empty0 = bodyIndices0 < 0;
                var s0 = (float*)(solverStates + (bodyIndices0 & BodyIndexMask));
                var bodyIndices1 = encodedBodyIndices[1];
                var empty1 = bodyIndices1 < 0;
                var s1 = (float*)(solverStates + (bodyIndices1 & BodyIndexMask));
                var bodyIndices2 = encodedBodyIndices[2];
                var empty2 = bodyIndices2 < 0;
                var s2 = (float*)(solverStates + (bodyIndices2 & BodyIndexMask));
                var bodyIndices3 = encodedBodyIndices[3];
                var empty3 = bodyIndices3 < 0;
                var s3 = (float*)(solverStates + (bodyIndices3 & BodyIndexMask));
                var bodyIndices4 = encodedBodyIndices[4];
                var empty4 = bodyIndices4 < 0;
                var s4 = (float*)(solverStates + (bodyIndices4 & BodyIndexMask));
                var bodyIndices5 = encodedBodyIndices[5];
                var empty5 = bodyIndices5 < 0;
                var s5 = (float*)(solverStates + (bodyIndices5 & BodyIndexMask));
                var bodyIndices6 = encodedBodyIndices[6];
                var empty6 = bodyIndices6 < 0;
                var s6 = (float*)(solverStates + (bodyIndices6 & BodyIndexMask));
                var bodyIndices7 = encodedBodyIndices[7];
                var empty7 = bodyIndices7 < 0;
                var s7 = (float*)(solverStates + (bodyIndices7 & BodyIndexMask));

                //for (int i = 0; i < 8; ++i)
                //{
                //    s0[i] = i;
                //    s1[i] = i + 100;
                //    s2[i] = i + 200;
                //    s3[i] = i + 300;
                //    s4[i] = i + 400;
                //    s5[i] = i + 500;
                //    s6[i] = i + 600;
                //    s7[i] = i + 700;
                //}

                {
                    //Load every body for the first half of the motion state.
                    //Note that buffers are allocated on cache line boundaries, so we can use aligned loads for all that matters.
                    var m0 = empty0 ? Vector256<float>.Zero : Avx.LoadAlignedVector256(s0);
                    var m1 = empty1 ? Vector256<float>.Zero : Avx.LoadAlignedVector256(s1);
                    var m2 = empty2 ? Vector256<float>.Zero : Avx.LoadAlignedVector256(s2);
                    var m3 = empty3 ? Vector256<float>.Zero : Avx.LoadAlignedVector256(s3);
                    var m4 = empty4 ? Vector256<float>.Zero : Avx.LoadAlignedVector256(s4);
                    var m5 = empty5 ? Vector256<float>.Zero : Avx.LoadAlignedVector256(s5);
                    var m6 = empty6 ? Vector256<float>.Zero : Avx.LoadAlignedVector256(s6);
                    var m7 = empty7 ? Vector256<float>.Zero : Avx.LoadAlignedVector256(s7);

                    var n0 = Avx.UnpackLow(m0, m1);
                    var n1 = Avx.UnpackLow(m2, m3);
                    var n2 = Avx.UnpackLow(m4, m5);
                    var n3 = Avx.UnpackLow(m6, m7);
                    var n4 = Avx.UnpackHigh(m0, m1);
                    var n5 = Avx.UnpackHigh(m2, m3);
                    var n6 = Avx.UnpackHigh(m4, m5);
                    var n7 = Avx.UnpackHigh(m6, m7);

                    var o0 = Avx.Shuffle(n0, n1, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                    var o1 = Avx.Shuffle(n2, n3, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                    var o2 = Avx.Shuffle(n4, n5, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                    var o3 = Avx.Shuffle(n6, n7, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                    var o4 = Avx.Shuffle(n0, n1, 2 | (3 << 2) | (2 << 4) | (3 << 6));
                    var o5 = Avx.Shuffle(n2, n3, 2 | (3 << 2) | (2 << 4) | (3 << 6));
                    var o6 = Avx.Shuffle(n4, n5, 2 | (3 << 2) | (2 << 4) | (3 << 6));
                    var o7 = Avx.Shuffle(n6, n7, 2 | (3 << 2) | (2 << 4) | (3 << 6));

                    if (filter.GatherOrientation)
                    {
                        orientation.X = Avx.Permute2x128(o0, o1, 0 | (2 << 4)).AsVector();
                        orientation.Y = Avx.Permute2x128(o4, o5, 0 | (2 << 4)).AsVector();
                        orientation.Z = Avx.Permute2x128(o2, o3, 0 | (2 << 4)).AsVector();
                        orientation.W = Avx.Permute2x128(o6, o7, 0 | (2 << 4)).AsVector();
                    }
                    else
                    {
                        Unsafe.SkipInit(out orientation);
                    }
                    if (filter.GatherPosition)
                    {
                        position.X = Avx.Permute2x128(o0, o1, 1 | (3 << 4)).AsVector();
                        position.Y = Avx.Permute2x128(o4, o5, 1 | (3 << 4)).AsVector();
                        position.Z = Avx.Permute2x128(o2, o3, 1 | (3 << 4)).AsVector();
                    }
                    else
                    {
                        Unsafe.SkipInit(out position);
                    }
                }

                {
                    //Second half.
                    var m0 = empty0 ? Vector256<float>.Zero : Avx.LoadAlignedVector256(s0 + 8);
                    var m1 = empty1 ? Vector256<float>.Zero : Avx.LoadAlignedVector256(s1 + 8);
                    var m2 = empty2 ? Vector256<float>.Zero : Avx.LoadAlignedVector256(s2 + 8);
                    var m3 = empty3 ? Vector256<float>.Zero : Avx.LoadAlignedVector256(s3 + 8);
                    var m4 = empty4 ? Vector256<float>.Zero : Avx.LoadAlignedVector256(s4 + 8);
                    var m5 = empty5 ? Vector256<float>.Zero : Avx.LoadAlignedVector256(s5 + 8);
                    var m6 = empty6 ? Vector256<float>.Zero : Avx.LoadAlignedVector256(s6 + 8);
                    var m7 = empty7 ? Vector256<float>.Zero : Avx.LoadAlignedVector256(s7 + 8);

                    var n0 = Avx.UnpackLow(m0, m1);
                    var n1 = Avx.UnpackLow(m2, m3);
                    var n2 = Avx.UnpackLow(m4, m5);
                    var n3 = Avx.UnpackLow(m6, m7);
                    var n4 = Avx.UnpackHigh(m0, m1);
                    var n5 = Avx.UnpackHigh(m2, m3);
                    var n6 = Avx.UnpackHigh(m4, m5);
                    var n7 = Avx.UnpackHigh(m6, m7);

                    var o0 = Avx.Shuffle(n0, n1, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                    var o1 = Avx.Shuffle(n2, n3, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                    var o2 = Avx.Shuffle(n4, n5, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                    var o3 = Avx.Shuffle(n6, n7, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                    var o4 = Avx.Shuffle(n0, n1, 2 | (3 << 2) | (2 << 4) | (3 << 6));
                    var o5 = Avx.Shuffle(n2, n3, 2 | (3 << 2) | (2 << 4) | (3 << 6));

                    if (filter.AccessLinearVelocity)
                    {
                        velocity.Linear.X = Avx.Permute2x128(o0, o1, 0 | (2 << 4)).AsVector();
                        velocity.Linear.Y = Avx.Permute2x128(o4, o5, 0 | (2 << 4)).AsVector();
                        velocity.Linear.Z = Avx.Permute2x128(o2, o3, 0 | (2 << 4)).AsVector();
                    }
                    else
                    {
                        Unsafe.SkipInit(out velocity.Linear);
                    }
                    if (filter.AccessAngularVelocity)
                    {
                        velocity.Angular.X = Avx.Permute2x128(o0, o1, 1 | (3 << 4)).AsVector();
                        velocity.Angular.Y = Avx.Permute2x128(o4, o5, 1 | (3 << 4)).AsVector();
                        velocity.Angular.Z = Avx.Permute2x128(o2, o3, 1 | (3 << 4)).AsVector();
                    }
                    else
                    {
                        Unsafe.SkipInit(out velocity.Angular);
                    }
                }

                {
                    var offsetInFloats = worldInertia ? 24 : 16;

                    //Load every inertia vector.
                    //Assuming here that most bodies are dynamic, so it's not worth the extra work extracting a dynamic-only mask to avoid loading some kinematic zeroes.
                    var m0 = empty0 ? Vector256<float>.Zero : Avx.LoadAlignedVector256(s0 + offsetInFloats);
                    var m1 = empty1 ? Vector256<float>.Zero : Avx.LoadAlignedVector256(s1 + offsetInFloats);
                    var m2 = empty2 ? Vector256<float>.Zero : Avx.LoadAlignedVector256(s2 + offsetInFloats);
                    var m3 = empty3 ? Vector256<float>.Zero : Avx.LoadAlignedVector256(s3 + offsetInFloats);
                    var m4 = empty4 ? Vector256<float>.Zero : Avx.LoadAlignedVector256(s4 + offsetInFloats);
                    var m5 = empty5 ? Vector256<float>.Zero : Avx.LoadAlignedVector256(s5 + offsetInFloats);
                    var m6 = empty6 ? Vector256<float>.Zero : Avx.LoadAlignedVector256(s6 + offsetInFloats);
                    var m7 = empty7 ? Vector256<float>.Zero : Avx.LoadAlignedVector256(s7 + offsetInFloats);

                    var n0 = Avx.UnpackLow(m0, m1);
                    var n1 = Avx.UnpackLow(m2, m3);
                    var n2 = Avx.UnpackLow(m4, m5);
                    var n3 = Avx.UnpackLow(m6, m7);
                    var n4 = Avx.UnpackHigh(m0, m1);
                    var n5 = Avx.UnpackHigh(m2, m3);
                    var n6 = Avx.UnpackHigh(m4, m5);
                    var n7 = Avx.UnpackHigh(m6, m7);

                    var o0 = Avx.Shuffle(n0, n1, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                    var o1 = Avx.Shuffle(n2, n3, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                    var o2 = Avx.Shuffle(n4, n5, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                    var o3 = Avx.Shuffle(n6, n7, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                    var o4 = Avx.Shuffle(n0, n1, 2 | (3 << 2) | (2 << 4) | (3 << 6));
                    var o5 = Avx.Shuffle(n2, n3, 2 | (3 << 2) | (2 << 4) | (3 << 6));
                    var o6 = Avx.Shuffle(n4, n5, 2 | (3 << 2) | (2 << 4) | (3 << 6));
                    var o7 = Avx.Shuffle(n6, n7, 2 | (3 << 2) | (2 << 4) | (3 << 6));

                    if (filter.GatherInertiaTensor)
                    {
                        inertia.InverseInertiaTensor.XX = Avx.Permute2x128(o0, o1, 0 | (2 << 4)).AsVector();
                        inertia.InverseInertiaTensor.YX = Avx.Permute2x128(o4, o5, 0 | (2 << 4)).AsVector();
                        inertia.InverseInertiaTensor.YY = Avx.Permute2x128(o2, o3, 0 | (2 << 4)).AsVector();
                        inertia.InverseInertiaTensor.ZX = Avx.Permute2x128(o6, o7, 0 | (2 << 4)).AsVector();
                        inertia.InverseInertiaTensor.ZY = Avx.Permute2x128(o0, o1, 1 | (3 << 4)).AsVector();
                        inertia.InverseInertiaTensor.ZZ = Avx.Permute2x128(o4, o5, 1 | (3 << 4)).AsVector();
                    }
                    else
                    {
                        Unsafe.SkipInit(out inertia.InverseInertiaTensor);
                    }
                    if (filter.GatherMass)
                    {
                        inertia.InverseMass = Avx.Permute2x128(o2, o3, 1 | (3 << 4)).AsVector();
                    }
                    else
                    {
                        Unsafe.SkipInit(out inertia.InverseMass);
                    }
                }

            }
            else
            {
                Unsafe.SkipInit(out position);
                Unsafe.SkipInit(out orientation);
                Unsafe.SkipInit(out velocity);
                Unsafe.SkipInit(out inertia);
                FallbackGatherMotionState(solverStates, encodedBodyIndices, ref position, ref orientation, ref velocity);
                FallbackGatherInertia(solverStates, encodedBodyIndices, ref inertia, worldInertia ? 24 : 16);
            }
        }


        ////[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //unsafe void GatherInertia<TAccessFilter>(ref Vector<int> bodyIndices, int count, int offsetInFloats, out BodyInertiaWide inertia) where TAccessFilter : unmanaged, IBodyAccessFilter
        //{
        //    var inertias = ActiveSet.SolverStates.Memory;
        //    if (Avx.IsSupported && Vector<float>.Count == 8)
        //    {
        //        Unsafe.SkipInit(out TAccessFilter filter);
        //        var indices = (int*)Unsafe.AsPointer(ref bodyIndices);
        //        {
        //            var s0 = (float*)(inertias + indices[0]) + offsetInFloats;
        //            var s1 = (float*)(inertias + indices[1]) + offsetInFloats;
        //            var s2 = (float*)(inertias + indices[2]) + offsetInFloats;
        //            var s3 = (float*)(inertias + indices[3]) + offsetInFloats;
        //            var s4 = (float*)(inertias + indices[4]) + offsetInFloats;
        //            var s5 = (float*)(inertias + indices[5]) + offsetInFloats;
        //            var s6 = (float*)(inertias + indices[6]) + offsetInFloats;
        //            var s7 = (float*)(inertias + indices[7]) + offsetInFloats;

        //            //Load every inertia vector.
        //            var m0 = Avx.LoadAlignedVector256(s0);
        //            var m1 = count > 1 ? Avx.LoadAlignedVector256(s1) : Vector256<float>.Zero;
        //            var m2 = count > 2 ? Avx.LoadAlignedVector256(s2) : Vector256<float>.Zero;
        //            var m3 = count > 3 ? Avx.LoadAlignedVector256(s3) : Vector256<float>.Zero;
        //            var m4 = count > 4 ? Avx.LoadAlignedVector256(s4) : Vector256<float>.Zero;
        //            var m5 = count > 5 ? Avx.LoadAlignedVector256(s5) : Vector256<float>.Zero;
        //            var m6 = count > 6 ? Avx.LoadAlignedVector256(s6) : Vector256<float>.Zero;
        //            var m7 = count > 7 ? Avx.LoadAlignedVector256(s7) : Vector256<float>.Zero;

        //            var n0 = Avx.UnpackLow(m0, m1);
        //            var n1 = Avx.UnpackLow(m2, m3);
        //            var n2 = Avx.UnpackLow(m4, m5);
        //            var n3 = Avx.UnpackLow(m6, m7);
        //            var n4 = Avx.UnpackHigh(m0, m1);
        //            var n5 = Avx.UnpackHigh(m2, m3);
        //            var n6 = Avx.UnpackHigh(m4, m5);
        //            var n7 = Avx.UnpackHigh(m6, m7);

        //            var o0 = Avx.Shuffle(n0, n1, 0 | (1 << 2) | (0 << 4) | (1 << 6));
        //            var o1 = Avx.Shuffle(n2, n3, 0 | (1 << 2) | (0 << 4) | (1 << 6));
        //            var o2 = Avx.Shuffle(n4, n5, 0 | (1 << 2) | (0 << 4) | (1 << 6));
        //            var o3 = Avx.Shuffle(n6, n7, 0 | (1 << 2) | (0 << 4) | (1 << 6));
        //            var o4 = Avx.Shuffle(n0, n1, 2 | (3 << 2) | (2 << 4) | (3 << 6));
        //            var o5 = Avx.Shuffle(n2, n3, 2 | (3 << 2) | (2 << 4) | (3 << 6));
        //            var o6 = Avx.Shuffle(n4, n5, 2 | (3 << 2) | (2 << 4) | (3 << 6));
        //            var o7 = Avx.Shuffle(n6, n7, 2 | (3 << 2) | (2 << 4) | (3 << 6));

        //            if (filter.GatherInertiaTensor)
        //            {
        //                inertia.InverseInertiaTensor.XX = Avx.Permute2x128(o0, o1, 0 | (2 << 4)).AsVector();
        //                inertia.InverseInertiaTensor.YX = Avx.Permute2x128(o4, o5, 0 | (2 << 4)).AsVector();
        //                inertia.InverseInertiaTensor.YY = Avx.Permute2x128(o2, o3, 0 | (2 << 4)).AsVector();
        //                inertia.InverseInertiaTensor.ZX = Avx.Permute2x128(o6, o7, 0 | (2 << 4)).AsVector();
        //                inertia.InverseInertiaTensor.ZY = Avx.Permute2x128(o0, o1, 1 | (3 << 4)).AsVector();
        //                inertia.InverseInertiaTensor.ZZ = Avx.Permute2x128(o4, o5, 1 | (3 << 4)).AsVector();
        //            }
        //            else
        //            {
        //                Unsafe.SkipInit(out inertia.InverseInertiaTensor);
        //            }
        //            if (filter.GatherMass)
        //            {
        //                inertia.InverseMass = Avx.Permute2x128(o2, o3, 1 | (3 << 4)).AsVector();
        //            }
        //            else
        //            {
        //                Unsafe.SkipInit(out inertia.InverseMass);
        //            }
        //        }
        //    }
        //    else
        //    {
        //        Unsafe.SkipInit(out inertia);
        //        FallbackGatherInertia(count, ActiveSet.SolverStates.Memory, ref bodyIndices, ref inertia, offsetInFloats);
        //    }
        //}

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public unsafe void GatherWorldInertia<TAccessFilter>(ref Vector<int> bodyIndices, int count, out BodyInertiaWide inertia) where TAccessFilter : unmanaged, IBodyAccessFilter
        //{
        //    GatherInertia<TAccessFilter>(ref bodyIndices, count, 24, out inertia);
        //}
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public unsafe void GatherLocalInertia<TAccessFilter>(ref Vector<int> bodyIndices, int count, out BodyInertiaWide inertia) where TAccessFilter : unmanaged, IBodyAccessFilter
        //{
        //    GatherInertia<TAccessFilter>(ref bodyIndices, count, 16, out inertia);
        //}

        /// <summary>
        /// Gathers orientations and relative positions for a two body bundle into an AOSOA bundle.
        /// </summary>
        /// <param name="references">Active body indices being gathered.</param>
        /// <param name="count">Number of body pairs in the bundle.</param>
        /// <param name="ab">Gathered offsets from body A to body B.</param>
        /// <param name="orientationA">Gathered orientation of body A.</param>
        /// <param name="orientationB">Gathered orientation of body B.</param>
        /// <param name="velocityA">Gathered velocity of body A.</param>
        /// <param name="velocityB">Gathered velocity of body B.</param>
        /// <param name="inertiaA">Gathered inertia of body A.</param>
        /// <param name="inertiaB">Gathered inertia of body B.</param>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void GatherState(ref TwoBodyReferences references,
        out QuaternionWide orientationA, out BodyVelocityWide velocityA, out BodyInertiaWide inertiaA,
        out Vector3Wide ab,
        out QuaternionWide orientationB, out BodyVelocityWide velocityB, out BodyInertiaWide inertiaB)
        {
            ref var states = ref ActiveSet.SolverStates;

            GatherState<AccessAll>(references.IndexA, true, out var positionA, out orientationA, out velocityA, out inertiaA);
            GatherState<AccessAll>(references.IndexB, true, out var positionB, out orientationB, out velocityB, out inertiaB);

            ////Grab the base references for the body indices. Note that we make use of the references memory layout again.
            //ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
            //ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);
            //for (int i = 0; i < count; ++i)
            //{
            //    //WriteGatherState(ref baseIndexA, i, ref states, ref positionA, ref orientationA, ref velocityA);
            //    //WriteGatherInertia(ref baseIndexA, i, ref Inertias, ref inertiaA);
            //    //WriteGatherState(ref baseIndexB, i, ref states, ref positionB, ref orientationB, ref velocityB);
            //    //WriteGatherInertia(ref baseIndexB, i, ref Inertias, ref inertiaB);
            //    WriteGatherState(ref baseIndexA, i, ref states, ref positionA, ref orientationA, ref velocityA, ref inertiaA);
            //    WriteGatherState(ref baseIndexB, i, ref states, ref positionB, ref orientationB, ref velocityB, ref inertiaB);
            //}

            //TODO: In future versions, we will likely store the body position in different forms to allow for extremely large worlds.
            //That will be an opt-in feature. The default implementation will use the FP32 representation, but the user could choose to swap it out for a fp64 or fixed64 representation.
            //This affects other systems- AABB calculation, pose integration, solving, and in extreme (64 bit) cases, the broadphase.
            //We want to insulate other systems from direct knowledge about the implementation of positions when possible.
            //These functions support the solver's needs while hiding absolute positions.
            //In order to support other absolute positions, we'll need alternate implementations of this and other functions.
            //But for the most part, we don't want to pay the overhead of an abstract invocation within the inner loop of the solver. 
            //Given the current limits of C# and the compiler, the best option seems to be conditional compilation.
            Vector3Wide.Subtract(positionB, positionA, out ab);
        }


        /// <summary>
        /// Gathers orientations and relative positions for a three body bundle into an AOSOA bundle.
        /// </summary>
        /// <param name="references">Active body indices being gathered.</param>
        /// <param name="count">Number of body pairs in the bundle.</param>
        /// <param name="ab">Gathered offsets from body A to body B.</param>
        /// <param name="ac">Gathered offsets from body A to body C.</param>
        /// <param name="orientationA">Gathered orientation of body A.</param>
        /// <param name="orientationB">Gathered orientation of body B.</param>
        /// <param name="orientationC">Gathered orientation of body C.</param>
        /// <param name="velocityA">Gathered velocity of body A.</param>
        /// <param name="velocityB">Gathered velocity of body B.</param>
        /// <param name="velocityC">Gathered velocity of body C.</param>
        /// <param name="inertiaA">Gathered inertia of body A.</param>
        /// <param name="inertiaB">Gathered inertia of body B.</param>
        /// <param name="inertiaC">Gathered inertia of body C.</param>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GatherState(ref ThreeBodyReferences references,
            out QuaternionWide orientationA, out BodyVelocityWide velocityA, out BodyInertiaWide inertiaA,
            out Vector3Wide ab,
            out QuaternionWide orientationB, out BodyVelocityWide velocityB, out BodyInertiaWide inertiaB,
            out Vector3Wide ac,
            out QuaternionWide orientationC, out BodyVelocityWide velocityC, out BodyInertiaWide inertiaC)
        {

            GatherState<AccessAll>(references.IndexA, true, out var positionA, out orientationA, out velocityA, out inertiaA);
            GatherState<AccessAll>(references.IndexB, true, out var positionB, out orientationB, out velocityB, out inertiaB);
            GatherState<AccessAll>(references.IndexC, true, out var positionC, out orientationC, out velocityC, out inertiaC);

            //TODO: In future versions, we will likely store the body position in different forms to allow for extremely large worlds.
            //That will be an opt-in feature. The default implementation will use the FP32 representation, but the user could choose to swap it out for a fp64 or fixed64 representation.
            //This affects other systems- AABB calculation, pose integration, solving, and in extreme (64 bit) cases, the broadphase.
            //We want to insulate other systems from direct knowledge about the implementation of positions when possible.
            //These functions support the solver's needs while hiding absolute positions.
            //In order to support other absolute positions, we'll need alternate implementations of this and other functions.
            //But for the most part, we don't want to pay the overhead of an abstract invocation within the inner loop of the solver. 
            //Given the current limits of C# and the compiler, the best option seems to be conditional compilation.
            Vector3Wide.Subtract(positionB, positionA, out ab);
            Vector3Wide.Subtract(positionC, positionA, out ac);
        }

        /// <summary>
        /// Gathers orientations and relative positions for a four body bundle into an AOSOA bundle.
        /// </summary>
        /// <param name="references">Active body indices being gathered.</param>
        /// <param name="count">Number of body pairs in the bundle.</param>
        /// <param name="ab">Gathered offsets from body A to body B.</param>
        /// <param name="ac">Gathered offsets from body A to body C.</param>
        /// <param name="ad">Gathered offsets from body A to body D.</param>
        /// <param name="orientationA">Gathered orientation of body A.</param>
        /// <param name="orientationB">Gathered orientation of body B.</param>
        /// <param name="orientationC">Gathered orientation of body C.</param>
        /// <param name="orientationD">Gathered orientation of body D.</param>
        /// <param name="velocityA">Gathered velocity of body A.</param>
        /// <param name="velocityB">Gathered velocity of body B.</param>
        /// <param name="velocityC">Gathered velocity of body C.</param>
        /// <param name="velocityD">Gathered velocity of body D.</param>
        /// <param name="inertiaA">Gathered inertia of body A.</param>
        /// <param name="inertiaB">Gathered inertia of body B.</param>
        /// <param name="inertiaC">Gathered inertia of body C.</param>
        /// <param name="inertiaD">Gathered inertia of body C.</param>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GatherState(ref FourBodyReferences references,
            out QuaternionWide orientationA, out BodyVelocityWide velocityA, out BodyInertiaWide inertiaA,
            out Vector3Wide ab,
            out QuaternionWide orientationB, out BodyVelocityWide velocityB, out BodyInertiaWide inertiaB,
            out Vector3Wide ac,
            out QuaternionWide orientationC, out BodyVelocityWide velocityC, out BodyInertiaWide inertiaC,
            out Vector3Wide ad,
            out QuaternionWide orientationD, out BodyVelocityWide velocityD, out BodyInertiaWide inertiaD)
        {

            GatherState<AccessAll>(references.IndexA, true, out var positionA, out orientationA, out velocityA, out inertiaA);
            GatherState<AccessAll>(references.IndexB, true, out var positionB, out orientationB, out velocityB, out inertiaB);
            GatherState<AccessAll>(references.IndexC, true, out var positionC, out orientationC, out velocityC, out inertiaC);
            GatherState<AccessAll>(references.IndexD, true, out var positionD, out orientationD, out velocityD, out inertiaD);
            //TODO: In future versions, we will likely store the body position in different forms to allow for extremely large worlds.
            //That will be an opt-in feature. The default implementation will use the FP32 representation, but the user could choose to swap it out for a fp64 or fixed64 representation.
            //This affects other systems- AABB calculation, pose integration, solving, and in extreme (64 bit) cases, the broadphase.
            //We want to insulate other systems from direct knowledge about the implementation of positions when possible.
            //These functions support the solver's needs while hiding absolute positions.
            //In order to support other absolute positions, we'll need alternate implementations of this and other functions.
            //But for the most part, we don't want to pay the overhead of an abstract invocation within the inner loop of the solver. 
            //Given the current limits of C# and the compiler, the best option seems to be conditional compilation.
            Vector3Wide.Subtract(positionB, positionA, out ab);
            Vector3Wide.Subtract(positionC, positionA, out ac);
            Vector3Wide.Subtract(positionD, positionA, out ad);
        }

        //Note that ScatterPose and ScatterInertia do not need to check body references for empty lanes or for kinematicity.
        //The mask is only set for lanes which were subject to constraint integration responsibility; empty lanes and kinematics cannot be integrated by constraints.

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void ScatterPose(
            ref Vector3Wide position, ref QuaternionWide orientation, Vector<int> encodedBodyIndices, Vector<int> mask)
        {
            if (Avx.IsSupported && Vector<float>.Count == 8)
            {
                var states = ActiveSet.SolverStates.Memory;
                {
                    var m0 = orientation.X.AsVector256();
                    var m1 = orientation.Y.AsVector256();
                    var m2 = orientation.Z.AsVector256();
                    var m3 = orientation.W.AsVector256();
                    var m4 = position.X.AsVector256();
                    var m5 = position.Y.AsVector256();
                    var m6 = position.Z.AsVector256();

                    var n0 = Avx.UnpackLow(m0, m1);
                    var n1 = Avx.UnpackLow(m2, m3);
                    var n2 = Avx.UnpackLow(m4, m5);
                    var n3 = Avx.UnpackLow(m6, m6); //Laze alert.
                    var n4 = Avx.UnpackHigh(m0, m1);
                    var n5 = Avx.UnpackHigh(m2, m3);
                    var n6 = Avx.UnpackHigh(m4, m5);
                    var n7 = Avx.UnpackHigh(m6, m6); //Laze alert.

                    var o0 = Avx.Shuffle(n0, n1, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                    var o1 = Avx.Shuffle(n2, n3, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                    var o2 = Avx.Shuffle(n4, n5, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                    var o3 = Avx.Shuffle(n6, n7, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                    var o4 = Avx.Shuffle(n0, n1, 2 | (3 << 2) | (2 << 4) | (3 << 6));
                    var o5 = Avx.Shuffle(n2, n3, 2 | (3 << 2) | (2 << 4) | (3 << 6));
                    var o6 = Avx.Shuffle(n4, n5, 2 | (3 << 2) | (2 << 4) | (3 << 6));
                    var o7 = Avx.Shuffle(n6, n7, 2 | (3 << 2) | (2 << 4) | (3 << 6));

                    if (mask[0] != 0) Avx.StoreAligned((float*)(states + encodedBodyIndices[0]), Avx.Permute2x128(o0, o1, 0 | (2 << 4)));
                    if (mask[1] != 0) Avx.StoreAligned((float*)(states + encodedBodyIndices[1]), Avx.Permute2x128(o4, o5, 0 | (2 << 4)));
                    if (mask[2] != 0) Avx.StoreAligned((float*)(states + encodedBodyIndices[2]), Avx.Permute2x128(o2, o3, 0 | (2 << 4)));
                    if (mask[3] != 0) Avx.StoreAligned((float*)(states + encodedBodyIndices[3]), Avx.Permute2x128(o6, o7, 0 | (2 << 4)));
                    if (mask[4] != 0) Avx.StoreAligned((float*)(states + encodedBodyIndices[4]), Avx.Permute2x128(o0, o1, 1 | (3 << 4)));
                    if (mask[5] != 0) Avx.StoreAligned((float*)(states + encodedBodyIndices[5]), Avx.Permute2x128(o4, o5, 1 | (3 << 4)));
                    if (mask[6] != 0) Avx.StoreAligned((float*)(states + encodedBodyIndices[6]), Avx.Permute2x128(o2, o3, 1 | (3 << 4)));
                    if (mask[7] != 0) Avx.StoreAligned((float*)(states + encodedBodyIndices[7]), Avx.Permute2x128(o6, o7, 1 | (3 << 4)));

                    //if (maskPointer[0] != 0) { states[indices[0]].Motion.Pose.Position.Validate(); states[indices[0]].Motion.Pose.Orientation.Validate(); }
                    //if (maskPointer[1] != 0) { states[indices[1]].Motion.Pose.Position.Validate(); states[indices[1]].Motion.Pose.Orientation.Validate(); }
                    //if (maskPointer[2] != 0) { states[indices[2]].Motion.Pose.Position.Validate(); states[indices[2]].Motion.Pose.Orientation.Validate(); }
                    //if (maskPointer[3] != 0) { states[indices[3]].Motion.Pose.Position.Validate(); states[indices[3]].Motion.Pose.Orientation.Validate(); }
                    //if (maskPointer[4] != 0) { states[indices[4]].Motion.Pose.Position.Validate(); states[indices[4]].Motion.Pose.Orientation.Validate(); }
                    //if (maskPointer[5] != 0) { states[indices[5]].Motion.Pose.Position.Validate(); states[indices[5]].Motion.Pose.Orientation.Validate(); }
                    //if (maskPointer[6] != 0) { states[indices[6]].Motion.Pose.Position.Validate(); states[indices[6]].Motion.Pose.Orientation.Validate(); }
                    //if (maskPointer[7] != 0) { states[indices[7]].Motion.Pose.Position.Validate(); states[indices[7]].Motion.Pose.Orientation.Validate(); }
                }
            }
            else
            {
            }

        }


        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void ScatterInertia(
            ref BodyInertiaWide inertia, Vector<int> encodedBodyIndices, Vector<int> mask)
        {
            if (Avx.IsSupported && Vector<float>.Count == 8)
            {
                var states = ActiveSet.SolverStates.Memory;
                {
                    var m0 = inertia.InverseInertiaTensor.XX.AsVector256();
                    var m1 = inertia.InverseInertiaTensor.YX.AsVector256();
                    var m2 = inertia.InverseInertiaTensor.YY.AsVector256();
                    var m3 = inertia.InverseInertiaTensor.ZX.AsVector256();
                    var m4 = inertia.InverseInertiaTensor.ZY.AsVector256();
                    var m5 = inertia.InverseInertiaTensor.ZZ.AsVector256();
                    var m6 = inertia.InverseMass.AsVector256();

                    var n0 = Avx.UnpackLow(m0, m1);
                    var n1 = Avx.UnpackLow(m2, m3);
                    var n2 = Avx.UnpackLow(m4, m5);
                    var n3 = Avx.UnpackLow(m6, m6); //Laze alert.
                    var n4 = Avx.UnpackHigh(m0, m1);
                    var n5 = Avx.UnpackHigh(m2, m3);
                    var n6 = Avx.UnpackHigh(m4, m5);
                    var n7 = Avx.UnpackHigh(m6, m6); //Laze alert.

                    var o0 = Avx.Shuffle(n0, n1, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                    var o1 = Avx.Shuffle(n2, n3, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                    var o2 = Avx.Shuffle(n4, n5, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                    var o3 = Avx.Shuffle(n6, n7, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                    var o4 = Avx.Shuffle(n0, n1, 2 | (3 << 2) | (2 << 4) | (3 << 6));
                    var o5 = Avx.Shuffle(n2, n3, 2 | (3 << 2) | (2 << 4) | (3 << 6));
                    var o6 = Avx.Shuffle(n4, n5, 2 | (3 << 2) | (2 << 4) | (3 << 6));
                    var o7 = Avx.Shuffle(n6, n7, 2 | (3 << 2) | (2 << 4) | (3 << 6));

                    //Note the offset; we're scattering into the world inertias.
                    if (mask[0] != 0) Avx.StoreAligned((float*)(states + encodedBodyIndices[0]) + 24, Avx.Permute2x128(o0, o1, 0 | (2 << 4)));
                    if (mask[1] != 0) Avx.StoreAligned((float*)(states + encodedBodyIndices[1]) + 24, Avx.Permute2x128(o4, o5, 0 | (2 << 4)));
                    if (mask[2] != 0) Avx.StoreAligned((float*)(states + encodedBodyIndices[2]) + 24, Avx.Permute2x128(o2, o3, 0 | (2 << 4)));
                    if (mask[3] != 0) Avx.StoreAligned((float*)(states + encodedBodyIndices[3]) + 24, Avx.Permute2x128(o6, o7, 0 | (2 << 4)));
                    if (mask[4] != 0) Avx.StoreAligned((float*)(states + encodedBodyIndices[4]) + 24, Avx.Permute2x128(o0, o1, 1 | (3 << 4)));
                    if (mask[5] != 0) Avx.StoreAligned((float*)(states + encodedBodyIndices[5]) + 24, Avx.Permute2x128(o4, o5, 1 | (3 << 4)));
                    if (mask[6] != 0) Avx.StoreAligned((float*)(states + encodedBodyIndices[6]) + 24, Avx.Permute2x128(o2, o3, 1 | (3 << 4)));
                    if (mask[7] != 0) Avx.StoreAligned((float*)(states + encodedBodyIndices[7]) + 24, Avx.Permute2x128(o6, o7, 1 | (3 << 4)));

                    //if (maskPointer[0] != 0) { states[indices[0]].Inertia.Local.InverseInertiaTensor.Validate(); states[indices[0]].Inertia.Local.InverseMass.Validate(); }
                    //if (maskPointer[1] != 0) { states[indices[1]].Inertia.Local.InverseInertiaTensor.Validate(); states[indices[1]].Inertia.Local.InverseMass.Validate(); }
                    //if (maskPointer[2] != 0) { states[indices[2]].Inertia.Local.InverseInertiaTensor.Validate(); states[indices[2]].Inertia.Local.InverseMass.Validate(); }
                    //if (maskPointer[3] != 0) { states[indices[3]].Inertia.Local.InverseInertiaTensor.Validate(); states[indices[3]].Inertia.Local.InverseMass.Validate(); }
                    //if (maskPointer[4] != 0) { states[indices[4]].Inertia.Local.InverseInertiaTensor.Validate(); states[indices[4]].Inertia.Local.InverseMass.Validate(); }
                    //if (maskPointer[5] != 0) { states[indices[5]].Inertia.Local.InverseInertiaTensor.Validate(); states[indices[5]].Inertia.Local.InverseMass.Validate(); }
                    //if (maskPointer[6] != 0) { states[indices[6]].Inertia.Local.InverseInertiaTensor.Validate(); states[indices[6]].Inertia.Local.InverseMass.Validate(); }
                    //if (maskPointer[7] != 0) { states[indices[7]].Inertia.Local.InverseInertiaTensor.Validate(); states[indices[7]].Inertia.Local.InverseMass.Validate(); }
                }
            }
            else
            {
            }
        }


        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void ScatterVelocities<TAccessFilter>(ref BodyVelocityWide sourceVelocities, ref Vector<int> encodedBodyIndices) where TAccessFilter : unmanaged, IBodyAccessFilter
        {
            if (Avx.IsSupported && Vector<float>.Count == 8)
            {
                //TODO: High precision poses means we'll end up with 64 bits of the second lane containing either orientation components or a position component.
                //That'll require a revamp of this approach to mask out any writes to the pose components, but it'll all be compile time conditional.
                Unsafe.SkipInit(out TAccessFilter filter);

                if (filter.AccessLinearVelocity ^ filter.AccessAngularVelocity)
                {
                    //for (int i = 0; i < 8; ++i)
                    //{
                    //    Get(ref sourceVelocities.Linear.X, i) = i + 100;
                    //    Get(ref sourceVelocities.Linear.Y, i) = i + 200;
                    //    Get(ref sourceVelocities.Linear.Z, i) = i + 300;
                    //    Get(ref sourceVelocities.Angular.X, i) = i + 500;
                    //    Get(ref sourceVelocities.Angular.Y, i) = i + 600;
                    //    Get(ref sourceVelocities.Angular.Z, i) = i + 700;
                    //}
                    //We can't write the entire lane if we only want linear or angular velocity; that would overwrite existing values with invalid data.
                    Vector256<float> m0, m1, m2;
                    int targetOffset;
                    if (filter.AccessLinearVelocity)
                    {
                        targetOffset = 8;
                        m0 = sourceVelocities.Linear.X.AsVector256();
                        m1 = sourceVelocities.Linear.Y.AsVector256();
                        m2 = sourceVelocities.Linear.Z.AsVector256();
                    }
                    else
                    {
                        targetOffset = 12;
                        m0 = sourceVelocities.Angular.X.AsVector256();
                        m1 = sourceVelocities.Angular.Y.AsVector256();
                        m2 = sourceVelocities.Angular.Z.AsVector256();
                    }
                    //We're being a bit lazy here- you could reduce the instructions a bit more given the two empty source lanes.
                    var n0 = Avx.UnpackLow(m0, m1);
                    var n1 = Avx.UnpackLow(m2, m2);
                    var n4 = Avx.UnpackHigh(m0, m1);
                    var n5 = Avx.UnpackHigh(m2, m2);

                    var o0 = Avx.Shuffle(n0, n1, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                    var o2 = Avx.Shuffle(n4, n5, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                    var o4 = Avx.Shuffle(n0, n1, 2 | (3 << 2) | (2 << 4) | (3 << 6));
                    var o6 = Avx.Shuffle(n4, n5, 2 | (3 << 2) | (2 << 4) | (3 << 6));

                    var indices = (uint*)Unsafe.AsPointer(ref encodedBodyIndices);
                    var states = ActiveSet.SolverStates.Memory;
                    if (indices[0] < DynamicLimit) Sse.StoreAligned((float*)(states + indices[0]) + targetOffset, o0.GetLower());
                    if (indices[1] < DynamicLimit) Sse.StoreAligned((float*)(states + indices[1]) + targetOffset, o4.GetLower());
                    if (indices[2] < DynamicLimit) Sse.StoreAligned((float*)(states + indices[2]) + targetOffset, o2.GetLower());
                    if (indices[3] < DynamicLimit) Sse.StoreAligned((float*)(states + indices[3]) + targetOffset, o6.GetLower());
                    if (indices[4] < DynamicLimit) Sse.StoreAligned((float*)(states + indices[4]) + targetOffset, o0.GetUpper());
                    if (indices[5] < DynamicLimit) Sse.StoreAligned((float*)(states + indices[5]) + targetOffset, o4.GetUpper());
                    if (indices[6] < DynamicLimit) Sse.StoreAligned((float*)(states + indices[6]) + targetOffset, o2.GetUpper());
                    if (indices[7] < DynamicLimit) Sse.StoreAligned((float*)(states + indices[7]) + targetOffset, o6.GetUpper());

                }
                else
                {
                    //Just like the gather but... transposed, we transpose from the wide representation into per-body velocities.
                    //The motion state struct puts the velocities into the second 32 byte chunk, so we can do a single write per body.
                    var m0 = sourceVelocities.Linear.X.AsVector256();
                    var m1 = sourceVelocities.Linear.Y.AsVector256();
                    var m2 = sourceVelocities.Linear.Z.AsVector256();
                    var m4 = sourceVelocities.Angular.X.AsVector256();
                    var m5 = sourceVelocities.Angular.Y.AsVector256();
                    var m6 = sourceVelocities.Angular.Z.AsVector256();

                    //We're being a bit lazy here- you could reduce the instructions a bit more given the two empty source lanes.
                    var n0 = Avx.UnpackLow(m0, m1);
                    var n1 = Avx.UnpackLow(m2, m2);
                    var n2 = Avx.UnpackLow(m4, m5);
                    var n3 = Avx.UnpackLow(m6, m6);
                    var n4 = Avx.UnpackHigh(m0, m1);
                    var n5 = Avx.UnpackHigh(m2, m2);
                    var n6 = Avx.UnpackHigh(m4, m5);
                    var n7 = Avx.UnpackHigh(m6, m6);

                    var o0 = Avx.Shuffle(n0, n1, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                    var o1 = Avx.Shuffle(n2, n3, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                    var o2 = Avx.Shuffle(n4, n5, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                    var o3 = Avx.Shuffle(n6, n7, 0 | (1 << 2) | (0 << 4) | (1 << 6));
                    var o4 = Avx.Shuffle(n0, n1, 2 | (3 << 2) | (2 << 4) | (3 << 6));
                    var o5 = Avx.Shuffle(n2, n3, 2 | (3 << 2) | (2 << 4) | (3 << 6));
                    var o6 = Avx.Shuffle(n4, n5, 2 | (3 << 2) | (2 << 4) | (3 << 6));
                    var o7 = Avx.Shuffle(n6, n7, 2 | (3 << 2) | (2 << 4) | (3 << 6));

                    var indices = (uint*)Unsafe.AsPointer(ref encodedBodyIndices);
                    var states = ActiveSet.SolverStates.Memory;
                    if (indices[0] < DynamicLimit) Avx.StoreAligned((float*)(states + indices[0]) + 8, Avx.Permute2x128(o0, o1, 0 | (2 << 4)));
                    if (indices[1] < DynamicLimit) Avx.StoreAligned((float*)(states + indices[1]) + 8, Avx.Permute2x128(o4, o5, 0 | (2 << 4)));
                    if (indices[2] < DynamicLimit) Avx.StoreAligned((float*)(states + indices[2]) + 8, Avx.Permute2x128(o2, o3, 0 | (2 << 4)));
                    if (indices[3] < DynamicLimit) Avx.StoreAligned((float*)(states + indices[3]) + 8, Avx.Permute2x128(o6, o7, 0 | (2 << 4)));
                    if (indices[4] < DynamicLimit) Avx.StoreAligned((float*)(states + indices[4]) + 8, Avx.Permute2x128(o0, o1, 1 | (3 << 4)));
                    if (indices[5] < DynamicLimit) Avx.StoreAligned((float*)(states + indices[5]) + 8, Avx.Permute2x128(o4, o5, 1 | (3 << 4)));
                    if (indices[6] < DynamicLimit) Avx.StoreAligned((float*)(states + indices[6]) + 8, Avx.Permute2x128(o2, o3, 1 | (3 << 4)));
                    if (indices[7] < DynamicLimit) Avx.StoreAligned((float*)(states + indices[7]) + 8, Avx.Permute2x128(o6, o7, 1 | (3 << 4)));
                }

                //{
                //    var indices = (int*)Unsafe.AsPointer(ref references);
                //    var states = ActiveSet.SolverStates.Memory;
                //    if (indices[0] >= 0) { states[indices[0]].Motion.Velocity.Linear.Validate(); states[indices[0]].Motion.Velocity.Angular.Validate(); }
                //    if (indices[1] >= 0) { states[indices[1]].Motion.Velocity.Linear.Validate(); states[indices[1]].Motion.Velocity.Angular.Validate(); }
                //    if (indices[2] >= 0) { states[indices[2]].Motion.Velocity.Linear.Validate(); states[indices[2]].Motion.Velocity.Angular.Validate(); }
                //    if (indices[3] >= 0) { states[indices[3]].Motion.Velocity.Linear.Validate(); states[indices[3]].Motion.Velocity.Angular.Validate(); }
                //    if (indices[4] >= 0) { states[indices[4]].Motion.Velocity.Linear.Validate(); states[indices[4]].Motion.Velocity.Angular.Validate(); }
                //    if (indices[5] >= 0) { states[indices[5]].Motion.Velocity.Linear.Validate(); states[indices[5]].Motion.Velocity.Angular.Validate(); }
                //    if (indices[6] >= 0) { states[indices[6]].Motion.Velocity.Linear.Validate(); states[indices[6]].Motion.Velocity.Angular.Validate(); }
                //    if (indices[7] >= 0) { states[indices[7]].Motion.Velocity.Linear.Validate(); states[indices[7]].Motion.Velocity.Angular.Validate(); }
                //}
            }
            else
            {
                var indices = (uint*)Unsafe.AsPointer(ref encodedBodyIndices);
                for (int innerIndex = 0; innerIndex < Vector<int>.Count; ++innerIndex)
                {
                    if (indices[innerIndex] >= DynamicLimit)
                        continue;
                    ref var sourceSlot = ref GetOffsetInstance(ref sourceVelocities, innerIndex);
                    ref var target = ref ActiveSet.SolverStates[indices[innerIndex]].Motion.Velocity;
                    target.Linear = new Vector3(sourceSlot.Linear.X[0], sourceSlot.Linear.Y[0], sourceSlot.Linear.Z[0]);
                    target.Angular = new Vector3(sourceSlot.Angular.X[0], sourceSlot.Angular.Y[0], sourceSlot.Angular.Z[0]);
                }
            }
        }
    }
}
