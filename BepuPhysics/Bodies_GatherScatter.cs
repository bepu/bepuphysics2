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
        private static void WriteGatherInertia(ref int bundleBaseBodyIndexInSet, int bodyIndexInBundle, ref Buffer<SolverState> states, ref BodyInertiaWide gatheredInertias)
        {
            ref var source = ref states[Unsafe.Add(ref bundleBaseBodyIndexInSet, bodyIndexInBundle)].Inertia.World;
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
        private static void WriteGatherMotionState(ref int bundleBaseBodyIndexInSet, int bodyIndexInBundle, ref Buffer<SolverState> states,
            ref Vector3Wide position, ref QuaternionWide orientation, ref BodyVelocityWide velocity)
        {
            ref var state = ref states[Unsafe.Add(ref bundleBaseBodyIndexInSet, bodyIndexInBundle)].Motion;
            Vector3Wide.WriteFirst(state.Pose.Position, ref GetOffsetInstance(ref position, bodyIndexInBundle));
            QuaternionWide.WriteFirst(state.Pose.Orientation, ref GetOffsetInstance(ref orientation, bodyIndexInBundle));
            Vector3Wide.WriteFirst(state.Velocity.Linear, ref GetOffsetInstance(ref velocity.Linear, bodyIndexInBundle));
            Vector3Wide.WriteFirst(state.Velocity.Angular, ref GetOffsetInstance(ref velocity.Angular, bodyIndexInBundle));
        }

        /// <summary>
        /// Gathers motion state information for a body bundle into an AOSOA bundle.
        /// </summary>
        /// <param name="references">Active body indices being gathered.</param>
        /// <param name="count">Number of body pairs in the bundle.</param>
        /// <param name="position">Gathered absolute position of the body.</param>
        /// <param name="orientation">Gathered orientation of the body.</param>
        /// <param name="velocity">Gathered velocity of the body.</param>
        /// <param name="inertia">Gathered inertia of the body.</param>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GatherState(ref Vector<int> references, int count,
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
            Debug.Assert(count >= 0 && count <= Vector<float>.Count);
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndex = ref Unsafe.As<Vector<int>, int>(ref references);

            for (int i = 0; i < count; ++i)
            {
                WriteGatherMotionState(ref baseIndex, i, ref ActiveSet.SolverStates, ref position, ref orientation, ref velocity);
                WriteGatherInertia(ref baseIndex, i, ref ActiveSet.SolverStates, ref inertia);
            }
        }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe static void FallbackGatherMotionState(int count, SolverState* states, ref Vector<int> baseIndex, ref Vector3Wide position, ref QuaternionWide orientation, ref BodyVelocityWide velocity)
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

            for (int i = 0; i < count; ++i)
            {
                var index = indices[i];
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
        unsafe static void FallbackGatherInertia(int count, SolverState* states, ref Vector<int> baseIndex, ref BodyInertiaWide inertia, int offsetInFloats)
        {
            var indices = (int*)Unsafe.AsPointer(ref baseIndex);
            var pMass = (float*)Unsafe.AsPointer(ref inertia.InverseMass);
            var pInertiaXX = (float*)Unsafe.AsPointer(ref inertia.InverseInertiaTensor.XX);
            var pInertiaYX = (float*)Unsafe.AsPointer(ref inertia.InverseInertiaTensor.YX);
            var pInertiaYY = (float*)Unsafe.AsPointer(ref inertia.InverseInertiaTensor.YY);
            var pInertiaZX = (float*)Unsafe.AsPointer(ref inertia.InverseInertiaTensor.ZX);
            var pInertiaZY = (float*)Unsafe.AsPointer(ref inertia.InverseInertiaTensor.ZY);
            var pInertiaZZ = (float*)Unsafe.AsPointer(ref inertia.InverseInertiaTensor.ZZ);

            for (int i = 0; i < count; ++i)
            {
                var index = indices[i];
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

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void GatherState<TAccessFilter>(ref Vector<int> bodyIndices, int count, bool worldInertia, out Vector3Wide position, out QuaternionWide orientation, out BodyVelocityWide velocity, out BodyInertiaWide inertia)
            where TAccessFilter : unmanaged, IBodyAccessFilter
        {
            var solverStates = ActiveSet.SolverStates.Memory;
            Unsafe.SkipInit(out TAccessFilter filter);
            if (Avx.IsSupported && Vector<float>.Count == 8)
            {
                var indices = (int*)Unsafe.AsPointer(ref bodyIndices);

                var s0 = (float*)(solverStates + indices[0]);
                var s1 = (float*)(solverStates + indices[1]);
                var s2 = (float*)(solverStates + indices[2]);
                var s3 = (float*)(solverStates + indices[3]);
                var s4 = (float*)(solverStates + indices[4]);
                var s5 = (float*)(solverStates + indices[5]);
                var s6 = (float*)(solverStates + indices[6]);
                var s7 = (float*)(solverStates + indices[7]);

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
                    var m0 = Avx.LoadAlignedVector256(s0);
                    var m1 = count > 1 ? Avx.LoadAlignedVector256(s1) : Vector256<float>.Zero;
                    var m2 = count > 2 ? Avx.LoadAlignedVector256(s2) : Vector256<float>.Zero;
                    var m3 = count > 3 ? Avx.LoadAlignedVector256(s3) : Vector256<float>.Zero;
                    var m4 = count > 4 ? Avx.LoadAlignedVector256(s4) : Vector256<float>.Zero;
                    var m5 = count > 5 ? Avx.LoadAlignedVector256(s5) : Vector256<float>.Zero;
                    var m6 = count > 6 ? Avx.LoadAlignedVector256(s6) : Vector256<float>.Zero;
                    var m7 = count > 7 ? Avx.LoadAlignedVector256(s7) : Vector256<float>.Zero;

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
                    var m0 = Avx.LoadAlignedVector256(s0 + 8);
                    var m1 = count > 1 ? Avx.LoadAlignedVector256(s1 + 8) : Vector256<float>.Zero;
                    var m2 = count > 2 ? Avx.LoadAlignedVector256(s2 + 8) : Vector256<float>.Zero;
                    var m3 = count > 3 ? Avx.LoadAlignedVector256(s3 + 8) : Vector256<float>.Zero;
                    var m4 = count > 4 ? Avx.LoadAlignedVector256(s4 + 8) : Vector256<float>.Zero;
                    var m5 = count > 5 ? Avx.LoadAlignedVector256(s5 + 8) : Vector256<float>.Zero;
                    var m6 = count > 6 ? Avx.LoadAlignedVector256(s6 + 8) : Vector256<float>.Zero;
                    var m7 = count > 7 ? Avx.LoadAlignedVector256(s7 + 8) : Vector256<float>.Zero;

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
                    var m0 = Avx.LoadAlignedVector256(s0 + offsetInFloats);
                    var m1 = count > 1 ? Avx.LoadAlignedVector256(s1 + offsetInFloats) : Vector256<float>.Zero;
                    var m2 = count > 2 ? Avx.LoadAlignedVector256(s2 + offsetInFloats) : Vector256<float>.Zero;
                    var m3 = count > 3 ? Avx.LoadAlignedVector256(s3 + offsetInFloats) : Vector256<float>.Zero;
                    var m4 = count > 4 ? Avx.LoadAlignedVector256(s4 + offsetInFloats) : Vector256<float>.Zero;
                    var m5 = count > 5 ? Avx.LoadAlignedVector256(s5 + offsetInFloats) : Vector256<float>.Zero;
                    var m6 = count > 6 ? Avx.LoadAlignedVector256(s6 + offsetInFloats) : Vector256<float>.Zero;
                    var m7 = count > 7 ? Avx.LoadAlignedVector256(s7 + offsetInFloats) : Vector256<float>.Zero;

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
                FallbackGatherMotionState(count, solverStates, ref bodyIndices, ref position, ref orientation, ref velocity);
                FallbackGatherInertia(count, solverStates, ref bodyIndices, ref inertia, worldInertia ? 24 : 16);
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
        public unsafe void GatherState(ref TwoBodyReferences references, int count,
        out QuaternionWide orientationA, out BodyVelocityWide velocityA, out BodyInertiaWide inertiaA,
        out Vector3Wide ab,
        out QuaternionWide orientationB, out BodyVelocityWide velocityB, out BodyInertiaWide inertiaB)
        {
            Debug.Assert(count >= 0 && count <= Vector<float>.Count);
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
            ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);

            ref var states = ref ActiveSet.SolverStates;

            GatherState<AccessAll>(ref references.IndexA, count, true, out var positionA, out orientationA, out velocityA, out inertiaA);
            GatherState<AccessAll>(ref references.IndexB, count, true, out var positionB, out orientationB, out velocityB, out inertiaB);

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
        public void GatherState(ref ThreeBodyReferences references, int count,
            out QuaternionWide orientationA, out BodyVelocityWide velocityA, out BodyInertiaWide inertiaA,
            out Vector3Wide ab,
            out QuaternionWide orientationB, out BodyVelocityWide velocityB, out BodyInertiaWide inertiaB,
            out Vector3Wide ac,
            out QuaternionWide orientationC, out BodyVelocityWide velocityC, out BodyInertiaWide inertiaC)
        {
            Unsafe.SkipInit(out Vector3Wide positionA);
            Unsafe.SkipInit(out Vector3Wide positionB);
            Unsafe.SkipInit(out Vector3Wide positionC);
            Unsafe.SkipInit(out orientationA);
            Unsafe.SkipInit(out orientationB);
            Unsafe.SkipInit(out orientationC);
            Unsafe.SkipInit(out velocityA);
            Unsafe.SkipInit(out velocityB);
            Unsafe.SkipInit(out velocityC);
            Unsafe.SkipInit(out inertiaA);
            Unsafe.SkipInit(out inertiaB);
            Unsafe.SkipInit(out inertiaC);
            Debug.Assert(count >= 0 && count <= Vector<float>.Count);
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
            ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);
            ref var baseIndexC = ref Unsafe.As<Vector<int>, int>(ref references.IndexC);

            ref var states = ref ActiveSet.SolverStates;
            for (int i = 0; i < count; ++i)
            {
                WriteGatherMotionState(ref baseIndexA, i, ref states, ref positionA, ref orientationA, ref velocityA);
                WriteGatherInertia(ref baseIndexA, i, ref states, ref inertiaA);
                WriteGatherMotionState(ref baseIndexB, i, ref states, ref positionB, ref orientationB, ref velocityB);
                WriteGatherInertia(ref baseIndexB, i, ref states, ref inertiaB);
                WriteGatherMotionState(ref baseIndexC, i, ref states, ref positionC, ref orientationC, ref velocityC);
                WriteGatherInertia(ref baseIndexC, i, ref states, ref inertiaC);
            }
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
        public void GatherState(ref FourBodyReferences references, int count,
            out QuaternionWide orientationA, out BodyVelocityWide velocityA, out BodyInertiaWide inertiaA,
            out Vector3Wide ab,
            out QuaternionWide orientationB, out BodyVelocityWide velocityB, out BodyInertiaWide inertiaB,
            out Vector3Wide ac,
            out QuaternionWide orientationC, out BodyVelocityWide velocityC, out BodyInertiaWide inertiaC,
            out Vector3Wide ad,
            out QuaternionWide orientationD, out BodyVelocityWide velocityD, out BodyInertiaWide inertiaD)
        {
            Unsafe.SkipInit(out Vector3Wide positionA);
            Unsafe.SkipInit(out Vector3Wide positionB);
            Unsafe.SkipInit(out Vector3Wide positionC);
            Unsafe.SkipInit(out Vector3Wide positionD);
            Unsafe.SkipInit(out orientationA);
            Unsafe.SkipInit(out orientationB);
            Unsafe.SkipInit(out orientationC);
            Unsafe.SkipInit(out orientationD);
            Unsafe.SkipInit(out velocityA);
            Unsafe.SkipInit(out velocityB);
            Unsafe.SkipInit(out velocityC);
            Unsafe.SkipInit(out velocityD);
            Unsafe.SkipInit(out inertiaA);
            Unsafe.SkipInit(out inertiaB);
            Unsafe.SkipInit(out inertiaC);
            Unsafe.SkipInit(out inertiaD);
            Debug.Assert(count >= 0 && count <= Vector<float>.Count);
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
            ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);
            ref var baseIndexC = ref Unsafe.As<Vector<int>, int>(ref references.IndexC);
            ref var baseIndexD = ref Unsafe.As<Vector<int>, int>(ref references.IndexD);

            ref var states = ref ActiveSet.SolverStates;
            for (int i = 0; i < count; ++i)
            {
                WriteGatherMotionState(ref baseIndexA, i, ref states, ref positionA, ref orientationA, ref velocityA);
                WriteGatherInertia(ref baseIndexA, i, ref states, ref inertiaA);
                WriteGatherMotionState(ref baseIndexB, i, ref states, ref positionB, ref orientationB, ref velocityB);
                WriteGatherInertia(ref baseIndexB, i, ref states, ref inertiaB);
                WriteGatherMotionState(ref baseIndexC, i, ref states, ref positionC, ref orientationC, ref velocityC);
                WriteGatherInertia(ref baseIndexC, i, ref states, ref inertiaC);
                WriteGatherMotionState(ref baseIndexD, i, ref states, ref positionD, ref orientationD, ref velocityD);
                WriteGatherInertia(ref baseIndexD, i, ref states, ref inertiaD);
            }
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
            Vector3Wide.Subtract(positionC, positionA, out ad);
        }


        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void ScatterPose(
            ref Vector3Wide position, ref QuaternionWide orientation, ref Vector<int> references, ref Vector<int> mask)
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

                    var maskPointer = (int*)Unsafe.AsPointer(ref mask);
                    var indices = (int*)Unsafe.AsPointer(ref references);
                    if (maskPointer[0] != 0) Avx.StoreAligned((float*)(states + indices[0]), Avx.Permute2x128(o0, o1, 0 | (2 << 4)));
                    if (maskPointer[1] != 0) Avx.StoreAligned((float*)(states + indices[1]), Avx.Permute2x128(o4, o5, 0 | (2 << 4)));
                    if (maskPointer[2] != 0) Avx.StoreAligned((float*)(states + indices[2]), Avx.Permute2x128(o2, o3, 0 | (2 << 4)));
                    if (maskPointer[3] != 0) Avx.StoreAligned((float*)(states + indices[3]), Avx.Permute2x128(o6, o7, 0 | (2 << 4)));
                    if (maskPointer[4] != 0) Avx.StoreAligned((float*)(states + indices[4]), Avx.Permute2x128(o0, o1, 1 | (3 << 4)));
                    if (maskPointer[5] != 0) Avx.StoreAligned((float*)(states + indices[5]), Avx.Permute2x128(o4, o5, 1 | (3 << 4)));
                    if (maskPointer[6] != 0) Avx.StoreAligned((float*)(states + indices[6]), Avx.Permute2x128(o2, o3, 1 | (3 << 4)));
                    if (maskPointer[7] != 0) Avx.StoreAligned((float*)(states + indices[7]), Avx.Permute2x128(o6, o7, 1 | (3 << 4)));
                }
            }
            else
            {
            }
        }


        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void ScatterInertia(
            ref BodyInertiaWide inertia, ref Vector<int> references, ref Vector<int> mask)
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

                    var maskPointer = (int*)Unsafe.AsPointer(ref mask);
                    var indices = (int*)Unsafe.AsPointer(ref references);
                    //Note the offset; we're scattering into the world inertias.
                    if (maskPointer[0] != 0) Avx.StoreAligned((float*)(states + indices[0]) + 24, Avx.Permute2x128(o0, o1, 0 | (2 << 4)));
                    if (maskPointer[1] != 0) Avx.StoreAligned((float*)(states + indices[1]) + 24, Avx.Permute2x128(o4, o5, 0 | (2 << 4)));
                    if (maskPointer[2] != 0) Avx.StoreAligned((float*)(states + indices[2]) + 24, Avx.Permute2x128(o2, o3, 0 | (2 << 4)));
                    if (maskPointer[3] != 0) Avx.StoreAligned((float*)(states + indices[3]) + 24, Avx.Permute2x128(o6, o7, 0 | (2 << 4)));
                    if (maskPointer[4] != 0) Avx.StoreAligned((float*)(states + indices[4]) + 24, Avx.Permute2x128(o0, o1, 1 | (3 << 4)));
                    if (maskPointer[5] != 0) Avx.StoreAligned((float*)(states + indices[5]) + 24, Avx.Permute2x128(o4, o5, 1 | (3 << 4)));
                    if (maskPointer[6] != 0) Avx.StoreAligned((float*)(states + indices[6]) + 24, Avx.Permute2x128(o2, o3, 1 | (3 << 4)));
                    if (maskPointer[7] != 0) Avx.StoreAligned((float*)(states + indices[7]) + 24, Avx.Permute2x128(o6, o7, 1 | (3 << 4)));
                }
            }
            else
            {
            }
        }


        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void ScatterVelocities<TAccessFilter>(ref BodyVelocityWide sourceVelocities, ref Vector<int> references, int count) where TAccessFilter : unmanaged, IBodyAccessFilter
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

                    var indices = (int*)Unsafe.AsPointer(ref references);
                    var states = ActiveSet.SolverStates.Memory;
                    Sse.StoreAligned((float*)(states + indices[0]) + targetOffset, o0.GetLower());
                    if (count > 1) Sse.StoreAligned((float*)(states + indices[1]) + targetOffset, o4.GetLower());
                    if (count > 2) Sse.StoreAligned((float*)(states + indices[2]) + targetOffset, o2.GetLower());
                    if (count > 3) Sse.StoreAligned((float*)(states + indices[3]) + targetOffset, o6.GetLower());
                    if (count > 4) Sse.StoreAligned((float*)(states + indices[4]) + targetOffset, o0.GetUpper());
                    if (count > 5) Sse.StoreAligned((float*)(states + indices[5]) + targetOffset, o4.GetUpper());
                    if (count > 6) Sse.StoreAligned((float*)(states + indices[6]) + targetOffset, o2.GetUpper());
                    if (count > 7) Sse.StoreAligned((float*)(states + indices[7]) + targetOffset, o6.GetUpper());

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

                    var indices = (int*)Unsafe.AsPointer(ref references);
                    var states = ActiveSet.SolverStates.Memory;
                    Avx.StoreAligned((float*)(states + indices[0]) + 8, Avx.Permute2x128(o0, o1, 0 | (2 << 4)));
                    if (count > 1) Avx.StoreAligned((float*)(states + indices[1]) + 8, Avx.Permute2x128(o4, o5, 0 | (2 << 4)));
                    if (count > 2) Avx.StoreAligned((float*)(states + indices[2]) + 8, Avx.Permute2x128(o2, o3, 0 | (2 << 4)));
                    if (count > 3) Avx.StoreAligned((float*)(states + indices[3]) + 8, Avx.Permute2x128(o6, o7, 0 | (2 << 4)));
                    if (count > 4) Avx.StoreAligned((float*)(states + indices[4]) + 8, Avx.Permute2x128(o0, o1, 1 | (3 << 4)));
                    if (count > 5) Avx.StoreAligned((float*)(states + indices[5]) + 8, Avx.Permute2x128(o4, o5, 1 | (3 << 4)));
                    if (count > 6) Avx.StoreAligned((float*)(states + indices[6]) + 8, Avx.Permute2x128(o2, o3, 1 | (3 << 4)));
                    if (count > 7) Avx.StoreAligned((float*)(states + indices[7]) + 8, Avx.Permute2x128(o6, o7, 1 | (3 << 4)));
                }
            }
            else
            {
                for (int innerIndex = 0; innerIndex < count; ++innerIndex)
                {
                    ref var sourceSlot = ref GetOffsetInstance(ref sourceVelocities, innerIndex);
                    var indices = (int*)Unsafe.AsPointer(ref references);
                    ref var target = ref ActiveSet.SolverStates[indices[innerIndex]].Motion.Velocity;
                    target.Linear = new Vector3(sourceSlot.Linear.X[0], sourceSlot.Linear.Y[0], sourceSlot.Linear.Z[0]);
                    target.Angular = new Vector3(sourceSlot.Angular.X[0], sourceSlot.Angular.Y[0], sourceSlot.Angular.Z[0]);
                }
            }
        }
        ////[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //private unsafe void ScatterVelocities(ref BodyVelocityWide sourceVelocities, ref int baseIndex, int innerIndex)
        //{
        //    //TODO: How much value would there be in branching on kinematic state and avoiding a write? Depends a lot on the number of kinematics.
        //    ref var sourceSlot = ref GetOffsetInstance(ref sourceVelocities, innerIndex);
        //    ref var target = ref ActiveSet.SolverStates[Unsafe.Add(ref baseIndex, innerIndex)].Motion.Velocity;
        //    target.Linear = new Vector3(sourceSlot.Linear.X[0], sourceSlot.Linear.Y[0], sourceSlot.Linear.Z[0]);
        //    target.Angular = new Vector3(sourceSlot.Angular.X[0], sourceSlot.Angular.Y[0], sourceSlot.Angular.Z[0]);
        //}

        ///// <summary>
        ///// Scatters velocities for one body bundle into the active body set.
        ///// </summary>
        ///// <param name="sourceVelocities">Velocities of body bundle A to scatter.</param>
        ///// <param name="references">Active set indices of the bodies to scatter velocity data to.</param>
        ///// <param name="count">Number of body pairs in the bundle.</param>
        ////[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public unsafe void ScatterVelocities(ref BodyVelocityWide sourceVelocities, ref Vector<int> references, int count)
        //{
        //    Debug.Assert(count >= 0 && count <= Vector<float>.Count);
        //    //Grab the base references for the body indices. Note that we make use of the references memory layout again.
        //    ref var baseIndex = ref Unsafe.As<Vector<int>, int>(ref references);
        //    for (int i = 0; i < count; ++i)
        //    {
        //        ScatterVelocities(ref sourceVelocities, ref baseIndex, i);
        //    }
        //}

        ///// <summary>
        ///// Scatters velocities for two body bundles into the active body set.
        ///// </summary>
        ///// <param name="sourceVelocitiesA">Velocities of body bundle A to scatter.</param>
        ///// <param name="sourceVelocitiesA">Velocities of body bundle B to scatter.</param>
        ///// <param name="references">Active set indices of the bodies to scatter velocity data to.</param>
        ///// <param name="count">Number of body pairs in the bundle.</param>
        ////[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public unsafe void ScatterVelocities(ref BodyVelocityWide sourceVelocitiesA, ref BodyVelocityWide sourceVelocitiesB,
        //    ref TwoBodyReferences references, int count)
        //{
        //    Debug.Assert(count >= 0 && count <= Vector<float>.Count);
        //    ScatterVelocities(ref sourceVelocitiesA, ref references.IndexA, count);
        //    ScatterVelocities(ref sourceVelocitiesB, ref references.IndexB, count);
        //    ////Grab the base references for the body indices. Note that we make use of the references memory layout again.
        //    //ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
        //    //ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);
        //    //for (int i = 0; i < count; ++i)
        //    //{
        //    //    ScatterVelocities(ref sourceVelocitiesA, ref baseIndexA, i);
        //    //    ScatterVelocities(ref sourceVelocitiesB, ref baseIndexB, i);
        //    //}
        //}

        ///// <summary>
        ///// Scatters velocities for three body bundles into the active body set.
        ///// </summary>
        ///// <param name="sourceVelocitiesA">Velocities of body bundle A to scatter.</param>
        ///// <param name="sourceVelocitiesB">Velocities of body bundle B to scatter.</param>
        ///// <param name="sourceVelocitiesC">Velocities of body bundle C to scatter.</param>
        ///// <param name="references">Active set indices of the bodies to scatter velocity data to.</param>
        ///// <param name="count">Number of body pairs in the bundle.</param>
        ////[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public unsafe void ScatterVelocities(
        //    ref BodyVelocityWide sourceVelocitiesA, ref BodyVelocityWide sourceVelocitiesB, ref BodyVelocityWide sourceVelocitiesC,
        //    ref ThreeBodyReferences references, int count)
        //{
        //    Debug.Assert(count >= 0 && count <= Vector<float>.Count);
        //    //Grab the base references for the body indices. Note that we make use of the references memory layout again.
        //    ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
        //    ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);
        //    ref var baseIndexC = ref Unsafe.As<Vector<int>, int>(ref references.IndexC);
        //    for (int i = 0; i < count; ++i)
        //    {
        //        ScatterVelocities(ref sourceVelocitiesA, ref baseIndexA, i);
        //        ScatterVelocities(ref sourceVelocitiesB, ref baseIndexB, i);
        //        ScatterVelocities(ref sourceVelocitiesC, ref baseIndexC, i);
        //    }
        //}

        ///// <summary>
        ///// Scatters velocities for four body bundles into the active body set.
        ///// </summary>
        ///// <param name="sourceVelocitiesA">Velocities of body bundle A to scatter.</param>
        ///// <param name="sourceVelocitiesB">Velocities of body bundle B to scatter.</param>
        ///// <param name="sourceVelocitiesC">Velocities of body bundle C to scatter.</param>
        ///// <param name="sourceVelocitiesD">Velocities of body bundle D to scatter.</param>
        ///// <param name="references">Active set indices of the bodies to scatter velocity data to.</param>
        ///// <param name="count">Number of body pairs in the bundle.</param>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public unsafe void ScatterVelocities(
        //    ref BodyVelocityWide sourceVelocitiesA, ref BodyVelocityWide sourceVelocitiesB, ref BodyVelocityWide sourceVelocitiesC, ref BodyVelocityWide sourceVelocitiesD,
        //    ref FourBodyReferences references, int count)
        //{
        //    Debug.Assert(count >= 0 && count <= Vector<float>.Count);
        //    //Grab the base references for the body indices. Note that we make use of the references memory layout again.
        //    ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
        //    ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);
        //    ref var baseIndexC = ref Unsafe.As<Vector<int>, int>(ref references.IndexC);
        //    ref var baseIndexD = ref Unsafe.As<Vector<int>, int>(ref references.IndexD);
        //    for (int i = 0; i < count; ++i)
        //    {
        //        ScatterVelocities(ref sourceVelocitiesA, ref baseIndexA, i);
        //        ScatterVelocities(ref sourceVelocitiesB, ref baseIndexB, i);
        //        ScatterVelocities(ref sourceVelocitiesC, ref baseIndexC, i);
        //        ScatterVelocities(ref sourceVelocitiesD, ref baseIndexD, i);
        //    }
        //}
    }
}
