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
        private static void WriteGatherInertia(ref int bundleBaseBodyIndexInSet, int bodyIndexInBundle, ref Buffer<BodyInertia> states, ref BodyInertias gatheredInertias)
        {
            ref var source = ref states[Unsafe.Add(ref bundleBaseBodyIndexInSet, bodyIndexInBundle)];
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
        private static void WriteGatherState(ref int bundleBaseBodyIndexInSet, int bodyIndexInBundle, ref Buffer<MotionState> states,
            ref Vector3Wide position, ref QuaternionWide orientation, ref BodyVelocities velocity)
        {
            ref var state = ref states[Unsafe.Add(ref bundleBaseBodyIndexInSet, bodyIndexInBundle)];
            Vector3Wide.WriteFirst(state.Pose.Position, ref GetOffsetInstance(ref position, bodyIndexInBundle));
            QuaternionWide.WriteFirst(state.Pose.Orientation, ref GetOffsetInstance(ref orientation, bodyIndexInBundle));
            Vector3Wide.WriteFirst(state.Velocity.Linear, ref GetOffsetInstance(ref velocity.Linear, bodyIndexInBundle));
            Vector3Wide.WriteFirst(state.Velocity.Angular, ref GetOffsetInstance(ref velocity.Angular, bodyIndexInBundle));
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void WriteGatherState(ref int bundleBaseBodyIndexInSet, int bodyIndexInBundle, ref Buffer<MotionState> states,
            ref Vector3Wide position, ref QuaternionWide orientation, ref BodyVelocities velocity, ref BodyInertias inertia)
        {
            ref var state = ref states[Unsafe.Add(ref bundleBaseBodyIndexInSet, bodyIndexInBundle)];
            Vector3Wide.WriteFirst(state.Pose.Position, ref GetOffsetInstance(ref position, bodyIndexInBundle));
            QuaternionWide.WriteFirst(state.Pose.Orientation, ref GetOffsetInstance(ref orientation, bodyIndexInBundle));
            Vector3Wide.WriteFirst(state.Velocity.Linear, ref GetOffsetInstance(ref velocity.Linear, bodyIndexInBundle));
            Vector3Wide.WriteFirst(state.Velocity.Angular, ref GetOffsetInstance(ref velocity.Angular, bodyIndexInBundle));

            //ref var targetSlot = ref GetOffsetInstance(ref inertia, bodyIndexInBundle);
            //GetFirst(ref targetSlot.InverseInertiaTensor.XX) = (float)state.PackedLocalInertia.InverseNormalizedInertiaXX * state.PackedLocalInertia.InverseMass;
            //GetFirst(ref targetSlot.InverseInertiaTensor.YX) = 0;
            //GetFirst(ref targetSlot.InverseInertiaTensor.YY) = (float)state.PackedLocalInertia.InverseNormalizedInertiaYY * state.PackedLocalInertia.InverseMass;
            //GetFirst(ref targetSlot.InverseInertiaTensor.ZX) = 0;
            //GetFirst(ref targetSlot.InverseInertiaTensor.ZY) = 0;
            //GetFirst(ref targetSlot.InverseInertiaTensor.ZZ) = (float)state.PackedLocalInertia.InverseNormalizedInertiaZZ * state.PackedLocalInertia.InverseMass;
            //GetFirst(ref targetSlot.InverseMass) = state.PackedLocalInertia.InverseMass;
            ref var targetSlot = ref GetOffsetInstance(ref inertia, bodyIndexInBundle);
            GetFirst(ref targetSlot.InverseInertiaTensor.XX) = state.PackedLocalInertia.InverseMass;
            GetFirst(ref targetSlot.InverseInertiaTensor.YX) = 0;
            GetFirst(ref targetSlot.InverseInertiaTensor.YY) = state.PackedLocalInertia.InverseMass;
            GetFirst(ref targetSlot.InverseInertiaTensor.ZX) = 0;
            GetFirst(ref targetSlot.InverseInertiaTensor.ZY) = 0;
            GetFirst(ref targetSlot.InverseInertiaTensor.ZZ) = state.PackedLocalInertia.InverseMass;
            GetFirst(ref targetSlot.InverseMass) = state.PackedLocalInertia.InverseMass;
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
            out Vector3Wide position, out QuaternionWide orientation, out BodyVelocities velocity, out BodyInertias inertia)
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

            ref var states = ref ActiveSet.MotionStates;
            for (int i = 0; i < count; ++i)
            {
                WriteGatherState(ref baseIndex, i, ref states, ref position, ref orientation, ref velocity);
                WriteGatherInertia(ref baseIndex, i, ref Inertias, ref inertia);
            }
        }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe static void ScalarGather(int count, MotionState* motionStates, ref Vector<int> baseIndex, ref Vector3Wide position, ref QuaternionWide orientation, ref BodyVelocities velocity, ref BodyInertias inertia)
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
                var stateValues = (float*)(motionStates + index);
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
                pMass[i] = stateValues[MotionState.OffsetToInverseMass];
                pInertiaXX[i] = stateValues[MotionState.OffsetToInverseInertiaXX];
                pInertiaYX[i] = 0;
                pInertiaYY[i] = stateValues[MotionState.OffsetToInverseInertiaYY];
                pInertiaZX[i] = 0;
                pInertiaZY[i] = 0;
                pInertiaZZ[i] = stateValues[MotionState.OffsetToInverseInertiaZZ];
            }
        }
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        unsafe static void TransposingGather(int count, MotionState* motionStates, ref Vector<int> baseIndex, ref Vector3Wide position, ref QuaternionWide orientation, ref BodyVelocities velocity, ref BodyInertias inertia)
        {
            if (Avx.IsSupported)
            {
                var indices = (int*)Unsafe.AsPointer(ref baseIndex);

                var s0 = (float*)(motionStates + indices[0]);
                var s1 = (float*)(motionStates + indices[1]);
                var s2 = (float*)(motionStates + indices[2]);
                var s3 = (float*)(motionStates + indices[3]);
                var s4 = (float*)(motionStates + indices[4]);
                var s5 = (float*)(motionStates + indices[5]);
                var s6 = (float*)(motionStates + indices[6]);
                var s7 = (float*)(motionStates + indices[7]);

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
                    var m0 = Avx.LoadVector256(s0);
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

                    orientation.X = Avx.Permute2x128(o0, o1, 0 | (2 << 4)).AsVector();
                    orientation.Y = Avx.Permute2x128(o4, o5, 0 | (2 << 4)).AsVector();
                    orientation.Z = Avx.Permute2x128(o2, o3, 0 | (2 << 4)).AsVector();
                    orientation.W = Avx.Permute2x128(o6, o7, 0 | (2 << 4)).AsVector();
                    position.X = Avx.Permute2x128(o0, o1, 1 | (3 << 4)).AsVector();
                    position.Y = Avx.Permute2x128(o4, o5, 1 | (3 << 4)).AsVector();
                    position.Z = Avx.Permute2x128(o2, o3, 1 | (3 << 4)).AsVector();
                    inertia.InverseMass = Avx.Permute2x128(o6, o7, 1 | (3 << 4)).AsVector();
                    inertia.InverseInertiaTensor.XX = inertia.InverseMass;
                    inertia.InverseInertiaTensor.YY = inertia.InverseMass;
                    inertia.InverseInertiaTensor.ZZ = inertia.InverseMass;
                    inertia.InverseInertiaTensor.YX = default;
                    inertia.InverseInertiaTensor.ZX = default;
                    inertia.InverseInertiaTensor.ZY = default;
                }

                {
                    //Second half.
                    var m0 = Avx.LoadVector256(s0 + 8);
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

                    velocity.Linear.X = Avx.Permute2x128(o0, o1, 0 | (2 << 4)).AsVector();
                    velocity.Linear.Y = Avx.Permute2x128(o4, o5, 0 | (2 << 4)).AsVector();
                    velocity.Linear.Z = Avx.Permute2x128(o2, o3, 0 | (2 << 4)).AsVector();
                    velocity.Angular.X = Avx.Permute2x128(o6, o7, 0 | (2 << 4)).AsVector();
                    velocity.Angular.Y = Avx.Permute2x128(o0, o1, 1 | (3 << 4)).AsVector();
                    velocity.Angular.Z = Avx.Permute2x128(o4, o5, 1 | (3 << 4)).AsVector();
                }
            }
            else
            {
                ScalarGather(count, motionStates, ref baseIndex, ref position, ref orientation, ref velocity, ref inertia);
            }
        }

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
        out QuaternionWide orientationA, out BodyVelocities velocityA, out BodyInertias inertiaA,
        out Vector3Wide ab,
        out QuaternionWide orientationB, out BodyVelocities velocityB, out BodyInertias inertiaB)
        {
            Unsafe.SkipInit(out Vector3Wide positionA);
            Unsafe.SkipInit(out Vector3Wide positionB);
            Unsafe.SkipInit(out orientationA);
            Unsafe.SkipInit(out orientationB);
            Unsafe.SkipInit(out velocityA);
            Unsafe.SkipInit(out velocityB);
            Unsafe.SkipInit(out inertiaA);
            Unsafe.SkipInit(out inertiaB);
            Debug.Assert(count >= 0 && count <= Vector<float>.Count);
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
            ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);

            ref var states = ref ActiveSet.MotionStates;

            TransposingGather(count, states.Memory, ref references.IndexA, ref positionA, ref orientationA, ref velocityA, ref inertiaA);
            TransposingGather(count, states.Memory, ref references.IndexB, ref positionB, ref orientationB, ref velocityB, ref inertiaB);

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
            out QuaternionWide orientationA, out BodyVelocities velocityA, out BodyInertias inertiaA,
            out Vector3Wide ab,
            out QuaternionWide orientationB, out BodyVelocities velocityB, out BodyInertias inertiaB,
            out Vector3Wide ac,
            out QuaternionWide orientationC, out BodyVelocities velocityC, out BodyInertias inertiaC)
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

            ref var states = ref ActiveSet.MotionStates;
            for (int i = 0; i < count; ++i)
            {
                WriteGatherState(ref baseIndexA, i, ref states, ref positionA, ref orientationA, ref velocityA);
                WriteGatherInertia(ref baseIndexA, i, ref Inertias, ref inertiaA);
                WriteGatherState(ref baseIndexB, i, ref states, ref positionB, ref orientationB, ref velocityB);
                WriteGatherInertia(ref baseIndexB, i, ref Inertias, ref inertiaB);
                WriteGatherState(ref baseIndexC, i, ref states, ref positionC, ref orientationC, ref velocityC);
                WriteGatherInertia(ref baseIndexC, i, ref Inertias, ref inertiaC);
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
            out QuaternionWide orientationA, out BodyVelocities velocityA, out BodyInertias inertiaA,
            out Vector3Wide ab,
            out QuaternionWide orientationB, out BodyVelocities velocityB, out BodyInertias inertiaB,
            out Vector3Wide ac,
            out QuaternionWide orientationC, out BodyVelocities velocityC, out BodyInertias inertiaC,
            out Vector3Wide ad,
            out QuaternionWide orientationD, out BodyVelocities velocityD, out BodyInertias inertiaD)
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

            ref var states = ref ActiveSet.MotionStates;
            for (int i = 0; i < count; ++i)
            {
                WriteGatherState(ref baseIndexA, i, ref states, ref positionA, ref orientationA, ref velocityA);
                WriteGatherInertia(ref baseIndexA, i, ref Inertias, ref inertiaA);
                WriteGatherState(ref baseIndexB, i, ref states, ref positionB, ref orientationB, ref velocityB);
                WriteGatherInertia(ref baseIndexB, i, ref Inertias, ref inertiaB);
                WriteGatherState(ref baseIndexC, i, ref states, ref positionC, ref orientationC, ref velocityC);
                WriteGatherInertia(ref baseIndexC, i, ref Inertias, ref inertiaC);
                WriteGatherState(ref baseIndexD, i, ref states, ref positionD, ref orientationD, ref velocityD);
                WriteGatherInertia(ref baseIndexD, i, ref Inertias, ref inertiaD);
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
        //private static void WriteLinearGatherState(ref int baseBodyIndexInSet, int bodyIndexInBundle, ref Buffer<MotionState> states, ref Vector3Wide linearVelocity, ref Vector3Wide position)
        //{
        //    ref var state = ref states[Unsafe.Add(ref baseBodyIndexInSet, bodyIndexInBundle)];
        //    Vector3Wide.WriteFirst(state.Pose.Position, ref GetOffsetInstance(ref position, bodyIndexInBundle));
        //    Vector3Wide.WriteFirst(state.Velocity.Linear, ref GetOffsetInstance(ref linearVelocity, bodyIndexInBundle));
        //}
        ///// <summary>
        ///// Gathers relative positions and linear velocities for a two body bundle into an AOSOA bundle.
        ///// </summary>
        ///// <param name="references">Active body indices being gathered.</param>
        ///// <param name="count">Number of body pairs in the bundle.</param>
        ///// <param name="ab">Gathered offset from body A to body B.</param>
        ///// <param name="linearVelocityA">Linear velocity of body A.</param>
        ///// <param name="linearVelocityB">Linear velocity of body B.</param>
        ////[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public void GatherLinearState(ref TwoBodyReferences references, int count, out Vector3Wide ab, out Vector3Wide linearVelocityA, out Vector3Wide linearVelocityB)
        //{
        //    Unsafe.SkipInit(out Vector3Wide positionA);
        //    Unsafe.SkipInit(out Vector3Wide positionB);
        //    Unsafe.SkipInit(out linearVelocityA);
        //    Unsafe.SkipInit(out linearVelocityB);
        //    Debug.Assert(count >= 0 && count <= Vector<float>.Count);
        //    //Grab the base references for the body indices. Note that we make use of the references memory layout again.
        //    ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
        //    ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);

        //    ref var states = ref ActiveSet.MotionStates;
        //    for (int i = 0; i < count; ++i)
        //    {
        //        WriteLinearGatherState(ref baseIndexA, i, ref states, ref positionA, ref linearVelocityA);
        //        WriteLinearGatherState(ref baseIndexB, i, ref states, ref positionB, ref linearVelocityB);
        //    }
        //    //Same as other gather case; this is sensitive to changes in the representation of body position. In high precision modes, this'll need to change.
        //    Vector3Wide.Subtract(positionB, positionA, out ab);
        //}


        ///// <summary>
        ///// Gathers relative positions and linear velocities for a three body bundle into an AOSOA bundle.
        ///// </summary>
        ///// <param name="references">Active body indices being gathered.</param>
        ///// <param name="count">Number of body pairs in the bundle.</param>
        ///// <param name="ab">Gathered offset from body A to body B.</param>
        ///// <param name="ac">Gathered offset from body A to body C.</param>
        ///// <param name="linearVelocityA">Linear velocity of body A.</param>
        ///// <param name="linearVelocityB">Linear velocity of body B.</param>
        ///// <param name="linearVelocityC">Linear velocity of body C.</param>
        ////[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public void GatherLinearState(ref ThreeBodyReferences references, int count, out Vector3Wide ab, out Vector3Wide ac, out Vector3Wide linearVelocityA, out Vector3Wide linearVelocityB, out Vector3Wide linearVelocityC)
        //{
        //    Unsafe.SkipInit(out Vector3Wide positionA);
        //    Unsafe.SkipInit(out Vector3Wide positionB);
        //    Unsafe.SkipInit(out Vector3Wide positionC);
        //    Unsafe.SkipInit(out linearVelocityA);
        //    Unsafe.SkipInit(out linearVelocityB);
        //    Unsafe.SkipInit(out linearVelocityC);
        //    Debug.Assert(count >= 0 && count <= Vector<float>.Count);
        //    //Grab the base references for the body indices. Note that we make use of the references memory layout again.
        //    ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
        //    ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);
        //    ref var baseIndexC = ref Unsafe.As<Vector<int>, int>(ref references.IndexC);

        //    ref var states = ref ActiveSet.MotionStates;
        //    for (int i = 0; i < count; ++i)
        //    {
        //        WriteLinearGatherState(ref baseIndexA, i, ref states, ref positionA, ref linearVelocityA);
        //        WriteLinearGatherState(ref baseIndexB, i, ref states, ref positionB, ref linearVelocityB);
        //        WriteLinearGatherState(ref baseIndexC, i, ref states, ref positionC, ref linearVelocityC);
        //    }
        //    //Same as other gather case; this is sensitive to changes in the representation of body position. In high precision modes, this'll need to change.
        //    Vector3Wide.Subtract(positionB, positionA, out ab);
        //    Vector3Wide.Subtract(positionC, positionA, out ac);
        //}


        ///// <summary>
        ///// Gathers relative positions and linear velocities for a four body bundle into an AOSOA bundle.
        ///// </summary>
        ///// <param name="references">Active body indices being gathered.</param>
        ///// <param name="count">Number of body pairs in the bundle.</param>
        ///// <param name="ab">Gathered offset from body A to body B.</param>
        ///// <param name="ac">Gathered offset from body A to body C.</param>
        ///// <param name="ad">Gathered offset from body A to body D.</param>
        ///// <param name="linearVelocityA">Linear velocity of body A.</param>
        ///// <param name="linearVelocityB">Linear velocity of body B.</param>
        ///// <param name="linearVelocityC">Linear velocity of body C.</param>
        ///// <param name="linearVelocityD">Linear velocity of body D.</param>
        ////[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public void GatherLinearState(ref FourBodyReferences references, int count, out Vector3Wide ab, out Vector3Wide ac, out Vector3Wide ad,
        //    out Vector3Wide linearVelocityA, out Vector3Wide linearVelocityB, out Vector3Wide linearVelocityC, out Vector3Wide linearVelocityD)
        //{
        //    Unsafe.SkipInit(out Vector3Wide positionA);
        //    Unsafe.SkipInit(out Vector3Wide positionB);
        //    Unsafe.SkipInit(out Vector3Wide positionC);
        //    Unsafe.SkipInit(out Vector3Wide positionD);
        //    Unsafe.SkipInit(out linearVelocityA);
        //    Unsafe.SkipInit(out linearVelocityB);
        //    Unsafe.SkipInit(out linearVelocityC);
        //    Unsafe.SkipInit(out linearVelocityD);
        //    Debug.Assert(count >= 0 && count <= Vector<float>.Count);
        //    //Grab the base references for the body indices. Note that we make use of the references memory layout again.
        //    ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
        //    ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);
        //    ref var baseIndexC = ref Unsafe.As<Vector<int>, int>(ref references.IndexC);
        //    ref var baseIndexD = ref Unsafe.As<Vector<int>, int>(ref references.IndexD);

        //    ref var states = ref ActiveSet.MotionStates;
        //    for (int i = 0; i < count; ++i)
        //    {
        //        WriteLinearGatherState(ref baseIndexA, i, ref states, ref positionA, ref linearVelocityA);
        //        WriteLinearGatherState(ref baseIndexB, i, ref states, ref positionB, ref linearVelocityB);
        //        WriteLinearGatherState(ref baseIndexC, i, ref states, ref positionC, ref linearVelocityC);
        //        WriteLinearGatherState(ref baseIndexD, i, ref states, ref positionD, ref linearVelocityD);
        //    }
        //    //Same as other gather case; this is sensitive to changes in the representation of body position. In high precision modes, this'll need to change.
        //    Vector3Wide.Subtract(positionB, positionA, out ab);
        //    Vector3Wide.Subtract(positionC, positionA, out ac);
        //    Vector3Wide.Subtract(positionD, positionA, out ad);
        //}


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private unsafe void ScatterVelocities(ref BodyVelocities sourceVelocities, ref int baseIndex, int innerIndex)
        {
            //TODO: How much value would there be in branching on kinematic state and avoiding a write? Depends a lot on the number of kinematics.
            ref var sourceSlot = ref GetOffsetInstance(ref sourceVelocities, innerIndex);
            ref var target = ref ActiveSet.MotionStates[Unsafe.Add(ref baseIndex, innerIndex)].Velocity;
            target.Linear = new Vector3(sourceSlot.Linear.X[0], sourceSlot.Linear.Y[0], sourceSlot.Linear.Z[0]);
            target.Angular = new Vector3(sourceSlot.Angular.X[0], sourceSlot.Angular.Y[0], sourceSlot.Angular.Z[0]);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private unsafe void TransposeScatterVelocities(ref BodyVelocities sourceVelocities, MotionState* motionStates, ref Vector<int> references, int count)
        {
            if (Avx.IsSupported)
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
                Avx.StoreAligned((float*)(motionStates + indices[0]) + 8, Avx.Permute2x128(o0, o1, 0 | (2 << 4)));
                if (count > 1) Avx.StoreAligned((float*)(motionStates + indices[1]) + 8, Avx.Permute2x128(o4, o5, 0 | (2 << 4)));
                if (count > 2) Avx.StoreAligned((float*)(motionStates + indices[2]) + 8, Avx.Permute2x128(o2, o3, 0 | (2 << 4)));
                if (count > 3) Avx.StoreAligned((float*)(motionStates + indices[3]) + 8, Avx.Permute2x128(o6, o7, 0 | (2 << 4)));
                if (count > 4) Avx.StoreAligned((float*)(motionStates + indices[4]) + 8, Avx.Permute2x128(o0, o1, 1 | (3 << 4)));
                if (count > 5) Avx.StoreAligned((float*)(motionStates + indices[5]) + 8, Avx.Permute2x128(o4, o5, 1 | (3 << 4)));
                if (count > 6) Avx.StoreAligned((float*)(motionStates + indices[6]) + 8, Avx.Permute2x128(o2, o3, 1 | (3 << 4)));
                if (count > 7) Avx.StoreAligned((float*)(motionStates + indices[7]) + 8, Avx.Permute2x128(o6, o7, 1 | (3 << 4)));
            }
            else
            {

            }
        }


        /// <summary>
        /// Scatters velocities for one body bundle into the active body set.
        /// </summary>
        /// <param name="sourceVelocities">Velocities of body bundle A to scatter.</param>
        /// <param name="references">Active set indices of the bodies to scatter velocity data to.</param>
        /// <param name="count">Number of body pairs in the bundle.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void ScatterVelocities(ref BodyVelocities sourceVelocities, ref Vector<int> references, int count)
        {
            Debug.Assert(count >= 0 && count <= Vector<float>.Count);
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndex = ref Unsafe.As<Vector<int>, int>(ref references);
            for (int i = 0; i < count; ++i)
            {
                ScatterVelocities(ref sourceVelocities, ref baseIndex, i);
            }
        }

        /// <summary>
        /// Scatters velocities for two body bundles into the active body set.
        /// </summary>
        /// <param name="sourceVelocitiesA">Velocities of body bundle A to scatter.</param>
        /// <param name="sourceVelocitiesA">Velocities of body bundle B to scatter.</param>
        /// <param name="references">Active set indices of the bodies to scatter velocity data to.</param>
        /// <param name="count">Number of body pairs in the bundle.</param>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        [MethodImpl(MethodImplOptions.NoInlining)]
        public unsafe void ScatterVelocities(ref BodyVelocities sourceVelocitiesA, ref BodyVelocities sourceVelocitiesB,
            ref TwoBodyReferences references, int count)
        {
            Debug.Assert(count >= 0 && count <= Vector<float>.Count);
            var motionStates = ActiveSet.MotionStates.Memory;
            TransposeScatterVelocities(ref sourceVelocitiesA, motionStates, ref references.IndexA, count);
            TransposeScatterVelocities(ref sourceVelocitiesB, motionStates, ref references.IndexB, count);
            ////Grab the base references for the body indices. Note that we make use of the references memory layout again.
            //ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
            //ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);
            //for (int i = 0; i < count; ++i)
            //{
            //    ScatterVelocities(ref sourceVelocitiesA, ref baseIndexA, i);
            //    ScatterVelocities(ref sourceVelocitiesB, ref baseIndexB, i);
            //}
        }

        /// <summary>
        /// Scatters velocities for three body bundles into the active body set.
        /// </summary>
        /// <param name="sourceVelocitiesA">Velocities of body bundle A to scatter.</param>
        /// <param name="sourceVelocitiesB">Velocities of body bundle B to scatter.</param>
        /// <param name="sourceVelocitiesC">Velocities of body bundle C to scatter.</param>
        /// <param name="references">Active set indices of the bodies to scatter velocity data to.</param>
        /// <param name="count">Number of body pairs in the bundle.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void ScatterVelocities(
            ref BodyVelocities sourceVelocitiesA, ref BodyVelocities sourceVelocitiesB, ref BodyVelocities sourceVelocitiesC,
            ref ThreeBodyReferences references, int count)
        {
            Debug.Assert(count >= 0 && count <= Vector<float>.Count);
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
            ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);
            ref var baseIndexC = ref Unsafe.As<Vector<int>, int>(ref references.IndexC);
            for (int i = 0; i < count; ++i)
            {
                ScatterVelocities(ref sourceVelocitiesA, ref baseIndexA, i);
                ScatterVelocities(ref sourceVelocitiesB, ref baseIndexB, i);
                ScatterVelocities(ref sourceVelocitiesC, ref baseIndexC, i);
            }
        }

        /// <summary>
        /// Scatters velocities for four body bundles into the active body set.
        /// </summary>
        /// <param name="sourceVelocitiesA">Velocities of body bundle A to scatter.</param>
        /// <param name="sourceVelocitiesB">Velocities of body bundle B to scatter.</param>
        /// <param name="sourceVelocitiesC">Velocities of body bundle C to scatter.</param>
        /// <param name="sourceVelocitiesD">Velocities of body bundle D to scatter.</param>
        /// <param name="references">Active set indices of the bodies to scatter velocity data to.</param>
        /// <param name="count">Number of body pairs in the bundle.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void ScatterVelocities(
            ref BodyVelocities sourceVelocitiesA, ref BodyVelocities sourceVelocitiesB, ref BodyVelocities sourceVelocitiesC, ref BodyVelocities sourceVelocitiesD,
            ref FourBodyReferences references, int count)
        {
            Debug.Assert(count >= 0 && count <= Vector<float>.Count);
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
            ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);
            ref var baseIndexC = ref Unsafe.As<Vector<int>, int>(ref references.IndexC);
            ref var baseIndexD = ref Unsafe.As<Vector<int>, int>(ref references.IndexD);
            for (int i = 0; i < count; ++i)
            {
                ScatterVelocities(ref sourceVelocitiesA, ref baseIndexA, i);
                ScatterVelocities(ref sourceVelocitiesB, ref baseIndexB, i);
                ScatterVelocities(ref sourceVelocitiesC, ref baseIndexC, i);
                ScatterVelocities(ref sourceVelocitiesD, ref baseIndexD, i);
            }
        }
    }
}
