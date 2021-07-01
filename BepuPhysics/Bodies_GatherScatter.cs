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


            if (Avx2.IsSupported)
            {
                var maskMemory = stackalloc int[8];
                for (int i = 0; i < count; ++i)
                {
                    maskMemory[i] = -1;
                }
                for (int i = count; i < 8; ++i)
                {
                    maskMemory[i] = 0;
                }
                var mask = Avx.LoadVector256(maskMemory);
                var statesMemory = (float*)states.Memory;
                var indexA = Avx2.ShiftLeftLogical(references.IndexA.AsVector256(), 6);
                var indexB = Avx2.ShiftLeftLogical(references.IndexB.AsVector256(), 6);
                var zero = Vector256<float>.Zero;
                var offsetX = Vector256.Create(MotionState.ByteOffsetToPositionX);
                positionA.X = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexA, offsetX), mask.AsSingle(), 1).AsVector();
                positionB.X = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexB, offsetX), mask.AsSingle(), 1).AsVector();
                var offsetY = Vector256.Create(MotionState.ByteOffsetToPositionY);
                positionA.Y = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexA, offsetY), mask.AsSingle(), 1).AsVector();
                positionB.Y = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexB, offsetY), mask.AsSingle(), 1).AsVector();
                var offsetZ = Vector256.Create(MotionState.ByteOffsetToPositionZ);
                positionA.Z = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexA, offsetZ), mask.AsSingle(), 1).AsVector();
                positionB.Z = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexB, offsetZ), mask.AsSingle(), 1).AsVector();

                var offsetOrientationX = Vector256.Create(MotionState.ByteOffsetToOrientationX);
                orientationA.X = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexA, offsetOrientationX), mask.AsSingle(), 1).AsVector();
                orientationB.X = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexB, offsetOrientationX), mask.AsSingle(), 1).AsVector();
                var offsetOrientationY = Vector256.Create(MotionState.ByteOffsetToOrientationY);
                orientationA.Y = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexA, offsetOrientationY), mask.AsSingle(), 1).AsVector();
                orientationB.Y = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexB, offsetOrientationY), mask.AsSingle(), 1).AsVector();
                var offsetOrientationZ = Vector256.Create(MotionState.ByteOffsetToOrientationZ);
                orientationA.Z = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexA, offsetOrientationZ), mask.AsSingle(), 1).AsVector();
                orientationB.Z = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexB, offsetOrientationZ), mask.AsSingle(), 1).AsVector();
                var offsetOrientationW = Vector256.Create(MotionState.ByteOffsetToOrientationW);
                orientationA.W = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexA, offsetOrientationW), mask.AsSingle(), 1).AsVector();
                orientationB.W = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexB, offsetOrientationW), mask.AsSingle(), 1).AsVector();

                var offsetLinearX = Vector256.Create(MotionState.ByteOffsetToLinearX);
                velocityA.Linear.X = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexA, offsetLinearX), mask.AsSingle(), 1).AsVector();
                velocityB.Linear.X = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexB, offsetLinearX), mask.AsSingle(), 1).AsVector();
                var offsetLinearY = Vector256.Create(MotionState.ByteOffsetToLinearY);
                velocityA.Linear.Y = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexA, offsetLinearY), mask.AsSingle(), 1).AsVector();
                velocityB.Linear.Y = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexB, offsetLinearY), mask.AsSingle(), 1).AsVector();
                var offsetLinearZ = Vector256.Create(MotionState.ByteOffsetToLinearZ);
                velocityA.Linear.Z = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexA, offsetLinearZ), mask.AsSingle(), 1).AsVector();
                velocityB.Linear.Z = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexB, offsetLinearZ), mask.AsSingle(), 1).AsVector();

                var offsetAngularX = Vector256.Create(MotionState.ByteOffsetToAngularX);
                velocityA.Angular.X = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexA, offsetAngularX), mask.AsSingle(), 1).AsVector();
                velocityB.Angular.X = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexB, offsetAngularX), mask.AsSingle(), 1).AsVector();
                var offsetAngularY = Vector256.Create(MotionState.ByteOffsetToAngularY);
                velocityA.Angular.Y = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexA, offsetAngularY), mask.AsSingle(), 1).AsVector();
                velocityB.Angular.Y = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexB, offsetAngularY), mask.AsSingle(), 1).AsVector();
                var offsetAngularZ = Vector256.Create(MotionState.ByteOffsetToAngularZ);
                velocityA.Angular.Z = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexA, offsetAngularZ), mask.AsSingle(), 1).AsVector();
                velocityB.Angular.Z = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexB, offsetAngularZ), mask.AsSingle(), 1).AsVector();

                var offsetInverseMass = Vector256.Create(MotionState.ByteOffsetToInverseMass);
                inertiaA.InverseMass = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexA, offsetInverseMass), mask.AsSingle(), 1).AsVector();
                inertiaB.InverseMass = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexB, offsetInverseMass), mask.AsSingle(), 1).AsVector();
                var offsetInverseInertiaXX = Vector256.Create(MotionState.ByteOffsetToInverseInertiaXX);
                inertiaA.InverseInertiaTensor.XX = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexA, offsetInverseInertiaXX), mask.AsSingle(), 1).AsVector();
                inertiaB.InverseInertiaTensor.XX = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexB, offsetInverseInertiaXX), mask.AsSingle(), 1).AsVector();
                var offsetInverseInertiaYY = Vector256.Create(MotionState.ByteOffsetToInverseInertiaYY);
                inertiaA.InverseInertiaTensor.YY = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexA, offsetInverseInertiaYY), mask.AsSingle(), 1).AsVector();
                inertiaB.InverseInertiaTensor.YY = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexB, offsetInverseInertiaYY), mask.AsSingle(), 1).AsVector();
                var offsetInverseInertiaZZ = Vector256.Create(MotionState.ByteOffsetToInverseInertiaZZ);
                inertiaA.InverseInertiaTensor.ZZ = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexA, offsetInverseInertiaZZ), mask.AsSingle(), 1).AsVector();
                inertiaB.InverseInertiaTensor.ZZ = Avx2.GatherMaskVector256(zero, statesMemory, Avx2.Add(indexB, offsetInverseInertiaZZ), mask.AsSingle(), 1).AsVector();
                inertiaA.InverseInertiaTensor.YX = default;
                inertiaB.InverseInertiaTensor.YX = default;
                inertiaB.InverseInertiaTensor.ZX = default;
                inertiaA.InverseInertiaTensor.ZX = default;
                inertiaA.InverseInertiaTensor.ZY = default;
                inertiaB.InverseInertiaTensor.ZY = default;
            }
            else
            {
                ScalarGather(count, states.Memory, ref references.IndexA, ref positionA, ref orientationA, ref velocityA, ref inertiaA);
                ScalarGather(count, states.Memory, ref references.IndexB, ref positionB, ref orientationB, ref velocityB, ref inertiaB);


                //for (int i = 0; i < count; ++i)
                //{
                //    //WriteGatherState(ref baseIndexA, i, ref states, ref positionA, ref orientationA, ref velocityA);
                //    //WriteGatherInertia(ref baseIndexA, i, ref Inertias, ref inertiaA);
                //    //WriteGatherState(ref baseIndexB, i, ref states, ref positionB, ref orientationB, ref velocityB);
                //    //WriteGatherInertia(ref baseIndexB, i, ref Inertias, ref inertiaB);
                //    WriteGatherState(ref baseIndexA, i, ref states, ref positionA, ref orientationA, ref velocityA, ref inertiaA);
                //    WriteGatherState(ref baseIndexB, i, ref states, ref positionB, ref orientationB, ref velocityB, ref inertiaB);
                //}
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
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void ScatterVelocities(ref BodyVelocities sourceVelocitiesA, ref BodyVelocities sourceVelocitiesB,
            ref TwoBodyReferences references, int count)
        {
            Debug.Assert(count >= 0 && count <= Vector<float>.Count);
            //Grab the base references for the body indices. Note that we make use of the references memory layout again.
            ref var baseIndexA = ref Unsafe.As<Vector<int>, int>(ref references.IndexA);
            ref var baseIndexB = ref Unsafe.As<Vector<int>, int>(ref references.IndexB);
            for (int i = 0; i < count; ++i)
            {
                ScatterVelocities(ref sourceVelocitiesA, ref baseIndexA, i);
                ScatterVelocities(ref sourceVelocitiesB, ref baseIndexB, i);
            }
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
