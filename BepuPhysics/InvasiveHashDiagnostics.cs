using BepuUtilities.Collections;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace BepuPhysics
{
    /// <summary>
    /// Hardcoded hash types used by invasive hash diagnostics.
    /// </summary>
    public enum HashDiagnosticType
    {
        AwakeBodyStates0,
        AwakeBodyStates1,
        AwakeBodyStates2,
        AwakeBodyStates3,
        AwakeBodyStates4,
        AwakeBodyStates5,
        AwakeBodyStates6,
        AwakeBodyStates7,
        AwakeBodyStates8,
        AwakeBodyStates9,
        AwakeBodyStates10,
        AwakeBodyCollidableStates0,
        AwakeBodyCollidableStates1,
        AwakeBodyCollidableStates2,
        AwakeBodyCollidableStates3,
        AwakeBodyCollidableStates4,
        AwakeBodyCollidableStates5,
        AwakeBodyCollidableStates6,
        AwakeBodyCollidableStates7,
        AwakeBodyCollidableStates8,
        AwakeBodyCollidableStates9,
        AwakeBodyCollidableStates10,
        AddSleepingToActiveForFallback,
        SolverBodyReferenceBeforeCollisionDetection,
        SolverBodyReferenceBeforePreflush,
        SolverBodyReferenceAfterPreflushPhase1,
        SolverBodyReferenceAfterPreflushPhase2,
        SolverBodyReferenceAfterPreflushPhase3,
        SolverBodyReferenceAfterPreflush,
        SolverBodyReferenceBeforeSolver,
        SolverBodyReferenceAfterSolver,
        SolverBodyReferenceAtEnd,
        DeterministicConstraintAdd,
        AddToSimulationSpeculative,
        AddToSimulationSpeculativeFallbackSolverReferences,
        EnqueueStaleRemoval,
        RemoveConstraintsFromFallbackBatchReferencedHandles,
        RemoveConstraintsFromBatchReferencedHandles,
        RemoveConstraintsFromBodyLists,
        RemoveConstraintsFromTypeBatch,
        ReturnConstraintHandles,
        PreflushJobs,
        AllocateInTypeBatchForFallback,
        AllocateInTypeBatchForFallbackProbes,
        AllocateInBatch,
        TypeProcessorRemove
    }
    /// <summary>
    /// Helper diagnostics class for monitoring internal state determinism across runs.
    /// Typically used by inserting tests into engine internals.
    /// </summary>
    public class InvasiveHashDiagnostics
    {
        /// <summary>
        /// This is meant as an internal diagnostic utility, so hardcoding some things is totally fine.
        /// </summary>
        const int HashTypeCount = 46;
        public static InvasiveHashDiagnostics Instance;
        public static void Initialize(int runCount, int hashCapacityPerType)
        {
            var instance = new InvasiveHashDiagnostics();
            instance.Hashes = new int[runCount][][];
            for (int runIndex = 0; runIndex < runCount; ++runIndex)
            {
                instance.Hashes[runIndex] = new int[HashTypeCount][];
                for (int hashTypeIndex = 0; hashTypeIndex < HashTypeCount; ++hashTypeIndex)
                {
                    instance.Hashes[runIndex][hashTypeIndex] = new int[hashCapacityPerType];
                }
            }
            Instance = instance;
        }

        public int CurrentRunIndex;
        public int CurrentHashIndex;
        public int[][][] Hashes;

        public bool TypeIsActive(HashDiagnosticType hashType)
        {
            return CurrentRunIndex >= 0 && CurrentRunIndex < Hashes.Length && (int)hashType >= 0 && (int)hashType < Hashes[CurrentRunIndex].Length && CurrentHashIndex >= 0 && CurrentHashIndex < Hashes[CurrentRunIndex][(int)hashType].Length;
        }

        public void MoveToNextRun()
        {
            ++CurrentRunIndex;
            CurrentHashIndex = 0;
        }

        public void MoveToNextHashFrame()
        {
            if (CurrentRunIndex > Hashes.Length)
                return;
            if (CurrentHashIndex < 0)
                throw new ArgumentException($"Invalid hash index: {CurrentHashIndex}");
            bool anyFailed = false;
            for (int hashTypeIndex = 0; hashTypeIndex < Hashes[CurrentRunIndex].Length; ++hashTypeIndex)
            {
                if (CurrentHashIndex >= Hashes[CurrentRunIndex][hashTypeIndex].Length)
                    continue;
                for (int previousRunIndex = 0; previousRunIndex < CurrentRunIndex; ++previousRunIndex)
                {
                    if (Hashes[CurrentRunIndex][hashTypeIndex][CurrentHashIndex] != Hashes[previousRunIndex][hashTypeIndex][CurrentHashIndex])
                    {
                        Console.WriteLine($"Hash failure on {(HashDiagnosticType)hashTypeIndex} frame {CurrentHashIndex}, current run {CurrentRunIndex} vs previous {previousRunIndex}: {Hashes[CurrentRunIndex][hashTypeIndex][CurrentHashIndex] } vs {Hashes[previousRunIndex][hashTypeIndex][CurrentHashIndex]}.");
                        anyFailed = true;
                    }
                }
            }
            if (anyFailed)
            {
                Console.WriteLine("Press enter to continue.");
                Console.ReadLine();
            }
            ++CurrentHashIndex;
        }

        public ref int GetHashForType(HashDiagnosticType hashType)
        {
            return ref Hashes[CurrentRunIndex][(int)hashType][CurrentHashIndex];
        }

        public void ContributeToHash(ref int hash, int value)
        {
            hash = HashHelper.Rehash(hash ^ value);
        }
        public void ContributeToHash<T>(ref int hash, T value) where T : unmanaged
        {
            var intCount = Unsafe.SizeOf<T>() / 4;
            ref var intBase = ref Unsafe.As<T, int>(ref value);
            for (int i = 0; i < intCount; ++i)
            {
                ContributeToHash(ref hash, Unsafe.Add(ref intBase, i));
            }
            ref var byteBase = ref Unsafe.As<int, byte>(ref Unsafe.Add(ref intBase, intCount));
            var byteRemainder = Unsafe.SizeOf<T>() - intCount * 4;
            for (int i = 0; i < byteRemainder; ++i)
            {
                ContributeToHash(ref hash, Unsafe.Add(ref byteBase, i));
            }
        }

        public void ContributeToHash(HashDiagnosticType hashType, int value)
        {
            ContributeToHash(ref GetHashForType(hashType), value);
        }
        public void ContributeToHash<T>(HashDiagnosticType hashType, T value) where T : unmanaged
        {
            ContributeToHash(ref GetHashForType(hashType), value);
        }
    }
}
