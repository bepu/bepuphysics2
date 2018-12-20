using BepuPhysics.CollisionDetection;
using BepuUtilities;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Text;

namespace BepuPhysics
{
    public partial class Solver
    {
        struct IncrementalContactDataUpdateFilter : ITypeBatchSolveFilter
        {
            public bool AllowFallback { get { return false; } }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool AllowType(int typeId)
            {
                return NarrowPhase.IsContactConstraintType(typeId);
            }
        }

        struct IncrementalContactUpdateStageFunction : IStageFunction
        {
            public float Dt;
            public float InverseDt;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void Execute(Solver solver, int blockIndex)
            {
                ref var block = ref solver.context.ConstraintBlocks.Blocks[blockIndex];
                ref var typeBatch = ref solver.ActiveSet.Batches[block.BatchIndex].TypeBatches[block.TypeBatchIndex];
                solver.TypeProcessors[typeBatch.TypeId].IncrementallyUpdateContactData(ref typeBatch, solver.bodies, Dt, InverseDt, block.StartBundle, block.End);
            }
        }

        void IncrementalContactUpdateWorker(int workerIndex)
        {
            int start = GetUniformlyDistributedStart(workerIndex, context.ConstraintBlocks.Blocks.Count, context.WorkerCount, 0);

            int syncStage = 0;
            //The claimed and unclaimed state swap after every usage of both pingpong claims buffers.
            int claimedState = 1;
            int unclaimedState = 0;
            var bounds = context.WorkerBoundsA;
            var boundsBackBuffer = context.WorkerBoundsB;
            //Note that every batch has a different start position. Each covers a different subset of constraints, so they require different start locations.
            //The same concept applies to the prestep- the prestep covers all constraints at once, rather than batch by batch.
            var incrementalContactUpdateStage = new IncrementalContactUpdateStageFunction { Dt = context.Dt, InverseDt = 1f / context.Dt };
            Debug.Assert(ActiveSet.Batches.Count > 0, "Don't dispatch if there are no constraints.");
            //Technically this could mutate prestep starts, but at the moment we rebuild starts every frame anyway so it doesn't matter one way or the other.
            ExecuteStage(ref incrementalContactUpdateStage, ref context.ConstraintBlocks, ref bounds, ref boundsBackBuffer, workerIndex, 0, context.ConstraintBlocks.Blocks.Count,
                ref start, ref syncStage, claimedState, unclaimedState);
        }

        internal void IncrementallyUpdateContactConstraints(float dt, IThreadDispatcher threadDispatcher = null)
        {
            if (threadDispatcher == null)
            {
                var inverseDt = 1f / dt;
                ref var activeSet = ref ActiveSet;
                for (int i = 0; i < activeSet.Batches.Count; ++i)
                {
                    ref var batch = ref activeSet.Batches[i];
                    for (int j = 0; j < batch.TypeBatches.Count; ++j)
                    {
                        ref var typeBatch = ref batch.TypeBatches[j];
                        if (NarrowPhase.IsContactConstraintType(typeBatch.TypeId))
                        {
                            TypeProcessors[typeBatch.TypeId].IncrementallyUpdateContactData(ref typeBatch, bodies, dt, inverseDt, 0, typeBatch.BundleCount);
                        }
                    }
                }
            }
            else
            {
                ExecuteMultithreaded<IncrementalContactDataUpdateFilter>(dt, threadDispatcher, incrementalContactUpdateWorker);
            }
        }

    }
}
