using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;

namespace BepuPhysics
{
    //TODO: You could change this into a simulation generic type parameter, but the value beyond consistency is unclear.
#if PROFILE
    /// <summary>
    /// Stores profiling information for the previous simulation execution.
    /// </summary>
    public class SimulationProfiler
    {
        //Really, using dictionaries here is overkill. There will likely never be more than a handful of stages, so a list would actually be faster.
        //But then we'd have to set up a key-value association and so on. It matters very little, so shrug.
        PassthroughArrayPool<int> intPool = new PassthroughArrayPool<int>();
        PassthroughArrayPool<object> objectPool = new PassthroughArrayPool<object>();
        PassthroughArrayPool<double> doublePool = new PassthroughArrayPool<double>();
        PassthroughArrayPool<long> longPool = new PassthroughArrayPool<long>();
        QuickDictionary<object, double, Array<object>, Array<double>, Array<int>, ReferenceComparer<object>> stages;
        QuickDictionary<object, long, Array<object>, Array<long>, Array<int>, ReferenceComparer<object>> startTimeStamps;

        /// <summary>
        /// Gets the time it took to complete the last execution of the given stage. If no stage matching the given object ran, returns -1.
        /// </summary>
        /// <param name="stage">Stage to look up the time for.</param>
        /// <returns>Time it took to complete the last execution of the given stage.</returns>
        public double this[object stage]
        {
            get
            {
                if(stages.TryGetValue(stage, out var time))
                {
                    return time;
                }
                return -1;
            }
        }

        public SimulationProfiler(int initialStageCount = 8)
        {
            QuickDictionary<object, double, Array<object>, Array<double>, Array<int>, ReferenceComparer<object>>.Create(
                objectPool, doublePool, intPool, SpanHelper.GetContainingPowerOf2(initialStageCount), 3, out stages);
            QuickDictionary<object, long, Array<object>, Array<long>, Array<int>, ReferenceComparer<object>>.Create(
                objectPool, longPool, intPool, SpanHelper.GetContainingPowerOf2(initialStageCount), 3, out startTimeStamps);
        }

        internal void Start(object o)
        {
            //This technically does a double lookup, but the performance of this doesn't matter in the slightest. It'll be invoked a handful of times per frame.
            //Even the value of deferring the measurement until after the add is questionable.
            if (!startTimeStamps.Add(o, 0, objectPool, longPool, intPool))
            {
                throw new InvalidOperationException("Can't start a stage which was already started without ending it first.");
            }
            var index = startTimeStamps.IndexOf(o);
            startTimeStamps.Values[index] = Stopwatch.GetTimestamp();
        }

        internal void End(object o)
        {
            var endTimeStamp = Stopwatch.GetTimestamp();
            //Could avoid a double lookup here too. If it mattered. But it doesn't!
            var startIndex = startTimeStamps.IndexOf(o);
            if(startIndex < 0)
                throw new InvalidOperationException("Can't end a stage which wasn't started first.");
            var stageTime = (endTimeStamp - startTimeStamps.Values[startIndex]) / (double)Stopwatch.Frequency;
            startTimeStamps.FastRemove(o);

            //We allow stages to be started multiple times. Could be useful at some point, maybe.
            var stageIndex = stages.IndexOf(o);
            if(stageIndex < 0)
            {
                stages.Add(o, stageTime, objectPool, doublePool, intPool);
            }
            else
            {
                stages.Values[stageIndex] += stageTime;
            }
        }

        internal void Clear()
        {
            Debug.Assert(startTimeStamps.Count == 0, "It's likely that some stage was left unended from the previous frame.");
            stages.Clear();
        }
    }
#endif
    partial class Simulation
    {
        //We're basically requiring users to also have conditionally compiled code when using the profiler (if they ever don't compile with profiling), which isn't too crazy.
        //There is a chance that convenience demands making this property unconditional, but it's a pretty small detail and the object would just be dead weight.
        //If anyone complains about it with good reason, we can fiddle with it then.
#if PROFILE
        public SimulationProfiler Timings { get; } = new SimulationProfiler();
#endif
        //These are out here to limit the exposure of the main simulation codebase to the conditionally compiled stuff. Don't want to stick #if PROFILER everywhere to access the Timings.
        [Conditional("PROFILE")]
        void ProfilerClear()
        {
            Timings.Clear();
        }
        [Conditional("PROFILE")]
        void ProfilerStart(object o)
        {
            Timings.Start(o);
        }
        [Conditional("PROFILE")]
        void ProfilerEnd(object o)
        {
            Timings.End(o);
        }
    }
}
