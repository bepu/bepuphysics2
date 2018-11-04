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
        Dictionary<object, double> stages;
        Dictionary<object, long> startTimeStamps;

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
            stages = new Dictionary<object, double>(initialStageCount);
            startTimeStamps = new Dictionary<object, long>(initialStageCount);
        }

        internal void Start(object o)
        {
            Debug.Assert(!startTimeStamps.ContainsKey(o), "Cannot start a stage that has already been started.");
            startTimeStamps.Add(o, Stopwatch.GetTimestamp());
        }

        internal void End(object o)
        {
            var endTimeStamp = Stopwatch.GetTimestamp();
            Debug.Assert(startTimeStamps.ContainsKey(o), "To end a stage, it must currently be active (started and not already stopped).");
            if (!stages.TryGetValue(o, out var accumulatedStageTime))
            {
                accumulatedStageTime = 0;
            }
            stages[o] = accumulatedStageTime + (endTimeStamp - startTimeStamps[o]) / (double)Stopwatch.Frequency;
            startTimeStamps.Remove(o);           
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
