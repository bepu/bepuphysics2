using BepuUtilities.Collections;
using BepuUtilities.Memory;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;

namespace BepuPhysics
{
    /// <summary>
    /// Stores profiling information for the previous simulation execution.
    /// </summary>
    public struct SimulationProfiler
    {
#if PROFILE
        //Really, using dictionaries here is overkill. There will likely never be more than a handful of stages, so a list would actually be faster.
        //But then we'd have to set up a key-value association and so on. It matters very little, so shrug.
        Dictionary<object, double> stages;
        Dictionary<object, long> startTimeStamps;
#endif

        /// <summary>
        /// Gets the time it took to complete the last execution of the given stage. If no stage matching the given object ran, returns -1.
        /// </summary>
        /// <param name="stage">Stage to look up the time for.</param>
        /// <returns>Time it took to complete the last execution of the given stage.</returns>
        public double this[object stage]
        {
            get
            {
#if PROFILE
                if (stages.TryGetValue(stage, out var time))
                {
                    return time;
                }
#endif
                return -1;
            }
        }

        public SimulationProfiler(int initialStageCount)
        {
#if PROFILE
            stages = new Dictionary<object, double>(initialStageCount);
            startTimeStamps = new Dictionary<object, long>(initialStageCount);
#endif
        }

        public void Start(object o)
        {
#if PROFILE
            Debug.Assert(!startTimeStamps.ContainsKey(o), "Cannot start a stage that has already been started.");
            startTimeStamps.Add(o, Stopwatch.GetTimestamp());
#endif
        }

        public void End(object o)
        {
#if PROFILE
            var endTimeStamp = Stopwatch.GetTimestamp();
            Debug.Assert(startTimeStamps.ContainsKey(o), "To end a stage, it must currently be active (started and not already stopped).");
            if (!stages.TryGetValue(o, out var accumulatedStageTime))
            {
                accumulatedStageTime = 0;
            }
            stages[o] = accumulatedStageTime + (endTimeStamp - startTimeStamps[o]) / (double)Stopwatch.Frequency;
            startTimeStamps.Remove(o);
#endif
        }

        public void Clear()
        {
#if PROFILE
            Debug.Assert(startTimeStamps.Count == 0, "It's likely that some stage was left unended from the previous frame.");
            stages.Clear();
#endif
        }
    }
}
