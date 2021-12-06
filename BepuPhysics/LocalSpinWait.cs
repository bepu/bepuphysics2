using System.Runtime.CompilerServices;
using System.Threading;

namespace BepuPhysics
{
    /// <summary>
    /// Behaves like a framework SpinWait, but never voluntarily relinquishes the timeslice to off-core threads.
    /// </summary>
    /// <remarks><para>There are two big reasons for using this over the regular framework SpinWait:</para>
    /// <para>1) The framework spinwait relies on spins for quite a while before resorting to any form of timeslice surrender.
    /// Empirically, this is not ideal for the solver- if the sync condition isn't met within several nanoseconds, it will tend to be some microseconds away.
    /// This spinwait is much more aggressive about moving to yields.</para>
    /// <para>2) After a number of yields, the framework SpinWait will resort to calling Sleep.
    /// This widens the potential set of schedulable threads to those not native to the current core. If we permit that transition, it is likely to evict cached solver data.
    /// (For very large simulations, the use of Sleep(0) isn't that concerning- every iteration can be large enough to evict all of cache- 
    /// but there still isn't much benefit to using it over yields in context.)</para>
    /// <para>SpinWait will also fall back to Sleep(1) by default which obliterates performance, but that behavior can be disabled.</para>
    /// <para>Note that this isn't an indication that the framework SpinWait should be changed, but rather that the solver's requirements are extremely specific and don't match
    /// a general purpose solution very well.</para></remarks>
    internal struct LocalSpinWait
    {
        public int WaitCount;

        //Empirically, being pretty aggressive about yielding produces the best results. This is pretty reasonable- 
        //a single constraint bundle can take hundreds of nanoseconds to finish.
        //That would be a whole lot of spinning that could be used by some other thread. At worst, we're being friendlier to other applications on the system.
        //This thread will likely be rescheduled on the same core, so it's unlikely that we'll lose any cache warmth (that we wouldn't have lost anyway).
        public const int YieldThreshold = 3;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void SpinOnce()
        {
            if (WaitCount >= YieldThreshold)
            {
                Thread.Yield();
            }
            else
            {
                //We are sacrificing one important feature of the newer framework provided waits- normalized spinning (RuntimeThread.OptimalMaxSpinWaitsPerSpinIteration).
                //Different platforms can spin at significantly different speeds, so a single constant value for the maximum spin duration doesn't map well to all hardware.
                //On the upside, we tend to be concerned about two modes- waiting a very short time, and waiting a medium amount of time.
                //The specific length of the 'short' time doesn't matter too much, so long as it's fairly short.
                Thread.SpinWait(1 << WaitCount);
                ++WaitCount;
            }

        }
    }
}
