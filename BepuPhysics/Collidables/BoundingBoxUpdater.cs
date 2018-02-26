using BepuUtilities.Collections;
using BepuUtilities.Memory;
using BepuPhysics.CollisionDetection;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuUtilities;

namespace BepuPhysics.Collidables
{
    /// <summary>
    /// Computes bounding boxes for bodies.
    /// </summary>
    /// <remarks>Note that only bodies need a dynamic bounding box updater. Bounding boxes for statics can be updated on demand- which should be extremely rare.</remarks>
    public struct BoundingBoxUpdater
    {
        BodyBundleSource bundleSource;
        Shapes shapes;
        BufferPool<int> pool;
        float dt;
        Buffer<QuickList<int, Buffer<int>>> batchesPerType;

        /// <summary>
        /// The number of bodies to accumulate per type before executing an AABB update. The more bodies per batch, the less virtual overhead and execution divergence.
        /// However, this should be kept low enough such that the data that has to be gathered by the bounding box update is still usually in L1.
        /// </summary>
        public const int CollidablesPerFlush = 16;

        public BoundingBoxUpdater(Bodies bodies, Shapes shapes, BroadPhase broadPhase, BufferPool pool, float dt)
        {
            bundleSource = new BodyBundleSource
            {
                Bodies = bodies,
                BroadPhase = broadPhase
            };
            this.shapes = shapes;
            this.pool = pool.SpecializeFor<int>();
            this.dt = dt;
            //The number of registered types cannot change mid-frame, because adding collidables mid-update is illegal. Can just allocate based on current count.
            pool.SpecializeFor<QuickList<int, Buffer<int>>>().Take(shapes.RegisteredTypeSpan, out batchesPerType);
            //Batches are lazily created. The underlying span is checked before lazy initialization, so we need to clear out any trash in the backing array.
            batchesPerType.Clear(0, shapes.RegisteredTypeSpan);
        }



        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void ExpandBoundingBoxes(ref Vector3Wide min, ref Vector3Wide max, ref BodyVelocities velocities, float dt,
            ref Vector<float> maximumRadius, ref Vector<float> maximumAngularExpansion, ref Vector<float> maximumExpansion)
        {
            /*
            If an object sitting on a plane had a raw (unexpanded) AABB that is just barely above the plane, no contacts would be generated. 
            If the velocity of the object would shove it down into the plane in the next frame, then it would generate contacts in the next frame- and, 
            potentially, this cycle would repeat and cause jitter.

            To solve this, there are a couple of options:
            1) Introduce an 'allowed penetration' so that objects can overlap a little bit. This tends to confuse people a little bit when they notice it, 
            and in some circumstances objects can be seen settling into the allowed penetration slowly. It looks a bit odd.
            2) Make contact constraints fight to maintain zero penetration depth, but expand the bounding box with velocity and allow contacts to be generated speculatively- 
            contacts with negative penetration depth.

            #2 is a form of continuous collision detection, but it's handy for general contact stability too. 
            In this version of the engine, all objects generate speculative contacts by default, though only within a per-collidable-tuned 'speculative margin'. 
            It's kind of like BEPUphysics v1's AllowedPenetration, except inverted. Speculative contacts that fall within the speculative margin- 
            that is, those with negative depth of a magnitude less than the margin- are kept.

            So, a user could choose to have a very large speculative margin, and the speculative contact generation would provide a form of continuous collision detection. 
            The main purpose, though, is just contact stability. With this in isolation, there's no strong reason to expand the bounding box more than the speculative margin. 
            This is the 'discrete' mode.

            However, consider what would happen if an object A with high velocity and this 'discrete' mode was headed towards an object B in a 'continuous' mode.
            Object B only expands its bounding box by its own velocity, and object A doesn't expand beyond its speculative margin. The collision between A and B could easily be missed. 
            To account for this, there is an intermediate mode- 'passive'- where the bounding box is allowed to expand beyond the margin, 
            but no further continuous collision detection is performed.

            The fully continuous modes fully expand the bounding boxes. Notably, the inner sphere continuous collision detection mode could get by with less, 
            but it would be pretty confusing to have the same kind of missed collision possibility if the other object in the pair was a substepping object.
            Two different inner sphere modes could be offered, but I'm unsure about the usefulness versus the complexity.

            (Note that there ARE situations where a bounding box which contains the full unconstrained motion will fail to capture constrained motion. 
            Consider object A flying at high speed to impact the stationary object B, which sits next to another stationary object C. 
            Object B's bounding box doesn't overlap with object C's bounding box- they're both stationary, so there's no velocity expansion. But during one frame, 
            object A slams into B, and object B's velocity during that frame now forces it to tunnel all the way through C unimpeded, because no contacts were generated between B and C. 
            There are ways to address this- all of which are a bit expensive- but CCD as implemented is not a hard guarantee. 
            It's a 'best effort' that compromises with performance. Later on, if it's really necessary, we could consider harder guarantees with higher costs, but... 
            given that no one seemed to have much of an issue with v1's rather limited CCD, it'll probably be fine.)

            So, how is the velocity expansion calculated?
            There's two parts, linear and angular.

            Linear is pretty simple- expand the bounding box in the direction of linear displacement (linearVelocity * dt).
            */

            Vector<float> vectorDt = new Vector<float>(dt);
            Vector3Wide.Scale(ref velocities.Linear, ref vectorDt, out var linearDisplacement);

            var zero = Vector<float>.Zero;
            Vector3Wide.Min(ref zero, ref linearDisplacement, out var minDisplacement);
            Vector3Wide.Max(ref zero, ref linearDisplacement, out var maxDisplacement);

            /*
            Angular requires a bit more care. Since the goal is to create a tight bound, simply using a v = w * r approximation isn't ideal. A slightly tighter can be found:
            1) The maximum displacement along ANY axis during an intermediate time is equal to the distance from a starting position at MaximumRadius 
            to the position of that point at the intermediate time.
            2) The expansion cannot exceed the maximum radius, so angular deltas greater than pi/3 do not need to be considered. 
            (An expansion equal to the maximum radius would result in an equilateral triangle, which has an angle of 60 degrees in each corner.) 
            Larger values can simply be clamped.
            3) The largest displacement along any axis, at any time, is the distance from the starting position to the position at dt. Note that this only holds because of the clamp: 
            if the angle was allowed to wrap around, it the distance would start to go down again.
            4) position(time) = {radius * sin(angular speed * time), radius * cos(angular speed * time)}
            5) largest expansion required = ||position(dt) - position(0)|| = sqrt(2 * radius^2 * (1 - cos(dt * w)))
            6) Don't have any true SIMD sin function, but we can approximate it using a taylor series, like: cos(x) = 1 - x^2 / 2! + x^4 / 4! - x^6 / 6!
            7) Note that the cosine approximation should stop at a degree where it is smaller than the true value of cosine for the interval 0 to pi/3: this guarantees that the distance,
            which is larger when the cosine is smaller, is conservative and fully bounds the angular motion.

            Why do this extra work?
            1) The bounding box calculation phase, as a part of the pose integration phase, tends to be severely memory bound. 
            Spending a little of ALU time to get a smaller bounding box isn't a big concern, even though it includes a couple of sqrts.
            An extra few dozen ALU cycles is unlikely to meaningfully change the execution time.
            2) Shrinking the bounding box reduces the number of collision pairs. Collision pairs are expensive- many times more expensive than the cost of shrinking the bounding box.
            */
            Vector3Wide.Length(ref velocities.Angular, out var angularVelocityMagnitude);
            var a = Vector.Min(angularVelocityMagnitude * vectorDt, new Vector<float>(MathHelper.Pi / 3f));
            var a2 = a * a;
            var a4 = a2 * a2;
            var a6 = a4 * a2;
            var cosAngleMinusOne = a2 * new Vector<float>(-1f / 2f) + a4 * new Vector<float>(1f / 24f) - a6 * new Vector<float>(1f / 720f);
            //Note that it's impossible for angular motion to cause an increase in bounding box size beyond (maximumRadius-minimumRadius) on any given axis.
            //That value, or a conservative approximation, is stored as the maximum angular expansion.
            var angularExpansion = Vector.Min(maximumAngularExpansion,
                Vector.SquareRoot(new Vector<float>(-2f) * maximumRadius * maximumRadius * cosAngleMinusOne));
            Vector3Wide.Subtract(ref minDisplacement, ref angularExpansion, out minDisplacement);
            Vector3Wide.Add(ref maxDisplacement, ref angularExpansion, out maxDisplacement);

            //The maximum expansion passed into this function is the speculative margin for discrete mode collidables, and ~infinity for passive or continuous ones.
            var negativeMaximum = -maximumExpansion;
            Vector3Wide.Max(ref negativeMaximum, ref minDisplacement, out minDisplacement);
            Vector3Wide.Min(ref maximumExpansion, ref maxDisplacement, out maxDisplacement);

            Vector3Wide.Add(ref min, ref minDisplacement, out min);
            Vector3Wide.Add(ref max, ref maxDisplacement, out max);

            //Note that this is an area that will need to adapt to changes to the body pose representation. If you use a 32 bit fixed point position or a 64 bit double position,
            //the bounding box calculation needs to be aware. (In the more extreme cases, so does the broad phase. The amount of conservative padding needed for a 32 bit bounding box
            //with 64 bit positions is silly.)
        }
        public void TryAdd(int bodyIndex)
        {
            //For convenience, this function handles the case where the collidable reference points to nothing.
            //Note that this touches the memory associated with the full collidable. That's okay- we'll be reading the rest of it shortly if it has a collidable.
            ref var collidable = ref bundleSource.Bodies.ActiveSet.Collidables[bodyIndex];
            //Technically, you could make a second pass that only processes collidables, rather than iterating over all bodies and doing last second branches.
            //But then you'd be evicting everything from cache L1/L2. And, 99.99% of the time, bodies are going to have shapes, so this isn't going to be a difficult branch to predict.
            //Even if it was 50%, the cache benefit of executing alongside the just-touched data source would outweigh the misprediction.
            if (collidable.Shape.Exists)
            {
                var typeIndex = collidable.Shape.Type;
                Debug.Assert(typeIndex >= 0 && typeIndex < batchesPerType.Length, "The preallocated type batch array should be able to hold every type index. Is the type index broken?");
                ref var batchSlot = ref batchesPerType[typeIndex];
                if (!batchSlot.Span.Allocated)
                {
                    //No list exists for this type yet.
                    QuickList<int, Buffer<int>>.Create(pool, CollidablesPerFlush, out batchSlot);
                }
                batchSlot.AddUnsafely(bodyIndex);
                if (batchSlot.Count == CollidablesPerFlush)
                {
                    bundleSource.BodyIndices = batchSlot;
                    shapes[typeIndex].ComputeBounds(ref bundleSource, dt);
                    batchSlot.Count = 0;
                }
            }
        }

        public void FlushAndDispose()
        {
            for (int typeIndex = 0; typeIndex < shapes.RegisteredTypeSpan; ++typeIndex)
            {
                ref var batch = ref batchesPerType[typeIndex];
                if (batch.Span.Allocated)
                {
                    bundleSource.BodyIndices = batch;
                    shapes[typeIndex].ComputeBounds(ref bundleSource, dt);
                    batch.Dispose(pool);
                }
            }
            pool.Raw.SpecializeFor<QuickList<int, Buffer<int>>>().Return(ref batchesPerType);
        }

    }
}
