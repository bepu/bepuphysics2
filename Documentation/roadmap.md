# Roadmap

This is a high level plan for future development. All dates and features are goals, not guarantees. For a detailed breakdown of tasks in progress, check the [issues](https://github.com/bepu/bepuphysics2/issues) page.

## Near future (Q2-Q3 2018)

The big goals for the initial version are:
- Performance should generally be an order of magnitude better than v1.
- Stuff should tend to be automatically stable with as few tuning knobs as possible. In particular, it should avoid *global* tuning knobs. The use of an iterative solver means there are still mass ratio issues, but fiddly details like scale should just go away.

Note that this release will lack many of v1's built in features, like:
- Buoyancy.
- Full featured character controller (though I will likely provide a simple example version in the demos, just without the full upstepping/downstepping/stance shifting feature set).
- Dedicated vehicle type.

The initial version will be split into incremental stages: alpha, beta, and release. The alpha was released on March 31, 2018.

### Beta (June 2018)
The beta will still be missing some important pieces, but should be able to support most games.
- Triangulated mesh colliders with boundary smoothing.
- Tree-accelerated compound for larger collections of shapes.
- Simulation-wide ray and shape casts.
- A few additional common constraint types.

### Release (July-September 2018)
Should be able to support the core features required by physically complex games.
- Better handling of some dangerous performance corner cases in the solver.
- Fully enabled CCD.
- Convex hulls.
- Additional ease of use features, especially for supplying custom body properties to collision filtering and velocity integration.
- More educational demos, including some deeper dives like showing how to create a custom voxel world collidable.

### Nice-to-haves 
There are a few features which aren't fundamentally required for a release, but which are still high value. These might get pulled into one of the above milestones if possible. If they don't make it into the first version, they'll probably show up later.
- Further broad phase improvements.
- Experimental general convex boundary smoothing.
- Cylinders and cones.
 
## Medium term (Q3 2018-Q2 2019)

Once the initial version is ready, most of my time will move to things other than core engine development. Much of that time will be spent actually using the engine, so expect to see incremental enhancements and fixes driven by practical use.

And of course, I'll still be watching for input from users and fixing bugs as they arise. I'll probably add more examples during this phase for people wanting to extend the engine in various ways.

I expect a new wave of SIMD intrinsics will become available during this period as well. When this happens, I will likely revisit the various SIMD implementations with some new tools. That will likely result in a nontrivial speedup for the non-bandwidth-bound paths.

## Long term

As I finish up the first phase of development in the other project, I'll likely swing back to physics for a little while. Hopefully, it won't involve a BEPUphysics v3.

The second phase of development will likely require some additional features in the engine.

Here, we get into the realm of the highly speculative. I make no guarantees about the nature or existence of these features, but they are things I'd like to at least investigate at some point:

- Full featured character controller, but without any instantaneous stepping or stance transitions. The character in v1 suffered incredible complexity due to those large instant changes, and they never work very well when combined with other constraints. Consider a rope attached to a character- when it steps up, there is a velocityless displacement that the constraint is forced to fix retroactively. Instead of instant transitions, everything would be incremental. The exact design is still uncertain and depends on some other details (like how far I decide to push motorized ragdoll animation).
- Buoyancy. In particular, a form of buoyancy that supports deterministic animated (nonplanar) surfaces. It would almost certainly still be a heightmap, but the goal would be to support synchronized multiplayer boats or similar use cases. Ideally, it would also allow for some approximate handling of concavity. Hollowed concave shapes would displace the appropriate amount of water, including handling of the case where a boat capsizes and the internal volume fills with water (though likely in a pretty hacky way). Would likely be easy to extend this to simple heightmap fluid simulation if the determinism requirement is relaxed.
- High precision body and static poses, plus associated broad phase changes, for worlds exceeding FP32 precision. This isn't actually too difficult, but it would come with tradeoffs. See https://github.com/bepu/bepuphysics2/issues/13.
- Fixed point math for cross platform determinism. This one is pretty questionable, but it would be nice to support deterministic physics across any platform. (It's important to note, however, that fixed point math alone is merely necessary and not necessarily *sufficient* to guarantee cross platform determinism...) It is unlikely that I will personally make use of this feature, so the likelihood of it being implemented is lower unless I can find a low effort path.