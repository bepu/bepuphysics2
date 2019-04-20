# Roadmap

This is a high level plan for future development. All dates and features are speculative. For a detailed breakdown of tasks in progress, check the [issues](https://github.com/bepu/bepuphysics2/issues) page.

## Near term (2019, early 2020)

After v2's initial release, most of my time will move to things other than core engine development. Much of that time will be spent actually using the engine, so expect to see incremental enhancements and fixes driven by practical use.

And of course, I'll still be watching for input from users and fixing bugs as they arise. I'll probably add more examples during this phase for people wanting to extend the engine in various ways.

Some things that didn't make it into the first release may show up. They may include (but are not limited to):
1. Further broad phase improvements.
2. Warm starting depth refinement for some expensive convex pair types.
3. Bounciness/coefficient of restitution, if it seems unavoidable.
4. Convex hull tooling improvements, like in-library simplification utilities.
5. Better allocators for temporary data on threads (could significantly reduce memory requirements on some simulations).
6. Mesh/compound intersection optimization.
7. Ray cast optimization, particularly with large batches of rays.
8. Lower deactivation/reactivation spike overhead.
9. Platform-specific intrinsics. I will likely (eventually) revisit the various SIMD implementations with some new tools for speeding things up, though the question of maintaining .NET Standard support may delay things.

## Long term

As I finish up the first phase of development in the other project, I'll likely swing back to physics for a little while. Hopefully, it won't involve a BEPUphysics v3.

The second phase of development will likely require some additional features in the engine.

Here, we get into the realm of the highly speculative. I make no guarantees about the nature or existence of these features, but they are things I'd like to at least investigate at some point:

- Generalized boundary smoothing to support non-mesh compounds. Would help avoid annoying hitches when scooting around complex geometry composed of a bunch of convex hulls or other non-triangle convexes.
- Buoyancy. In particular, a form of buoyancy that supports deterministic animated (nonplanar) surfaces. It would almost certainly still be a heightmap, but the goal would be to support synchronized multiplayer boats or similar use cases. Ideally, it would also allow for some approximate handling of concavity. Hollowed concave shapes would displace the appropriate amount of water, including handling of the case where a boat capsizes and the internal volume fills with water (though likely in a pretty hacky way). Would likely be easy to extend this to simple heightmap fluid simulation if the determinism requirement is relaxed.
- High precision body and static poses, plus associated broad phase changes, for worlds exceeding 32 bit floating point precision. This isn't actually too difficult, but it would come with tradeoffs. See https://github.com/bepu/bepuphysics2/issues/13.
- Fixed point math for cross platform determinism. This one is pretty questionable, but it would be nice to support deterministic physics across any platform. (It's important to note, however, that fixed point math alone is merely necessary and not necessarily *sufficient* to guarantee cross platform determinism...) It is unlikely that I will personally make use of this feature, so the likelihood of it being implemented is lower unless I can find a low effort path.