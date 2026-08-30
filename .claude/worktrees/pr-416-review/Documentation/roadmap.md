# Roadmap

This is a high level plan for future development. All dates and features are *extremely* speculative, and any specific detail on this roadmap is almost certainly wrong. Treat it as a snapshot of vibes unless noted otherwise. For a more detailed breakdown, check the [issues](https://github.com/bepu/bepuphysics2/issues) page.

Notably, I now have a "full time job" doing "important things" like some kind of weirdo, so I've given up on trying to guess when these things will actually be done. Think of this roughly as a priority queue.

## Near term

2.5 should be releasing relatively soon. It already includes a bunch of miscellaneous improvements, plus notable transformative improvements to tree building and refinement. The broad phase is a lot faster.

The only significant work remaining in 2.5 is to improve thread load balancing in the broad phase for smaller simulations.

## Medium term

The timing on these features are uncertain, but they're relatively low hanging fruit and I would like to get to them eventually.

1. Super secret special sauce solver changes that may or may not actually work at all. But if they *do* work, they'll be great!
2. Look into simplifying layouts with the latest generation of solver for usability and sleeper efficiency reasons.
3. More bandwidth optimizations in the solver for broad simulations: https://github.com/bepu/bepuphysics2/issues/193
4. Catch up with 512 bit instructions and improvements to vectorization.
5. Low hanging fruit in the API; e.g. allow normal delegates or function pointers on functions which currently require reified generics. (This would be a strictly opt-in cost; they'd be implemented through the reified generics API.)
6. Mesh/compound intersection optimization, especially in pairs with higher angular velocity.
7. Narrow phase flush improvements: https://github.com/bepu/bepuphysics2/issues/205
8. Sleeper improvements. Applies to actual sleep/wake and candidacy analysis. One exemplar: island management scales poorly in the limit (https://github.com/bepu/bepuphysics2/issues/284)
9. ARM specialized paths: https://github.com/bepu/bepuphysics2/issues/184
10. Convex hull test performance improvements.
11. Scalar-style API for lower pain contact and boolean queries.
12. Ray cast optimization, particularly with large batches of rays.
13. Convex hull tooling improvements, like in-library simplification utilities.
14. Try for partial cross platform determinism by reimplementing some platform-dependent functionality in software.
15. High precision body and static poses, plus associated broad phase changes, for worlds exceeding 32 bit floating point precision. This isn't actually too difficult, but it would come with tradeoffs. See https://github.com/bepu/bepuphysics2/issues/13. 2.4's revamp of the solver and body data layouts intentionally left the door open for higher precision poses.


## Long term

These features are even *more* speculative.

- Generalized boundary smoothing to support non-mesh compounds. Would help avoid annoying hitches when scooting around complex geometry composed of a bunch of convex hulls or other non-triangle convexes.
- Buoyancy. In particular, a form of buoyancy that supports deterministic animated (nonplanar) surfaces. It would almost certainly still be a heightmap, but the goal would be to support synchronized multiplayer boats or similar use cases. Ideally, it would also allow for some approximate handling of concavity. Hollowed concave shapes would displace the appropriate amount of water, including handling of the case where a boat capsizes and the internal volume fills with water (though likely in a pretty hacky way). Would likely be easy to extend this to simple heightmap fluid simulation if the determinism requirement is relaxed.