# Roadmap

This is a high level plan for future development. All dates and features are *extremely* speculative, and any specific detail on this roadmap is almost certainly wrong. Treat it as a snapshot of vibes unless noted otherwise. For a more detailed breakdown, check the [issues](https://github.com/bepu/bepuphysics2/issues) page.

## Near term (Q1 2024)

2.5 should be releasing relatively soon. It already includes a bunch of miscellaneous improvements, plus notable transformative improvements to tree building and refinement. The broad phase is a lot faster.

## Medium term

The timing on these features are uncertain, but they're relatively low hanging fruit and I would like to get to them eventually.

1. Mesh/compound intersection optimization, especially in pairs with higher angular velocity.
2. Narrow phase flush improvements: https://github.com/bepu/bepuphysics2/issues/205
3. Sleeper improvements. Applies to actual sleep/wake and candidacy analysis. One exemplar: island management scales poorly in the limit (https://github.com/bepu/bepuphysics2/issues/284)
4. More bandwidth optimizations in the solver for broad simulations: https://github.com/bepu/bepuphysics2/issues/193
5. ARM specialized paths: https://github.com/bepu/bepuphysics2/issues/184
6. Convex hull test performance improvements.
7. Scalar-style API for lower pain contact and boolean queries.
8. Ray cast optimization, particularly with large batches of rays.
9. Convex hull tooling improvements, like in-library simplification utilities.
10. Try for partial cross platform determinism by reimplementing some platform-dependent functionality in software.
11. High precision body and static poses, plus associated broad phase changes, for worlds exceeding 32 bit floating point precision. This isn't actually too difficult, but it would come with tradeoffs. See https://github.com/bepu/bepuphysics2/issues/13. 2.4's revamp of the solver and body data layouts intentionally left the door open for higher precision poses.


## Long term

These features are even *more* speculative.

- Generalized boundary smoothing to support non-mesh compounds. Would help avoid annoying hitches when scooting around complex geometry composed of a bunch of convex hulls or other non-triangle convexes.
- Buoyancy. In particular, a form of buoyancy that supports deterministic animated (nonplanar) surfaces. It would almost certainly still be a heightmap, but the goal would be to support synchronized multiplayer boats or similar use cases. Ideally, it would also allow for some approximate handling of concavity. Hollowed concave shapes would displace the appropriate amount of water, including handling of the case where a boat capsizes and the internal volume fills with water (though likely in a pretty hacky way). Would likely be easy to extend this to simple heightmap fluid simulation if the determinism requirement is relaxed.