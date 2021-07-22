# Roadmap

This is a high level plan for future development. All dates and features are speculative. For a detailed breakdown of tasks in progress, check the [issues](https://github.com/bepu/bepuphysics2/issues) page.

## Near term (H2 2021)

2.4.0 will launch with a revamped solver that should be dramatically faster. The library will also take a dependency on .NET 5 and assumes a runtime roughly as capable as the version of CoreCLR associated with .NET 5.

I'll be working in the background on other projects using bepuphysics2. Their requirements will drive various changes and future planning.

## Medium term

The timing on these features are uncertain, but they're relatively low hanging fruit and I would like to get to them eventually.

1. Tree revamp. Should help with both initialization costs (faster Mesh construction, for example) as well as faster and more flexible broad phase incremental refinement.
2. Lower deactivation/reactivation spike overhead. As a part of improving the Tree, I'd like to add batched subtree insertions with better multithreaded scaling. As insertions are a major sequential bottleneck in the current activation system, this could drop the sleep/wake costs by a lot.
3. High precision body and static poses, plus associated broad phase changes, for worlds exceeding 32 bit floating point precision. This isn't actually too difficult, but it would come with tradeoffs. See https://github.com/bepu/bepuphysics2/issues/13.
4. Mesh/compound intersection optimization, especially in pairs with higher angular velocity.
5. Warm starting depth refinement for some expensive convex pair types.
6. Better allocators for temporary data on threads (could significantly reduce memory requirements on some simulations).
7. Convex hull tooling improvements, like in-library simplification utilities.
8. Ray cast optimization, particularly with large batches of rays.


## Long term

Here, we get into the realm of the highly speculative. I make no guarantees about the nature or existence of these features, but they are things I'd like to at least investigate at some point:

- Generalized boundary smoothing to support non-mesh compounds. Would help avoid annoying hitches when scooting around complex geometry composed of a bunch of convex hulls or other non-triangle convexes.
- Buoyancy. In particular, a form of buoyancy that supports deterministic animated (nonplanar) surfaces. It would almost certainly still be a heightmap, but the goal would be to support synchronized multiplayer boats or similar use cases. Ideally, it would also allow for some approximate handling of concavity. Hollowed concave shapes would displace the appropriate amount of water, including handling of the case where a boat capsizes and the internal volume fills with water (though likely in a pretty hacky way). Would likely be easy to extend this to simple heightmap fluid simulation if the determinism requirement is relaxed.
- Fixed point math for cross platform determinism, or swapping to a restricted subset of reliable floating point operations. This one is pretty questionable, but it would be nice to support deterministic physics across any platform. (It's important to note, however, that fixed point math alone is merely necessary and not necessarily *sufficient* to guarantee cross platform determinism...) It is unlikely that I will personally make use of this feature, so the likelihood of it being implemented is lower unless I can find a low effort path.