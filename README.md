# bepuphysics v2

This is the repo for the bepuphysics v2 library, a complete rewrite of the C# 3d rigid body physics engine [BEPUphysics v1](https://github.com/bepu/bepuphysics1).

The library is still in alpha. Many important features are missing, and many bugs likely remain.

The alpha contains a basic featureset:
- Spheres, capsules, boxes
- Simple compounds of the above, mainly for use in smaller shapes where acceleration structures wouldn't be helpful
- Highly nonidiomatic APIs
- Automatic sleeping/waking management
- Ball socket, hinge, swivel hinge, and swing limit
- Speediness

It notably does *not* yet include:
- Convex hulls, cylinders, cones, or other complex shapes 
- Efficient compounds supporting many children
- Meshes
- Bounciness, other than the frequency/damping ratio tuning
- Continuous collision detection, other than the speculative margin
- Scene-wide queries like ray casts or volume queries
- Shape casts
- Many useful constraint types

If you are one of the brave early adopters, you can report bugs [here on github](../../issues). 

Visit the [forums](https://forum.bepuentertainment.com) for discussion and questions.

Check the [roadmap](Documentation/roadmap.md) for a high level look at where things are going.

There exists some sparse proto-documentation:

[Q&A](Documentation/QuestionsAndAnswers.md)

[Stability Tips](Documentation/StabilityTips.md)

[Performance Tips](Documentation/PerformanceTips.md)

[Packaging and Versioning](Documentation/PackagingAndVersioning.md)

