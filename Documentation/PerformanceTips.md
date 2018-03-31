# Performance Tips

*This is an early work in progress. More will show up over time as I remember to write them down.*
## Shape Optimization
-Use simple shapes whenever possible. Spheres and capsules are fastest followed by (TODO: measurement required) cylinders, cones, boxes, and finally convex hulls. While convex hulls are not slow in an absolute sense, convex hull contact generation can be an order of magnitude slower than some of the extremely fast simpler shapes.

-If you need to use a convex hull, use the minimum number of vertices needed to approximate the shape. The cost of hull collision detection is proportional to their complexity.

-For mobile concave shapes, first make sure they *really* need to be concave. Whenever you can get away with a simple convex shape, do so. If there's no choice, prefer using a compound of a minimum number of simple shapes like spheres and capsules rather than convex hulls (as per the earlier tip).

-If you *really, _definitely_* need a mobile mesh, especially one that needs to collide with other meshes, spend a while confirming that you *really*, **definitely**, ***seriously*** need it and there is no other option, and then use a compound of simple shapes instead.

-Okay, so maybe you actually truly really seriously need an actual mobile mesh. Keep the number of triangles to the minimum necessary to approximate the desired shape, and try to keep the triangles fairly uniform in size. Long sliver-like triangles can end up with large and inefficient bounding boxes. Static meshes follow the same optimization guidelines.

-Reuse shapes when convenient. In particular, avoid creating tons of duplicate convex hulls. They are much larger than the other types. Both the required memory bandwidth and cache size can become a bottleneck during the narrow phase.

-Prefer using the same shape types when convenient. The narrow phase works on batches of same-type collision pairs at a time. By using a lot of the same types, the narrow phase can get better SIMD efficiency. (This is a fairly minor effect. If you kinda want to use a cylinder for something even though you haven't used them anywhere else, don't feel too bad about it.)


## Solver Optimization
-Try using the minimum number of iterations sufficient to retain stability. The cost of the solver stage is linear with the number of iterations, and some simulations can get by with very few.

-Watch out when there are a large number of constraints associated with a single body. The solver cannot solve multiple constraints affecting the same body at the same time, potentially limiting both SIMD and multicore parallelism. 300 characters on a spaceship represented as a single body may cause a sequentialization bottleneck. If this appears to be a dominant cost, consider splitting the offending body into multiple pieces. (This tip should become unnecessary once the jacobi fallback solver is available.)