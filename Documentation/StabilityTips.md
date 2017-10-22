# Simulation Quality and Stability Tips

*This is an early work in progress. More will show up over time as I remember to write them down.*

## Solver Stabilization

-Whenever constraints are behaving oddly, try increasing the solver's iteration count to an extreme value (100+). If the problems go away, it is likely that the constraints or the bodies connected to them are configured in a way that is making the constraint system very hard to solve. If behavior doesn't improve, then the problem may not be related to constraints.

-While many simple simulations can work fine with only 1 solver iteration, using a minimum of 2 is recommended for most simulations. Simulations with greater degrees of complexity- articulated robots, ragdolls, stacks- will often need more.

-The "mass ratios" problem: avoid making heavy objects depend on light objects. A tiny box can sit on top of a tank without any issues at all, but a tank would squish a tiny box. Larger mass ratios yield larger stability problems.

-If constraints are taking too many iterations to converge and the design allows it, try softening the constraints. A little bit of softness can significantly stabilize a constraint system.

-Avoid configuring constraints to 'fight' each other. Opposing constraints tend to require more iterations to converge, and if they're extremely rigid, it can take an unreasonable number of iterations.

-Use the minimum number of constraints necessary to express the desired design. Less constraints means better performance and fewer opportunities for constraints to conflict.

-When trying to build rope-like constraint systems, it's likely that the solver will suffer from both mass ratios and the difficulty of propagating impulses through a long chain of bodies. In this situation, you can give the solver a little help by adding 'skip' constraints. That is, rather than creating distance limits only between adjacent bodies in a rope, try also connecting the first body to the last body (with an appropriately increased distance limit). In extreme cases, you could also create more skip constraints- connecting every other body or every third body, for example.

-When tuning the natural frequency of constraints, prefer values smaller than PI * 0.5 / timeStepDuration. Higher values risk oscillation that the time step can't represent, potentially leading to explosions.

-Once bugs and misconfiguration are ruled out and things are still behaving poorly, try decreasing the time step duration and updating the simulation more frequently to compensate. This is the single most powerful way to stabilize simulations, but also one of the most expensive.

## Contact Generation

-While nonzero speculative margins are required for stable contact, overly large margins can sometimes cause 'ghost' contacts when objects are moving quickly relative to each other. It might look like one object is bouncing off the air a foot away from the other shape. To avoid this, use a smaller speculative margin and, if continuous collision detection is desired, consider using one of the explicitly continuous modes.

-Prefer simpler shapes. In particular, avoid highly complex convex hulls with a bunch of faces separated by only a few degrees. The solver likes temporally coherent contacts, and excess complexity can cause the set of generated contacts to vary significantly with small rotations. Also, complicated hulls are slow!



