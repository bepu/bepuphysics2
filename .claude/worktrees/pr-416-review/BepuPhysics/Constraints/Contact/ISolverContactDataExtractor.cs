namespace BepuPhysics.Constraints.Contact
{
    /// <summary>
    /// Callbacks for direct references to the solver's contact constraint data.
    /// </summary>
    public interface ISolverContactDataExtractor
    {
        /// <summary>
        /// Provides a reference to a convex one body contact constraint. Constraint data is in the first lane of the direct reference (slot 0 of vectors).
        /// </summary>
        /// <typeparam name="TPrestep">Type of the prestep data returned.</typeparam>
        /// <typeparam name="TAccumulatedImpulses">Type of the accumulated impulses data returned.</typeparam>
        /// <param name="bodyHandle">Body handle referenced by the constraint.</param>
        /// <param name="prestep">Prestep data associated with the constraint.</param>
        /// <param name="impulses">Accumulated impulses associated with the constraint.</param>
        void ConvexOneBody<TPrestep, TAccumulatedImpulses>(BodyHandle bodyHandle, ref TPrestep prestep, ref TAccumulatedImpulses impulses)
            where TPrestep : struct, IConvexContactPrestep<TPrestep>
            where TAccumulatedImpulses : struct, IConvexContactAccumulatedImpulses<TAccumulatedImpulses>;

        /// <summary>
        /// Provides a reference to a convex two body contact constraint. Constraint data is in the first lane of the direct reference (slot 0 of vectors).
        /// </summary>
        /// <typeparam name="TPrestep">Type of the prestep data returned.</typeparam>
        /// <typeparam name="TAccumulatedImpulses">Type of the accumulated impulses data returned.</typeparam>
        /// <param name="bodyHandleA">First body handle referenced by the constraint.</param>
        /// <param name="bodyHandleB">Second body handle referenced by the constraint.</param>
        /// <param name="prestep">Prestep data associated with the constraint.</param>
        /// <param name="impulses">Accumulated impulses associated with the constraint.</param>
        void ConvexTwoBody<TPrestep, TAccumulatedImpulses>(BodyHandle bodyHandleA, BodyHandle bodyHandleB, ref TPrestep prestep, ref TAccumulatedImpulses impulses)
            where TPrestep : struct, ITwoBodyConvexContactPrestep<TPrestep>
            where TAccumulatedImpulses : struct, IConvexContactAccumulatedImpulses<TAccumulatedImpulses>;


        /// <summary>
        /// Provides a reference to a nonconvex one body contact constraint. Constraint data is in the first lane of the direct reference (slot 0 of vectors).
        /// </summary>
        /// <typeparam name="TPrestep">Type of the prestep data returned.</typeparam>
        /// <typeparam name="TAccumulatedImpulses">Type of the accumulated impulses data returned.</typeparam>
        /// <param name="bodyHandle">Body handle referenced by the constraint.</param>
        /// <param name="prestep">Prestep data associated with the constraint.</param>
        /// <param name="impulses">Accumulated impulses associated with the constraint.</param>
        void NonconvexOneBody<TPrestep, TAccumulatedImpulses>(BodyHandle bodyHandle, ref TPrestep prestep, ref TAccumulatedImpulses impulses)
            where TPrestep : struct, INonconvexContactPrestep<TPrestep>
            where TAccumulatedImpulses : struct, INonconvexContactAccumulatedImpulses<TAccumulatedImpulses>;

        /// <summary>
        /// Provides a reference to a nonconvex two body contact constraint. Constraint data is in the first lane of the direct reference (slot 0 of vectors).
        /// </summary>
        /// <typeparam name="TPrestep">Type of the prestep data returned.</typeparam>
        /// <typeparam name="TAccumulatedImpulses">Type of the accumulated impulses data returned.</typeparam>
        /// <param name="bodyHandleA">First body handle referenced by the constraint.</param>
        /// <param name="bodyHandleB">Second body handle referenced by the constraint.</param>
        /// <param name="prestep">Prestep data associated with the constraint.</param>
        /// <param name="impulses">Accumulated impulses associated with the constraint.</param>
        void NonconvexTwoBody<TPrestep, TAccumulatedImpulses>(BodyHandle bodyHandleA, BodyHandle bodyHandleB, ref TPrestep prestep, ref TAccumulatedImpulses impulses)
            where TPrestep : struct, ITwoBodyNonconvexContactPrestep<TPrestep>
            where TAccumulatedImpulses : struct, INonconvexContactAccumulatedImpulses<TAccumulatedImpulses>;
    }

    //Body references require a tiny bit of processing, so in extremely constrained use cases, being able to avoid them can be useful.
    /// <summary>
    /// Callbacks for direct references to the solver's contact constraint data. Includes only prestep and impulse data.
    /// </summary>
    public interface ISolverContactPrestepAndImpulsesExtractor
    {
        /// <summary>
        /// Provides a reference to a convex one body contact constraint. Constraint data is in the first lane of the direct reference (slot 0 of vectors).
        /// </summary>
        /// <typeparam name="TPrestep">Type of the prestep data returned.</typeparam>
        /// <typeparam name="TAccumulatedImpulses">Type of the accumulated impulses data returned.</typeparam>
        /// <param name="prestep">Prestep data associated with the constraint.</param>
        /// <param name="impulses">Accumulated impulses associated with the constraint.</param>
        void ConvexOneBody<TPrestep, TAccumulatedImpulses>(ref TPrestep prestep, ref TAccumulatedImpulses impulses)
            where TPrestep : struct, IConvexContactPrestep<TPrestep>
            where TAccumulatedImpulses : struct, IConvexContactAccumulatedImpulses<TAccumulatedImpulses>;

        /// <summary>
        /// Provides a reference to a convex two body contact constraint. Constraint data is in the first lane of the direct reference (slot 0 of vectors).
        /// </summary>
        /// <typeparam name="TPrestep">Type of the prestep data returned.</typeparam>
        /// <typeparam name="TAccumulatedImpulses">Type of the accumulated impulses data returned.</typeparam>
        /// <param name="prestep">Prestep data associated with the constraint.</param>
        /// <param name="impulses">Accumulated impulses associated with the constraint.</param>
        void ConvexTwoBody<TPrestep, TAccumulatedImpulses>(ref TPrestep prestep, ref TAccumulatedImpulses impulses)
            where TPrestep : struct, ITwoBodyConvexContactPrestep<TPrestep>
            where TAccumulatedImpulses : struct, IConvexContactAccumulatedImpulses<TAccumulatedImpulses>;


        /// <summary>
        /// Provides a reference to a nonconvex one body contact constraint. Constraint data is in the first lane of the direct reference (slot 0 of vectors).
        /// </summary>
        /// <typeparam name="TPrestep">Type of the prestep data returned.</typeparam>
        /// <typeparam name="TAccumulatedImpulses">Type of the accumulated impulses data returned.</typeparam>
        /// <param name="prestep">Prestep data associated with the constraint.</param>
        /// <param name="impulses">Accumulated impulses associated with the constraint.</param>
        void NonconvexOneBody<TPrestep, TAccumulatedImpulses>(ref TPrestep prestep, ref TAccumulatedImpulses impulses)
            where TPrestep : struct, INonconvexContactPrestep<TPrestep>
            where TAccumulatedImpulses : struct, INonconvexContactAccumulatedImpulses<TAccumulatedImpulses>;

        /// <summary>
        /// Provides a reference to a nonconvex two body contact constraint. Constraint data is in the first lane of the direct reference (slot 0 of vectors).
        /// </summary>
        /// <typeparam name="TPrestep">Type of the prestep data returned.</typeparam>
        /// <typeparam name="TAccumulatedImpulses">Type of the accumulated impulses data returned.</typeparam>
        /// <param name="prestep">Prestep data associated with the constraint.</param>
        /// <param name="impulses">Accumulated impulses associated with the constraint.</param>
        void NonconvexTwoBody<TPrestep, TAccumulatedImpulses>(ref TPrestep prestep, ref TAccumulatedImpulses impulses)
            where TPrestep : struct, ITwoBodyNonconvexContactPrestep<TPrestep>
            where TAccumulatedImpulses : struct, INonconvexContactAccumulatedImpulses<TAccumulatedImpulses>;
    }

}
