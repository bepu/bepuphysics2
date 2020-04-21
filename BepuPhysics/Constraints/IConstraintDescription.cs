using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BepuPhysics.Constraints
{
    /// <summary>
    /// Marks a type as a description of a constraint associated with a particular batch.
    /// </summary>
    /// <remarks>
    /// Note that one batch may have multiple description types associated with it, each one potentially offering a different subset of properties or translation logic.
    /// </remarks>
    /// <typeparam name="TDescription">Type of the description object.</typeparam>
    public interface IConstraintDescription<TDescription>
        where TDescription : unmanaged, IConstraintDescription<TDescription>
    {
        /// <summary>
        /// Changes the batch-held memory at a given location to match the given description.
        /// </summary>
        /// <param name="batch">Batch to modify.</param>
        /// <param name="bundleIndex">Index of the target constraint's bundle.</param>
        /// <param name="innerIndex">Index of the target constraint within its bundle.</param>
        void ApplyDescription(ref TypeBatch batch, int bundleIndex, int innerIndex);


        /// <summary>
        /// Creates a description from the batch-held memory at a given location.
        /// </summary>
        /// <param name="batch">Batch to read.</param>
        /// <param name="bundleIndex">Index of the source constraint's bundle.</param>
        /// <param name="innerIndex">Index of the source constraint within its bundle.</param>
        /// <param name="description">Description of the constraint.</param>
        void BuildDescription(ref TypeBatch batch, int bundleIndex, int innerIndex, out TDescription description);
        
        /// <summary>
        /// Gets the type id of the constraint that this is a description of.
        /// </summary>
        int ConstraintTypeId { get; }
        /// <summary>
        /// Gets the type of the type batch which contains described constraints.
        /// </summary>
        Type TypeProcessorType  { get; }
    }

    /// <summary>
    /// Marks a type as a one body constraint description.
    /// </summary>
    /// <typeparam name="TDescription">Type of the description.</typeparam>
    /// <remarks>This and the other body-count aware interfaces exist to give the compiler a way to report errors when using Solver.Add with different body counts.</remarks>
    public interface IOneBodyConstraintDescription<TDescription> : IConstraintDescription<TDescription> where TDescription : unmanaged, IOneBodyConstraintDescription<TDescription>
    {
    }

    /// <summary>
    /// Marks a type as a two body constraint description.
    /// </summary>
    /// <typeparam name="TDescription">Type of the description.</typeparam>
    /// <remarks>This and the other body-count aware interfaces exist to give the compiler a way to report errors when using Solver.Add with different body counts.</remarks>
    public interface ITwoBodyConstraintDescription<TDescription> : IConstraintDescription<TDescription> where TDescription : unmanaged, ITwoBodyConstraintDescription<TDescription>
    {
    }

    /// <summary>
    /// Marks a type as a three body constraint description.
    /// </summary>
    /// <typeparam name="TDescription">Type of the description.</typeparam>
    /// <remarks>This and the other body-count aware interfaces exist to give the compiler a way to report errors when using Solver.Add with different body counts.</remarks>
    public interface IThreeBodyConstraintDescription<TDescription> : IConstraintDescription<TDescription> where TDescription : unmanaged, IThreeBodyConstraintDescription<TDescription>
    {
    }

    /// <summary>
    /// Marks a type as a four body constraint description.
    /// </summary>
    /// <typeparam name="TDescription">Type of the description.</typeparam>
    /// <remarks>This and the other body-count aware interfaces exist to give the compiler a way to report errors when using Solver.Add with different body counts.</remarks>
    public interface IFourBodyConstraintDescription<TDescription> : IConstraintDescription<TDescription> where TDescription : unmanaged, IFourBodyConstraintDescription<TDescription>
    {
    }
}
