namespace BepuPhysics.Constraints
{
    //These are used as compile time specialization constants. The solver can preprocess constraint references to know whether a given batch has any constraints that need integration.
    //The first batch, for example, is known to require integration for every single constraint, since the first batch is necessarily the first time you see any body.
    //Similarly, there is a number of batches beyond which no constraints will have any integration responsibilities.
    //By marking those ahead of time, we avoid the nonzero cost of checking the integration flags.

    /// <summary>
    /// Marks a type as determining the integration mode for a solver batch.
    /// </summary>
    public interface IBatchIntegrationMode
    {
    }

    /// <summary>
    /// The batch was determined to have only constraints with integration responsibilities, so there's no need to check.
    /// </summary>
    public struct BatchShouldAlwaysIntegrate : IBatchIntegrationMode
    {

    }
    /// <summary>
    /// The batch was determined to have no constraints with integration responsibilities, so there's no need to check.
    /// </summary>
    public struct BatchShouldNeverIntegrate : IBatchIntegrationMode
    {

    }
    /// <summary>
    /// The batch was determined to have some constraints with integration responsibilities.
    /// </summary>
    public struct BatchShouldConditionallyIntegrate : IBatchIntegrationMode
    {

    }

    /// <summary>
    /// Marks a type as determining whether pose integration should be performed on bodies within the constraint batch.
    /// </summary>
    public interface IBatchPoseIntegrationAllowed
    {

    }

    /// <summary>
    /// Marks a batch as integrating poses for any bodies with integration responsibility within the constraint batch.
    /// Constraints which need to be updated in response to pose integration will also have their UpdateForNewPose function called.
    /// </summary>
    public struct AllowPoseIntegration : IBatchPoseIntegrationAllowed
    {
    }

    /// <summary>
    /// Marks a batch as not integrating poses for any bodies within the constraint batch.
    /// </summary>
    public struct DisallowPoseIntegration : IBatchPoseIntegrationAllowed
    {
    }

}
