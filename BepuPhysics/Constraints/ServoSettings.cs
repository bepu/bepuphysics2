using BepuUtilities;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace BepuPhysics.Constraints;

/// <summary>
/// Describes how a quickly and strongly a servo constraint should move towards a position target.
/// </summary>
/// <remarks>
/// The constraint will attempt to reach a speed between <see cref="BaseSpeed"/> and <see cref="MaximumSpeed"/> using a force no greater than <see cref="MaximumForce"/>.
/// The speed that the constraint will attempt to use before clamping is based on its spring settings.
/// </remarks>
public struct ServoSettings
{
    /// <summary>
    /// Maximum speed that the constraint can try to use to move towards the target.
    /// </summary>
    public float MaximumSpeed;
    /// <summary>
    /// Minimum speed that the constraint will try to use to move towards the target.
    /// If the speed implied by the spring configuration is higher than this, the servo will attempt to use the higher speed.
    /// Will be clamped by the MaximumSpeed.
    /// </summary>
    public float BaseSpeed;
    /// <summary>
    /// The maximum force that the constraint can apply to move towards the target.
    /// </summary>
    /// <remarks>
    /// This value is specified in terms of force: a change in momentum over time. It is approximated as a maximum impulse (an instantaneous change in momentum) on a per-substep basis. In other words, for a given velocity iteration, the constraint's impulse can be no larger than <see cref="MaximumForce"/> * dt where dt is the substep duration.
    /// </remarks>
    public float MaximumForce;

    /// <summary>
    /// Gets settings representing a servo with unlimited force, speed, and no base speed.
    /// A servo with these settings will behave like a conventional position-level constraint.
    /// </summary>
    public static ServoSettings Default { get { return new ServoSettings(float.MaxValue, 0, float.MaxValue); } }

    /// <summary>
    /// Checks servo settings to ensure valid values.
    /// </summary>
    /// <param name="settings">Settings to check.</param>
    /// <returns>True if the settings contain valid values, false otherwise.</returns>
    public static bool Validate(in ServoSettings settings)
    {
        return ConstraintChecker.IsNonnegativeNumber(settings.MaximumSpeed) && ConstraintChecker.IsNonnegativeNumber(settings.BaseSpeed) && ConstraintChecker.IsNonnegativeNumber(settings.MaximumForce);
    }

    /// <summary>
    /// Creates a new servo settings instance with the specified properties.
    /// </summary>
    /// <param name="maximumSpeed">Sets the <see cref="MaximumSpeed"/> property.</param>
    /// <param name="baseSpeed">Sets the <see cref="BaseSpeed"/> property.</param>
    /// <param name="maximumForce">Sets the <see cref="MaximumForce"/> property.</param>
    public ServoSettings(float maximumSpeed, float baseSpeed, float maximumForce)
    {
        MaximumSpeed = maximumSpeed;
        BaseSpeed = baseSpeed;
        MaximumForce = maximumForce;
        Debug.Assert(Validate(this), "Servo settings must have nonnegative maximum speed, base speed, and maximum force.");
    }
}
public struct ServoSettingsWide
{
    public Vector<float> MaximumSpeed;
    public Vector<float> BaseSpeed;
    public Vector<float> MaximumForce;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void ComputeClampedBiasVelocity(in Vector<float> error, in Vector<float> positionErrorToVelocity, in ServoSettingsWide servoSettings, float dt, float inverseDt, 
        out Vector<float> clampedBiasVelocity, out Vector<float> maximumImpulse)
    {
        //Can't request speed that would cause an overshoot.
        var baseSpeed = Vector.Min(servoSettings.BaseSpeed, Vector.Abs(error) * new Vector<float>(inverseDt));
        var biasVelocity = error * positionErrorToVelocity;
        clampedBiasVelocity = Vector.ConditionalSelect(Vector.LessThan(biasVelocity, Vector<float>.Zero),
            Vector.Max(-servoSettings.MaximumSpeed, Vector.Min(-baseSpeed, biasVelocity)),
            Vector.Min(servoSettings.MaximumSpeed, Vector.Max(baseSpeed, biasVelocity)));
        maximumImpulse = servoSettings.MaximumForce * new Vector<float>(dt);
    }
    
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void ComputeClampedBiasVelocity(in Vector2Wide errorAxis, in Vector<float> errorLength, in Vector<float> positionErrorToBiasVelocity, in ServoSettingsWide servoSettings,
        float dt, float inverseDt, out Vector2Wide clampedBiasVelocity, out Vector<float> maximumImpulse)
    {
        //Can't request speed that would cause an overshoot.
        var baseSpeed = Vector.Min(servoSettings.BaseSpeed, errorLength * new Vector<float>(inverseDt));
        var unclampedBiasSpeed = errorLength * positionErrorToBiasVelocity;
        var targetSpeed = Vector.Max(baseSpeed, unclampedBiasSpeed);
        var scale = Vector.Min(Vector<float>.One, servoSettings.MaximumSpeed / targetSpeed);
        //Protect against division by zero. The min would handle inf, but if MaximumSpeed is 0, it turns into a NaN.
        var useFallback = Vector.LessThan(targetSpeed, new Vector<float>(1e-10f));
        scale = Vector.ConditionalSelect(useFallback, Vector<float>.One, scale);
        Vector2Wide.Scale(errorAxis, scale * unclampedBiasSpeed, out clampedBiasVelocity);
        maximumImpulse = servoSettings.MaximumForce * new Vector<float>(dt);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void ComputeClampedBiasVelocity(in Vector2Wide error, in Vector<float> positionErrorToBiasVelocity, in ServoSettingsWide servoSettings,
        float dt, float inverseDt, out Vector2Wide clampedBiasVelocity, out Vector<float> maximumImpulse)
    {
        Vector2Wide.Length(error, out var errorLength);
        Vector2Wide.Scale(error, Vector<float>.One / errorLength, out var errorAxis);
        var useFallback = Vector.LessThan(errorLength, new Vector<float>(1e-10f));
        errorAxis.X = Vector.ConditionalSelect(useFallback, Vector<float>.Zero, errorAxis.X);
        errorAxis.Y = Vector.ConditionalSelect(useFallback, Vector<float>.Zero, errorAxis.Y);
        ComputeClampedBiasVelocity(errorAxis, errorLength, positionErrorToBiasVelocity, servoSettings, dt, inverseDt, out clampedBiasVelocity, out maximumImpulse);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void ComputeClampedBiasVelocity(in Vector3Wide errorAxis, in Vector<float> errorLength, in Vector<float> positionErrorToBiasVelocity, in ServoSettingsWide servoSettings,
        float dt, float inverseDt, out Vector3Wide clampedBiasVelocity, out Vector<float> maximumImpulse)
    {
        //Can't request speed that would cause an overshoot.
        var baseSpeed = Vector.Min(servoSettings.BaseSpeed, errorLength * new Vector<float>(inverseDt));
        var unclampedBiasSpeed = errorLength * positionErrorToBiasVelocity;
        var targetSpeed = Vector.Max(baseSpeed, unclampedBiasSpeed);
        var scale = Vector.Min(Vector<float>.One, servoSettings.MaximumSpeed / targetSpeed);
        //Protect against division by zero. The min would handle inf, but if MaximumSpeed is 0, it turns into a NaN.
        var useFallback = Vector.LessThan(targetSpeed, new Vector<float>(1e-10f));
        scale = Vector.ConditionalSelect(useFallback, Vector<float>.One, scale);
        Vector3Wide.Scale(errorAxis, scale * unclampedBiasSpeed, out clampedBiasVelocity);
        maximumImpulse = servoSettings.MaximumForce * new Vector<float>(dt);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void ComputeClampedBiasVelocity(in Vector3Wide error, in Vector<float> positionErrorToBiasVelocity, in ServoSettingsWide servoSettings,
       float dt, float inverseDt, out Vector3Wide clampedBiasVelocity, out Vector<float> maximumImpulse)
    {
        Vector3Wide.Length(error, out var errorLength);
        Vector3Wide.Scale(error, Vector<float>.One / errorLength, out var errorAxis);
        var useFallback = Vector.LessThan(errorLength, new Vector<float>(1e-10f));
        errorAxis.X = Vector.ConditionalSelect(useFallback, Vector<float>.Zero, errorAxis.X);
        errorAxis.Y = Vector.ConditionalSelect(useFallback, Vector<float>.Zero, errorAxis.Y);
        errorAxis.Z = Vector.ConditionalSelect(useFallback, Vector<float>.Zero, errorAxis.Z);
        ComputeClampedBiasVelocity(errorAxis, errorLength, positionErrorToBiasVelocity, servoSettings, dt, inverseDt, out clampedBiasVelocity, out maximumImpulse);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void ClampImpulse(in Vector<float> maximumImpulse, ref Vector<float> accumulatedImpulse, ref Vector<float> csi)
    {
        var previousImpulse = accumulatedImpulse;
        accumulatedImpulse = Vector.Max(-maximumImpulse, Vector.Min(maximumImpulse, accumulatedImpulse + csi));
        csi = accumulatedImpulse - previousImpulse;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void ClampImpulse(in Vector<float> maximumImpulse, ref Vector2Wide accumulatedImpulse, ref Vector2Wide csi)
    {
        var previousImpulse = accumulatedImpulse;
        Vector2Wide.Add(accumulatedImpulse, csi, out var unclamped);
        Vector2Wide.Length(unclamped, out var impulseMagnitude);
        var impulseScale = Vector.ConditionalSelect(
            Vector.LessThan(Vector.Abs(impulseMagnitude), new Vector<float>(1e-10f)),
            Vector<float>.One,
            Vector.Min(maximumImpulse / impulseMagnitude, Vector<float>.One));
        Vector2Wide.Scale(unclamped, impulseScale, out accumulatedImpulse);
        Vector2Wide.Subtract(accumulatedImpulse, previousImpulse, out csi);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void ClampImpulse(in Vector<float> maximumImpulse, ref Vector3Wide accumulatedImpulse, ref Vector3Wide csi)
    {
        var previousAccumulatedImpulse = accumulatedImpulse;
        Vector3Wide.Add(accumulatedImpulse, csi, out accumulatedImpulse);
        Vector3Wide.Length(accumulatedImpulse, out var impulseMagnitude);
        var impulseScale = Vector.ConditionalSelect(
            Vector.LessThan(Vector.Abs(impulseMagnitude), new Vector<float>(1e-10f)), 
            Vector<float>.One, 
            Vector.Min(maximumImpulse / impulseMagnitude, Vector<float>.One));
        Vector3Wide.Scale(accumulatedImpulse, impulseScale, out accumulatedImpulse);
        Vector3Wide.Subtract(accumulatedImpulse, previousAccumulatedImpulse, out csi);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void WriteFirst(in ServoSettings source, ref ServoSettingsWide target)
    {
        GatherScatter.GetFirst(ref target.MaximumSpeed) = source.MaximumSpeed;
        GatherScatter.GetFirst(ref target.BaseSpeed) = source.BaseSpeed;
        GatherScatter.GetFirst(ref target.MaximumForce) = source.MaximumForce;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void ReadFirst(in ServoSettingsWide source, out ServoSettings target)
    {
        target.MaximumSpeed = source.MaximumSpeed[0];
        target.BaseSpeed = source.BaseSpeed[0];
        target.MaximumForce = source.MaximumForce[0];
    }

}
