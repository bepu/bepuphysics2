using BepuUtilities;
using System;
using System.Numerics;

namespace DemoRenderer
{
    /// <summary>
    /// Simple yaw/pitch up-locked camera.
    /// </summary>
    public class Camera
    {
        /// <summary>
        /// Gets or sets the position of the camera.
        /// </summary>
        public Vector3 Position;

        float yaw;
        /// <summary>
        /// Gets or sets the yaw of the camera as a value from -PI to PI. At 0, Forward is aligned with -z. At PI/2, Forward is aligned with +x. In other words, higher values turn right.
        /// </summary>
        public float Yaw
        {
            get => yaw;
            set
            {
                var revolution = (value + Math.PI) / (2 * Math.PI);
                revolution -= Math.Floor(revolution);
                yaw = (float)(revolution * (Math.PI * 2) - Math.PI);
            }
        }
        float pitch;
        /// <summary>
        /// Gets or sets the pitch of the camera, clamped to a value from -MaximumPitch to MaximumPitch. Higher values look downward, lower values look upward.
        /// </summary>
        public float Pitch
        {
            get => pitch;
            set => pitch = Math.Clamp(value, -maximumPitch, maximumPitch);
        }

        float maximumPitch;
        /// <summary>
        /// Gets or sets the maximum pitch of the camera, a value from 0 to PI / 2.
        /// </summary>
        public float MaximumPitch
        {
            get => maximumPitch;
            set => maximumPitch = (float)Math.Clamp(value, 0, Math.PI / 2);
        }

        /// <summary>
        /// Gets or sets the aspect ratio of the camera.
        /// </summary>
        public float AspectRatio;

        /// <summary>
        /// Gets or sets the vertical field of view of the camera.
        /// </summary>
        public readonly float FieldOfView;

        /// <summary>
        /// Gets or sets the near plane of the camera.
        /// </summary>
        public readonly float NearClip;

        /// <summary>
        /// Gets or sets the far plane of the camera.
        /// </summary>
        public readonly float FarClip;

        //All of this could be quite a bit faster, but wasting a few thousand cycles per frame isn't exactly a concern.
        /// <summary>
        /// Gets the orientation quaternion of the camera.
        /// </summary>
        public Quaternion OrientationQuaternion
        {
            get
            {
                QuaternionEx.CreateFromYawPitchRoll(-yaw, -pitch, 0, out var orientationQuaternion);
                return orientationQuaternion;
            }
        }

        /// <summary>
        /// Gets the orientation transform of the camera.
        /// </summary>
        public Matrix Orientation => Matrix.CreateFromQuaternion(OrientationQuaternion);

        /// <summary>
        /// Gets the right direction of the camera. Equivalent to transforming (1,0,0) by Orientation.
        /// </summary>
        public Vector3 Right
        {
            get
            {
                var orientation = OrientationQuaternion;
                QuaternionEx.TransformUnitX(orientation, out var right);
                return right;
            }
        }
        /// <summary>
        /// Gets the left direction of the camera. Equivalent to transforming (-1,0,0) by Orientation.
        /// </summary>
        public Vector3 Left => -Right;
        /// <summary>
        /// Gets the up direction of the camera. Equivalent to transforming (0,1,0) by Orientation.
        /// </summary>
        public Vector3 Up
        {
            get
            {
                var orientation = OrientationQuaternion;
                QuaternionEx.TransformUnitY(orientation, out var up);
                return up;
            }
        }
        /// <summary>
        /// Gets the down direction of the camera. Equivalent to transforming (0,-1,0) by Orientation.
        /// </summary>
        public Vector3 Down => -Up;
        /// <summary>
        /// Gets the backward direction of the camera. Equivalent to transforming (0,0,1) by Orientation.
        /// </summary>
        public Vector3 Backward
        {
            get
            {
                var orientation = OrientationQuaternion;
                QuaternionEx.TransformUnitZ(orientation, out var backward);
                return backward;
            }
        }
        /// <summary>
        /// Gets the forward direction of the camera. Equivalent to transforming (0,0,-1) by Orientation.
        /// </summary>
        public Vector3 Forward => -Backward;

        /// <summary>
        /// Gets the world transform of the camera.
        /// </summary>
        public Matrix World
        {
            get
            {
                var world = Orientation;
                world.Translation = Position;
                return world;
            }
        }

        /// <summary>
        /// Gets the view transform of the camera.
        /// </summary>
        public Matrix View => Matrix.Invert(World);

        /// <summary>
        /// Gets the projection transform of the camera using reversed depth.
        /// </summary>
        public Matrix Projection
        {
            get
            {
                var f = 1f / (float)Math.Tan(FieldOfView * 0.5f);
                var nfi = 1f / (FarClip - NearClip);
                return new Matrix {
                    X = new Vector4(f / AspectRatio, 0f, 0f, 0f),
                    Y = new Vector4(0f, f, 0f, 0f),
                    Z = new Vector4(0f, 0f, (NearClip + FarClip) * nfi, -1f),
                    W = new Vector4(0f, 0f, 2f * NearClip * FarClip * nfi, 0f)
                };
            }
        }

        /// <summary>
        /// Gets the combined view * projection of the camera.
        /// </summary>
        public Matrix ViewProjection => View * Projection;

        /// <summary>
        /// Creates a new camera.
        /// </summary>
        /// <param name="aspectRatio">Aspect ratio of the camera's projection.</param>
        /// <param name="fieldOfView">Vertical field of view of the camera's projection.</param>
        /// <param name="nearClip">Near clip plane of the camera's projection.</param>
        /// <param name="farClip">Far clip plane of the camera's projection.</param>
        /// <param name="maximumPitch">Maximum angle that the camera can look up or down.</param>
        public Camera(float aspectRatio, float fieldOfView, float nearClip, float farClip, float maximumPitch = MathF.PI * 0.499f)
        {
            AspectRatio = aspectRatio;
            FieldOfView = fieldOfView;
            MaximumPitch = maximumPitch;
            NearClip = nearClip;
            FarClip = farClip;
        }

        /// <summary>
        /// Gets the ray direction for the given mouse state.
        /// </summary>
        /// <param name="mouseLocked">Whether the mouse is currently locked. If locked, the ray corresponding to the center of the screen will be used.</param>
        /// <param name="normalizedMousePosition">Location of the mouse normalized to [0, 1] relative to window bounds.</param>
        /// <returns>World space ray direction pointing the mouse's direction.</returns>
        public Vector3 GetRayDirection(bool mouseLocked, in Vector2 normalizedMousePosition)
        {
            //The ray direction depends on the camera and whether the camera is locked.
            if (mouseLocked)
            {
                return Forward;
            }
            var unitPlaneHalfHeight = MathF.Tan(FieldOfView * 0.5f);
            var unitPlaneHalfWidth = unitPlaneHalfHeight * AspectRatio;
            var localRayDirection = new Vector3(
                new Vector2(unitPlaneHalfWidth, unitPlaneHalfHeight) * 2 * new Vector2(normalizedMousePosition.X - 0.5f, 0.5f - normalizedMousePosition.Y), -1);
            QuaternionEx.TransformWithoutOverlap(localRayDirection, OrientationQuaternion, out var rayDirection);
            return rayDirection;
        }
    }
}
