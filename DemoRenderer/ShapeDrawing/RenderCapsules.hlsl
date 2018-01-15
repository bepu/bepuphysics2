/*[META]
vs
ps
[META]*/
#include "Common.hlsl"
cbuffer VertexConstants : register(b0)
{
	float4x4 Projection;
	float3 CameraPosition;
	float NearClip;
	float3 CameraRight;
	float Padding0;
	float3 CameraUp;
	float Padding1;
	float3 CameraBackward;
};

struct CapsuleInstance
{
	nointerpolation float3 Position : Position;
	nointerpolation float Radius : Radius;
	nointerpolation uint2 PackedOrientation : PackedOrientation;
	nointerpolation float HalfLength : HalfLength;
	nointerpolation uint PackedColor : PackedColor;
};

StructuredBuffer<CapsuleInstance> Instances : register(t0);

struct PSInput
{
	float4 Position : SV_Position;
	float3 ToAABB : RayDirection;
	CapsuleInstance Instance;
};
PSInput VSMain(uint vertexId : SV_VertexId)
{
	//The vertex id is used to position each vertex. 
	//Each AABB has 8 vertices; the position is based on the 3 least significant bits.
	int instanceId = vertexId >> 3;
	PSInput output;
	output.Instance = Instances[instanceId];
	//Note that we move the instance location into camera local translation.
	output.Instance.Position -= CameraPosition;

	//Convert the vertex id to local AABB coordinates, and then into view space.
	//Note that this id->coordinate transformation requires consistency with the index buffer
	//to ensure proper triangle winding. A set bit in a given position makes it higher along the axis.
	//So vertexId&1 == 1 => +x, vertexId&2 == 2 => +y, and vertexId&4 == 4 => +z.
	float3 aabbCoordinates = float3((vertexId & 1) << 1, vertexId & 2, (vertexId & 4) >> 1) - 1;

	//Note that in view space, -z moves away along the camera's forward axis by convention.
	float3 viewPosition = float3(dot(CameraRight, output.Instance.Position), dot(CameraUp, output.Instance.Position), dot(CameraBackward, output.Instance.Position));
	
	//Compute the bounds of the capsule in view space.
	float4 orientation = UnpackOrientation(output.Instance.PackedOrientation);
	float3 maxOffset = output.Instance.HalfLength * abs(TransformUnitY(orientation)) + output.Instance.Radius;

	float3 vertexViewPosition = viewPosition + maxOffset * aabbCoordinates;
	if (aabbCoordinates.z > 0)
	{
		//Clamp the near side of the AABB to the camera nearclip plane (unless the far side of the AABB passes the near plane).
		//This keeps the raytraced instance visible even during camera overlap.
		//Note the consequences of -z being forward.
		vertexViewPosition.z = max(min(-NearClip - 1e-5, vertexViewPosition.z), viewPosition.z - maxOffset.z);
	}
	output.ToAABB = vertexViewPosition.x * CameraRight + vertexViewPosition.y * CameraUp + vertexViewPosition.z * CameraBackward;
	output.Position = mul(float4(vertexViewPosition, 1), Projection);
	return output;
}

struct PSOutput
{
	float3 Color : SV_Target0;
	float Depth : SV_DepthLessEqual;
};

cbuffer PixelConstants : register(b1)
{
	float3 CameraRightPS;
	float Near;
	float3 CameraUpPS;
	float Far;
	float3 CameraBackwardPS;
	float Padding;
	float2 PixelSizeAtUnitPlane;
};


float GetProjectedDepth(float linearDepth, float near, float far)
{
	//Note the reversal of near and far relative to a standard depth projection.
	//We use 0 to mean furthest, and 1 to mean closest.
	float dn = linearDepth * near;
	return (far * near - dn) / (linearDepth * far - dn);
}

bool RayCastCapsule(float3 rayDirection, float3 spherePosition, float radius, float halfLength, float4 orientation,
	out float t, out float3 hitLocation, out float3 hitNormal)
{
	float directionLength = length(rayDirection);
	float3 normalizedDirection = rayDirection / directionLength;
	float3 m = -spherePosition;
	float b = dot(m, normalizedDirection);
	float c = dot(m, m) - radius * radius;

	//This isn't a very good GPU implementation, but only worry about that if it becomes an issue.
	if (c > 0 && b > 0)
	{
		t = 0;
		hitLocation = 0;
		hitNormal = 0;
		return false;
	}
	else
	{
		float discriminant = b * b - c;
		if (discriminant < 0)
		{
			t = 0;
			hitLocation = 0;
			hitNormal = 0;
			return false;
		}
		else
		{
			t = -b - sqrt(discriminant);
			if (t < 0)
			{
				t = 0;
				hitLocation = 0;
				hitNormal = 0;
				return false;
			}
			else
			{
				hitLocation = normalizedDirection * t;
				hitNormal = normalize(hitLocation - spherePosition);
				return true;
			}
		}
	}
}


PSOutput PSMain(PSInput input)
{
	PSOutput output;
	//float t;
	//float3 hitLocation, hitNormal;
	//if (RayCastCapsule(input.ToAABB, input.Instance.Position, input.Instance.Radius, t, hitLocation, hitNormal))
	//{
	//	float3 baseColor = UnpackR11G11B10_UNorm(input.Sphere.PackedColor);
	//	float z = -dot(CameraBackwardPS, hitLocation);
	//	float4 orientation = UnpackOrientation(input.Sphere.PackedOrientation);
	//	float3 dpdx, dpdy;
	//	GetScreenspaceDerivatives(hitLocation, hitNormal, input.ToAABB, CameraRightPS, CameraUpPS, CameraBackwardPS, PixelSizeAtUnitPlane, dpdx, dpdy);
	//	float3 color = ShadeSurface(
	//		hitLocation, hitNormal, UnpackR11G11B10_UNorm(input.Sphere.PackedColor), dpdx, dpdy,
	//		input.Sphere.Position, orientation, input.Sphere.Radius * 2, z);
	//	output.Color = color;
	//	output.Depth = GetProjectedDepth(z, Near, Far);
	//}
	//else
	{
		output.Color = 0;
		output.Depth = 0;
	}
	return output;
}