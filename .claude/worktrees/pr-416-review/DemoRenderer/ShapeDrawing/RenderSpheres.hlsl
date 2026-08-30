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

struct SphereInstance
{
	nointerpolation float3 Position : Position;
	nointerpolation float Radius : Radius;
	nointerpolation float3 PackedOrientation : PackedOrientation;
	nointerpolation uint PackedColor : PackedColor;
};

StructuredBuffer<SphereInstance> Instances : register(t0);

struct PSInput
{
	float4 Position : SV_Position;
	float3 ToAABB : RayDirection;
	SphereInstance Sphere;
};
PSInput VSMain(uint vertexId : SV_VertexId)
{
	//The vertex id is used to position each vertex. 
	//Each AABB has 8 vertices; the position is based on the 3 least significant bits.
	int instanceId = vertexId >> 3;
	PSInput output;
	output.Sphere = Instances[instanceId];
	//Note that we move the instance location into camera local translation.
	output.Sphere.Position -= CameraPosition;

	//Convert the vertex id to local AABB coordinates, and then into view space.
	//Note that this id->coordinate transformation requires consistency with the index buffer
	//to ensure proper triangle winding. A set bit in a given position makes it higher along the axis.
	//So vertexId&1 == 1 => +x, vertexId&2 == 2 => +y, and vertexId&4 == 4 => +z.
	float3 aabbCoordinates = float3((vertexId & 1) << 1, vertexId & 2, (vertexId & 4) >> 1) - 1;

	//Note that in view space, -z moves away along the camera's forward axis by convention.
	float3 sphereViewPosition = float3(dot(CameraRight, output.Sphere.Position), dot(CameraUp, output.Sphere.Position), dot(CameraBackward, output.Sphere.Position));
	float3 vertexViewPosition = sphereViewPosition + output.Sphere.Radius * aabbCoordinates;
	if (aabbCoordinates.z > 0)
	{
		//Clamp the near side of the AABB to the camera nearclip plane (unless the far side of the AABB passes the near plane).
		//This keeps the raytraced sphere visible even during camera overlap.
		//Note the consequences of -z being forward.
		vertexViewPosition.z = max(min(-NearClip - 1e-5, vertexViewPosition.z), sphereViewPosition.z - output.Sphere.Radius);
	}
	output.ToAABB = vertexViewPosition.x * CameraRight + vertexViewPosition.y * CameraUp + vertexViewPosition.z * CameraBackward;
	output.Position = mul(float4(vertexViewPosition, 1), Projection);
	return output;
}

struct PSOutput
{
	float4 Color : SV_Target0;
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

bool RayCastSphere(float3 d, float3 spherePosition, float radius,
	out float t, out float3 hitLocation, out float3 hitNormal)
{
	//Note that the ray direction is assumed to be normalized. Helps with numerical issues and simplifies the math.
	//Move the origin up to the earliest possible impact time. This isn't necessary for math reasons, but it does help avoid some numerical problems.
	float3 o = -spherePosition;
	float tOffset = max(0, -dot(o, d) - radius);
	o += d * tOffset;
	float b = dot(o, d);
	float c = dot(o, o) - radius * radius;

	float discriminant = b * b - c;
	t = max(0, -b - sqrt(discriminant));
	hitLocation = o + d * t;
	hitNormal = hitLocation / radius;
	hitLocation += spherePosition;
	t = t + tOffset;
	return (b <= 0 || c <= 0) && discriminant > 0;
}

float GetSignedDistance(float3 direction, float3 position, float radius, out float3 closestPointOnRay)
{
	//Compute closest point on the eye ray to the sphere center.
	closestPointOnRay = direction * dot(direction, position);
	return distance(closestPointOnRay, position) - radius;
}

PSOutput PSMain(PSInput input)
{
	PSOutput output;
	float t;
	float3 hitLocation, hitNormal;
	float3 direction = normalize(input.ToAABB);
	if (RayCastSphere(direction, input.Sphere.Position, input.Sphere.Radius, t, hitLocation, hitNormal))
	{
		float4 orientation = UnpackOrientation(input.Sphere.PackedOrientation);
		float3 dpdx, dpdy;
		GetScreenspaceDerivatives(hitLocation, hitNormal, input.ToAABB, CameraRightPS, CameraUpPS, CameraBackwardPS, PixelSizeAtUnitPlane, dpdx, dpdy);
		float3 color = ShadeSurface(
			hitLocation, hitNormal, UnpackR11G11B10_UNorm(input.Sphere.PackedColor), dpdx, dpdy,
			input.Sphere.Position, orientation);
		float3 closestPointOnRay;
		float signedDistance = GetSignedDistance(direction, input.Sphere.Position, input.Sphere.Radius, closestPointOnRay);
		output.Color = float4(color, GetCoverage(signedDistance, PixelSizeAtUnitPlane, -dot(closestPointOnRay, CameraBackwardPS)));
		output.Depth = GetProjectedDepth(-dot(CameraBackwardPS, hitLocation), Near, Far);
	}
	else
	{
		output.Color = 0;
		output.Depth = 0;
	}
	return output;
}