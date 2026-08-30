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

struct CylinderInstance
{
	nointerpolation float3 Position : Position;
	nointerpolation float Radius : Radius;
	nointerpolation uint2 PackedOrientation : PackedOrientation;
	nointerpolation float HalfLength : HalfLength;
	nointerpolation uint PackedColor : PackedColor;
};

StructuredBuffer<CylinderInstance> Instances : register(t0);

struct PSInput
{
	float4 Position : SV_Position;
	float3 ToAABB : RayDirection;
	CylinderInstance Instance;
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
	float3x3 worldToViewRotation = transpose(float3x3(CameraRight, CameraUp, CameraBackward));
	float3 viewPosition = mul(output.Instance.Position, worldToViewRotation);

	//Compute the bounds of the capsule in view space.
	float4 orientation = UnpackOrientation(output.Instance.PackedOrientation);
	float3x3 orientationMatrix = ConvertToRotationMatrix(orientation);
	float3x3 viewSpaceInstanceOrientation = mul(orientationMatrix, worldToViewRotation);
	float3 positiveDiscBoundOffsets = sqrt(saturate(1.0 - viewSpaceInstanceOrientation[1] * viewSpaceInstanceOrientation[1])) * output.Instance.Radius;
	float3 viewMaxOffset = abs(output.Instance.HalfLength * viewSpaceInstanceOrientation[1]) + positiveDiscBoundOffsets;

	float3 vertexViewPosition = viewPosition + viewMaxOffset * aabbCoordinates;
	if (aabbCoordinates.z > 0)
	{
		//Clamp the near side of the AABB to the camera nearclip plane (unless the far side of the AABB passes the near plane).
		//This keeps the raytraced instance visible even during camera overlap.
		//Note the consequences of -z being forward.
		vertexViewPosition.z = max(min(-NearClip - 1e-5, vertexViewPosition.z), viewPosition.z - viewMaxOffset.z);
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

//Like the capsule, this is just using the scalar version of the intersection routine. It's not very GPU friendly, but thaaaat's okay.
bool RayCastCylinder(float3 rayDirection, float3 cylinderPosition, float4 cylinderOrientation, float radius, float halfLength,
	out float t, out float3 hitLocation, out float3 hitNormal)
{
	//It's convenient to work in local space, so pull the ray into the capsule's local space.
	float3x3 orientationMatrix = ConvertToRotationMatrix(cylinderOrientation);
	float3 o = mul(-cylinderPosition, transpose(orientationMatrix));
	float3 d = mul(rayDirection, transpose(orientationMatrix));

	//Note that we assume the ray direction is unit length. That helps with some numerical issues and simplifies some things.
	float2 oh = float2(o.x, o.z);
	float2 dh = float2(d.x, d.z);
	float a = dot(dh, dh);
	float b = dot(oh, dh);
	float radiusSquared = radius * radius;
	float c = dot(oh, oh) - radiusSquared;

	hitLocation = hitNormal = t = 0;
	if (b > 0 && c > 0)
	{
		//Ray is outside and pointing away, no hit.
		return false;
	}

	float discY;
	if (a > 1e-8f)
	{
		float discriminant = b * b - a * c;
		if (discriminant < 0)
		{
			//The infinite cylinder isn't hit, so the finite cylinder can't be hit.
			return false;
		}
		t = (-b - sqrt(discriminant)) / a;
		t = max(t, 0);
		hitLocation.y = o.y + d.y * t;
		if (hitLocation.y < -halfLength)
		{
			discY = -halfLength;
		}
		else if (hitLocation.y > halfLength)
		{
			discY = halfLength;
		}
		else
		{
			//The hit is on the side of the cylinder.
			hitLocation.xz = o.xz + d.xz * t;
			hitNormal.xz = hitLocation.xz / radius;
			hitNormal.y = 0;
			hitNormal = mul(hitNormal, orientationMatrix);
			hitLocation = mul(hitLocation, orientationMatrix) + cylinderPosition;
			return true;
		}
	}
	else
	{
		//The ray is parallel to the axis; the impact is on a disc or nothing.
		discY = d.y > 0 ? -halfLength : halfLength;
	}

	//Intersect the ray with the plane anchored at discY with normal equal to (0,1,0).
	//t = dot(rayOrigin - (0,discY,0), (0,1,0)) / dot(rayDirection, (0,1,0)
	if (o.y * d.y >= 0)
	{
		//The ray can only hit the disc if the direction points toward the cylinder.
		return false;
	}
	t = (discY - o.y) / d.y;
	hitLocation.xz = o.xz + d.xz * t;
	if ((hitLocation.x * hitLocation.x + hitLocation.z * hitLocation.z) > radiusSquared)
	{
		//The hit missed the cap.
		return false;
	}
	hitLocation.y = discY;
	hitLocation = mul(hitLocation, orientationMatrix) + cylinderPosition;
	hitNormal = d.y < 0 ? orientationMatrix[1] : -orientationMatrix[1];
	return true;
}

PSOutput PSMain(PSInput input)
{
	PSOutput output;
	float t = 0;
	float3 hitLocation = 0, hitNormal = 0;
	float4 orientation = UnpackOrientation(input.Instance.PackedOrientation);
	//While we could create a signed distance without terrible difficulty here, cylinders have normal discontinuities that make coverage insufficient for full antialiasing.
	//It would also be possible to modify the ray cast to create a pixel-sensitive 'bevel' to reduce the edge's frequency.
	//Instead of any of that, we apologize to low end graphics cards and brute force multiple ray cast samples.
	//This still won't be quite as good as performing multiple shading samples, but...
	float3 directionDDX = ddx(input.ToAABB);
	float3 directionDDY = ddy(input.ToAABB);
	const int sampleCount = 4;
	float3 directions[sampleCount];
	directions[0] = normalize(input.ToAABB + directionDDX * -0.125 + directionDDY * -0.375);
	directions[1] = normalize(input.ToAABB + directionDDX * 0.375 + directionDDY * -0.125);
	directions[2] = normalize(input.ToAABB + directionDDX * -0.375 + directionDDY * 0.125);
	directions[3] = normalize(input.ToAABB + directionDDX * 0.125 + directionDDY * 0.375);
	float hitCount = 0;
	for (int i = 0; i < sampleCount; ++i)
	{
		float ti;
		float3 hitLocationi, hitNormali;
		if (RayCastCylinder(directions[i], input.Instance.Position, orientation, input.Instance.Radius, input.Instance.HalfLength, ti, hitLocationi, hitNormali))
		{
			t += ti;
			hitLocation += hitLocationi;
			hitNormal += hitNormali;
			hitCount++;
		}
	}
	if (hitCount > 0)
	{
		float inverseHitCount = 1.0 / hitCount;
		t *= inverseHitCount;
		hitLocation *= inverseHitCount;
		hitNormal = normalize(hitNormal);

		float3 dpdx, dpdy;
		GetScreenspaceDerivatives(hitLocation, hitNormal, input.ToAABB, CameraRightPS, CameraUpPS, CameraBackwardPS, PixelSizeAtUnitPlane, dpdx, dpdy);
		float3 color = ShadeSurface(
			hitLocation, hitNormal, UnpackR11G11B10_UNorm(input.Instance.PackedColor), dpdx, dpdy,
			input.Instance.Position, orientation);
		output.Color = float4(color, hitCount / float(sampleCount));
		output.Depth = GetProjectedDepth(-dot(CameraBackwardPS, hitLocation), Near, Far);
	}
	else
	{
		output.Color = 0;
		output.Depth = 0;
	}
	return output;
}