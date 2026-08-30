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
	float3x3 worldToViewRotation = transpose(float3x3(CameraRight, CameraUp, CameraBackward));
	float3 viewPosition = mul(output.Instance.Position, worldToViewRotation);

	//Compute the bounds of the capsule in view space.
	float4 orientation = UnpackOrientation(output.Instance.PackedOrientation);
	float3x3 orientationMatrix = ConvertToRotationMatrix(orientation);
	float3x3 viewSpaceInstanceOrientation = mul(orientationMatrix, worldToViewRotation);
	float3 viewMaxOffset = output.Instance.HalfLength * abs(viewSpaceInstanceOrientation[1]) + output.Instance.Radius;

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

//Like the cylinder, this is just using the scalar version of the intersection routine. It's not very GPU friendly, but thaaaat's okay.
bool RayCastCapsule(float3 rayDirection, float3 capsulePosition, float4 capsuleOrientation, float radius, float halfLength,
	out float t, out float3 hitLocation, out float3 hitNormal)
{
	//It's convenient to work in local space, so pull the ray into the capsule's local space.
	float3x3 orientationMatrix = ConvertToRotationMatrix(capsuleOrientation);
	float3 o = mul(-capsulePosition, transpose(orientationMatrix));
	float3 d = mul(rayDirection, transpose(orientationMatrix));

	//Note that we assume the ray direction is unit length. That helps with some numerical issues and simplifies some things.

	//Move the origin up to the earliest possible impact time. This isn't necessary for math reasons, but it does help avoid some numerical problems.
	float tOffset = 0;// max(0, -dot(o, d) - (halfLength + radius));
	o += d * tOffset;
	float2 oh = float2(o.x, o.z);
	float2 dh = float2(d.x, d.z);
	float a = dot(dh, dh);
	float b = dot(oh, dh);
	float radiusSquared = radius * radius;
	float c = dot(oh, oh) - radiusSquared;
	if (b > 0 && c > 0)
	{
		//Ray is outside and pointing away, no hit.
		hitLocation = hitNormal = t = 0;
		return false;
	}

	float sphereY;
	if (a > 1.0e-8)
	{
		float discriminant = b * b - a * c;
		if (discriminant < 0)
		{
			//The infinite cylinder isn't hit, so the capsule can't be hit.
			hitLocation = hitNormal = t = 0;
			return false;
		}
		t = max(-tOffset, (-b - sqrt(discriminant)) / a);
		float3 cylinderHitLocation = o + d * t;
		if (cylinderHitLocation.y < -halfLength)
		{
			sphereY = -halfLength;
		}
		else if (cylinderHitLocation.y > halfLength)
		{
			sphereY = halfLength;
		}
		else
		{
			//The hit is on the cylindrical portion of the capsule.
			hitNormal = mul(float3(cylinderHitLocation.x, 0, cylinderHitLocation.z) / radius, orientationMatrix);
			t = t + tOffset;
			hitLocation = rayDirection * t;
			return true;
		}
	}
	else
	{
		//The ray is parallel to the axis; the impact is on a spherical cap or nothing.
		sphereY = d.y > 0 ? -halfLength : halfLength;
	}
	float3 os = float3(o.x, o.y - sphereY, o.z);
	float capB = dot(os, d);
	float capC = dot(os, os) - radiusSquared;

	if (capB > 0 && capC > 0)
	{
		//Ray is outside and pointing away, no hit.
		hitLocation = hitNormal = t = 0;
		return false;
	}

	float capDiscriminant = capB * capB - capC;
	if (capDiscriminant < 0)
	{
		//Ray misses, no hit.
		hitLocation = hitNormal = t = 0;
		return false;
	}
	t = max(-tOffset, -capB - sqrt(capDiscriminant));
	hitNormal = mul((os + d * t) / radius, orientationMatrix);
	t = t + tOffset;
	hitLocation = rayDirection * t;
	return true;
}

float GetSignedDistance(float3 direction, float3 position, float4 orientation, float radius, float halfLength, out float3 closestPointOnRay)
{
	//Compute closest points between the infinite eye ray and the capsule's internal line segment.
	//Note that direction is assumed to be unit length.

	//Compute the closest points between the infinite eye ray and internal line segment. No clamping to begin with.
	//We want to minimize distance = ||(a + da * ta) - (b + db * tb)||.
	//Taking the derivative with respect to ta and doing some algebra (taking into account ||da|| == ||db|| == 1) to solve for ta yields:
	//ta = (da * (b - a) + (db * (a - b)) * (da * db)) / (1 - ((da * db) * (da * db))    
	//Treat the capsule's internal segment as 'a', and the eye ray as 'b'.
	float3 da = TransformUnitY(orientation);
	float daOffsetB = -dot(da, position);
	float dbOffsetB = -dot(direction, position);
	float dadb = dot(da, direction);
	//Note potential division by zero when the axes are parallel. Arbitrarily clamp; near zero values will instead produce extreme values which get clamped to reasonable results.
	float ta = clamp((daOffsetB - dbOffsetB * dadb) / max(1e-15, 1.0 - dadb * dadb), -halfLength, halfLength);
	//tb = ta * (da * db) - db * (b - a)
	float tb = ta * dadb - dbOffsetB;
	closestPointOnRay = tb * direction;
	float3 segmentLocation = position + da * ta;
	float3 offset = closestPointOnRay - segmentLocation;
	float internalDistance = length(offset);
	return internalDistance - radius;
}

PSOutput PSMain(PSInput input)
{
	PSOutput output;
	float t;
	float3 hitLocation, hitNormal;
	float4 orientation = UnpackOrientation(input.Instance.PackedOrientation);
	float3 direction = normalize(input.ToAABB);
	if (RayCastCapsule(direction, input.Instance.Position, orientation, input.Instance.Radius, input.Instance.HalfLength, t, hitLocation, hitNormal))
	{
		float3 dpdx, dpdy;
		GetScreenspaceDerivatives(hitLocation, hitNormal, input.ToAABB, CameraRightPS, CameraUpPS, CameraBackwardPS, PixelSizeAtUnitPlane, dpdx, dpdy);
		float3 color = ShadeSurface(
			hitLocation, hitNormal, UnpackR11G11B10_UNorm(input.Instance.PackedColor), dpdx, dpdy,
			input.Instance.Position, orientation);
		float3 closestPointOnRay;
		float signedDistance = GetSignedDistance(direction, input.Instance.Position, orientation, input.Instance.Radius, input.Instance.HalfLength, closestPointOnRay);
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