#include "ColorUnpacking.hlsl"
static const float3 SunColor = float3(1, 1, 1);
static const float3 ToSunDirection = normalize(float3(0.37, 0.93, 0.3));
static const float3 SkyColor = float3(0.128, 0.283, .855);
static const float3 BackgroundBase = float3(0.125, 0.125, 0.125);


float4 UnpackOrientation(float3 packedOrientation)
{
	//This packing is pretty simple and doesn't attempt to save much space. It's just targeting 32 bytes for the full struct.
	//Since we only need to open up 4 bytes, the quaternion is modified to guarantee that the W component is positive. We can then reconstruct the W component with a sqrt.
	return float4(packedOrientation, sqrt(saturate(1 - dot(packedOrientation, packedOrientation))));
}

float UnpackComponent(uint packed)
{
	float magnitude = (packed & ((1 << 15) - 1)) * (1.0 / ((1 << 15) - 1));
	uint unpackedUInt = asuint(magnitude);
	//Set the sign bit.
	unpackedUInt |= (packed & (1u << 15)) << 16;
	return asfloat(unpackedUInt | ((packed & (1u << 15)) << 16));
}
float4 UnpackOrientation(uint2 packedOrientation)
{
	float4 orientation = float4(
		UnpackComponent(packedOrientation.x),
		UnpackComponent(packedOrientation.x >> 16),
		UnpackComponent(packedOrientation.y),
		UnpackComponent(packedOrientation.y >> 16));
	return orientation / length(orientation);
}
float4 Conjugate(float4 rotation)
{
	return float4(rotation.xyz, -rotation.w);
}
float3 Transform(float3 v, float4 rotation)
{
	float x2 = rotation.x + rotation.x;
	float y2 = rotation.y + rotation.y;
	float z2 = rotation.z + rotation.z;
	float xx2 = rotation.x * x2;
	float xy2 = rotation.x * y2;
	float xz2 = rotation.x * z2;
	float yy2 = rotation.y * y2;
	float yz2 = rotation.y * z2;
	float zz2 = rotation.z * z2;
	float wx2 = rotation.w * x2;
	float wy2 = rotation.w * y2;
	float wz2 = rotation.w * z2;
	return float3(
		v.x * (1.0 - yy2 - zz2) + v.y * (xy2 - wz2) + v.z * (xz2 + wy2),
		v.x * (xy2 + wz2) + v.y * (1.0 - xx2 - zz2) + v.z * (yz2 - wx2),
		v.x * (xz2 - wy2) + v.y * (yz2 + wx2) + v.z * (1.0 - xx2 - yy2));
}

float3x3 ConvertToRotationMatrix(float4 rotation)
{
	float x2 = rotation.x + rotation.x;
	float y2 = rotation.y + rotation.y;
	float z2 = rotation.z + rotation.z;
	float xx2 = rotation.x * x2;
	float xy2 = rotation.x * y2;
	float xz2 = rotation.x * z2;
	float yy2 = rotation.y * y2;
	float yz2 = rotation.y * z2;
	float zz2 = rotation.z * z2;
	float wx2 = rotation.w * x2;
	float wy2 = rotation.w * y2;
	float wz2 = rotation.w * z2;
	return float3x3(
		float3(1.0 - yy2 - zz2, xy2 + wz2, xz2 - wy2),
		float3(xy2 - wz2, 1.0 - xx2 - zz2, yz2 + wx2),
		float3(xz2 + wy2, yz2 - wx2, 1.0 - xx2 - yy2));
}

float3 TransformUnitY(float4 rotation)
{
	float x2 = rotation.x + rotation.x;
	float y2 = rotation.y + rotation.y;
	float z2 = rotation.z + rotation.z;
	float xx2 = rotation.x * x2;
	float xy2 = rotation.x * y2;
	float yz2 = rotation.y * z2;
	float zz2 = rotation.z * z2;
	float wx2 = rotation.w * x2;
	float wz2 = rotation.w * z2;
	return float3(xy2 - wz2, 1.0 - xx2 - zz2, yz2 + wx2);
}

float3 AccumulateLight(float3 normal)
{
	//This uses a very simple ambient + diffuse + approximate hemispheric sky scheme.
	//There are two directional light sources- one from the sun, and one dimmer source in the opposite direction. 
	//The opposite direction roughly approximates the light from retroreflected skylight (in the infinite void...). A little extra character without just making everything fullbright. 
	float sunDot = dot(ToSunDirection, normal);
	return
		BackgroundBase +
		saturate(normal.y) * SkyColor * 0.3 +
		SunColor * (saturate(sunDot) + 0.2 * saturate(-sunDot));
}

float EvaluateIntegral(float x, float normalizedLineWidth)
{
	float halfWidth = normalizedLineWidth * 0.5;
	float shiftedX = x - halfWidth;
	//This is a piecewise function. It increases linearly every time it reaches a grid plane's covered interval, and remains flat everywhere else.
	return normalizedLineWidth * (floor(shiftedX + 1) + saturate((shiftedX - floor(shiftedX) - (1 - normalizedLineWidth)) / normalizedLineWidth));
}
float GetLocalGridPlaneCoverage(float2 interval, float inverseGridSpacing, float lineWidth)
{
	//Work in grid units.
	float start = interval.x * inverseGridSpacing;
	float end = interval.y * inverseGridSpacing;
	float normalizedLineWidth = lineWidth * inverseGridSpacing;
	//Note that we assume a simple uniform box region aligned with the local plane normal. That's not actually correct, but it's a decent approximation.
	//In order to get the coverage fraction, we analytically integrate how much of the interval is covered and divide that by the whole interval width.
	float normalizedCoveredSpan = EvaluateIntegral(end, normalizedLineWidth) - EvaluateIntegral(start, normalizedLineWidth);
	return saturate(normalizedCoveredSpan / max(1e-7, end - start));
}

float GetNormalFade(float axisLocalNormal)
{
	const float fadeStart = 0.707;
	const float fadeEnd = 0.71;
	return 1 - saturate((abs(axisLocalNormal) - fadeStart) / (fadeEnd - fadeStart));
}

float GetLocalGridCoverage(
	float3 localPosition, float3 localNormal,
	float inverseGridSpacing,
	float lineWidth,
	float3 halfSampleSpan)
{
	float x = GetLocalGridPlaneCoverage(float2(localPosition.x - halfSampleSpan.x, localPosition.x + halfSampleSpan.x), inverseGridSpacing, lineWidth) * GetNormalFade(localNormal.x);
	float y = GetLocalGridPlaneCoverage(float2(localPosition.y - halfSampleSpan.y, localPosition.y + halfSampleSpan.y), inverseGridSpacing, lineWidth) * GetNormalFade(localNormal.y);
	float z = GetLocalGridPlaneCoverage(float2(localPosition.z - halfSampleSpan.z, localPosition.z + halfSampleSpan.z), inverseGridSpacing, lineWidth) * GetNormalFade(localNormal.z);

	float contribution = x + y * (1 - x);
	contribution = contribution + z * (1 - contribution);
	return contribution;
}

float4 GetLocalGridContributions(float3 localPosition, float3 localNormal, float3 dpdx, float3 dpdy)
{
	const float smallGridSpacing = 1.0;
	const float mediumGridSpacing = 5.0;
	const float largeGridSpacing = 25.0;

	const float smallLineWidth = 0.01;
	const float mediumLineWidth = 0.035;
	const float largeLineWidth = .1;

	const float3 smallLineColor = 0.15;
	const float3 mediumLineColor = 0.075;
	const float3 largeLineColor = 0.05;

	//Create a local bounding box for the sample. We assume the screenspace derivatives dpdx and dpdy extend both positively and negatively from the central sample.
	//Since they're centered on the central sample, the extent in either direction is half of the derivative.
	float3 halfSampleSpan = 0.5 * max(abs(dpdx), abs(dpdy));
	float smallCoverage = GetLocalGridCoverage(localPosition, localNormal, 1.0 / smallGridSpacing,
		smallLineWidth, halfSampleSpan);
	float mediumCoverage = GetLocalGridCoverage(localPosition, localNormal, 1.0 / mediumGridSpacing,
		mediumLineWidth, halfSampleSpan);
	float largeCoverage = GetLocalGridCoverage(localPosition, localNormal, 1.0 / largeGridSpacing,
		largeLineWidth, halfSampleSpan);
	float4 smallContribution = float4(smallCoverage * smallLineColor, smallCoverage);
	float4 mediumContribution = float4(mediumCoverage * mediumLineColor, mediumCoverage);
	float4 largeContribution = float4(largeCoverage * largeLineColor, largeCoverage);

	float4 contribution = mediumContribution + smallContribution * (1 - mediumContribution.w);
	return largeContribution + contribution * (1 - largeContribution.w);
}


void GetScreenspaceDerivatives(float3 surfacePosition, float3 surfaceNormal, float3 currentRayDirection, float3 right, float3 up, float3 backward,
	float2 pixelSizeAtUnitPlane, out float3 dpdx, out float3 dpdy)
{
	//Pull the ray direction into view local space.
	float3 viewSpaceDirection = float3(
		dot(currentRayDirection, right),
		dot(currentRayDirection, up),
		dot(currentRayDirection, -backward));
	//Now build two adjacent pixel directions and bring them into world space, since the surface normal is in world space.
	float2 unitZDirection = viewSpaceDirection.xy / viewSpaceDirection.z;
	float2 adjacentX = unitZDirection + float2(pixelSizeAtUnitPlane.x, 0);
	float3 adjacentXDirection = adjacentX.x * right + adjacentX.y * up - backward;
	float2 adjacentY = unitZDirection + float2(0, pixelSizeAtUnitPlane.y);
	float3 adjacentYDirection = adjacentY.x * right + adjacentY.y * up - backward;
	float velocityX = dot(surfaceNormal, adjacentXDirection);
	float velocityY = dot(surfaceNormal, adjacentYDirection);
	float distance = dot(surfaceNormal, surfacePosition);
	float tX = min(1e7, distance / velocityX);
	float tY = min(1e7, distance / velocityY);
	dpdx = adjacentXDirection * tX - surfacePosition;
	dpdy = adjacentYDirection * tY - surfacePosition;


}

float GetCoverage(float signedDistanceFromShape, float2 pixelSizeAtUnitPlane, float z)
{
	float pixelSize = z * max(pixelSizeAtUnitPlane.x, pixelSizeAtUnitPlane.y);
	//0 distance = 0 coverage. -1 distance in pixels = 1 coverage.
	//Note that this effectively 'insets' the antialiasing compared to the ground truth. It's only a half pixel off and it's simple, so not a big deal.
	return saturate(-signedDistanceFromShape / pixelSize);
}

float3 ShadeSurface(float3 surfacePosition, float3 surfaceNormal, float3 surfaceColor, float3 dpdx, float3 dpdy,
	float3 instancePosition, float4 instanceOrientation)
{
	float3x3 worldToLocalRotation = ConvertToRotationMatrix(Conjugate(instanceOrientation));
	float3 shapeLocalPosition = mul(surfacePosition - instancePosition, worldToLocalRotation);
	float3 shapeLocalNormal = mul(surfaceNormal, worldToLocalRotation);
	float3 shapeLocalDpdx = mul(dpdx, worldToLocalRotation);
	float3 shapeLocalDpdy = mul(dpdy, worldToLocalRotation);
	float4 grid = GetLocalGridContributions(shapeLocalPosition, shapeLocalNormal, shapeLocalDpdx, shapeLocalDpdy);
	float3 compositedColor = grid.xyz + surfaceColor * (1 - grid.w);
	return compositedColor * AccumulateLight(surfaceNormal);
}

float GetProjectedDepth(float linearDepth, float near, float far)
{
	//Note the reversal of near and far relative to a standard depth projection.
	//We use 0 to mean furthest, and 1 to mean closest.
	float dn = linearDepth * near;
	return (far * near - dn) / (linearDepth * far - dn);
}
