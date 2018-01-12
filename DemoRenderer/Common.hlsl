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
	return normalizedLineWidth * (floor(x + 1 + halfWidth) + saturate((shiftedX - floor(shiftedX) - (1 - normalizedLineWidth)) / normalizedLineWidth));
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
	return normalizedCoveredSpan / max(1e-7, end - start);
}

float GetNormalFade(float axisLocalNormal)
{
	const float fadeStart = 0.707;
	const float fadeEnd = 0.71;
	return 1 - saturate((abs(axisLocalNormal) - fadeStart) / (fadeEnd - fadeStart));
}

float GetLocalGridCoverage(
	float3 localPosition, float3 localNormal, float distance,
	float inverseGridSpacing,
	float lineWidth,
	float fadeOutStart, float fadeOutEnd)
{
	float distanceFade = (1 - saturate((distance - fadeOutStart) / (fadeOutEnd - fadeOutStart)));
	float x = GetLocalGridPlaneCoverage(localPosition.x, inverseGridSpacing, lineWidth) * (distanceFade * GetNormalFade(localNormal.x));
	float y = GetLocalGridPlaneCoverage(localPosition.y, inverseGridSpacing, lineWidth) * (distanceFade * GetNormalFade(localNormal.y));
	float z = GetLocalGridPlaneCoverage(localPosition.z, inverseGridSpacing, lineWidth) * (distanceFade * GetNormalFade(localNormal.z));

	float contribution = x + y * (1 - x);
	contribution = contribution + z * (1 - contribution);
	return contribution;
}

float4 GetLocalGridContributions(float3 localPosition, float3 localNormal, float3 dpdx, float3 dpdy, float shapeSize, float distance)
{
	const float smallGridSpacing = 1.0;
	const float mediumGridSpacing = 5.0;
	const float largeGridSpacing = 25.0;

	const float smallGridFadeOutStart = 30;
	const float smallGridFadeOutEnd = 50;

	const float mediumGridFadeOutStart = 100;
	const float mediumGridFadeOutEnd = 150;

	const float largeGridFadeOutStart = 800;
	const float largeGridFadeOutEnd = 1500;

	float smallOuterLineWidth = 0.01;
	float smallInnerLineWidth = smallOuterLineWidth * 0.35;

	float mediumOuterLineWidth = 0.05;
	float mediumInnerLineWidth = mediumOuterLineWidth * 0.35;

	float largeOuterLineWidth = .1;
	float largeInnerLineWidth = largeOuterLineWidth * 0.35;

	float smallLineWidth = distance * 0.0025;
	float mediumLineWidth = distance * 0.0035;
	float largeLineWidth = distance * 0.0045;
	float innerLineWidthScale = 0.35;

	const float3 smallLineColor = 0.15;
	const float3 mediumLineColor = 0.1;
	const float3 largeLineColor = 0.05;

	float sizeFadeStart = shapeSize * 0.125;
	float sizeFadeEnd = shapeSize * 0.25;
	float sizeFade = 1;// 1 - saturate((largeLineWidth - sizeFadeStart) / (sizeFadeEnd - sizeFadeStart));

	float smallCoverage = GetLocalGridCoverage(localPosition, localNormal, distance, 1.0 / smallGridSpacing,
		smallLineWidth,
		smallGridFadeOutStart, smallGridFadeOutEnd) * sizeFade;
	float mediumCoverage = GetLocalGridCoverage(localPosition, localNormal, distance, 1.0 / mediumGridSpacing,
		mediumLineWidth,
		mediumGridFadeOutStart, mediumGridFadeOutEnd)* sizeFade;
	float largeCoverage = GetLocalGridCoverage(localPosition, localNormal, distance, 1.0 / largeGridSpacing,
		largeLineWidth,
		largeGridFadeOutStart, largeGridFadeOutEnd)* sizeFade;
	float4 smallContribution = float4(smallCoverage * smallLineColor, smallCoverage);
	float4 mediumContribution = float4(mediumCoverage * mediumLineColor, mediumCoverage);
	float4 largeContribution = float4(largeCoverage * largeLineColor, largeCoverage);
	float4 contribution = mediumContribution + smallContribution * (1 - mediumContribution.w);
	return largeContribution + contribution * (1 - largeContribution.w);

}

float3 TransformByConjugate(float3 v, float4 rotation)
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
	float wx2 = rotation.w * -x2;
	float wy2 = rotation.w * -y2;
	float wz2 = rotation.w * -z2;
	return float3(
		v.x * (1.0 - yy2 - zz2) + v.y * (xy2 - wz2) + v.z * (xz2 + wy2),
		v.x * (xy2 + wz2) + v.y * (1.0 - xx2 - zz2) + v.z * (yz2 - wx2),
		v.x * (xz2 - wy2) + v.y * (yz2 + wx2) + v.z * (1.0 - xx2 - yy2));
}

void GetScreenspaceDerivatives(float3 surfacePosition, float3 surfaceNormal, float3 currentRayDirection, float2 pixelSizeAtUnitPlane, out float3 dpdx, out float3 dpdy)
{
	float2 unitZDirection = currentRayDirection.xy / currentRayDirection.z;
	float3 adjacentXDirection = float3(unitZDirection + float2(pixelSizeAtUnitPlane.x, 0), 1);
	float3 adjacentYDirection = float3(unitZDirection + float2(0, pixelSizeAtUnitPlane.y), 1);
	float velocityX = dot(surfaceNormal, adjacentXDirection);
	float velocityY = dot(surfaceNormal, adjacentYDirection);
	float distance = dot(surfaceNormal, surfacePosition);
	float tX = distance / velocityX;
	float tY = distance / velocityY;
	dpdx = adjacentXDirection * tX - surfacePosition;
	dpdy = adjacentYDirection * tY - surfacePosition;


}

float3 ShadeSurface(float3 surfacePosition, float3 surfaceNormal, float3 surfaceColor, float3 dpdx, float3 dpdy,
	float3 instancePosition, float4 instanceOrientation, float shapeSize, float zDistance)
{
	float3 shapeLocalPosition = TransformByConjugate(surfacePosition - instancePosition, instanceOrientation);
	float3 shapeLocalNormal = TransformByConjugate(surfaceNormal, instanceOrientation);
	float3 shapeLocalDpdx = TransformByConjugate(dpdx, instanceOrientation);
	float3 shapeLocalDpdy = TransformByConjugate(dpdy, instanceOrientation);
	float4 grid = GetLocalGridContributions(shapeLocalPosition, shapeLocalNormal, shapeLocalDpdx, shapeLocalDpdy, shapeSize, zDistance);
	float3 compositedColor = grid.xyz + surfaceColor * (1 - grid.w);
	return compositedColor * AccumulateLight(surfaceNormal);
}
