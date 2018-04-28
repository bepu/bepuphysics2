/*[META]
vs
ps
[META]*/
cbuffer VertexConstants : register(b0)
{
	float4x4 ViewProjection;
	float2 NDCToScreenScale; //0 to 2 => 0 to resolution
	float2 Padding0;
	float3 CameraForward;
	float TanAnglePerPixel;
	float3 CameraRight;
	float Padding1;
	float3 CameraPosition;
};

#include "ColorUnpacking.hlsl"
struct LineInstance
{
	float3 Start;
	uint PackedBackgroundColor;
	float3 End;
	uint PackedColor;
};

StructuredBuffer<LineInstance> Instances : register(t0);

struct PSInput
{
	float4 Position : SV_Position;
	float3 ToBox : ToBoxOffset;
	nointerpolation float3 Start : LineStart;
	nointerpolation float3 Direction : LineDirection;
	nointerpolation float Length : LineLength;
	nointerpolation uint PackedColor : ScreenLineColor0;
	nointerpolation uint PackedBackgroundColor : ScreenLineColor1;
	nointerpolation float InverseLineRadius : InverseLineRadius;
	nointerpolation float TanAnglePerPixel : TanAnglePerPixel;
};
#define TargetRadiusInPixels 1.0
PSInput VSMain(uint vertexId : SV_VertexId)
{
	int instanceId = vertexId >> 3;
	LineInstance instance = Instances[instanceId];
	PSInput output;

	//Project the start and end points into NDC.
	//Output the line start, direction, and length in screenspace for use by the pixel shader. This is similar to the UI line renderer.
	float3 worldLine = instance.End - instance.Start;
	float worldLineLength = length(worldLine);
	worldLine = worldLineLength > 1e-7 ? worldLine / worldLineLength : float3(1, 0, 0);
	output.Start = instance.Start - CameraPosition;
	output.Direction = worldLine;
	output.Length = worldLineLength;
	output.PackedColor = instance.PackedColor;
	output.PackedBackgroundColor = instance.PackedBackgroundColor;
	output.TanAnglePerPixel = TanAnglePerPixel;
	//How wide is a pixel at the line, approximately?
	//Compute the closest point on the line to the camera.
	float t = clamp(-dot(worldLine, output.Start), 0, worldLineLength);
	float3 closestOnLine = output.Start + worldLine * t;
	float distanceFromCamera = length(closestOnLine);
	//Convert the vertex id to local box coordinates.
	//Note that this id->coordinate transformation requires consistency with the index buffer
	//to ensure proper triangle winding. A set bit in a given position makes it higher along the axis.
	//So vertexId&1 == 1 => +x, vertexId&2 == 2 => +y, and vertexId&4 == 4 => +z.
	float3 boxCoordinates = float3(vertexId & 1, (vertexId & 2) >> 1, (vertexId & 4) >> 2);
	float pixelSize = distanceFromCamera * TanAnglePerPixel;
	float lineRadius = TargetRadiusInPixels * pixelSize;
	output.InverseLineRadius = 1.0 / lineRadius;

	float3 worldLineX = cross(CameraForward, worldLine);
	float worldLineXLength = length(worldLineX);
	if (worldLineXLength < 1e-7)
	{
		worldLineX = cross(CameraRight, worldLine);
		worldLineXLength = length(worldLineX);
	}
	worldLineX /= worldLineXLength;
	float3 worldLineY = cross(worldLine, worldLineX);
	//Use the line radius to pad out the box.
	float3 paddingX = lineRadius * worldLineX;
	float3 paddingY = lineRadius * worldLineY;
	float3 paddingZ = lineRadius * worldLine;
	float3 position = boxCoordinates.z > 0 ?
		instance.End + paddingZ :
		instance.Start - paddingZ;
	position += (boxCoordinates.x * 2 - 1) * paddingX;
	position += (boxCoordinates.y * 2 - 1) * paddingY;
	output.ToBox = position - CameraPosition;
	output.Position = mul(float4(position, 1), ViewProjection);
	return output;
	//A couple of notes:
	//1) More accurate depths could be calculated for the vertices rather than simply assuming that they have the same depth as the endpoint.
	//That assumption overestimates the depth of the vertices associated with the near point and underestimates the depth of the far point.
	//Only bother if actually matters.
	//2) The pixel shader could interpolate linear depth and offset it with conservative depth output.
	//That would allow smoother transitions between lines during overlap.
}

struct PSOutput
{
	float3 Color : SV_Target0;
};


float EvaluateIntegral(float x)
{
	//        { 0       (-inf, -1) }
	// f(x) = { 1 + x   [-1, 0]	   }
	//        { 1 - x   (0, 1]	   }
	//        { 0       (1, inf)   }
	//						 { -0.5				 (-inf, -1)	}
	// integrate(f(x), dx) = { x + x^2/2	     [-1, 0]	}
	//					     { x - x^2/2		 (0, 1]		}
	//					     { 0.5				 (1, inf)	}
	float negativeX = clamp(x, -1, 0);
	float negativeInterval = negativeX + 0.5 * negativeX * negativeX;
	float positiveX = clamp(x, 0, 1);
	//Note that we're stacking the positive contribution on top of the negative contribution, so the 0.5 constant additive term is excluded.
	float positiveInterval = positiveX - 0.5 * positiveX * positiveX;
	return negativeInterval + positiveInterval;
}

PSOutput PSMain(PSInput input)
{
	PSOutput output;
	float3 rayDirection = input.ToBox;
	float rayLength = length(rayDirection);
	rayDirection /= rayLength;
	//Compute the closest points between the two line segments. No clamping to begin with.
	//We want to minimize distance = ||(a + da * ta) - (b + db * tb)||.
	//Taking the derivative with respect to ta and doing some algebra (taking into account ||da|| == ||db|| == 1) to solve for ta yields:
	//ta = (da * (b - a) + (db * (a - b)) * (da * db)) / (1 - ((da * db) * (da * db))        
	float daOffsetB = dot(input.Direction, -input.Start);
	float dbOffsetB = dot(rayDirection, -input.Start);
	float dadb = dot(input.Direction, rayDirection);
	//Note potential division by zero when the axes are parallel. Arbitrarily clamp; near zero values will instead produce extreme values which get clamped to reasonable results.
	float tLine = clamp((daOffsetB - dbOffsetB * dadb) / max(1e-15, 1.0 - dadb * dadb), 0, input.Length);
		
	float3 closestOnLine = input.Start + tLine * input.Direction;
	//Note rayDirection is unit length.
	float3 closestOnRay = rayDirection * dot(closestOnLine, rayDirection);
	float lineDistance = distance(closestOnRay, closestOnLine);
	float normalizedDistance = lineDistance * input.InverseLineRadius;
	float normalizedPixelSize = input.TanAnglePerPixel * rayLength * input.InverseLineRadius;

	//Integrate over the approximate span of the pixel along a 1d function representing the foreground intensity:
	float halfWidth = 0.5 * normalizedPixelSize;
	//Note that the integral is not allowed to reach outside of the radius; this avoids overdarkening. Coverage is handled by MSAA.
	float a = max(-1, normalizedDistance - halfWidth);
	float b = min(1, normalizedDistance + halfWidth);
	float colorIntensity = saturate((EvaluateIntegral(b) - EvaluateIntegral(a)) / (b - a));
	output.Color = lerp(UnpackR11G11B10_UNorm(input.PackedBackgroundColor), UnpackR11G11B10_UNorm(input.PackedColor), colorIntensity);
	return output;
}