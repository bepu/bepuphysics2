float2 GetWholeScreenTriangleVertexNDC(uint vertexId)
{
	return float2((vertexId << 2) & 4, (vertexId << 1) & 4) - 1;
} 