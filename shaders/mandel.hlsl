
// Vertex Shader
struct VS_INPUT
{
	float3 vPosition : POSITION;
	//float3 vColor: COLOR0;
};

struct PS_INPUT
{
	float4 vPosition : SV_POSITION;
	float height : FOG;
};

cbuffer SceneConstantBuffer : register(b0)
{
	float4x4 g_MVPMatrix;
	float4x4 g_TransposedMVPMatrix;
	float g_HeightFactor;
};

cbuffer ColorLookup : register(b2)
{
	float3 g_ColorLookup[1024];
};

PS_INPUT VSMain(VS_INPUT i)
{
	PS_INPUT o;
		
	o.vPosition = mul(g_MVPMatrix, float4(i.vPosition.x, g_HeightFactor * 0.5 * (i.vPosition.y - 0.5), i.vPosition.z, 1.0));
#ifdef VULKAN
	o.vPosition.y = -o.vPosition.y;
#endif

	o.height = 0.5 * i.vPosition.y;
	//o.height = g_HeightFactor * 0.5 * i.vPosition.y;
	//o.height = i.vPosition.y;
	return o;
}

float4 PSMain(PS_INPUT i) : SV_TARGET
{
    int c_index = (int)(1023.0 * i.height);
	float3 cl = g_ColorLookup[c_index];

	return float4(cl.x, cl.y, cl.z, 1.0);
	//return float4(i.vColor.x, i.vColor.y, i.vColor.z, 1.0);
}
