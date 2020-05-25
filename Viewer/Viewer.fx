//--------------------------------------------------------------------------------------
// File: Tutorial06.fx
//
// Copyright (c) Microsoft Corporation. All rights reserved.
//--------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------
// Constant Buffer Variables
//--------------------------------------------------------------------------------------
cbuffer ConstantBuffer : register( b0 )
{
	matrix World;
	matrix View;
	matrix Projection;
	float4 worldOffset;
	float4 vLightDir[2];
	float4 vLightColor[3];
	float4 vOutputColor[2];
}


//--------------------------------------------------------------------------------------
struct VS_INPUT
{
    float4 Pos : POSITION;
    float3 Norm : NORMAL;
};

struct PS_INPUT
{
    float4 Pos : SV_POSITION;
    float3 Norm : TEXCOORD0;
	float4 C : COLOR0;
};

//--------------------------------------------------------------------------------------
// Vertex Shader
//--------------------------------------------------------------------------------------
PS_INPUT VS( VS_INPUT input )
{
    PS_INPUT output = (PS_INPUT)0;
    output.Pos = mul( input.Pos, World );
    output.Pos = mul( output.Pos, View );
    output.Pos = mul( output.Pos, Projection );
    output.Norm = mul( float4( input.Norm, 1 ), World ).xyz;

	// Vertices origin of the word is the center of the
	// block. Calculate the real word coordinates of this
	// vertex so that we can apply the marking pattern
	float4 phyPos = input.Pos + worldOffset;

	// Apply a checker pattern of 1 inch size squares of alternating shades
	bool xToggle = (phyPos.x > 0) ? (((int)phyPos.x & 1) != 0) : (((int)phyPos.x & 1) == 0);
	bool zToggle = (phyPos.z > 0) ? (((int)phyPos.z & 1) != 0) : (((int)phyPos.z & 1) == 0);

	// Origin marker is a circle with inverted checker pattern
	// 1/16: 0.0009765625
	// 1/4 : 0.015635
	bool bCenter = (abs(phyPos.x * phyPos.x + phyPos.z * phyPos.z) < 0.015635);

	output.C = vLightColor[bCenter ? !(xToggle ^ zToggle) : xToggle ^ zToggle ];

    return output;
}


//--------------------------------------------------------------------------------------
// Pixel Shader
//--------------------------------------------------------------------------------------
float4 PS( PS_INPUT input) : SV_Target
{
    float4 finalColor = 0;    
	finalColor += saturate(dot((float3)vLightDir[0], input.Norm) * input.C);
    finalColor.a = 1;
    return finalColor;
}


//--------------------------------------------------------------------------------------
// PSSolid - render a solid color
//--------------------------------------------------------------------------------------
float4 PSSolid( PS_INPUT input) : SV_Target
{
	return vOutputColor[0];
}
