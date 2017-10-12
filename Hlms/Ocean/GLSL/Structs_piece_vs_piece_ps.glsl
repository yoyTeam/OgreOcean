
@piece( TerraMaterialDecl )
layout_constbuffer(binding = 1) uniform MaterialBuf
{
	/* kD is already divided by PI to make it energy conserving.
	  (formula is finalDiffuse = NdotL * surfaceDiffuse / PI)
	*/
	vec4 deepColour; //deepColour.w is wave intensity
	vec4 shallowColour; //shallowColour.w is wave scale

} material;
@end

@piece( TerraInstanceDecl )
struct CellData
{
	//.x = numVertsPerLine
	//.y = lodLevel
	//.z = vao->getPrimitiveCount() / m_verticesPerLine - 2u
	//.w = uvScale (float)
	uvec4 numVertsPerLine;
	ivec4 xzTexPosBounds;		//XZXZ
	vec4 pos;		//.w contains 1.0 / xzTexPosBounds.z
	vec4 scale;		//.w contains 1.0 / xzTexPosBounds.w
};

layout_constbuffer(binding = 2) uniform InstanceBuffer
{
	CellData cellData[256];
} instance;
@end

@piece( Terra_VStoPS_block )
	//flat uint drawId;
	vec3 pos;
	float waveHeight;
	float wavesIntensity;
	vec2 uv0;
	vec3 uv1;
	vec3 uv2;
	vec3 uv3;
	vec3 uv4;
	vec3 blendWeight;

	@foreach( hlms_num_shadow_map_lights, n )
		@property( !hlms_shadowmap@n_is_point_light )
			vec4 posL@n;@end @end
	@property( hlms_pssm_splits )float depth;@end
	@insertpiece( custom_VStoPS )
@end
