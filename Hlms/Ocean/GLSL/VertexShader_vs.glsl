@insertpiece( SetCrossPlatformSettings )
@insertpiece( SetCompatibilityLayer )

out gl_PerVertex
{
	vec4 gl_Position;
};

layout(std140) uniform;

//To render a 2x2 (quads) terrain:
//You'll normally need 6 vertices per line + 2 for degenerates.
//You'll need 8 vertices per line.
//So you'll need a total of 16 vertices.

//To render a 4x2 (quads) terrain:
//You'll need 10 vertices per line.
//If we include degenerate vertices, you'll need 12 per line
//So you'll need a total of 24 vertices.
//in int gl_VertexID;

@property( GL_ARB_base_instance )
	in uint drawId;
@end

@insertpiece( custom_vs_attributes )

out block
{
@insertpiece( Terra_VStoPS_block )
} outVs;

// START UNIFORM DECLARATION
@insertpiece( PassDecl )
@insertpiece( TerraInstanceDecl )
uniform sampler3D terrainData; // xy normal, z height, w foam
uniform sampler2D blendMap; //just a noise texture
@insertpiece( custom_vs_uniformDeclaration )
@property( !GL_ARB_base_instance )uniform uint baseInstance;@end
// END UNIFORM DECLARATION

@piece( VertexTransform )
	//Lighting is in view space
	outVs.pos		= ( vec4(worldPos.xyz, 1.0f) * passBuf.view ).xyz;
@property( !hlms_dual_paraboloid_mapping )
	gl_Position = vec4(worldPos.xyz, 1.0f) * passBuf.viewProj;@end
@property( hlms_dual_paraboloid_mapping )
	//Dual Paraboloid Mapping
	gl_Position.w	= 1.0f;
	gl_Position.xyz	= outVs.pos;
	float L = length( gl_Position.xyz );
	gl_Position.z	+= 1.0f;
	gl_Position.xy	/= gl_Position.z;
	gl_Position.z	= (L - NearPlane) / (FarPlane - NearPlane);@end
@end
@piece( ShadowReceive )
@foreach( hlms_num_shadow_map_lights, n )
	@property( !hlms_shadowmap@n_is_point_light )
		outVs.posL@n = vec4(worldPos.xyz, 1.0f) * passBuf.shadowRcv[@n].texViewProj;@end @end
@end

void main()
{
@property( !GL_ARB_base_instance )
    uint drawId = baseInstance + uint( gl_InstanceID );
@end

    @insertpiece( custom_vs_preExecution )

    CellData cellData = instance.cellData[drawId];

	//Map pointInLine from range [0; 12) to range [0; 9] so that it reads:
	// 0 0 1 2 3 4 5 6 7 8 9 9
	uint pointInLine = uint(gl_VertexID) % (cellData.numVertsPerLine.x); //cellData.numVertsPerLine.x = 12
	pointInLine = uint(clamp( int(pointInLine) - 1, 0, int(cellData.numVertsPerLine.x - 3u) ));

	uvec2 uVertexPos;

	uVertexPos.x = pointInLine >> 1u;
    //Even numbers are the next line, odd numbers are current line.
	uVertexPos.y = (pointInLine & 0x01u) == 0u ? 1u : 0u;
	uVertexPos.y += uint(gl_VertexID) / cellData.numVertsPerLine.x;
	//uVertexPos.y += floor( (float)gl_VertexID / (float)cellData.numVertsPerLine ); Could be faster on GCN.

@property( use_skirts )
	//Apply skirt.
	bool isSkirt =( pointInLine.x <= 1u ||
					pointInLine.x >= (cellData.numVertsPerLine.x - 4u) ||
					uVertexPos.y == 0u ||
					uVertexPos.y == (cellData.numVertsPerLine.z + 2u) );

	//Now shift X position for the left & right skirts
	uVertexPos.x = uint( max( int(uVertexPos.x) - 1, 0 ) );
	uVertexPos.x = min( uVertexPos.x, ((cellData.numVertsPerLine.x - 7u) >> 1u) );
	// uVertexPos.x becomes:
	// 0 0 0 1 1 2 2 3 3 4 4 4
	// 0 0 0 0 0 1 1 2 2 3 3 3
	// 0 0 0 0 0 1 1 2 2 2 2 2

	//Now shift Y position for the front & back skirts
	uVertexPos.y = uint( max( int(uVertexPos.y) - 1, 0 ) );
	uVertexPos.y = min( uVertexPos.y, cellData.numVertsPerLine.z );
@end

	uint lodLevel = cellData.numVertsPerLine.y;
	uVertexPos = uVertexPos << lodLevel;

	uVertexPos.xy = uvec2( clamp( ivec2(uVertexPos.xy) + cellData.xzTexPosBounds.xy,
                           ivec2( 0, 0 ), cellData.xzTexPosBounds.zw ) );

	vec3 worldPos;
	//calculate zx right away so we can use it to sample the textures
	worldPos.xz = uVertexPos.xy;
	worldPos.xz = worldPos.xz * cellData.scale.xz + cellData.pos.xz;

	outVs.wavesIntensity = cellData.scale.y;
	float timer = passBuf.timer.x; //need a timer!
	float scale = 1.0;
	float uvScale = uintBitsToFloat(cellData.numVertsPerLine.w);

	//generate some different uvs with arbitrary values for scale and rotation to decrease pattern/texture repetition
	float rotFactor = 0.48;
	outVs.uv1.xy = vec2( worldPos.xz ) * mat2(cos(rotFactor), sin(rotFactor), -sin(rotFactor), cos(rotFactor)) * vec2(0.24) * scale;
	outVs.uv1.xy *= uvScale;
	outVs.uv1.z = timer * 0.08;

	rotFactor = 0.17;
	outVs.uv2.xy = vec2( worldPos.xz ) * mat2(cos(rotFactor), sin(rotFactor), -sin(rotFactor), cos(rotFactor)) * vec2(0.08) * scale;
	outVs.uv2.xy *= uvScale;
	outVs.uv2.z = timer * 0.076;

	rotFactor = 0.09;
	outVs.uv3.xy = vec2( worldPos.xz ) * mat2(cos(rotFactor), sin(rotFactor), -sin(rotFactor), cos(rotFactor)) * vec2(0.17) * scale;
	outVs.uv3.xy *= uvScale;
	outVs.uv3.z = timer * 0.069;

	outVs.uv4.xy = vec2( worldPos.xz ) * vec2(0.3) * scale;
	outVs.uv4.xy *= uvScale;
	outVs.uv4.z = timer * 0.084;

	//sample a noise texture that is going to be used to blend the 4 samples from the different uvs
	outVs.blendWeight = texture( blendMap, vec2(outVs.uv1.xy*0.1) ).xyz;

	float textureValue = texture( terrainData, outVs.uv1 ).z;
	float textureValue2 = texture( terrainData, outVs.uv2 ).z;
	float textureValue3 = texture( terrainData, outVs.uv3 ).z;
	float textureValue4 = texture( terrainData, outVs.uv4 ).z;

	textureValue = mix( textureValue, textureValue2, outVs.blendWeight.x );
	textureValue = mix( textureValue, textureValue3, outVs.blendWeight.y );
	textureValue = mix( textureValue, textureValue4, outVs.blendWeight.z );


	worldPos.y  = textureValue * 3.0 - 1.7;
	worldPos.y *= uvScale;

@property( use_skirts )
	worldPos.y = isSkirt ? 0.001 : worldPos.y;
@end

	worldPos.y = worldPos.y * cellData.scale.y + cellData.pos.y;

	outVs.wpos = worldPos.xyz;

  outVs.waveHeight = pow( textureValue, 6 )*4;

	@insertpiece( VertexTransform )

	outVs.uv0.xy = vec2( uVertexPos.xy ) * vec2( cellData.pos.w, cellData.scale.w );

	@insertpiece( ShadowReceive )
@foreach( hlms_num_shadow_map_lights, n )
	@property( !hlms_shadowmap@n_is_point_light )
		outVs.posL@n.z = outVs.posL@n.z * passBuf.shadowRcv[@n].shadowDepthRange.y;
		outVs.posL@n.z = (outVs.posL@n.z * 0.5) + 0.5;
	@end
@end

@property( hlms_pssm_splits )	outVs.depth = gl_Position.z;@end

	//outVs.drawId = drawId;

	@insertpiece( custom_vs_posExecution )
}
