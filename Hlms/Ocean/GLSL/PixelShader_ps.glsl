@property( false )
@insertpiece( SetCrossPlatformSettings )
@insertpiece( SetCompatibilityLayer )

layout(std140) uniform;
#define FRAG_COLOR		0
layout(location = FRAG_COLOR, index = 0) out vec4 outColour;

in block
{
@insertpiece( Terra_VStoPS_block )
} inPs;

void main()
{
	outColour = vec4( inPs.uv0.xy, 0.0, 1.0 );
}

@end
@property( !false )
@insertpiece( SetCrossPlatformSettings )
@property( !GL430 )
@property( hlms_tex_gather )#extension GL_ARB_texture_gather: require@end
@end
@property( hlms_amd_trinary_minmax )#extension GL_AMD_shader_trinary_minmax: require@end
@insertpiece( SetCompatibilityLayer )

layout(std140) uniform;
#define FRAG_COLOR		0
layout(location = FRAG_COLOR, index = 0) out vec4 outColour;

@property( hlms_vpos )
in vec4 gl_FragCoord;
@end
// START UNIFORM DECLARATION
@insertpiece( PassDecl )
@insertpiece( TerraMaterialDecl )
@insertpiece( TerraInstanceDecl )
@insertpiece( custom_ps_uniformDeclaration )
// END UNIFORM DECLARATION
in block
{
@insertpiece( Terra_VStoPS_block )
} inPs;

uniform sampler3D terrainData; // xy normal, z height, w foam
uniform sampler2D terrainShadows;

@property( hlms_forwardplus )
/*layout(binding = 1) */uniform usamplerBuffer f3dGrid;
/*layout(binding = 2) */uniform samplerBuffer f3dLightList;
@end
@property( envprobe_map )uniform samplerCube	texEnvProbeMap;@end

vec4 diffuseCol;
@insertpiece( FresnelType ) F0;
float ROUGHNESS;

vec3 nNormal;

@property( hlms_lights_spot_textured )@insertpiece( DeclQuat_zAxis )
vec3 qmul( vec4 q, vec3 v )
{
	return v + 2.0 * cross( cross( v, q.xyz ) + q.w * v, q.xyz );
}
@end

@insertpiece( DeclareBRDF )

@insertpiece( DeclShadowMapMacros )
@insertpiece( DeclShadowSamplers )
@insertpiece( DeclShadowSamplingFuncs )

void main()
{
	@insertpiece( custom_ps_preExecution )

	@insertpiece( custom_ps_posMaterialLoad )


	float timer = passBuf.timer.x; //need a timer!

	//sample the data texture using diferent uvs to randomize patterns a little
	vec4 textureValue = texture( terrainData, inPs.uv1 );
	vec4 textureValue2 = texture( terrainData, inPs.uv2 );
	vec4 textureValue3 = texture( terrainData, inPs.uv3 );
	vec4 textureValue4 = texture( terrainData, inPs.uv4 );
	//blend them out
	textureValue = mix( textureValue, textureValue2, vec4(inPs.blendWeight.x) );
	textureValue = mix( textureValue, textureValue3, vec4(inPs.blendWeight.y) );
	textureValue = mix( textureValue, textureValue4, vec4(inPs.blendWeight.z) );

	float waveIntensity = inPs.wavesIntensity;
	//just a visually tweaked formula for foam factor
	float foam = pow( textureValue.w, 3-waveIntensity ) * 0.5 * waveIntensity * waveIntensity;

	//fixed roughness
	ROUGHNESS = 0.02 + foam;

	//get the diffuse colour, higher parts uses shallowColour, lower parts uses deepColour, this fakes scattering a little
	diffuseCol.xyz = mix( material.deepColour.xyz, material.shallowColour.xyz, vec3( inPs.waveHeight * waveIntensity ) );
	diffuseCol.w = 1.0;
	diffuseCol.xyz += foam;

	//non metal
	F0 = vec3( 0.03f );

	//get the normal from the texture
	nNormal.xy = textureValue.xy * 2.0 - 1.0;
  nNormal.xy *= waveIntensity;
	nNormal.z = 1.0;

@insertpiece( custom_ps_posSampleNormal );

		//always a plane so geometric normal is unitY, real normals are baked in the texture
		vec3 geomNormal = vec3(0,1,0);
		geomNormal = geomNormal * mat3(passBuf.view);

		//Get the TBN matrix
		vec3 viewSpaceUnitX	= vec3( passBuf.view[0].x, passBuf.view[1].x, passBuf.view[2].x );
		vec3 vTangent		= normalize( cross( geomNormal, viewSpaceUnitX ) );
		vec3 vBinormal		= cross( vTangent, geomNormal );
		mat3 TBN			= mat3( vBinormal, vTangent, geomNormal );

		nNormal = normalize( TBN * nNormal );

@property( terra_enabled )
//TODO Terra shaodws not working yet
	float fTerrainShadow = texture( terrainShadows, inPs.uv0.xy ).x;
@end
	@property( !(hlms_pssm_splits || (!hlms_pssm_splits && hlms_num_shadow_map_lights && hlms_lights_directional)) )
		float fShadow = 1.0f;
	@end
	@insertpiece( DoDirectionalShadowMaps )
	@property( terra_enabled )
		fShadow *= fTerrainShadow;
	@end


	//Everything's in Camera space
@property( hlms_lights_spot || ambient_hemisphere || envprobe_map || hlms_forwardplus )
	vec3 viewDir	= normalize( -inPs.pos );
	float NdotV		= clamp( dot( nNormal, viewDir ), 0.0, 1.0 );@end

@property( !ambient_fixed )
	vec3 finalColour = vec3(0);
@end @property( ambient_fixed )
	vec3 finalColour = passBuf.ambientUpperHemi.xyz * @insertpiece( kD ).xyz;
@end

	@insertpiece( custom_ps_preLights )

@property( !custom_disable_directional_lights )
@property( hlms_lights_directional )
	finalColour += BRDF( passBuf.lights[0].position.xyz, viewDir, NdotV, passBuf.lights[0].diffuse, passBuf.lights[0].specular )@insertpiece(DarkenWithShadowFirstLight);
@end
@foreach( hlms_lights_directional, n, 1 )
	finalColour += BRDF( passBuf.lights[@n].position.xyz, viewDir, NdotV, passBuf.lights[@n].diffuse, passBuf.lights[@n].specular )@insertpiece( DarkenWithShadow );@end
@foreach( hlms_lights_directional_non_caster, n, hlms_lights_directional )
	finalColour += BRDF( passBuf.lights[@n].position.xyz, viewDir, NdotV, passBuf.lights[@n].diffuse, passBuf.lights[@n].specular );@end
@end

@property( hlms_lights_point || hlms_lights_spot )	vec3 lightDir;
	float fDistance;
	vec3 tmpColour;
	float spotCosAngle;@end

	//Point lights
@foreach( hlms_lights_point, n, hlms_lights_directional_non_caster )
	lightDir = passBuf.lights[@n].position - inPs.pos;
	fDistance= length( lightDir );
	if( fDistance <= passBuf.lights[@n].attenuation.x )
	{
		lightDir *= 1.0 / fDistance;
		tmpColour = BRDF( lightDir, viewDir, NdotV, passBuf.lights[@n].diffuse, passBuf.lights[@n].specular )@insertpiece( DarkenWithShadowPoint );
		float atten = 1.0 / (0.5 + (passBuf.lights[@n].attenuation.y + passBuf.lights[@n].attenuation.z * fDistance) * fDistance );
		finalColour += tmpColour * atten;
	}@end

	//Spot lights
	//spotParams[@value(spot_params)].x = 1.0 / cos( InnerAngle ) - cos( OuterAngle )
	//spotParams[@value(spot_params)].y = cos( OuterAngle / 2 )
	//spotParams[@value(spot_params)].z = falloff
@foreach( hlms_lights_spot, n, hlms_lights_point )
	lightDir = passBuf.lights[@n].position - inPs.pos;
	fDistance= length( lightDir );
@property( !hlms_lights_spot_textured )	spotCosAngle = dot( normalize( inPs.pos - passBuf.lights[@n].position ), passBuf.lights[@n].spotDirection );@end
@property( hlms_lights_spot_textured )	spotCosAngle = dot( normalize( inPs.pos - passBuf.lights[@n].position ), zAxis( passBuf.lights[@n].spotQuaternion ) );@end
	if( fDistance <= passBuf.lights[@n].attenuation.x && spotCosAngle >= passBuf.lights[@n].spotParams.y )
	{
		lightDir *= 1.0 / fDistance;
	@property( hlms_lights_spot_textured )
		vec3 posInLightSpace = qmul( spotQuaternion[@value(spot_params)], inPs.pos );
		float spotAtten = texture( texSpotLight, normalize( posInLightSpace ).xy ).x;
	@end
	@property( !hlms_lights_spot_textured )
		float spotAtten = clamp( (spotCosAngle - passBuf.lights[@n].spotParams.y) * passBuf.lights[@n].spotParams.x, 0.0, 1.0 );
		spotAtten = pow( spotAtten, passBuf.lights[@n].spotParams.z );
	@end
		tmpColour = BRDF( lightDir, viewDir, NdotV, passBuf.lights[@n].diffuse, passBuf.lights[@n].specular )@insertpiece( DarkenWithShadow );
		float atten = 1.0 / (0.5 + (passBuf.lights[@n].attenuation.y + passBuf.lights[@n].attenuation.z * fDistance) * fDistance );
		finalColour += tmpColour * (atten * spotAtten);
	}@end

@insertpiece( forward3dLighting )

@property( envprobe_map || ambient_hemisphere )
	vec3 reflDir = 2.0 * dot( viewDir, nNormal ) * nNormal - viewDir;

	@property( envprobe_map )
		vec3 envColourS = passBuf.ambientUpperHemi.xyz * textureLod( texEnvProbeMap, reflDir * passBuf.invViewMatCubemap, ROUGHNESS * 12.0 ).xyz @insertpiece( ApplyEnvMapScale );// * 0.0152587890625;
		vec3 envColourD = passBuf.ambientUpperHemi.xyz * textureLod( texEnvProbeMap, nNormal * passBuf.invViewMatCubemap, 11.0 ).xyz @insertpiece( ApplyEnvMapScale );// * 0.0152587890625;
		@property( !hw_gamma_read )	//Gamma to linear space
			envColourS = envColourS * envColourS;
			envColourD = envColourD * envColourD;
		@end
	@end
	@property( ambient_hemisphere )
		float ambientWD = dot( passBuf.ambientHemisphereDir.xyz, nNormal ) * 0.5 + 0.5;
		float ambientWS = dot( passBuf.ambientHemisphereDir.xyz, reflDir ) * 0.5 + 0.5;

		@property( envprobe_map )
			envColourS	+= mix( passBuf.ambientLowerHemi.xyz, passBuf.ambientUpperHemi.xyz, ambientWD );
			envColourD	+= mix( passBuf.ambientLowerHemi.xyz, passBuf.ambientUpperHemi.xyz, ambientWS );
		@end @property( !envprobe_map )
			vec3 envColourS = mix( passBuf.ambientLowerHemi.xyz, passBuf.ambientUpperHemi.xyz, ambientWD );
			vec3 envColourD = mix( passBuf.ambientLowerHemi.xyz, passBuf.ambientUpperHemi.xyz, ambientWS );
		@end
	@end

	@insertpiece( BRDF_EnvMap )
@end
@property( !hw_gamma_write )
	//Linear to Gamma space
	outColour.xyz	= sqrt( finalColour );
@end @property( hw_gamma_write )
	outColour.xyz	= finalColour;
@end

@property( hlms_alphablend )
	@property( use_texture_alpha )
		outColour.w		= material.F0.w * diffuseCol.w;
	@end @property( !use_texture_alpha )
		outColour.w		= material.F0.w;
	@end
@end @property( !hlms_alphablend )
	outColour.w		= 1.0;@end

	@property( debug_pssm_splits )
		outColour.xyz = mix( outColour.xyz, debugPssmSplit.xyz, 0.2f );
	@end

	@insertpiece( custom_ps_posExecution )
}
@end
