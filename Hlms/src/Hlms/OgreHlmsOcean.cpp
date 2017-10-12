/*
-----------------------------------------------------------------------------
This source file is part of OGRE
    (Object-oriented Graphics Rendering Engine)
For the latest info, see http://www.ogre3d.org/

Copyright (c) 2000-2014 Torus Knot Software Ltd

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-----------------------------------------------------------------------------
*/

#include "Ocean/Hlms/OgreHlmsOcean.h"
#include "Ocean/Hlms/OgreHlmsOceanDatablock.h"
#include "OgreHlmsManager.h"
#include "OgreHlmsListener.h"
#include "OgreLwString.h"

#include "OgreViewport.h"
#include "OgreRenderTarget.h"
#include "OgreHighLevelGpuProgramManager.h"
#include "OgreHighLevelGpuProgram.h"
#include "OgreForward3D.h"
#include "OgreCamera.h"
#include <OgreRoot.h>
#include <OgreTextureManager.h>

#include "OgreSceneManager.h"
#include "Compositor/OgreCompositorShadowNode.h"
#include "Vao/OgreVaoManager.h"
#include "Vao/OgreConstBufferPacked.h"

#include "CommandBuffer/OgreCommandBuffer.h"
#include "CommandBuffer/OgreCbTexture.h"
#include "CommandBuffer/OgreCbShaderBuffer.h"

#include "Ocean/Ocean.h"
#include "OgreTimer.h"

#include <iostream>

namespace Ogre
{
    const IdString OceanProperty::HwGammaRead       = IdString( "hw_gamma_read" );
    const IdString OceanProperty::HwGammaWrite      = IdString( "hw_gamma_write" );
    const IdString OceanProperty::SignedIntTex      = IdString( "signed_int_textures" );
    const IdString OceanProperty::MaterialsPerBuffer= IdString( "materials_per_buffer" );
    const IdString OceanProperty::DebugPssmSplits   = IdString( "debug_pssm_splits" );

    const IdString OceanProperty::UseSkirts         = IdString( "use_skirts" );

    const char *OceanProperty::EnvProbeMap          = "envprobe_map";

    const IdString OceanProperty::FresnelScalar     = IdString( "fresnel_scalar" );
    const IdString OceanProperty::MetallicWorkflow  = IdString( "metallic_workflow" );
    const IdString OceanProperty::ReceiveShadows    = IdString( "receive_shadows" );

    const IdString OceanProperty::Pcf3x3            = IdString( "pcf_3x3" );
    const IdString OceanProperty::Pcf4x4            = IdString( "pcf_4x4" );
    const IdString OceanProperty::PcfIterations     = IdString( "pcf_iterations" );

    const IdString OceanProperty::EnvMapScale       = IdString( "envmap_scale" );
    const IdString OceanProperty::AmbientFixed      = IdString( "ambient_fixed" );
    const IdString OceanProperty::AmbientHemisphere = IdString( "ambient_hemisphere" );

    const IdString OceanProperty::BrdfDefault       = IdString( "BRDF_Default" );
    const IdString OceanProperty::BrdfCookTorrance  = IdString( "BRDF_CookTorrance" );
    const IdString OceanProperty::BrdfBlinnPhong    = IdString( "BRDF_BlinnPhong" );
    const IdString OceanProperty::FresnelSeparateDiffuse  = IdString( "fresnel_separate_diffuse" );
    const IdString OceanProperty::GgxHeightCorrelated     = IdString( "GGX_height_correlated" );


    HlmsOcean::HlmsOcean( Archive *dataFolder, ArchiveVec *libraryFolders ) :
        HlmsBufferManager( HLMS_USER2, "Ocean", dataFolder, libraryFolders ),
        ConstBufferPool( HlmsOceanDatablock::MaterialSizeInGpuAligned,
                         ConstBufferPool::ExtraBufferParams() ),
        mShadowmapSamplerblock( 0 ),
        mShadowmapCmpSamplerblock( 0 ),
        mCurrentShadowmapSamplerblock( 0 ),
        mOceanSamplerblock( 0 ),
        mCurrentPassBuffer( 0 ),
        mLastBoundPool( 0 ),
        mLastTextureHash( 0 ),
        mLastMovableObject( 0 ),
        mDebugPssmSplits( false ),
        mShadowFilter( PCF_3x3 ),
        mAmbientLightMode( AmbientAuto )
    {
        //Override defaults
        mLightGatheringMode = LightGatherForwardPlus;
        mProbe.setNull();
    }
    //-----------------------------------------------------------------------------------
    HlmsOcean::~HlmsOcean()
    {
        destroyAllBuffers();
    }
    //-----------------------------------------------------------------------------------
    void HlmsOcean::_changeRenderSystem( RenderSystem *newRs )
    {
        ConstBufferPool::_changeRenderSystem( newRs );

        //Force the pool to contain only 1 entry.
        mSlotsPerPool   = 1u;
        mBufferSize     = HlmsOceanDatablock::MaterialSizeInGpuAligned;

        HlmsBufferManager::_changeRenderSystem( newRs );

        if( newRs )
        {
            HlmsDatablockMap::const_iterator itor = mDatablocks.begin();
            HlmsDatablockMap::const_iterator end  = mDatablocks.end();

            while( itor != end )
            {
                assert( dynamic_cast<HlmsOceanDatablock*>( itor->second.datablock ) );
                HlmsOceanDatablock *datablock = static_cast<HlmsOceanDatablock*>( itor->second.datablock );

                requestSlot( datablock->mTextureHash, datablock, false );
                ++itor;
            }

            HlmsSamplerblock samplerblock;
            samplerblock.mU             = TAM_BORDER;
            samplerblock.mV             = TAM_BORDER;
            samplerblock.mW             = TAM_CLAMP;
            samplerblock.mBorderColour  = ColourValue( std::numeric_limits<float>::max(),
                                                       std::numeric_limits<float>::max(),
                                                       std::numeric_limits<float>::max(),
                                                       std::numeric_limits<float>::max() );

            if( mShaderProfile != "hlsl" )
            {
                samplerblock.mMinFilter = FO_POINT;
                samplerblock.mMagFilter = FO_POINT;
                samplerblock.mMipFilter = FO_NONE;

                if( !mShadowmapSamplerblock )
                    mShadowmapSamplerblock = mHlmsManager->getSamplerblock( samplerblock );
            }

            samplerblock.mMinFilter     = FO_LINEAR;
            samplerblock.mMagFilter     = FO_LINEAR;
            samplerblock.mMipFilter     = FO_NONE;
            samplerblock.mCompareFunction   = CMPF_LESS_EQUAL;

            if( !mShadowmapCmpSamplerblock )
                mShadowmapCmpSamplerblock = mHlmsManager->getSamplerblock( samplerblock );

            if( !mOceanSamplerblock )
                mOceanSamplerblock = mHlmsManager->getSamplerblock( HlmsSamplerblock() );
        }
    }
    //-----------------------------------------------------------------------------------
    const HlmsCache* HlmsOcean::createShaderCacheEntry( uint32 renderableHash,
                                                            const HlmsCache &passCache,
                                                            uint32 finalHash,
                                                            const QueuedRenderable &queuedRenderable )
    {
        const HlmsCache *retVal = Hlms::createShaderCacheEntry( renderableHash, passCache, finalHash,
                                                                queuedRenderable );

        if( mShaderProfile == "hlsl" )
        {
            mListener->shaderCacheEntryCreated( mShaderProfile, retVal, passCache,
                                                mSetProperties, queuedRenderable );
            return retVal; //D3D embeds the texture slots in the shader.
        }

        //Set samplers.
        if( !retVal->pso.pixelShader.isNull() )
        {
            GpuProgramParametersSharedPtr psParams = retVal->pso.pixelShader->getDefaultParameters();

            int texUnit = 3; //Vertex shader consumes 1 slot with its heightmap,
                             //PS consumes 2 slot with its normalmap & shadow texture.

            psParams->setNamedConstant( "terrainData", 0 );

            //Forward3D consumes 2 more slots.
            if( mGridBuffer )
            {
                psParams->setNamedConstant( "f3dGrid",      3 );
                psParams->setNamedConstant( "f3dLightList", 4 );
                texUnit += 2;
            }

            if( !mPreparedPass.shadowMaps.empty() )
            {
                char tmpData[32];
                LwString texName = LwString::FromEmptyPointer( tmpData, sizeof(tmpData) );
                texName = "texShadowMap";
                const size_t baseTexSize = texName.size();

                vector<int>::type shadowMaps;
                shadowMaps.reserve( mPreparedPass.shadowMaps.size() );
                for( size_t i=0; i<mPreparedPass.shadowMaps.size(); ++i )
                {
                    texName.resize( baseTexSize );
                    texName.a( (uint32)i );   //texShadowMap0
                    psParams->setNamedConstant( texName.c_str(), &texUnit, 1, 1 );
                    shadowMaps.push_back( texUnit++ );
                }
            }

            if( getProperty( OceanProperty::EnvProbeMap ) )
            {
                psParams->setNamedConstant( "texEnvProbeMap", texUnit++ );
            }

        }

        GpuProgramParametersSharedPtr vsParams = retVal->pso.vertexShader->getDefaultParameters();
        vsParams->setNamedConstant( "terrainData", 0 );
        vsParams->setNamedConstant( "blendMap", 1 );

        mListener->shaderCacheEntryCreated( mShaderProfile, retVal, passCache,
                                            mSetProperties, queuedRenderable );

        mRenderSystem->_setPipelineStateObject( &retVal->pso );

        mRenderSystem->bindGpuProgramParameters( GPT_VERTEX_PROGRAM, vsParams, GPV_ALL );
        if( !retVal->pso.pixelShader.isNull() )
        {
            GpuProgramParametersSharedPtr psParams = retVal->pso.pixelShader->getDefaultParameters();
            mRenderSystem->bindGpuProgramParameters( GPT_FRAGMENT_PROGRAM, psParams, GPV_ALL );
        }

        return retVal;
    }
    //-----------------------------------------------------------------------------------
    void HlmsOcean::calculateHashForPreCreate( Renderable *renderable, PiecesMap *inOutPieces )
    {
        assert( dynamic_cast<OceanCell*>(renderable) &&
                "This Hlms can only be used on a Ocean object!" );

        Ogre::OceanCell *OceanCell = static_cast<Ogre::OceanCell*>(renderable);
        setProperty( OceanProperty::UseSkirts, OceanCell->getUseSkirts() );

        assert( dynamic_cast<HlmsOceanDatablock*>( renderable->getDatablock() ) );
        HlmsOceanDatablock *datablock = static_cast<HlmsOceanDatablock*>(
                                                        renderable->getDatablock() );

        setProperty( OceanProperty::ReceiveShadows, 1 );

        setProperty( "terra_enabled", 0 );
        setProperty( OceanProperty::EnvProbeMap, 1 );

        uint32 brdf = datablock->getBrdf();
        if( (brdf & OceanBrdf::BRDF_MASK) == OceanBrdf::Default )
        {
            setProperty( OceanProperty::BrdfDefault, 1 );

            if( !(brdf & OceanBrdf::FLAG_UNCORRELATED) )
                setProperty( OceanProperty::GgxHeightCorrelated, 1 );
        }
        else if( (brdf & OceanBrdf::BRDF_MASK) == OceanBrdf::CookTorrance )
            setProperty( OceanProperty::BrdfCookTorrance, 1 );
        else if( (brdf & OceanBrdf::BRDF_MASK) == OceanBrdf::BlinnPhong )
            setProperty( OceanProperty::BrdfBlinnPhong, 1 );

        if( brdf & OceanBrdf::FLAG_SPERATE_DIFFUSE_FRESNEL )
            setProperty( OceanProperty::FresnelSeparateDiffuse, 1 );
    }
    //-----------------------------------------------------------------------------------
    HlmsCache HlmsOcean::preparePassHash( const CompositorShadowNode *shadowNode, bool casterPass,
                                          bool dualParaboloid, SceneManager *sceneManager )
    {
        mSetProperties.clear();

        if( casterPass )
        {
            HlmsCache retVal = Hlms::preparePassHashBase( shadowNode, casterPass,
                                                          dualParaboloid, sceneManager );
            return retVal;
        }

        //The properties need to be set before preparePassHash so that
        //they are considered when building the HlmsCache's hash.
        if( shadowNode && !casterPass )
        {
            //Shadow receiving can be improved in performance by using gather sampling.
            //(it's the only feature so far that uses gather)
            const RenderSystemCapabilities *capabilities = mRenderSystem->getCapabilities();
            if( capabilities->hasCapability( RSC_TEXTURE_GATHER ) )
                setProperty( HlmsBaseProp::TexGather, 1 );

            if( mShadowFilter == PCF_3x3 )
            {
                setProperty( OceanProperty::Pcf3x3, 1 );
                setProperty( OceanProperty::PcfIterations, 4 );
            }
            else if( mShadowFilter == PCF_4x4 )
            {
                setProperty( OceanProperty::Pcf4x4, 1 );
                setProperty( OceanProperty::PcfIterations, 9 );
            }
            else
            {
                setProperty( OceanProperty::PcfIterations, 1 );
            }

            if( mDebugPssmSplits )
            {
                int32 numPssmSplits = 0;
                const vector<Real>::type *pssmSplits = shadowNode->getPssmSplits( 0 );
                if( pssmSplits )
                {
                    numPssmSplits = static_cast<int32>( pssmSplits->size() - 1 );
                    if( numPssmSplits > 0 )
                        setProperty( OceanProperty::DebugPssmSplits, 1 );
                }
            }
        }

        AmbientLightMode ambientMode = mAmbientLightMode;
        ColourValue upperHemisphere = sceneManager->getAmbientLightUpperHemisphere();
        ColourValue lowerHemisphere = sceneManager->getAmbientLightLowerHemisphere();

        const float envMapScale = upperHemisphere.a;
        //Ignore alpha channel
        upperHemisphere.a = lowerHemisphere.a = 1.0;

        if( !casterPass )
        {
            if( mAmbientLightMode == AmbientAuto )
            {
                if( upperHemisphere == lowerHemisphere )
                {
                    if( upperHemisphere == ColourValue::Black )
                        ambientMode = AmbientNone;
                    else
                        ambientMode = AmbientFixed;
                }
                else
                {
                    ambientMode = AmbientHemisphere;
                }
            }

            if( ambientMode == AmbientFixed )
                setProperty( OceanProperty::AmbientFixed, 1 );
            if( ambientMode == AmbientHemisphere )
                setProperty( OceanProperty::AmbientHemisphere, 1 );

            if( envMapScale != 1.0f )
                setProperty( OceanProperty::EnvMapScale, 1 );
        }

        setProperty( OceanProperty::FresnelScalar, 1 );
        setProperty( OceanProperty::MetallicWorkflow, 1 );

        HlmsCache retVal = Hlms::preparePassHashBase( shadowNode, casterPass,
                                                      dualParaboloid, sceneManager );

        if( getProperty( HlmsBaseProp::LightsDirNonCaster ) > 0 )
        {
            //First directional light always cast shadows thanks to our Ocean shadows.
            int32 shadowCasterDirectional = getProperty( HlmsBaseProp::LightsDirectional );
            shadowCasterDirectional = std::max( shadowCasterDirectional, 1 );
            setProperty( HlmsBaseProp::LightsDirectional, shadowCasterDirectional );
        }

        RenderTarget *renderTarget = sceneManager->getCurrentViewport()->getTarget();

        const RenderSystemCapabilities *capabilities = mRenderSystem->getCapabilities();
        setProperty( OceanProperty::HwGammaRead, capabilities->hasCapability( RSC_HW_GAMMA ) );
        setProperty( OceanProperty::HwGammaWrite, capabilities->hasCapability( RSC_HW_GAMMA ) &&
                                                        renderTarget->isHardwareGammaEnabled() );
        setProperty( OceanProperty::SignedIntTex, capabilities->hasCapability(
                                                            RSC_TEXTURE_SIGNED_INT ) );
        retVal.setProperties = mSetProperties;

        Camera *camera = sceneManager->getCameraInProgress();
        Matrix4 viewMatrix = camera->getViewMatrix(true);

        Matrix4 projectionMatrix = camera->getProjectionMatrixWithRSDepth();

        if( renderTarget->requiresTextureFlipping() )
        {
            projectionMatrix[1][0] = -projectionMatrix[1][0];
            projectionMatrix[1][1] = -projectionMatrix[1][1];
            projectionMatrix[1][2] = -projectionMatrix[1][2];
            projectionMatrix[1][3] = -projectionMatrix[1][3];
        }

        int32 numLights             = getProperty( HlmsBaseProp::LightsSpot );
        int32 numDirectionalLights  = getProperty( HlmsBaseProp::LightsDirNonCaster );
        int32 numShadowMapLights   = getProperty( HlmsBaseProp::NumShadowMapLights );
        int32 numPssmSplits         = getProperty( HlmsBaseProp::PssmSplits );

        //mat4 viewProj + mat4 view;
        size_t mapSize = (16 + 16) * 4;

        mGridBuffer             = 0;
        mGlobalLightListBuffer  = 0;

        ForwardPlusBase *forwardPlus = sceneManager->_getActivePassForwardPlus();
        if( forwardPlus )
        {
            mapSize += forwardPlus->getConstBufferSize();
            mGridBuffer             = forwardPlus->getGridBuffer( camera );
            mGlobalLightListBuffer  = forwardPlus->getGlobalLightListBuffer( camera );
        }

        //mat4 shadowRcv[numShadowMapLights].texViewProj +
        //              vec2 shadowRcv[numShadowMapLights].shadowDepthRange +
        //              vec2 padding +
        //              vec4 shadowRcv[numShadowMapLights].invShadowMapSize +
        //mat3 invViewMatCubemap (upgraded to three vec4)
        mapSize += ( (16 + 2 + 2 + 4) * numShadowMapLights + 4 * 3 ) * 4;

        //vec3 ambientUpperHemi + float envMapScale
        if( ambientMode == AmbientFixed || ambientMode == AmbientHemisphere || envMapScale != 1.0f )
            mapSize += 4 * 4;

        //vec3 ambientLowerHemi + padding + vec3 ambientHemisphereDir + padding
        if( ambientMode == AmbientHemisphere )
            mapSize += 8 * 4;

        //float pssmSplitPoints N times.
        mapSize += numPssmSplits * 4;
        mapSize = alignToNextMultiple( mapSize, 16 );

        if( shadowNode )
        {
            //Six variables * 4 (padded vec3) * 4 (bytes) * numLights
            mapSize += ( 6 * 4 * 4 ) * numLights;
        }
        else
        {
            //Three variables * 4 (padded vec3) * 4 (bytes) * numDirectionalLights
            mapSize += ( 3 * 4 * 4 ) * numDirectionalLights;
        }

        mapSize += mListener->getPassBufferSize( shadowNode, casterPass, dualParaboloid,
                                                 sceneManager );

        //Arbitrary 8kb, should be enough.
        const size_t maxBufferSize = 8 * 1024;

        assert( mapSize <= maxBufferSize );

        if( mCurrentPassBuffer >= mPassBuffers.size() )
        {
            mPassBuffers.push_back( mVaoManager->createConstBuffer( maxBufferSize, BT_DYNAMIC_PERSISTENT,
                                                                    0, false ) );
        }

        ConstBufferPacked *passBuffer = mPassBuffers[mCurrentPassBuffer++];
        float *passBufferPtr = reinterpret_cast<float*>( passBuffer->map( 0, mapSize ) );

#ifndef NDEBUG
        const float *startupPtr = passBufferPtr;
#endif

        //---------------------------------------------------------------------------
        //                          ---- VERTEX SHADER ----
        //---------------------------------------------------------------------------

        //mat4 viewProj;
        Matrix4 viewProjMatrix = projectionMatrix * viewMatrix;
        for( size_t i=0; i<16; ++i )
            *passBufferPtr++ = (float)viewProjMatrix[0][i];

        mPreparedPass.viewMatrix        = viewMatrix;

        mPreparedPass.shadowMaps.clear();

        //mat4 view;
        for( size_t i=0; i<16; ++i )
            *passBufferPtr++ = (float)viewMatrix[0][i];

        size_t shadowMapTexIdx = 0;
        const TextureVec &contiguousShadowMapTex = shadowNode->getContiguousShadowMapTex();

        for( int32 i=0; i<numShadowMapLights; ++i )
        {
            //Skip inactive lights (e.g. no directional lights are available
            //and there's a shadow map that only accepts dir lights)
            while( !shadowNode->isShadowMapIdxActive( shadowMapTexIdx ) )
                ++shadowMapTexIdx;

            //mat4 shadowRcv[numShadowMapLights].texViewProj
            Matrix4 viewProjTex = shadowNode->getViewProjectionMatrix( shadowMapTexIdx );
            for( size_t j=0; j<16; ++j )
                *passBufferPtr++ = (float)viewProjTex[0][j];

            //vec2 shadowRcv[numShadowMapLights].shadowDepthRange
            Real fNear, fFar;
            shadowNode->getMinMaxDepthRange( shadowMapTexIdx, fNear, fFar );
            const Real depthRange = fFar - fNear;
            *passBufferPtr++ = fNear;
            *passBufferPtr++ = 1.0f / depthRange;
            ++passBufferPtr; //Padding
            ++passBufferPtr; //Padding


            //vec2 shadowRcv[numShadowMapLights].invShadowMapSize
            size_t shadowMapTexContigIdx =
                    shadowNode->getIndexToContiguousShadowMapTex( (size_t)shadowMapTexIdx );
            uint32 texWidth  = contiguousShadowMapTex[shadowMapTexContigIdx]->getWidth();
            uint32 texHeight = contiguousShadowMapTex[shadowMapTexContigIdx]->getHeight();
            *passBufferPtr++ = 1.0f / texWidth;
            *passBufferPtr++ = 1.0f / texHeight;
            *passBufferPtr++ = static_cast<float>( texWidth );
            *passBufferPtr++ = static_cast<float>( texHeight );

            ++shadowMapTexIdx;
        }

        //---------------------------------------------------------------------------
        //                          ---- PIXEL SHADER ----
        //---------------------------------------------------------------------------

        Matrix3 viewMatrix3, invViewMatrixCubemap;
        viewMatrix.extract3x3Matrix( viewMatrix3 );
        invViewMatrixCubemap = viewMatrix3.Inverse();
        //Cubemaps are left-handed.
        invViewMatrixCubemap = viewMatrix3;
        invViewMatrixCubemap[0][2] = -invViewMatrixCubemap[0][2];
        invViewMatrixCubemap[1][2] = -invViewMatrixCubemap[1][2];
        invViewMatrixCubemap[2][2] = -invViewMatrixCubemap[2][2];
        invViewMatrixCubemap = invViewMatrixCubemap.Inverse();

        //mat3 invViewMatCubemap
        for( size_t i=0; i<9; ++i )
        {
#ifdef OGRE_GLES2_WORKAROUND_2
            Matrix3 xRot( 1.0f, 0.0f, 0.0f,
                          0.0f, 0.0f, -1.0f,
                          0.0f, 1.0f, 0.0f );
            xRot = xRot * invViewMatrixCubemap;
            *passBufferPtr++ = (float)xRot[0][i];
#else
            *passBufferPtr++ = (float)invViewMatrixCubemap[0][i];
#endif

            //Alignment: each row/column is one vec4, despite being 3x3
            if( !( (i+1) % 3 ) )
                ++passBufferPtr;
        }

        //vec3 ambientUpperHemi + padding
        if( ambientMode == AmbientFixed || ambientMode == AmbientHemisphere || envMapScale != 1.0f )
        {
            *passBufferPtr++ = static_cast<float>( upperHemisphere.r );
            *passBufferPtr++ = static_cast<float>( upperHemisphere.g );
            *passBufferPtr++ = static_cast<float>( upperHemisphere.b );
            *passBufferPtr++ = envMapScale;
        }

        //vec3 ambientLowerHemi + padding + vec3 ambientHemisphereDir + padding
        if( ambientMode == AmbientHemisphere )
        {
            *passBufferPtr++ = static_cast<float>( lowerHemisphere.r );
            *passBufferPtr++ = static_cast<float>( lowerHemisphere.g );
            *passBufferPtr++ = static_cast<float>( lowerHemisphere.b );
            *passBufferPtr++ = 1.0f;

            Vector3 hemisphereDir = viewMatrix3 * sceneManager->getAmbientLightHemisphereDir();
            hemisphereDir.normalise();
            *passBufferPtr++ = static_cast<float>( hemisphereDir.x );
            *passBufferPtr++ = static_cast<float>( hemisphereDir.y );
            *passBufferPtr++ = static_cast<float>( hemisphereDir.z );
            *passBufferPtr++ = 1.0f;
        }

        //float pssmSplitPoints
        for( int32 i=0; i<numPssmSplits; ++i )
            *passBufferPtr++ = (*shadowNode->getPssmSplits(0))[i+1];

        passBufferPtr += alignToNextMultiple( numPssmSplits, 4 ) - numPssmSplits;

        if( numShadowMapLights > 0 )
        {
            //All directional lights (caster and non-caster) are sent.
            //Then non-directional shadow-casting shadow lights are sent.
            size_t shadowLightIdx = 0;
            size_t nonShadowLightIdx = 0;
            const LightListInfo &globalLightList = sceneManager->getGlobalLightList();
            const LightClosestArray &lights = shadowNode->getShadowCastingLights();

            const CompositorShadowNode::LightsBitSet &affectedLights =
                    shadowNode->getAffectedLightsBitSet();

            int32 shadowCastingDirLights = getProperty( HlmsBaseProp::LightsDirectional );

            for( int32 i=0; i<numLights; ++i )
            {
                Light const *light = 0;

                if( i >= shadowCastingDirLights && i < numDirectionalLights )
                {
                    while( affectedLights[nonShadowLightIdx] )
                        ++nonShadowLightIdx;

                    light = globalLightList.lights[nonShadowLightIdx++];

                    assert( light->getType() == Light::LT_DIRECTIONAL );
                }
                else
                {
                    //Skip inactive lights (e.g. no directional lights are available
                    //and there's a shadow map that only accepts dir lights)
                    while( !lights[shadowLightIdx].light )
                        ++shadowLightIdx;
                    light = lights[shadowLightIdx++].light;
                }

                Vector4 lightPos4 = light->getAs4DVector();
                Vector3 lightPos;

                if( light->getType() == Light::LT_DIRECTIONAL )
                    lightPos = viewMatrix3 * Vector3( lightPos4.x, lightPos4.y, lightPos4.z );
                else
                    lightPos = viewMatrix * Vector3( lightPos4.x, lightPos4.y, lightPos4.z );

                //vec3 lights[numLights].position
                *passBufferPtr++ = lightPos.x;
                *passBufferPtr++ = lightPos.y;
                *passBufferPtr++ = lightPos.z;
                ++passBufferPtr;

                //vec3 lights[numLights].diffuse
                ColourValue colour = light->getDiffuseColour() *
                                     light->getPowerScale();
                *passBufferPtr++ = colour.r;
                *passBufferPtr++ = colour.g;
                *passBufferPtr++ = colour.b;
                ++passBufferPtr;

                //vec3 lights[numLights].specular
                colour = light->getSpecularColour() * light->getPowerScale();
                *passBufferPtr++ = colour.r;
                *passBufferPtr++ = colour.g;
                *passBufferPtr++ = colour.b;
                ++passBufferPtr;

                //vec3 lights[numLights].attenuation;
                Real attenRange     = light->getAttenuationRange();
                Real attenLinear    = light->getAttenuationLinear();
                Real attenQuadratic = light->getAttenuationQuadric();
                *passBufferPtr++ = attenRange;
                *passBufferPtr++ = attenLinear;
                *passBufferPtr++ = attenQuadratic;
                ++passBufferPtr;

                //vec3 lights[numLights].spotDirection;
                Vector3 spotDir = viewMatrix3 * light->getDerivedDirection();
                *passBufferPtr++ = spotDir.x;
                *passBufferPtr++ = spotDir.y;
                *passBufferPtr++ = spotDir.z;
                ++passBufferPtr;

                //vec3 lights[numLights].spotParams;
                Radian innerAngle = light->getSpotlightInnerAngle();
                Radian outerAngle = light->getSpotlightOuterAngle();
                *passBufferPtr++ = 1.0f / ( cosf( innerAngle.valueRadians() * 0.5f ) -
                                         cosf( outerAngle.valueRadians() * 0.5f ) );
                *passBufferPtr++ = cosf( outerAngle.valueRadians() * 0.5f );
                *passBufferPtr++ = light->getSpotlightFalloff();
                ++passBufferPtr;
            }
        }
        else
        {
            //No shadow maps, only send directional lights
            const LightListInfo &globalLightList = sceneManager->getGlobalLightList();

            for( int32 i=0; i<numDirectionalLights; ++i )
            {
                assert( globalLightList.lights[i]->getType() == Light::LT_DIRECTIONAL );

                Vector4 lightPos4 = globalLightList.lights[i]->getAs4DVector();
                Vector3 lightPos = viewMatrix3 * Vector3( lightPos4.x, lightPos4.y, lightPos4.z );

                //vec3 lights[numLights].position
                *passBufferPtr++ = lightPos.x;
                *passBufferPtr++ = lightPos.y;
                *passBufferPtr++ = lightPos.z;
                ++passBufferPtr;

                //vec3 lights[numLights].diffuse
                ColourValue colour = globalLightList.lights[i]->getDiffuseColour() *
                                     globalLightList.lights[i]->getPowerScale();
                *passBufferPtr++ = colour.r;
                *passBufferPtr++ = colour.g;
                *passBufferPtr++ = colour.b;
                ++passBufferPtr;

                //vec3 lights[numLights].specular
                colour = globalLightList.lights[i]->getSpecularColour() * globalLightList.lights[i]->getPowerScale();
                *passBufferPtr++ = colour.r;
                *passBufferPtr++ = colour.g;
                *passBufferPtr++ = colour.b;
                ++passBufferPtr;
            }
        }

        if( shadowNode )
        {
            mPreparedPass.shadowMaps.reserve( contiguousShadowMapTex.size() );

            TextureVec::const_iterator itShadowMap = contiguousShadowMapTex.begin();
            TextureVec::const_iterator enShadowMap = contiguousShadowMapTex.end();

            while( itShadowMap != enShadowMap )
            {
                mPreparedPass.shadowMaps.push_back( itShadowMap->get() );
                ++itShadowMap;
            }
        }

        if( forwardPlus )
        {
            forwardPlus->fillConstBufferData( renderTarget, passBufferPtr );
            passBufferPtr += forwardPlus->getConstBufferSize() >> 2;
        }

        //Timer
        *passBufferPtr++ = Ogre::Root::getSingletonPtr()->getTimer()->getMilliseconds()*0.0005;
        *passBufferPtr++ = 0;
        *passBufferPtr++ = 0;
        *passBufferPtr++ = 0;


        passBufferPtr = mListener->preparePassBuffer( shadowNode, casterPass, dualParaboloid,
                                                      sceneManager, passBufferPtr );

        assert( (size_t)(passBufferPtr - startupPtr) * 4u == mapSize );

        passBuffer->unmap( UO_KEEP_PERSISTENT );

        mLastTextureHash = 0;
        mLastMovableObject = 0;

        mLastBoundPool = 0;

        if( mShadowmapSamplerblock && !getProperty( HlmsBaseProp::ShadowUsesDepthTexture ) )
            mCurrentShadowmapSamplerblock = mShadowmapSamplerblock;
        else
            mCurrentShadowmapSamplerblock = mShadowmapCmpSamplerblock;

        uploadDirtyDatablocks();

        return retVal;
    }
    //-----------------------------------------------------------------------------------
    uint32 HlmsOcean::fillBuffersFor( const HlmsCache *cache, const QueuedRenderable &queuedRenderable,
                                    bool casterPass, uint32 lastCacheHash,
                                    uint32 lastTextureHash )
    {
        OGRE_EXCEPT( Exception::ERR_NOT_IMPLEMENTED,
                     "Trying to use slow-path on a desktop implementation. "
                     "Change the RenderQueue settings.",
                     "HlmsOcean::fillBuffersFor" );
    }
    //-----------------------------------------------------------------------------------
    uint32 HlmsOcean::fillBuffersForV1( const HlmsCache *cache,
                                      const QueuedRenderable &queuedRenderable,
                                      bool casterPass, uint32 lastCacheHash,
                                      CommandBuffer *commandBuffer )
    {
        return fillBuffersFor( cache, queuedRenderable, casterPass,
                               lastCacheHash, commandBuffer, true );
    }
    //-----------------------------------------------------------------------------------
    uint32 HlmsOcean::fillBuffersForV2( const HlmsCache *cache,
                                      const QueuedRenderable &queuedRenderable,
                                      bool casterPass, uint32 lastCacheHash,
                                      CommandBuffer *commandBuffer )
    {
        return fillBuffersFor( cache, queuedRenderable, casterPass,
                               lastCacheHash, commandBuffer, false );
    }
    //-----------------------------------------------------------------------------------
    uint32 HlmsOcean::fillBuffersFor( const HlmsCache *cache, const QueuedRenderable &queuedRenderable,
                                    bool casterPass, uint32 lastCacheHash,
                                    CommandBuffer *commandBuffer, bool isV1 )
    {
        assert( dynamic_cast<const HlmsOceanDatablock*>( queuedRenderable.renderable->getDatablock() ) );
        const HlmsOceanDatablock *datablock = static_cast<const HlmsOceanDatablock*>(
                                                queuedRenderable.renderable->getDatablock() );

        if( OGRE_EXTRACT_HLMS_TYPE_FROM_CACHE_HASH( lastCacheHash ) != mType )
        {
            //layout(binding = 0) uniform PassBuffer {} pass
            ConstBufferPacked *passBuffer = mPassBuffers[mCurrentPassBuffer-1];
            *commandBuffer->addCommand<CbShaderBuffer>() = CbShaderBuffer( VertexShader,
                                                                           0, passBuffer, 0,
                                                                           passBuffer->
                                                                           getTotalSizeBytes() );
            *commandBuffer->addCommand<CbShaderBuffer>() = CbShaderBuffer( PixelShader,
                                                                           0, passBuffer, 0,
                                                                           passBuffer->
                                                                           getTotalSizeBytes() );

            size_t texUnit = 3;

            if( mGridBuffer )
            {
                texUnit = 5;
                *commandBuffer->addCommand<CbShaderBuffer>() =
                        CbShaderBuffer( PixelShader, 3, mGridBuffer, 0, 0 );
                *commandBuffer->addCommand<CbShaderBuffer>() =
                        CbShaderBuffer( PixelShader, 4, mGlobalLightListBuffer, 0, 0 );
            }

            //We changed HlmsType, rebind the shared textures.
            FastArray<Texture*>::const_iterator itor = mPreparedPass.shadowMaps.begin();
            FastArray<Texture*>::const_iterator end  = mPreparedPass.shadowMaps.end();
            while( itor != end )
            {
                *commandBuffer->addCommand<CbTexture>() = CbTexture( texUnit, true, *itor,
                                                                     mCurrentShadowmapSamplerblock );
                ++texUnit;
                ++itor;
            }

            mLastTextureHash = 0;
            mLastMovableObject = 0;
            mLastBoundPool = 0;

            //layout(binding = 2) uniform InstanceBuffer {} instance
            if( mCurrentConstBuffer < mConstBuffers.size() &&
                (size_t)((mCurrentMappedConstBuffer - mStartMappedConstBuffer) + 16) <=
                    mCurrentConstBufferSize )
            {
                *commandBuffer->addCommand<CbShaderBuffer>() =
                        CbShaderBuffer( VertexShader, 2, mConstBuffers[mCurrentConstBuffer], 0, 0 );
                *commandBuffer->addCommand<CbShaderBuffer>() =
                        CbShaderBuffer( PixelShader, 2, mConstBuffers[mCurrentConstBuffer], 0, 0 );
            }

            mListener->hlmsTypeChanged( casterPass, commandBuffer, datablock );
        }

        //Don't bind the material buffer on caster passes (important to keep
        //MDI & auto-instancing running on shadow map passes)
        if( mLastBoundPool != datablock->getAssignedPool() &&
            (!casterPass || datablock->getAlphaTest() != CMPF_ALWAYS_PASS) )
        {
            //layout(binding = 1) uniform MaterialBuf {} materialArray
            const ConstBufferPool::BufferPool *newPool = datablock->getAssignedPool();
            *commandBuffer->addCommand<CbShaderBuffer>() = CbShaderBuffer( PixelShader,
                                                                           1, newPool->materialBuffer, 0,
                                                                           newPool->materialBuffer->
                                                                           getTotalSizeBytes() );
            mLastBoundPool = newPool;
        }

        if( mLastMovableObject != queuedRenderable.movableObject )
        {
            {
                Ogre::TexturePtr tex = Ogre::TextureManager::getSingletonPtr()->getByName("oceanData.dds");
                if (tex.isNull()){
                    tex = Ogre::TextureManager::getSingleton().load("oceanData.dds", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_3D);
                }
                Ogre::HlmsSamplerblock samplerblock;
                samplerblock.mU = Ogre::TextureAddressingMode::TAM_WRAP;
                samplerblock.mV = Ogre::TextureAddressingMode::TAM_WRAP;
                samplerblock.mW = Ogre::TextureAddressingMode::TAM_WRAP;
                const Ogre::HlmsSamplerblock* finalSamplerblock = Ogre::Root::getSingleton().getHlmsManager()->getSamplerblock(samplerblock);
                *commandBuffer->addCommand<Ogre::CbTexture>() = Ogre::CbTexture( 0, true, tex.getPointer(), finalSamplerblock);
            }
            {
                Ogre::TexturePtr tex = Ogre::TextureManager::getSingletonPtr()->getByName("weight.dds");
                if (tex.isNull()){
                    tex = Ogre::TextureManager::getSingleton().load("weight.dds", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D);
                }
                Ogre::HlmsSamplerblock samplerblock;
                samplerblock.mU = Ogre::TextureAddressingMode::TAM_WRAP;
                samplerblock.mV = Ogre::TextureAddressingMode::TAM_WRAP;
                const Ogre::HlmsSamplerblock* finalSamplerblock = Ogre::Root::getSingleton().getHlmsManager()->getSamplerblock(samplerblock);
                *commandBuffer->addCommand<Ogre::CbTexture>() = Ogre::CbTexture( 1, true, tex.getPointer(), finalSamplerblock);
            }
            if( !mProbe.isNull() ){

                size_t texUnit = mPreparedPass.shadowMaps.size() + (!mGridBuffer ? 3 : 5);

                *commandBuffer->addCommand<CbTexture>() =
                        CbTexture( texUnit, true, mProbe.get(), mOceanSamplerblock );
            }


            mLastMovableObject = queuedRenderable.movableObject;
        }

        uint32 * RESTRICT_ALIAS currentMappedConstBuffer    = mCurrentMappedConstBuffer;

        //---------------------------------------------------------------------------
        //                          ---- VERTEX SHADER ----
        //---------------------------------------------------------------------------
        //We need to correct currentMappedConstBuffer to point to the right texture buffer's
        //offset, which may not be in sync if the previous draw had skeletal animation.
        bool exceedsConstBuffer = (size_t)((currentMappedConstBuffer - mStartMappedConstBuffer) + 12)
                                    > mCurrentConstBufferSize;

        if( exceedsConstBuffer )
            currentMappedConstBuffer = mapNextConstBuffer( commandBuffer );

        const Ogre::OceanCell *OceanCell = static_cast<const Ogre::OceanCell*>( queuedRenderable.renderable );

        OceanCell->uploadToGpu( currentMappedConstBuffer );
        currentMappedConstBuffer += 16u;

        //---------------------------------------------------------------------------
        //                          ---- PIXEL SHADER ----
        //---------------------------------------------------------------------------

        mCurrentMappedConstBuffer   = currentMappedConstBuffer;

        return ((mCurrentMappedConstBuffer - mStartMappedConstBuffer) >> 4) - 1;
    }
    //-----------------------------------------------------------------------------------
    void HlmsOcean::destroyAllBuffers(void)
    {
        HlmsBufferManager::destroyAllBuffers();

        mCurrentPassBuffer  = 0;

        {
            ConstBufferPackedVec::const_iterator itor = mPassBuffers.begin();
            ConstBufferPackedVec::const_iterator end  = mPassBuffers.end();

            while( itor != end )
            {
                if( (*itor)->getMappingState() != MS_UNMAPPED )
                    (*itor)->unmap( UO_UNMAP_ALL );
                mVaoManager->destroyConstBuffer( *itor );
                ++itor;
            }

            mPassBuffers.clear();
        }
    }
    //-----------------------------------------------------------------------------------
    void HlmsOcean::frameEnded(void)
    {
        HlmsBufferManager::frameEnded();
        mCurrentPassBuffer  = 0;
    }
    //-----------------------------------------------------------------------------------
    void HlmsOcean::setDebugPssmSplits( bool bDebug )
    {
        mDebugPssmSplits = bDebug;
    }
    //-----------------------------------------------------------------------------------
    void HlmsOcean::setShadowSettings( ShadowFilter filter )
    {
        mShadowFilter = filter;
    }
    //-----------------------------------------------------------------------------------
    void HlmsOcean::setAmbientLightMode( AmbientLightMode mode )
    {
        mAmbientLightMode = mode;
    }
    //-----------------------------------------------------------------------------------
    void HlmsOcean::setEnvProbe( Ogre::TexturePtr probe ){
        mProbe = probe;
    }
    //-----------------------------------------------------------------------------------
    HlmsDatablock* HlmsOcean::createDatablockImpl( IdString datablockName,
                                                       const HlmsMacroblock *macroblock,
                                                       const HlmsBlendblock *blendblock,
                                                       const HlmsParamVec &paramVec )
    {
        return OGRE_NEW HlmsOceanDatablock( datablockName, this, macroblock, blendblock, paramVec );
    }
}
