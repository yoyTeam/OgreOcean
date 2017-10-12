
#include "Ocean/Ocean.h"

#include "OgreImage.h"
#include "OgreTextureManager.h"
#include "OgreHardwarePixelBuffer.h"

#include "OgreCamera.h"
#include "OgreSceneManager.h"
#include "Compositor/OgreCompositorManager2.h"
#include "Compositor/OgreCompositorWorkspace.h"
#include "Compositor/OgreCompositorChannel.h"
#include "OgreMaterialManager.h"
#include "OgreTechnique.h"
#include "OgreRenderTexture.h"

namespace Ogre
{
    Ocean::Ocean( IdType id, ObjectMemoryManager *objectMemoryManager,
                  SceneManager *sceneManager, uint8 renderQueueId,
                  CompositorManager2 *compositorManager, Camera *camera ) :
        MovableObject( id, objectMemoryManager, sceneManager, renderQueueId ),
        m_width( 0u ),
        m_depth( 0u ),
        m_depthWidthRatio( 1.0f ),
        m_invWidth( 1.0f ),
        m_invDepth( 1.0f ),
        m_uvScale( 1.0f ),
        m_xzDimensions( Vector2::UNIT_SCALE ),
        m_xzInvDimensions( Vector2::UNIT_SCALE ),
        m_xzRelativeSize( Vector2::UNIT_SCALE ),
        m_height( 1.0f ),
        m_WavesIntensity( 0.5f ),
        m_OceanOrigin( Vector3::ZERO ),
        m_basePixelDimension( 256u ),
        m_currentCell( 0u ),
        m_prevLightDir( Vector3::ZERO ),
        m_compositorManager( compositorManager ),
        m_camera( camera )
    {
    }
    //-----------------------------------------------------------------------------------
    Ocean::~Ocean()
    {
        m_OceanCells.clear();
    }
    //-----------------------------------------------------------------------------------
    inline OceanGridPoint Ocean::worldToGrid( const Vector3 &vPos ) const
    {
        OceanGridPoint retVal;
        const float fWidth = static_cast<float>( m_width );
        const float fDepth = static_cast<float>( m_depth );

        retVal.x = (uint32)floorf( ((vPos.x - m_OceanOrigin.x) * m_xzInvDimensions.x) * fWidth );
        retVal.z = (uint32)floorf( ((vPos.z - m_OceanOrigin.z) * m_xzInvDimensions.y) * fDepth );

        return retVal;
    }
    //-----------------------------------------------------------------------------------
    inline Vector2 Ocean::gridToWorld( const OceanGridPoint &gPos ) const
    {
        Vector2 retVal;
        const float fWidth = static_cast<float>( m_width );
        const float fDepth = static_cast<float>( m_depth );

        retVal.x = (gPos.x / fWidth) * m_xzDimensions.x + m_OceanOrigin.x;
        retVal.y = (gPos.z / fDepth) * m_xzDimensions.y + m_OceanOrigin.z;

        return retVal;
    }
    //-----------------------------------------------------------------------------------
    bool Ocean::isVisible( const OceanGridPoint &gPos, const OceanGridPoint &gSize ) const
    {
        if( gPos.x >= static_cast<int32>( m_width ) ||
            gPos.z >= static_cast<int32>( m_depth ) ||
            gPos.x + gSize.x <= 0 ||
            gPos.z + gSize.z <= 0 )
        {
            //Outside Ocean bounds.
            return false;
        }

//        return true;

        const Vector2 cellPos = gridToWorld( gPos );
        const Vector2 cellSize( (gSize.x + 1u) * m_xzRelativeSize.x,
                                (gSize.z + 1u) * m_xzRelativeSize.y );

        const Vector3 vHalfSize = Vector3( cellSize.x, m_height, cellSize.y ) * 0.5f;
        const Vector3 vCenter = Vector3( cellPos.x, m_OceanOrigin.y, cellPos.y ) + vHalfSize;

        for( int i=0; i<6; ++i )
        {
            //Skip far plane if view frustum is infinite
            if( i == FRUSTUM_PLANE_FAR && m_camera->getFarClipDistance() == 0 )
                continue;

            Plane::Side side = m_camera->getFrustumPlane(i).getSide( vCenter, vHalfSize );

            //We only need one negative match to know the obj is outside the frustum
            if( side == Plane::NEGATIVE_SIDE )
                return false;
        }

        return true;
    }
    //-----------------------------------------------------------------------------------
    void Ocean::addRenderable( const OceanGridPoint &gridPos, const OceanGridPoint &cellSize, uint32 lodLevel )
    {
        OceanCell *cell = &m_OceanCells[m_currentCell++];
        cell->setOrigin( gridPos, cellSize.x, cellSize.z, lodLevel );
        m_collectedCells[0].push_back( cell );
    }
    //-----------------------------------------------------------------------------------
    void Ocean::optimizeCellsAndAdd(void)
    {
        //Keep iterating until m_collectedCells[0] stops shrinking
        size_t numCollectedCells = std::numeric_limits<size_t>::max();
        while( numCollectedCells != m_collectedCells[0].size() )
        {
            numCollectedCells = m_collectedCells[0].size();

            if( m_collectedCells[0].size() > 1 )
            {
                m_collectedCells[1].clear();

                std::vector<OceanCell*>::const_iterator itor = m_collectedCells[0].begin();
                std::vector<OceanCell*>::const_iterator end  = m_collectedCells[0].end();

                while( end - itor >= 2u )
                {
                    OceanCell *currCell = *itor;
                    OceanCell *nextCell = *(itor+1);

                    m_collectedCells[1].push_back( currCell );
                    if( currCell->merge( nextCell ) )
                        itor += 2;
                    else
                        ++itor;
                }

                while( itor != end )
                    m_collectedCells[1].push_back( *itor++ );

                m_collectedCells[1].swap( m_collectedCells[0] );
            }
        }

        std::vector<OceanCell*>::const_iterator itor = m_collectedCells[0].begin();
        std::vector<OceanCell*>::const_iterator end  = m_collectedCells[0].end();
        while( itor != end )
            mRenderables.push_back( *itor++ );

        m_collectedCells[0].clear();
    }
    //-----------------------------------------------------------------------------------
    void Ocean::update()
    {
        m_height = m_WavesIntensity*0.9 + 0.1; //from 10% to 100%, less than 10 looks too flat

        mRenderables.clear();
        m_currentCell = 0;

        Vector3 camPos = m_camera->getDerivedPosition();

        const uint32 basePixelDimension = m_basePixelDimension;
        const uint32 vertPixelDimension = static_cast<uint32>(m_basePixelDimension * m_depthWidthRatio);

        OceanGridPoint cellSize;
        cellSize.x = basePixelDimension;
        cellSize.z = vertPixelDimension;

        //Quantize the camera position to basePixelDimension steps
        OceanGridPoint camCenter = worldToGrid( camPos );
        camCenter.x = (camCenter.x / basePixelDimension) * basePixelDimension;
        camCenter.z = (camCenter.z / vertPixelDimension) * vertPixelDimension;

        uint32 currentLod = 0;

//        camCenter.x = 64;
//        camCenter.z = 64;

        //LOD 0: Add full 4x4 grid
        for( int32 z=-2; z<2; ++z )
        {
            for( int32 x=-2; x<2; ++x )
            {
                OceanGridPoint pos = camCenter;
                pos.x += x * cellSize.x;
                pos.z += z * cellSize.z;

                if( isVisible( pos, cellSize ) )
                    addRenderable( pos, cellSize, currentLod );
            }
        }

        optimizeCellsAndAdd();

        m_currentCell = 16u; //The first 16 cells don't use skirts.

        const uint32 maxRes = std::max( m_width, m_depth );
        //TODO: When we're too far (outside the Ocean), just display a 4x4 grid or something like that.

        size_t numObjectsAdded = std::numeric_limits<size_t>::max();
        //LOD n: Add 4x4 grid, ignore 2x2 center (which
        //is the same as saying the borders of the grid)
        while( numObjectsAdded != m_currentCell ||
               (mRenderables.empty() && (1u << currentLod) <= maxRes) )
        {
            numObjectsAdded = m_currentCell;

            cellSize.x <<= 1u;
            cellSize.z <<= 1u;
            ++currentLod;

            //Row 0
            {
                const int32 z = 1;
                for( int32 x=-2; x<2; ++x )
                {
                    OceanGridPoint pos = camCenter;
                    pos.x += x * cellSize.x;
                    pos.z += z * cellSize.z;

                    if( isVisible( pos, cellSize ) )
                        addRenderable( pos, cellSize, currentLod );
                }
            }
            //Row 3
            {
                const int32 z = -2;
                for( int32 x=-2; x<2; ++x )
                {
                    OceanGridPoint pos = camCenter;
                    pos.x += x * cellSize.x;
                    pos.z += z * cellSize.z;

                    if( isVisible( pos, cellSize ) )
                        addRenderable( pos, cellSize, currentLod );
                }
            }
            //Cells [0, 1] & [0, 2];
            {
                const int32 x = -2;
                for( int32 z=-1; z<1; ++z )
                {
                    OceanGridPoint pos = camCenter;
                    pos.x += x * cellSize.x;
                    pos.z += z * cellSize.z;

                    if( isVisible( pos, cellSize ) )
                        addRenderable( pos, cellSize, currentLod );
                }
            }
            //Cells [3, 1] & [3, 2];
            {
                const int32 x = 1;
                for( int32 z=-1; z<1; ++z )
                {
                    OceanGridPoint pos = camCenter;
                    pos.x += x * cellSize.x;
                    pos.z += z * cellSize.z;

                    if( isVisible( pos, cellSize ) )
                        addRenderable( pos, cellSize, currentLod );
                }
            }

            optimizeCellsAndAdd();
        }
    }
    //-----------------------------------------------------------------------------------
    void Ocean::create( const Vector3 center, const Vector2 &dimensions )
    {
        m_OceanOrigin = center - Ogre::Vector3( dimensions.x, 0, dimensions.y ) * 0.5f;
        m_xzDimensions = dimensions;
        m_xzInvDimensions = 1.0f / m_xzDimensions;
        m_basePixelDimension = 64u;

        m_width = 4096;
        m_depth = 4096;
        m_depthWidthRatio = m_depth / (float)(m_width);
        m_invWidth = 1.0f / m_width;
        m_invDepth = 1.0f / m_depth;

        m_xzRelativeSize = m_xzDimensions / Vector2( static_cast<Real>(m_width),
                                                     static_cast<Real>(m_depth) );

        {
            //Find out how many OceanCells we need. I think this might be
            //solved analitically with a power series. But my math is rusty.
            const uint32 basePixelDimension = m_basePixelDimension;
            const uint32 vertPixelDimension = static_cast<uint32>( m_basePixelDimension *
                                                                   m_depthWidthRatio );
            const uint32 maxPixelDimension = std::max( basePixelDimension, vertPixelDimension );
            const uint32 maxRes = std::max( m_width, m_depth );

            uint32 numCells = 16u; //4x4
            uint32 accumDim = 0u;
            uint32 iteration = 1u;
            while( accumDim < maxRes )
            {
                numCells += 12u; //4x4 - 2x2
                accumDim += maxPixelDimension * (1u << iteration);
                ++iteration;
            }

            numCells += 12u;
            accumDim += maxPixelDimension * (1u << iteration);
            ++iteration;

            m_OceanCells.clear();
            m_OceanCells.resize( numCells, OceanCell( this ) );
        }

        VaoManager *vaoManager = mManager->getDestinationRenderSystem()->getVaoManager();
        std::vector<OceanCell>::iterator itor = m_OceanCells.begin();
        std::vector<OceanCell>::iterator end  = m_OceanCells.end();

        const std::vector<OceanCell>::iterator begin = itor;

        while( itor != end )
        {
            itor->initialize( vaoManager, (itor - begin) >= 16u );
            ++itor;
        }
    }
    //-----------------------------------------------------------------------------------
    void Ocean::setWavesIntensity( float intensity ){
        m_WavesIntensity = intensity;
    }
    //-----------------------------------------------------------------------------------
    void Ocean::setWavesScale( float scale ){
        m_uvScale = 1.0f/scale;
    }
    //-----------------------------------------------------------------------------------
    void Ocean::setDatablock( HlmsDatablock *datablock )
    {
        std::vector<OceanCell>::iterator itor = m_OceanCells.begin();
        std::vector<OceanCell>::iterator end  = m_OceanCells.end();

        while( itor != end )
        {
            itor->setDatablock( datablock );
            ++itor;
        }
    }
    //-----------------------------------------------------------------------------------
    const String& Ocean::getMovableType(void) const
    {
        static const String movType = "Ocean";
        return movType;
    }
}
