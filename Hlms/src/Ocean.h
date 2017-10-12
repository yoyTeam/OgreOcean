
#ifndef _OgreOcean_H_
#define _OgreOcean_H_

#include "OgrePrerequisites.h"
#include "OgreMovableObject.h"
#include "OgreShaderParams.h"

#include "Ocean/OceanCell.h"

namespace Ogre
{
    struct OceanGridPoint
    {
        int32 x;
        int32 z;
    };

    struct OceanGridDirection
    {
        int x;
        int z;
    };


    class Ocean : public MovableObject
    {
        friend class OceanCell;

        std::vector<float>          m_heightMap;
        uint32                      m_width;
        uint32                      m_depth; //PNG's Height
        float                       m_depthWidthRatio;
        float                       m_invWidth;
        float                       m_invDepth;
        float                       m_uvScale;

        Vector2     m_xzDimensions;
        Vector2     m_xzInvDimensions;
        Vector2     m_xzRelativeSize; // m_xzDimensions / [m_width, m_height]
        float       m_height;
        float       m_WavesIntensity;
        Vector3     m_OceanOrigin;
        uint32      m_basePixelDimension;

        std::vector<OceanCell>   m_OceanCells;
        std::vector<OceanCell*>  m_collectedCells[2];
        size_t                     m_currentCell;

        Ogre::TexturePtr    m_heightMapTex;
        Ogre::TexturePtr    m_normalMapTex;

        Vector3             m_prevLightDir;

        //Ogre stuff
        CompositorManager2      *m_compositorManager;
        Camera                  *m_camera;

        inline OceanGridPoint worldToGrid( const Vector3 &vPos ) const;
        inline Vector2 gridToWorld( const OceanGridPoint &gPos ) const;

        bool isVisible( const OceanGridPoint &gPos, const OceanGridPoint &gSize ) const;

        void addRenderable( const OceanGridPoint &gridPos, const OceanGridPoint &cellSize, uint32 lodLevel );

        void optimizeCellsAndAdd(void);

    public:
        Ocean( IdType id, ObjectMemoryManager *objectMemoryManager, SceneManager *sceneManager,
               uint8 renderQueueId, CompositorManager2 *compositorManager, Camera *camera );
        ~Ocean();

        /** Must be called every frame so we can check the camera's position
            (passed in the constructor) and update our visible batches (and LODs).
        */
        void update();

        void create(const Vector3 center, const Vector2 &dimensions);

        void setWavesIntensity( float intensity );

        void setWavesScale( float scale );

        /// load must already have been called.
        void setDatablock( HlmsDatablock *datablock );

        //MovableObject overloads
        const String& getMovableType(void) const;

        Camera* getCamera() const                       { return m_camera; }
        void setCamera( Camera *camera )                { m_camera = camera; }

        const Vector2& getXZDimensions(void) const      { return m_xzDimensions; }
        const Vector2& getXZInvDimensions(void) const   { return m_xzInvDimensions; }
        const Vector3& getOceanOrigin(void) const     { return m_OceanOrigin; }
    };
}

#endif
