
#ifndef _OgreOceanCell_H_
#define _OgreOceanCell_H_

#include "OgrePrerequisites.h"
#include "OgreRenderable.h"

namespace Ogre
{
    class Ocean;
    struct OceanGridPoint;

    class OceanCell : public Renderable
    {
        int32  m_gridX;
        int32  m_gridZ;
        uint32 m_lodLevel;
        uint32  m_verticesPerLine;

        uint32  m_sizeX;
        uint32  m_sizeZ;

        VaoManager *m_vaoManager;

        Ocean *m_parentOcean;

        bool m_useSkirts;

    public:
        OceanCell( Ocean *parentOcean );
        virtual ~OceanCell();

        bool getUseSkirts(void) const                   { return m_useSkirts; }

        void initialize( VaoManager *vaoManager, bool useSkirts );

        void setOrigin( const OceanGridPoint &gridPos, uint32 horizontalPixelDim,
                        uint32 verticalPixelDim, uint32 lodLevel );

        /** Merges another OceanCell into 'this' for reducing batch counts.
            e.g.
                Two 32x32 cells will merge into one 64x32 or 32x64
                Two 64x32 cells will merge into one 64x64
                A 32x64 cell cannot merge with a 32x32 one.
                A 64x32 cell cannot merge with a 32x32 one.
        @remarks
            Merge will only happen if the cells are of the same LOD level and are contiguous.
        @param next
            The other OceanCell to merge with.
        @return
            False if couldn't merge, true on success.
        */
        bool merge( OceanCell *next );

        void uploadToGpu( uint32 * RESTRICT_ALIAS gpuPtr ) const;

        //Renderable overloads
        virtual const LightList& getLights(void) const;
        virtual void getRenderOperation( v1::RenderOperation& op, bool casterPass );
        virtual void getWorldTransforms( Matrix4* xform ) const;
        virtual bool getCastsShadows(void) const;
    };
}

#endif
