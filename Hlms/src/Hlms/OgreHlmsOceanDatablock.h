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
#ifndef _OgreHlmsOceanDatablock_H_
#define _OgreHlmsOceanDatablock_H_

#include "OgreHlmsDatablock.h"
#include "OgreHlmsTextureManager.h"
#include "OgreConstBufferPool.h"
#include "OgreVector4.h"
#include "OgreHeaderPrefix.h"

namespace Ogre
{
    /** \addtogroup Core
    *  @{
    */
    /** \addtogroup Resources
    *  @{
    */

    namespace OceanBrdf
    {
    enum OceanBrdf
    {
        FLAG_UNCORRELATED                           = 0x80000000,
        FLAG_SPERATE_DIFFUSE_FRESNEL                = 0x40000000,
        BRDF_MASK                                   = 0x00000FFF,

        /// Most physically accurate BRDF we have. Good for representing
        /// the majority of materials.
        /// Uses:
        ///     * Roughness/Distribution/NDF term: GGX
        ///     * Geometric/Visibility term: Smith GGX Height-Correlated
        ///     * Normalized Disney Diffuse BRDF,see
        ///         "Moving Frostbite to Physically Based Rendering" from
        ///         Sebastien Lagarde & Charles de Rousiers
        Default         = 0x00000000,

        /// Implements Cook-Torrance BRDF.
        /// Uses:
        ///     * Roughness/Distribution/NDF term: Beckmann
        ///     * Geometric/Visibility term: Cook-Torrance
        ///     * Lambertian Diffuse.
        ///
        /// Ideal for silk (use high roughness values), synthetic fabric
        CookTorrance    = 0x00000001,

        /// Implements Normalized Blinn Phong using a normalization
        /// factor of (n + 8) / (8 * pi)
        /// The main reason to use this BRDF is performance. It's cheap.
        BlinnPhong      = 0x00000002,

        /// Same as Default, but the geometry term is not height-correlated
        /// which most notably causes edges to be dimmer and is less correct.
        /// Unity (Marmoset too?) use an uncorrelated term, so you may want to
        /// use this BRDF to get the closest look for a nice exchangeable
        /// pipeline workflow.
        DefaultUncorrelated             = Default|FLAG_UNCORRELATED,

        /// Same as Default but the fresnel of the diffuse is calculated
        /// differently. Normally the diffuse component would be multiplied against
        /// the inverse of the specular's fresnel to maintain energy conservation.
        /// This has the nice side effect that to achieve a perfect mirror effect,
        /// you just need to raise the fresnel term to 1; which is very intuitive
        /// to artists (specially if using coloured fresnel)
        ///
        /// When using this BRDF, the diffuse fresnel will be calculated differently,
        /// causing the diffuse component to still affect the colour even when
        /// the fresnel = 1 (although subtly). To achieve a perfect mirror you will
        /// have to set the fresnel to 1 *and* the diffuse colour to black;
        /// which can be unintuitive for artists.
        ///
        /// This BRDF is very useful for representing surfaces with complex refractions
        /// and reflections like glass, transparent plastics, fur, and surface with
        /// refractions and multiple rescattering that cannot be represented well
        /// with the default BRDF.
        DefaultSeparateDiffuseFresnel   = Default|FLAG_SPERATE_DIFFUSE_FRESNEL,

        /// @see DefaultSeparateDiffuseFresnel. This is the same
        /// but the Cook Torrance model is used instead.
        ///
        /// Ideal for shiny objects like glass toy marbles, some types of rubber.
        /// silk, synthetic fabric.
        CookTorranceSeparateDiffuseFresnel  = CookTorrance|FLAG_SPERATE_DIFFUSE_FRESNEL,

        BlinnPhongSeparateDiffuseFresnel    = BlinnPhong|FLAG_SPERATE_DIFFUSE_FRESNEL,
    };
    }

    /** Contains information needed by Ocean (Physically Based Shading) for OpenGL 3+ & D3D11+
    */
    class HlmsOceanDatablock : public HlmsDatablock, public ConstBufferPoolUser
    {
        friend class HlmsOcean;

    protected:
        float   mDeepColuorR, mDeepColourG, mDeepColourB;
        float   _padding0;
        float   mShallowColourR, mShallowColourG, mShallowColourB;
        float   mWavesScale;

        /// @see OceanBrdf::OceanBrdf
        uint32  mBrdf;

        void scheduleConstBufferUpdate(void);
        virtual void uploadToConstBuffer( char *dstPtr );

    public:
        HlmsOceanDatablock( IdString name, HlmsOcean *creator,
                          const HlmsMacroblock *macroblock,
                          const HlmsBlendblock *blendblock,
                          const HlmsParamVec &params );
        virtual ~HlmsOceanDatablock();

        void setDeepColour( const Vector3 &deepColour );
        Vector3 getDeepColour(void) const;

        void setShallowColour( const Vector3 &shallowColour );
        Vector3 getShallowColour(void) const;

        void setWavesScale( float scale );
        float getWavesScale() const;

        /// Overloaded to tell it's unsupported
        virtual void setAlphaTestThreshold( float threshold );

        /// Changes the BRDF in use. Calling this function may trigger an
        /// HlmsDatablock::flushRenderables
        void setBrdf( OceanBrdf::OceanBrdf brdf );
        uint32 getBrdf(void) const;

        virtual void calculateHash();

        static const size_t MaterialSizeInGpu;
        static const size_t MaterialSizeInGpuAligned;
    };

    /** @} */
    /** @} */

}

#include "OgreHeaderSuffix.h"

#endif
