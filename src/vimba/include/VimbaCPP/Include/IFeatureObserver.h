/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        IFeatureObserver.h

  Description: Definition of interface AVT::VmbAPI::IFeatureObserver.

-------------------------------------------------------------------------------

  THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF TITLE,
  NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR  PURPOSE ARE
  DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, 
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED  
  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

=============================================================================*/

#ifndef AVT_VMBAPI_IFEATUREOBSERVER_H
#define AVT_VMBAPI_IFEATUREOBSERVER_H

#include <VimbaCPP/Include/VimbaCPPCommon.h>
#include <VimbaCPP/Include/SharedPointerDefines.h>
#include <VimbaCPP/Include/Feature.h>
#include <vector>

namespace AVT {
namespace VmbAPI {

class IFeatureObserver 
{
  public:
    //
    // Method:      FeatureChanged()
    //
    // Purpose:     The event handler function that gets called whenever
    //              a feature has changed
    //
    // Parameters:
    //
    // [in]     const FeaturePtr&      pFeature          The frame that has changed
    //
    IMEXPORT virtual void FeatureChanged( const FeaturePtr &pFeature ) = 0;

    //
    // Method:      IFeatureObserver destructor
    //
    // Purpose:     Destroys an instance of class IFeatureObserver
    //
    IMEXPORT virtual ~IFeatureObserver() {}

  protected:
    IMEXPORT IFeatureObserver() {}
    IMEXPORT IFeatureObserver( const IFeatureObserver& ) { /* No copy ctor */ }
};
typedef std::vector<IFeatureObserverPtr> IFeatureObserverPtrVector;

}} // namespace AVT::VmbAPI

#endif
