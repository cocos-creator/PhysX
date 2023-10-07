#pragma once

#if defined(PX_SUPPORT_PVD) && PX_SUPPORT_PVD
    #define PX_WEB_BINDINGS_PVD 1
#else
    #define PX_WEB_BINDINGS_PVD 0
#endif

void pvdBindings();
