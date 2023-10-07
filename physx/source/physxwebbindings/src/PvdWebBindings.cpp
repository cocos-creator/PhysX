#include "PxWebBindings.h"
#include "PxPhysicsAPI.h"
#include <emscripten.h>
#include <emscripten/bind.h>

using namespace physx;
using namespace emscripten;

class PxPvdTransportWrapper: public wrapper<PxPvdTransport> {
public:
    EMSCRIPTEN_WRAPPER(PxPvdTransportWrapper)

    bool connect() {
        return call<bool>("connect");
    }

	void disconnect() {
        return call<void>("disconnect");
    }

    bool isConnected() {
        return call<bool>("isConnected");
    }

    bool write(const uint8_t* inBytes, uint32_t inLength) {
        return call<bool>("write", val(typed_memory_view(inLength, inBytes)));
    }

    PxPvdTransport& lock() {
        call<void>("lock");
        return *this;
    }

    void unlock() {
        return call<void>("unlock");
    }

    void flush() {
        return call<void>("flush");
    }

    uint64_t getWrittenDataSize() {
        return call<uint64_t>("getWrittenDataSize");
    }

    void release() {
        return call<void>("release");
    }
};

void pvdBindings() {
    constant("PX_WEB_BINDINGS_PVD", !!PX_WEB_BINDINGS_PVD);

#if !PX_WEB_BINDINGS_PVD
    // Even if PVD is not involved in bindings.
    // We still need to export an empty `PxPvd` class binding.
    // Since `PxCreatePhysics` depends on that.
    class_<PxPvd>("PxPvd");
#else
    class_<PxPvd>("PxPvd")
       .function("connect", &PxPvd::connect);

    function("PxCreatePvd", &PxCreatePvd, allow_raw_pointers());

    class_<PxPvdInstrumentationFlags>("PxPvdInstrumentationFlags").constructor<int>();
    enum_<PxPvdInstrumentationFlag::Enum>("PxPvdInstrumentationFlag")
      .value("eALL", PxPvdInstrumentationFlag::Enum::eALL)
      .value("eDEBUG", PxPvdInstrumentationFlag::Enum::eDEBUG)
      .value("ePROFILE", PxPvdInstrumentationFlag::Enum::ePROFILE)
      .value("eMEMORY", PxPvdInstrumentationFlag::Enum::eMEMORY);

    class_<PxPvdSceneClient>("PxPvdSceneClient")
        .function("setScenePvdFlag", &PxPvdSceneClient::setScenePvdFlag);

    enum_<PxPvdSceneFlag::Enum>("PxPvdSceneFlag")
        .value("eTRANSMIT_CONSTRAINTS", PxPvdSceneFlag::Enum::eTRANSMIT_CONSTRAINTS)
        .value("eTRANSMIT_CONTACTS", PxPvdSceneFlag::Enum::eTRANSMIT_CONTACTS)
        .value("eTRANSMIT_SCENEQUERIES", PxPvdSceneFlag::Enum::eTRANSMIT_SCENEQUERIES);

    class_<PxPvdTransport>("PxPvdTransport")
        .allow_subclass<PxPvdTransportWrapper>("PxPvdTransportWrapper")
        ;
#endif
}

namespace emscripten {
namespace internal {
template <> void raw_destructor<PxPvd>(PxPvd *) { /* do nothing */ }
template <> void raw_destructor<PxPvdTransport>(PxPvdTransport *) { /* do nothing */ }
template <> void raw_destructor<PxPvdSceneClient>(PxPvdSceneClient *) { /* do nothing */ }
}
}
