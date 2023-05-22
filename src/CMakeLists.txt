set(
        SOURCES
        SickPLS.cc
        SickPLSMessage.cc
        SickPLSBufferMonitor.cc
)

set(
        INCLUDES
        "./"
)


add_library(sickpls SHARED ${SOURCES})
add_library(sickpls::sickpls ALIAS sickpls)
target_include_directories(sickpls PUBLIC ${INCLUDES})
