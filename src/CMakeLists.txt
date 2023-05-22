set(
        LIB_SOURCES
        SickPLS.cc
        SickPLSMessage.cc
        SickPLSBufferMonitor.cc
)

set(
        EXAMPLE_SOURCES
        example.cpp
)

set(
        INCLUDES
        "./"
)


add_library(sickpls SHARED ${LIB_SOURCES})
add_library(sickpls::sickpls ALIAS sickpls)
target_include_directories(sickpls PUBLIC ${INCLUDES})

add_executable(example ${EXAMPLE_SOURCES})
target_include_directories(example PUBLIC ${INCLUDES})
target_link_libraries(example PRIVATE sickpls)