cmake_minimum_required(VERSION 3.10.0)

project(provizio_dds_idls)

include(ExternalProject)

set(PROVIZIO_DDS_IDLS_GITHUB_PROJECT "provizio/provizio_dds_idls")

if(NOT PROVIZIO_DDS_IDLS_VERSION)
    set(PROVIZIO_DDS_IDLS_VERSION "develop")
endif(NOT PROVIZIO_DDS_IDLS_VERSION)

set(PROVIZIO_DDS_IDLS_SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/src")
ExternalProject_Add(libprovizio_dds_idls
    GIT_REPOSITORY "https://github.com/${PROVIZIO_DDS_IDLS_GITHUB_PROJECT}.git"
    GIT_TAG "${PROVIZIO_DDS_IDLS_VERSION}"
    SOURCE_DIR "${PROVIZIO_DDS_IDLS_SOURCE_DIR}"
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    CMAKE_ARGS "-DSKIP_CODE_GENERATION=TRUE"
)

add_custom_target(get_provizio_dds_idls ALL)
add_dependencies(get_provizio_dds_idls libprovizio_dds_idls)
