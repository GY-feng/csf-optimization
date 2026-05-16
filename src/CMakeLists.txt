set(CSF_SOURCES
    c2cdist.cpp
    c2cdistSoA.cpp
    Cloth.cpp
    ClothSoA.cpp
    Constraint.cpp
    CSF.cpp
    Particle.cpp
    point_cloud.cpp
    Rasterization.cpp
    RasterizationSoA.cpp
    XYZReader.cpp
)

set(CSF_HEADERS
    c2cdist.h
    c2cdistSoA.h
    Cloth.h
    ClothSoA.h
    Constraint.h
    CSF.h
    Particle.h
    point_cloud.h
    Rasterization.h
    RasterizationSoA.h
    Vec3.h
    XYZReader.h
)

if(CSF_ENABLE_CUDA)
    list(APPEND CSF_SOURCES
        cuda/ClothCuda.cu
        cuda/CudaReduction.cu
    )
    list(APPEND CSF_HEADERS
        cuda/ClothCuda.cuh
        cuda/CudaUtils.cuh
    )
endif()

add_library(CSF ${CSF_SOURCES} ${CSF_HEADERS})

if(MSVC)
    target_compile_options(CSF PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/W4>)
else()
    target_compile_options(CSF PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -pedantic>)
endif()

target_include_directories(CSF PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if(OpenMP_CXX_FOUND)
    target_link_libraries(CSF PUBLIC OpenMP::OpenMP_CXX)
endif()

if(CSF_ENABLE_CUDA)
    target_include_directories(CSF PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/cuda)
    target_link_libraries(CSF PUBLIC CUDA::cudart)
    set_target_properties(CSF PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
endif()

install(TARGETS CSF LIBRARY DESTINATION lib)
install(TARGETS CSF ARCHIVE DESTINATION lib)
install(FILES ${CSF_HEADERS} DESTINATION include)
