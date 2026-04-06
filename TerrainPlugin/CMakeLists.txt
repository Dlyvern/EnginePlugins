
add_library(TerrainPlugin SHARED
    src/TerrainTools.cpp
    src/TerrainPlugin.cpp
)

set_target_properties(TerrainPlugin PROPERTIES PREFIX "")

target_include_directories(TerrainPlugin
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        # Engine transitive headers not propagated through VelixSDK
        ${CMAKE_SOURCE_DIR}/external/volk
        ${CMAKE_SOURCE_DIR}/external/imgui
        ${CMAKE_SOURCE_DIR}/external/imgui/backends
        ${CMAKE_SOURCE_DIR}/external/stb
        ${CMAKE_SOURCE_DIR}/external/json/include
        ${CMAKE_SOURCE_DIR}/external/tinyobjloader
        ${CMAKE_SOURCE_DIR}/external/SPIRV-Cross
        ${CMAKE_SOURCE_DIR}/external/shaderc/libshaderc/include
        ${CMAKE_SOURCE_DIR}/external/VulkanMemoryAllocator/include
        ${CMAKE_SOURCE_DIR}/external/glfw/include
        ${CMAKE_BINARY_DIR}/external/freetype/include
        ${CMAKE_SOURCE_DIR}/external/freetype/include
)

target_include_directories(TerrainPlugin SYSTEM PRIVATE
        ${CMAKE_SOURCE_DIR}/external/fbx_sdk/include
        ${CMAKE_SOURCE_DIR}/external/fmod/include
)

target_link_libraries(TerrainPlugin
    PRIVATE
        VelixSDK
)

# Copy the built plugin into build/resources/plugins/ so the engine finds it.
add_custom_command(TARGET TerrainPlugin POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
        "${CMAKE_BINARY_DIR}/resources/plugins"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_FILE:TerrainPlugin>"
        "${CMAKE_BINARY_DIR}/resources/plugins/$<TARGET_FILE_NAME:TerrainPlugin>"
    COMMENT "Copying TerrainPlugin to build/resources/plugins/"
)
