
add_library(AdvancedAnimationPlugin SHARED
    src/AdvancedAnimationPlugin.cpp
    src/IK/IKComponent.cpp
    src/IK/FABRIKSolver.cpp
    src/Kalman/KalmanCharacterComponent.cpp
    src/Kalman/KalmanFilter6D.cpp
    src/MotionMatching/MotionMatchingComponent.cpp
    src/MotionMatching/MotionDatabase.cpp
    src/MotionMatching/MotionFeatureExtractor.cpp
)

set_target_properties(AdvancedAnimationPlugin PROPERTIES PREFIX "")

target_include_directories(AdvancedAnimationPlugin
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/external/volk
        ${CMAKE_SOURCE_DIR}/external/imgui
        ${CMAKE_SOURCE_DIR}/external/imgui/backends
        ${CMAKE_SOURCE_DIR}/external/stb
        ${CMAKE_SOURCE_DIR}/external/json/include
        ${CMAKE_SOURCE_DIR}/external/VulkanMemoryAllocator/include
        ${CMAKE_SOURCE_DIR}/external/glfw/include
        ${CMAKE_BINARY_DIR}/external/freetype/include
        ${CMAKE_SOURCE_DIR}/external/freetype/include
)

target_include_directories(AdvancedAnimationPlugin SYSTEM PRIVATE
    ${CMAKE_SOURCE_DIR}/external/fbx_sdk/include
    ${CMAKE_SOURCE_DIR}/external/fmod/include
)

target_link_libraries(AdvancedAnimationPlugin
    PRIVATE
        VelixSDK
)

add_custom_command(TARGET AdvancedAnimationPlugin POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
        "${CMAKE_BINARY_DIR}/resources/plugins"
    COMMAND ${CMAKE_COMMAND} -E rm -f
        "${CMAKE_BINARY_DIR}/resources/plugins/AdvancedAnimationPlugin.so"
        "${CMAKE_BINARY_DIR}/resources/plugins/AdvancedAnimationPlugin_d.so"
        "${CMAKE_BINARY_DIR}/resources/plugins/AdvancedAnimationPlugin.dll"
        "${CMAKE_BINARY_DIR}/resources/plugins/AdvancedAnimationPlugin_d.dll"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_FILE:AdvancedAnimationPlugin>"
        "${CMAKE_BINARY_DIR}/resources/plugins/$<TARGET_FILE_NAME:AdvancedAnimationPlugin>"
    COMMENT "Copying AdvancedAnimationPlugin to build/resources/plugins/"
)
