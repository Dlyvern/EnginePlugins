#include "AdvancedAnimationPlugin/IK/IKComponent.hpp"
#include "AdvancedAnimationPlugin/Kalman/KalmanCharacterComponent.hpp"
#include "AdvancedAnimationPlugin/MotionMatching/MotionMatchingComponent.hpp"

#include "VelixSDK/EditorPlugin.hpp"

#include "Engine/Components/AnimatorComponent.hpp"
#include "Engine/Components/CharacterMovementComponent.hpp"
#include "Engine/Components/SkeletalMeshComponent.hpp"
#include "Engine/Entity.hpp"
#include "Engine/PluginSystem/ComponentRegistry.hpp"

#include <imgui.h>

namespace elix::plugin::advancedanim
{

    class AdvancedAnimationPlugin final : public elix::sdk::IEditorPlugin
    {
    public:
        const char *getName() const override { return "AdvancedAnimationPlugin"; }
        const char *getVersion() const override { return "1.0.0"; }

        void onLoad() override
        {
            using namespace elix::engine;
            auto &reg = ComponentRegistry::instance();

            reg.registerComponent("IK Chain", "Animation",
                                  [](Entity *entity, Scene * /*scene*/, ComponentAddContext &ctx)
                                  {
                                      if (!entity->getComponent<AnimatorComponent>())
                                      {
                                          if (ctx.showWarning)
                                              ctx.showWarning("IK Chain requires an AnimatorComponent on this entity.");
                                          ctx.closePopup = false;
                                          return;
                                      }
                                      entity->addComponent<IKComponent>();
                                  });

            reg.registerComponent("Kalman Character", "Animation",
                                  [](Entity *entity, Scene * /*scene*/, ComponentAddContext &ctx)
                                  {
                                      if (!entity->getComponent<CharacterMovementComponent>())
                                      {
                                          if (ctx.showWarning)
                                              ctx.showWarning("Kalman Character requires a CharacterMovementComponent on this entity.");
                                          ctx.closePopup = false;
                                          return;
                                      }
                                      entity->addComponent<KalmanCharacterComponent>();
                                  });

            reg.registerComponent("Motion Matching", "Animation",
                                  [](Entity *entity, Scene * /*scene*/, ComponentAddContext &ctx)
                                  {
                                      if (!entity->getComponent<SkeletalMeshComponent>())
                                      {
                                          if (ctx.showWarning)
                                              ctx.showWarning("Motion Matching requires a SkeletalMeshComponent on this entity.");
                                          ctx.closePopup = false;
                                          return;
                                      }
                                      entity->addComponent<MotionMatchingComponent>();
                                  });
        }

        void onUnload() override {}

        void onEditorFrame(elix::sdk::EditorContext &ctx) override
        {
            if (!m_showWindow)
                return;

            ImGui::SetNextWindowSize(ImVec2(480, 400), ImGuiCond_FirstUseEver);
            if (!ImGui::Begin("Advanced Animation Tools", &m_showWindow))
            {
                ImGui::End();
                return;
            }

            if (ImGui::BeginTabBar("AdvAnimTabs"))
            {
                if (ImGui::BeginTabItem("IK Chains"))
                {
                    drawIKEditor(ctx);
                    ImGui::EndTabItem();
                }
                if (ImGui::BeginTabItem("Kalman Debug"))
                {
                    drawKalmanDebug(ctx);
                    ImGui::EndTabItem();
                }
                if (ImGui::BeginTabItem("Motion DB"))
                {
                    drawMotionMatchingEditor(ctx);
                    ImGui::EndTabItem();
                }
                ImGui::EndTabBar();
            }

            ImGui::End();
        }

        const char *getToolbarButtonLabel() const override { return "Anim Tools"; }
        void toggleToolbarWindow() override { m_showWindow = !m_showWindow; }

    private:
        void drawIKEditor(elix::sdk::EditorContext &ctx)
        {
            auto *entity = ctx.selectedEntity;
            if (!entity)
            {
                ImGui::TextDisabled("No entity selected.");
                return;
            }

            auto *ik = entity->getComponent<IKComponent>();
            if (!ik)
            {
                ImGui::TextDisabled("Selected entity has no IK Chain component.");
                return;
            }

            ImGui::Text("IK Chains on: %s", entity->getName().c_str());
            ImGui::Separator();

            for (auto &chain : const_cast<std::vector<IKChain> &>(ik->getChains()))
            {
                if (ImGui::TreeNode(chain.name.empty() ? "(unnamed)" : chain.name.c_str()))
                {
                    ImGui::SliderFloat("Weight", &chain.weight, 0.f, 1.f);
                    ImGui::SliderInt("Max Iterations", reinterpret_cast<int *>(&chain.maxIterations), 1, 32);
                    ImGui::SliderFloat("Tolerance (m)", &chain.tolerance, 0.001f, 0.1f);

                    ImGui::Text("Bones: %zu", chain.boneNames.size());
                    for (size_t i = 0; i < chain.boneNames.size(); ++i)
                        ImGui::Text("  [%zu] %s", i, chain.boneNames[i].c_str());

                    ImGui::TreePop();
                }
            }

            if (ImGui::Button("Refresh Hook"))
                ik->registerHook();
        }

        void drawKalmanDebug(elix::sdk::EditorContext &ctx)
        {
            auto *entity = ctx.selectedEntity;
            if (!entity)
            {
                ImGui::TextDisabled("No entity selected.");
                return;
            }

            auto *k = entity->getComponent<KalmanCharacterComponent>();
            if (!k)
            {
                ImGui::TextDisabled("Selected entity has no Kalman Character component.");
                return;
            }

            ImGui::Text("Kalman Character on: %s", entity->getName().c_str());
            ImGui::Separator();

            const glm::vec3 pos = k->getSmoothedPosition();
            const glm::vec3 vel = k->getSmoothedVelocity();
            ImGui::Text("Smoothed Position:  %.2f  %.2f  %.2f", pos.x, pos.y, pos.z);
            ImGui::Text("Smoothed Velocity:  %.2f  %.2f  %.2f", vel.x, vel.y, vel.z);
            ImGui::Text("Horizontal Speed:   %.3f m/s", k->getSmoothedHorizontalSpeed());

            ImGui::Separator();
            ImGui::TextDisabled("Tune noise via setProcessNoise() / setMeasurementNoise() in a script.");
        }

        void drawMotionMatchingEditor(elix::sdk::EditorContext &ctx)
        {
            auto *entity = ctx.selectedEntity;
            if (!entity)
            {
                ImGui::TextDisabled("No entity selected.");
                return;
            }

            auto *mm = entity->getComponent<MotionMatchingComponent>();
            if (!mm)
            {
                ImGui::TextDisabled("Selected entity has no Motion Matching component.");
                return;
            }

            ImGui::Text("Motion Matching on: %s", entity->getName().c_str());
            ImGui::Separator();

            const MotionFeature *feat = mm->getCurrentMatchedFeature();
            if (feat)
            {
                ImGui::Text("Matched clip:  %u  frame:  %u  time: %.3f s",
                            feat->clipIndex, feat->frameIndex, feat->timeInClip);
                const glm::vec3 hipV = feat->hipVelocity;
                ImGui::Text("Hip velocity:  %.2f  %.2f  %.2f", hipV.x, hipV.y, hipV.z);
            }
            else
            {
                ImGui::TextDisabled("No match yet — set a database and animations.");
            }

            ImGui::Separator();
            const glm::vec3 animVel = mm->getCurrentAnimatedVelocity();
            ImGui::Text("Animated velocity: %.2f  %.2f  %.2f", animVel.x, animVel.y, animVel.z);
        }

        bool m_showWindow{false};
    };

} // namespace elix::plugin::advancedanim

REGISTER_EDITOR_PLUGIN(elix::plugin::advancedanim::AdvancedAnimationPlugin)
