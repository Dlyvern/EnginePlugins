#include "TerrainPlugin/TerrainTools.hpp"

#include "VelixSDK/EditorPlugin.hpp"

#include <memory>

namespace elix
{
    namespace plugin
    {
        namespace terrain
        {

            class TerrainPlugin final : public elix::sdk::IEditorPlugin
            {
            public:
                const char *getName() const override { return "TerrainPlugin"; }
                const char *getVersion() const override { return "1.0.0"; }
                void onLoad() override {}
                void onUnload() override { m_terrainTools.cancelBrushStroke(); }

                void onEditorFrame(elix::sdk::EditorContext &ctx) override
                {
                    if (ctx.scene)
                        m_terrainTools.setScene(std::shared_ptr<elix::engine::Scene>(ctx.scene, [](elix::engine::Scene *) {}));

                    if (ctx.projectRootPath)
                        m_terrainTools.setProjectRootPath(*ctx.projectRootPath);

                    m_terrainTools.draw(&m_showWindow);

                    if (m_showWindow)
                    {
                        ctx.wantsBrushInput = true;

                        if (ctx.brushStrokeActive)
                        {
                            m_terrainTools.applyBrushStrokeFromNdc(
                                ctx.brushNdcPosition,
                                ctx.editorCamera,
                                ctx.selectedEntity,
                                ctx.deltaTime,
                                ctx.brushStrokeStart);
                        }
                        else
                            m_terrainTools.cancelBrushStroke();
                    }
                    else
                        m_terrainTools.cancelBrushStroke();
                }

                const char *getToolbarButtonLabel() const override { return "Terrain Tools"; }
                void toggleToolbarWindow() override { m_showWindow = !m_showWindow; }

            private:
                TerrainTools m_terrainTools;
                bool m_showWindow{false};
            };

        } // namespace terrain
    } // namespace plugin
} // namespace elix

REGISTER_EDITOR_PLUGIN(elix::plugin::terrain::TerrainPlugin)