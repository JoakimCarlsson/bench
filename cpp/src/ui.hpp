#pragma once

#include <cstdint>
#include <vector>

#include "ui_container.hpp"
#include "ui_control.hpp"
#include "ui_draw.hpp"
#include "ui_widgets.hpp"

namespace bench {

/// The controls the frames of the `ui` kernel change, picked while the screen is built.
struct UiTargets {
    std::vector<ui::Label*> labels;
    std::vector<ui::Button*> buttons;
    std::vector<ui::LineEdit*> edits;
    std::vector<ui::Control*> resizable;
    std::vector<ui::Slider*> sliders;
};

/// The engine's game-facing UI: a tree of about sixteen thousand controls
/// (an editor screen with a toolbar, a long hierarchy, a tabbed asset grid
/// and an inspector with long property lists) laid out and drawn into a
/// batched vertex, index and command list. Each of the frames resizes the
/// root, changes the text and minimum size of some controls so the
/// invalidation climbs the tree, runs the layout pass, draws the whole
/// tree, and folds every control rect and all draw data into the checksum.
class Ui {
public:
    static constexpr const char* name = "ui";

    Ui();
    uint64_t run();

private:
    /// Changes the text, minimum sizes and slider values of the targets for `frame`.
    void mutate(uint32_t frame);

    ui::Font font_;
    ui::DrawCanvas canvas_;
    ui::Context context_;
    UiTargets targets_;
};

} // namespace bench
