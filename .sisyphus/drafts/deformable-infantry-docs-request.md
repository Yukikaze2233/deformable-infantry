# Draft: Deformable Infantry Docs

## Requirements (confirmed)
- Create a documentation set named `deformable_infantry_docs` at the workspace/RMCS root.
- Reference the format/style of `rmcs_notebook`.
- Use Typst, not Markdown.
- Make the docs math-first, with strict derivations, formulas, and explanations similar to `rmcs_notebook` algorithm chapters.
- Cover these topics:
  - deformable infantry v1 screw kinematics and control
  - deformable infantry v2 direct-drive motor control
  - active suspension
  - deformable infantry omni kinematics
  - deformable infantry v2 steering kinematics

## Technical Decisions
- Current mode cannot write to workspace root; only `.sisyphus/` draft artifacts are allowed.
- Produce a root-ready Typst package structure and content in `.sisyphus/drafts/` first.
- Align structure with local `rmcs_notebook/` repository layout: `main.typ + template + chapters/.../*.typ`.
- Use formula-heavy chapter style similar to `rmcs_notebook/chapters/algorithm/omni_wheel.typ` and `steering_wheel.typ`.

## Research Findings
- `plan.md`: defines 3-layer active suspension target and ADRC-based joint-local architecture.
- `code-plan.md`: describes ADRC-oriented implementation direction, z2/z3 exposure, and config evolution.
- `todo.md`: records current implementation progress, preload issues, suspension-first restoration, and current pain points.
- `deformable_infantry_omni_active_suspension_flow.md`: documents actual data/control flow for omni active suspension.
- `rmcs_notebook/main.typ`: book-style root with chapter includes.
- `rmcs_notebook/template/template.typ`: shared Typst template with equation helpers such as `num_eq`, `vb`, `dcases`, Chinese typography config, TOC/index support.
- `rmcs_notebook/chapters/algorithm/omni_wheel.typ` and `steering_wheel.typ`: math-first algorithm chapters using explicit modeling assumptions, coordinate definitions, least-squares / pseudo-inverse / force derivations, and stepwise explanation.

## Open Questions
- Whether the final deliverable should mirror `rmcs_notebook` as a standalone mini-book (`main.typ`, local `template`, `chapters/`) or as a simpler `deformable_infantry_docs/` folder with one `main.typ` and several chapter `.typ` files.
- Whether to include rendered figures/diagrams placeholders now, or keep the first version purely text + formulas.

## Scope Boundaries
- INCLUDE: Typst directory structure proposal, chapter breakdown, formula-first drafts, root-ready content.
- EXCLUDE: direct creation of root files while still in planning-only mode.
