# Task figure archive

This is the repository's home for reusable **maps, plots and diagrams**, organized by task. Use it when preparing project documentation, presentations or thesis figures.

## Figure index

| Task / collection | Saved figures | Location |
|---|---|---|
| SLAM-003E — manual GLIM mapping | Top-down map PNG, motion-speed PNG, interactive map HTML | [Gallery and captions](SLAM-003E/README.md) |
| SLAM-003D-R — static route review | Swept-footprint comparison PNG | [Figure record](SLAM-003D-R/README.md) |
| TF-001 — historical TF inspection | Frame-tree PDF | [Figure record](TF-001/README.md) |
| Architecture diagrams | Existing system, simulation, subsystem and switching diagrams | [Collection index](ARCHITECTURE/README.md) |

Legacy files remain at their original paths so existing documentation links continue to work. New task exports belong inside their task folder.

## After each task

1. Create `evidence/figures/<TASK-ID>/` when there are figures to save.
2. Copy the actual exports into it and preserve the original files. For ChatGPT-generated exports, download or locate the files first; a chat preview alone is not an archived asset.
3. Create a task `README.md` using [TEMPLATE.md](TEMPLATE.md). Record a caption, source run/data, date, frames/units and any interpretation limits. Include editable source or plotting code when available.
4. Add links to this index and the relevant topic document. Link only files that exist.
5. When copying supplied exports, check source/destination SHA256 and record provenance, following the [SLAM-003E asset manifest](SLAM-003E/assets.json).

Preferred filenames:

```text
YYYY-MM-DD_TASK-ID_subject_v01.png
YYYY-MM-DD_TASK-ID_subject_v01.svg
YYYY-MM-DD_TASK-ID_subject_v01.pdf
YYYY-MM-DD_TASK-ID_subject_v01.html
```

Keep the same stem for related export formats. For a revision, add `v02` rather than replacing `v01`. PNG is convenient for previews; SVG/PDF preserve vector graphics when the source supports them. Preserve a self-contained interactive HTML export when available, alongside a static preview. Source files may live in a task-local `source/` directory.

This archive holds documentation figures, not bulk datasets. Native GLIM dumps, bags, compressed experiment packages and raw logs stay in the private experiment directories; record the source run ID here. Storage/visual inspection does not establish accuracy or validate the underlying experiment.

## Open the latest map folder in Windows

From WSL:

```bash
cd /home/feras/clearpath_ws/evidence/figures/SLAM-003E
explorer.exe .
```

Double-click the HTML map to open your exported interactive visualization in a browser. It was archived without running its JavaScript during import. No commit or push is implied by saving figures here.

**SLAM-004C — partial manual localization validation (2026-11-21):**
[Figure gallery and provenance](SLAM-004C/README.md) contains three diagnostic
plots in PNG/SVG with CSV data and editable source: aligned base trajectories,
synchronized pose errors and actual command limits. The trial was aborted at
the reviewed region boundary; the gallery records that limitation and the
unestimated shutdown tail.
