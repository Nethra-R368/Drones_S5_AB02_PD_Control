# AB02 documentation

| Folder / file | Content |
|---|---|
| `complete_documentation/AB02_Drone_Documentation.pdf` | Full report: controller in code, reproduction of the paper (Part 1, Part 2), Monte-Carlo and sensitivity studies, findings, Gazebo X3 integration and fault analysis, X3 results (E0–E5), MuJoCo/browser demos, MATLAB toy model guide, claims register |
| `mathematical_models/Mathematical_Models.pdf` | Learning guide: every equation (paper blocks 1–6, X3 blocks 7–10) with a boxed statement, symbol table with dimensions and units, derivation, hand-checkable toy example and "in our project" explanation |
| `mathematical_models/figures/flow_paper.*`, `flow_x3.*` | The two equation-flow diagrams (TikZ sources + PDF/PNG) |
| `figures/` | Every figure, written by `paper_reproduction.py` and `x3_sim/run_x3_experiments.py` (plus exported MuJoCo notebook outputs) |
| `results/metrics.md/.json`, `results/x3_metrics.md/.json` | Every number of the report |
| `common_preamble.tex` | Shared LaTeX preamble of both documents |
| `make_crosscheck.py` | Builds the Python-vs-MATLAB table from `toy_model/results_log.txt` |

Rebuild (TeX Live or MiKTeX): in `mathematical_models/figures/` run `pdflatex flow_paper.tex` and `pdflatex flow_x3.tex`;
then `pdflatex` twice in `mathematical_models/` (`Mathematical_Models.tex`) and in `complete_documentation/`
(`AB02_Drone_Documentation.tex`). Code listings are read directly from the repository files and the MATLAB figures from
`../toy_model/figures/`, so the PDFs always show the current code and results.
