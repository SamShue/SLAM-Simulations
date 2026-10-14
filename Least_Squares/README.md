# Least Squares

This folder contains a series of documents that build up the **least squares** method from scratch: the tool used throughout this repository (and in GraphSLAM in particular) to combine many imperfect measurements into the single best estimate.

If you are new to the topic, start with the introduction. It assumes no background beyond ordinary algebra — not even what a matrix is.

## Contents

| Path | What it is |
|---|---|
| [Documents/](Documents) | The documentation, with the figures it uses in [Documents/images/](Documents/images) |

## Documentation

The documents in [Documents/](Documents) are meant to be read in order. Each one assumes the ones before it.

| Document | What it covers |
|---|---|
| [Introduction to Least Squares](Documents/least_squares_introduction.md) | What problem least squares solves, why errors are squared instead of just added up, and the formulas for the best-fit line through a small, hand-worked example |
| [Least Squares with Linear Algebra](Documents/least_squares_linear_algebra.md) | The same problem written as $A\mathbf{x}=\mathbf{b}$: a concrete case with no exact solution, the column space, projecting onto it, and the normal equations $A^\top A\hat{\mathbf{x}}=A^\top\mathbf{b}$ |

More documents will be added to this series, extending the same idea to weighted least squares, the method GraphSLAM itself is built on. See the [GraphSLAM](../Graph_SLAM) folder for that application.

## Regenerating the Figures

The figures in the documents are produced by scripts in [Documents/images/](Documents/images), next to the images themselves. Each script also checks the numbers that its document quotes. Run it from the root of the repository:

```
python Least_Squares/Documents/images/generate_intro_figures.py
python Least_Squares/Documents/images/generate_linear_algebra_figures.py
```

| Script | Produces the figures for |
|---|---|
| [generate_intro_figures.py](Documents/images/generate_intro_figures.py) | The introduction |
| [generate_linear_algebra_figures.py](Documents/images/generate_linear_algebra_figures.py) | Least Squares with Linear Algebra |

The scripts need NumPy and Matplotlib.
