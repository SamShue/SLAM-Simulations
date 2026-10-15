# Least Squares

This folder contains a series of documents that build up the **least squares** method from scratch: the tool used throughout this repository (and in GraphSLAM in particular) to combine many imperfect measurements into the single best estimate.

## Contents

| Path | What it is |
|---|---|
| [Documents/](Documents) | The documentation, with the figures it uses in [Documents/images/](Documents/images) |

## Documentation

| Document | What it covers |
|---|---|
| [Linear Least Squares](Documents/linear_least_squares.md) | Linear systems of equations written as $A\mathbf{x}=\mathbf{b}$, the column space of a matrix, and reframing a solved system as a line fit through data points |

More documents will be added to this series, extending this foundation to the least squares problem itself and to weighted least squares, the method GraphSLAM itself is built on. See the [GraphSLAM](../Graph_SLAM) folder for that application.

## Regenerating the Figures

The figures in the documents are produced by scripts in [Documents/images/](Documents/images), next to the images themselves. Each script also checks the numbers that its document quotes. Run it from the root of the repository:

```
python Least_Squares/Documents/images/generate_linear_least_squares_figures.py
```

| Script | Produces the figures for |
|---|---|
| [generate_linear_least_squares_figures.py](Documents/images/generate_linear_least_squares_figures.py) | Linear Least Squares |

The script needs NumPy and Matplotlib.
