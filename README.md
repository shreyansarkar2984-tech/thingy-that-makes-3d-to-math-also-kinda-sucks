# 3D Equation Tool

This is a native C++ desktop app that loads an OBJ model and fits a mathematical representation to it.

It is designed as a practical approximation tool:

- Loads `OBJ` meshes.
- Renders the model in a lightweight OpenGL viewport.
- Supports `plane`, `sphere`, `cylinder`, `quadric`, `cubic`, and `RBF` fitting.
- Keeps the main UI minimal with `Insert OBJ`, `Analyze`, and `Marching Cubes`, and moves advanced options into a `Settings` tab.
- Uses background fitting so the UI remains responsive on modest hardware.

There are now three desktop builds:

- `normal.exe`: keeps the OBJ at full resolution.
- `light.exe`: automatically downsamples large OBJ files to `1500` vertices before analysis.
- `heavy.exe`: matches the normal build, but uses GPU compute for RBF marching-cubes field sampling when OpenGL 4.3 compute shaders are available.

## What "convert to an equation" means here

Arbitrary 3D meshes usually cannot be represented exactly by a single short equation.

Depending on the selected conversion mode, the app can produce:

- A plane
- A sphere
- A cylinder
- A quadric or cubic implicit fit
- An RBF surface

For arbitrary shapes, the fallback is an approximate implicit surface:

`f(u, v, w) = 0`

where:

- `u = (x - cx) / s`
- `v = (y - cy) / s`
- `w = (z - cz) / s`

`cx`, `cy`, `cz`, and `s` are computed from the mesh bounds so the fit is numerically stable.

## Build

The workspace already has `clang++`, so you can build with:

```bat
build.bat
```

The executables will be written to a fresh run folder inside `build`, for example:

- `build\run_30204\normal.exe`
- `build\run_30204\light.exe`
- `build\run_30204\heavy.exe`

The main source files live in `code\main.cpp`, `code\model.cpp`, `code\analysis.cpp`, `code\gpu.cpp`, and `code\view.cpp`.

## Notes

- Smooth or organic meshes generally fit better than sharp-edged models.
- A cubic fit usually captures more detail than a quadratic fit.
- Organic meshes generally fit best with the `RBF` method.
- The implicit polynomial residuals are algebraic fit metrics, not exact geometric distances.
- `Marching Cubes` builds a reconstructed preview mesh from the current fit and shows it in the viewport.
- The GPU build accelerates the marching-cubes field sampling stage for `RBF` fits and falls back to CPU sampling if GPU compute is unavailable.
