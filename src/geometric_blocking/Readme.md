

# Feasible Region from Blocking Direction

The following is an exhaustive list of return behavior from the function

`compute_feasible_region_from_block_dir(block_dirs):`



## Planar - Circle Example

Feasible Region: Plane

Represented by: Two Orthogonal Vectors (both in lin_set)

```python
block_dirs = [[1.0, 0.0, 0.0], [-1.0, 0.0, 0.0]]
rays = [[0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
lin_set = [0, 1]
```

```python
block_dirs = [[0.0, 1.0, 0.0], [0.0, -1.0, 0.0]]
rays = [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
lin_set = [0, 1]
```

## Planar - Half Circle Example

Feasible Region: Planar Half Circle

Represented by: Two Orthogonal Vectors (only one of them in lin_set)

```python
block_dirs = [[1.0, -0.0, 0.0], [-1.0, -0.0, 0.0], [0.0, 1.0, 0.0]]
rays = [[0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
lin_set = [1]
```



## Planar - Fan Shape Example (< 180deg)

Feasible Region: Planar, fan shaped region. 

(Note: Refer to the planar half-circle example above for fan = 180deg. Note it is not possible for fan to be larger than 180deg.)

Represented by: Two Vectors  (both not in lin_set)

```python
# Example bounded by 5 Vectors
block_dirs = [[1.0, -0.0, -0.3], [-1.0, -0.0, -0.3], [0.0, 1.0, 0.0], [-0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]
rays = [[0.28734788556634538, 0.0, 0.95782628522115132], [-0.28734788556634538, 0.0, 0.95782628522115132]]
lin_set = []
```

```python
# Example bounded by 4 Vectors
block_dirs = [[1.0, -0.0, 0.7], [0.0, 1.0, 0.0], [-0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]
rays = [[-1.0, 0.0, 0.0], [-0.57346234436332832, 0.0, 0.81923192051904048]]
lin_set = []
```



## Linear - Bi-direction Example

Feasible Region: Single bidirectional linear movement.

Represented by: One Vector (in lin_set)

```python
block_dirs = [[1.0, -0.0, 0.0], [-1.0, -0.0, 0.0], [0.0, 1.0, 0.0], [0.0, -1.0, 0.0]]
rays = [[0.0, 0.0, 1.0]]
lin_set = [0]
```

```python
block_dirs = [[1.0, -0.3, 0.0], [-1.0, -0.3, 0.0], [0.0, 1.0, 0.0]]
rays = [[0.0, 0.0, 1.0]]
lin_set = [0]
```

## Linear - Single Direction Example

Feasible Region: Only one direction.

Represented by: One Vector (not in lin_set)

```python
block_dirs = [[1.0, -0.0, 0.0], [-1.0, -0.0, 0.0], [0.0, 1.0, 0.0], [-0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]
rays = [[0.0, 0.0, 1.0]]
lin_set = []
```

## 3D - Half Sphere Example

Feasible Region: Half sphere space

Represented by: Three Vectors (two vectors in lin_set)

Note the two vector in lin_set represent the boundary plane, the third vector is the normal of the plane.

```python
block_dirs = [[0.0, 0.0, 1.0]]
rays = [[0.0, 0.0, -1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
lin_set = [1, 2]
```





## 3D - Two Face Wedge Example

Feasible Region: Wedge shape space with 2 bounded face

Represented by: Three Vectors (one vector in lin_set)

```python
block_dirs = [[1.0, 0.5, -0.0], [-1.0, 0.5, -0.0]]
rays = [[-0.44721359549995793, -0.89442719099991586, 0.0], [0.44721359549995793, -0.89442719099991586, 0.0], [0.0, 0.0, 1.0]]
lin_set = [2]
```



## 3D - Polygonal Face Wedge Example

Feasible Region: Wedge shape space with more than 2 bounded face

Represented by: Three or more vectors (none in lin_set)

```python
# Example bounded by 4 vectors. Result is 4 face polygon boundary
block_dirs = [(0.4, 1.0, -0.0), (-0.9, -0.0, -0.0), (0.0, 1.0, 1.0), (-0.0, 0.5, -1.0)]
rays = [[0.8703882797784892, -0.3481553119113957, 0.3481553119113957], [0.9128709291752769, -0.36514837167011072, -0.18257418583505536], [0.0, -0.89442719099991586, -0.44721359549995793], [0.0, -0.70710678118654746, 0.70710678118654746]]
lin_set = []

```

# Interpretation of result

A quick way to interpret the result is to count how many `rays` and `lin_set` are returned:

| len(rays) | len(lin_set) | Free Region Type           | Details                          |
| --------- | ------------ | -------------------------- | -------------------------------- |
| 1         | 0            | Linear                     | Single Direction                 |
| 1         | 1            | Linear                     | Bi-Direction                     |
| 2         | 0            | Planar                     | Fan Shape (<180deg)              |
| 2         | 1            | Planar                     | Half Circle (=180deg)            |
| 2         | 2            | Planar                     | Full Circle (=360deg)            |
| 3 or more | 0            | 3D                         | Polygonal Face Wedge             |
| 3         | 1            | 3D                         | Two Face Wedge (<180 face angle) |
| 3         | 2            | 3D                         | Half Sphere                      |
| 3         | 3            | I think this never happens |                                  |

