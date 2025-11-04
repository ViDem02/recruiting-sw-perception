import math
import random

def generate_cone_ply(
        filename,
        x0=0.0,
        y0=0.0,
        height=1.0,
        radius=1.0,
        num_base_vertices=100,
        num_surface_points=1000,
        include_base=True
):
    """
    Generate a PLY file for a cone with random points on its surface.

    The cone stands upright along the Y axis, with:
      - base centered at (x0, y0, 0)
      - apex at (x0, y0 + height, 0)

    Parameters
    ----------
    filename : str
        Output .ply file path.
    x0, y0 : float
        Base center coordinates.
    height : float
        Cone height.
    radius : float
        Radius of the base.
    num_base_vertices : int
        Number of vertices used for the base circle in the mesh.
    num_surface_points : int
        Number of random points to sample on the cone surface.
    include_base : bool
        If True, include random points on the base.
    """

    # Apex vertex
    apex = (x0, y0 + height, 0.0)

    # Base circle vertices
    base_vertices = []
    for i in range(num_base_vertices):
        theta = 2 * math.pi * i / num_base_vertices
        x = x0 + radius * math.cos(theta)
        z = radius * math.sin(theta)
        base_vertices.append((x, y0, z))

    vertices = [apex] + base_vertices

    # Triangular side faces
    faces = []
    for i in range(num_base_vertices):
        i_next = 1 + (i + 1) % num_base_vertices
        faces.append([0, 1 + i, i_next])

    # Base face (polygon)
    faces.append(list(range(1, num_base_vertices + 1)))

    # --- Random point generation ---
    def random_point_on_cone_surface():
        """
        Uniform sampling on the lateral surface of a cone.
        Derived from surface area parameterization.
        """
        # Random along the slant height (s = 0→1)
        s = math.sqrt(random.random())  # sqrt for uniform area
        theta = 2 * math.pi * random.random()

        # Radius shrinks linearly from base to apex
        r = radius * (1 - s)

        # Compute position
        x = x0 + r * math.cos(theta)
        z = r * math.sin(theta)
        y = y0 + s * height
        return (x, y, z)

    # Side surface points
    surface_points = [random_point_on_cone_surface() for _ in range(num_surface_points)]

    # Optional random points on the *base* (flat circle at y = y0)
    base_points = []
    if include_base:
        for _ in range(num_surface_points // 4):
            r = radius * math.sqrt(random.random())
            theta = random.random() * 2 * math.pi
            x = x0 + r * math.cos(theta)
            z = r * math.sin(theta)
            y = y0  # bottom of the cone
            base_points.append((x, y, z))

    # Combine all vertices
    all_points = vertices + surface_points + base_points

    # --- Write PLY ---
    with open(filename, 'w') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write("comment Cone with random surface points (bottom base)\n")
        f.write(f"element vertex {len(all_points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write(f"element face {len(faces)}\n")
        f.write("property list uchar int vertex_indices\n")
        f.write("end_header\n")

        # Write vertex coordinates
        for v in all_points:
            f.write(f"{v[0]} {v[1]} {v[2]}\n")

        # Write faces (for the cone mesh only)
        for face in faces:
            f.write(f"{len(face)} {' '.join(map(str, face))}\n")

    print(f"✅ Generated cone with {len(all_points)} vertices → {filename}")


if __name__ == "__main__":
    generate_cone_ply(
        filename="cmake-build-debug/cone_with_points.ply",
        x0=0.0,
        y0=0.0,
        height=2.0,
        radius=1.0,
        num_base_vertices=0,
        num_surface_points=2000,
        include_base=False
    )
