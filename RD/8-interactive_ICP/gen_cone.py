import numpy as np

def generate_cone_surface_ply(
        filename,
        x0=0.0,
        y0=0.0,
        height=1.0,
        base_radius=0.3,
        num_surface_points=1000,
):
    """
    Generate a PLY file with only points on the *outer surface* of a cone.
    No points on the base or inside the cone.

    Parameters:
        filename (str): Output PLY file path
        x0, y0 (float): Base center of the cone
        height (float): Cone height
        base_radius (float): Radius at the base
        num_surface_points (int): Number of random points on surface
    """
    points = []

    for _ in range(num_surface_points):
        # Random height along the cone (0 = base, 1 = tip)
        h_ratio = np.random.rand()
        r = base_radius * (1 - h_ratio)
        theta = np.random.rand() * 2 * np.pi  # full 360° surface

        # Surface coordinates
        x = x0 + r * np.cos(theta)
        y = y0 + h_ratio * height
        z = r * np.sin(theta)

        points.append([x, y, z])

    points = np.array(points)

    # Write to PLY
    with open(filename, "w") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        for p in points:
            f.write(f"{p[0]} {p[1]} {p[2]}\n")

    print(f"✅ Saved {len(points)} surface points to {filename}")


if __name__ == "__main__":
    generate_cone_surface_ply(
        "cmake-build-debug/cone_surface_only.ply",
        x0=0.0,
        y0=0.0,
        height=0.4,
        base_radius=0.2,
        num_surface_points=200,
    )
