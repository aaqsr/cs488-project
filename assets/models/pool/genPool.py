def generate_pool_obj(obj_filename, size_of_all_columns=(80-1) * 0.05 + 0.05, size_of_all_rows=(100-1) * 0.05 + 0.05, depth=1.5, wall_thickness=0.25,
                      mtl_name="pool_material",
                      texture_file="pool_diffuse.png",
                      specular_file="pool_specular.png"):
    import os

    mtl_filename = os.path.splitext(obj_filename)[0] + ".mtl"
    sz_col = size_of_all_columns
    sz_row = size_of_all_rows

    # Vertices: outer and inner
    outer = [
        # Top outer perimeter (Y = depth)
        (-wall_thickness, depth, -wall_thickness),                   # 0
        (sz_col + wall_thickness, depth, -wall_thickness),           # 1
        (sz_col + wall_thickness, depth, sz_row + wall_thickness),   # 2
        (-wall_thickness, depth, sz_row + wall_thickness),           # 3

        # Bottom outer perimeter (Y = 0)
        (-wall_thickness, 0, -wall_thickness),                       # 4
        (sz_col + wall_thickness, 0, -wall_thickness),               # 5
        (sz_col + wall_thickness, 0, sz_row + wall_thickness),       # 6
        (-wall_thickness, 0, sz_row + wall_thickness),               # 7
    ]

    inner = [
        # Top inner perimeter (Y = depth)
        (0, depth, 0),                        # 8
        (sz_col, depth, 0),                   # 9
        (sz_col, depth, sz_row),              # 10
        (0, depth, sz_row),                   # 11

        # Bottom inner floor perimeter (Y = wall_thickness)
        (0, wall_thickness, 0),               # 12
        (sz_col, wall_thickness, 0),          # 13
        (sz_col, wall_thickness, sz_row),     # 14
        (0, wall_thickness, sz_row),          # 15
    ]

    vertices = outer + inner

    # Faces (quads): indices into vertices list
    faces = [
        # Outer Floor
        # (4, 5, 6, 7),

        # Outer walls
        (0, 1, 5, 4),  # front
        (1, 2, 6, 5),  # right
        (2, 3, 7, 6),  # back
        (3, 0, 4, 7),  # left

        # Top lip (bridge between outer and inner)
        (0, 8, 9, 1),    # front
        (1, 9, 10, 2),   # right
        (2, 10, 11, 3),  # back
        (3, 11, 8, 0),   # left

        # Inner walls (pointing in)
        (12, 13, 9, 8),    # front inner
        (13, 14, 10, 9),   # right inner
        (14, 15, 11, 10),  # back inner
        (15, 12, 8, 11),   # left inner

        # Inner floor
        (12, 15, 14, 13),
    ]

    # --- Generate per-face UVs with correct projection ---

    uv_coords = []
    uv_map = {}  # (vertex_index, face_index) -> uv_index

    def get_uv_index(v_idx, f_idx, uv):
        key = (v_idx, f_idx)
        if key in uv_map:
            return uv_map[key]
        uv_coords.append(uv)
        uv_map[key] = len(uv_coords)  # 1-based
        return uv_map[key]

    uv_face_indices = []

    for f_idx, face in enumerate(faces):
        vpos = [vertices[i] for i in face]

        # Determine face orientation by edges
        # Compute vector edges
        edge1 = (vpos[1][0] - vpos[0][0], vpos[1][1] -
                 vpos[0][1], vpos[1][2] - vpos[0][2])
        edge2 = (vpos[2][0] - vpos[1][0], vpos[2][1] -
                 vpos[1][1], vpos[2][2] - vpos[1][2])

        # Compute normal by cross product
        nx = edge1[1]*edge2[2] - edge1[2]*edge2[1]
        ny = edge1[2]*edge2[0] - edge1[0]*edge2[2]
        nz = edge1[0]*edge2[1] - edge1[1]*edge2[0]

        nx, ny, nz = abs(nx), abs(ny), abs(nz)

        # Decide projection plane based on dominant normal axis
        if ny > nx and ny > nz:
            # Face is mostly horizontal → project XZ plane
            # UV = (x,z)
            uv_proj = [(v[0], v[2]) for v in vpos]
        elif nx > nz:
            # Face mostly vertical facing X → project YZ plane
            uv_proj = [(v[2], v[1]) for v in vpos]
        else:
            # Face mostly vertical facing Z → project XY plane
            uv_proj = [(v[0], v[1]) for v in vpos]

        face_uv = []
        for vi, uv in zip(face, uv_proj):
            # UV tiling scale can be adjusted here (currently 1:1 world units)
            face_uv.append(get_uv_index(vi, f_idx, uv))
        uv_face_indices.append(face_uv)

    # --- Write OBJ ---

    with open(obj_filename, "w") as f:
        f.write("# Swimming pool with MTL and per-face UVs\n")
        f.write(f"mtllib {os.path.basename(mtl_filename)}\n")
        f.write(f"usemtl {mtl_name}\n")

        for v in vertices:
            f.write(f"v {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}\n")

        for uv in uv_coords:
            f.write(f"vt {uv[0]:.4f} {uv[1]:.4f}\n")

        for face, uv_inds in zip(faces, uv_face_indices):
            face_line = " ".join(f"{vi+1}/{uv_i}" for vi,
                                 uv_i in zip(face, uv_inds))
            f.write(f"f {face_line}\n")

    # --- Write MTL ---

    with open(mtl_filename, "w") as mtl:
        mtl.write(f"newmtl {mtl_name}\n")
        mtl.write("Ka 1.000 1.000 1.000\n")
        mtl.write("Kd 1.000 1.000 1.000\n")
        mtl.write("Ks 0.500 0.500 0.500\n")
        mtl.write("Ns 96.078431\n")
        mtl.write("illum 2\n")
        mtl.write(f"map_Kd {texture_file}\n")
        mtl.write(f"map_Ks {specular_file}\n")

    print(f"[✓] OBJ saved: {obj_filename}")
    print(f"[✓] MTL saved: {mtl_filename}")


generate_pool_obj("pool.obj")
