import math


def write_stl(filename):

    length = 1.0
    radius = 0.15

    sections = 32

    vertices = []

    # 两端圆面
    for x in [-length/2, length/2]:

        for i in range(sections):

            theta = 2 * math.pi * i / sections

            y = radius * math.cos(theta)
            z = radius * math.sin(theta)

            vertices.append((x, y, z))


    triangles = []


    # 侧面
    for i in range(sections):

        n = (i+1) % sections

        triangles.append(
            (
                vertices[i],
                vertices[n],
                vertices[sections+n]
            )
        )

        triangles.append(
            (
                vertices[i],
                vertices[sections+n],
                vertices[sections+i]
            )
        )


    # 写STL

    with open(filename, "w") as f:

        f.write("solid auv\n")

        for tri in triangles:

            f.write("facet normal 0 0 0\n")
            f.write("outer loop\n")

            for v in tri:

                f.write(
                    f"vertex {v[0]} {v[1]} {v[2]}\n"
                )

            f.write("endloop\n")
            f.write("endfacet\n")


        f.write("endsolid auv\n")


write_stl("auv.stl")

print("auv.stl created")