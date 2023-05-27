from svgnest import nest
import svgwrite
import trimesh
from adaptmesh import triangulate
import numpy as np
import open3d as o3d
import shapley as sh
import matplotlib.pyplot as plt
from SVGnest import nest
from confmap.confmap import BFFfrom svgnest import nest
import svgwrite
import trimesh
from adaptmesh import triangulate
import numpy as np
import open3d as o3d
import shapley as sh
import matplotlib.pyplot as plt
from SVGnest import nest
from confmap.confmap import BFF
from confmap.io_utils import read_obj, write_obj


# read the scanned mesh and generate the uv map

mesh = o3d.io.read_triangle_mesh("../data/bumpcap.obj")


# m.p are the points
# m.t are the elements

# read the uv map and generate the confmap

vertices, faces = read_obj('../data/bumpcap.obj')
cm = BFF(vertices, faces)
image = cm.layout()
write_obj('bumpcap_with_uv.obj', cm.vertices,
          cm.faces, image.vertices, image.faces)


# read the svg of the mesh edges and generate the svg of the mesh edges with the confmap

mesh = trimesh.load('your_file.PLY', force='mesh')
lines = mesh.vertices[mesh.edges_unique]
sharp = mesh.face_adjacency_angles > np.radians(40)
edges = mesh.face_adjacency_edges[sharp]
lines = mesh.vertices[edges]

edges = mesh.edges
edges_unique = mesh.edges_unique
edges_sorted = np.sort(edges_unique, axis=1)
edges_sorted = np.unique(edges_sorted, axis=0)

lines = mesh.vertices[edges_sorted]

vertices, faces = read_obj('bumpcap_with_uv.obj')


dwg = svgwrite.Drawing('bumpcap_with_confmap.svg', profile='tiny')
for line in lines:
    dwg.add(dwg.line(line[0][:2], line[1][:2], stroke='black'))
dwg.save()


# import the svg of parts to be packed and nest them in svg of Confmap

files = {
    'part1.svg': 1,
    'part2.svg': 2
}
// nest in a 600x300 mm plate, saving to combined.svg

nest('combined.svg', files, 600, 300)


# map the svg of the packed parts to the uv map of the mesh

# read the svg of the packed parts

dwg = svgwrite.Drawing('combined.svg', profile='tiny')
for line in lines:
    dwg.add(dwg.line(line[0][:2], line[1][:2], stroke='black'))
dwg.save()

__file__ = 'bumpcap_with_confmap.svg'

# get the transforms of the packed parts in relation to the uv map



from confmap.io_utils import read_obj, write_obj


# read the scanned mesh and generate the uv map

mesh = o3d.io.read_triangle_mesh("../data/bumpcap.obj")


# m.p are the points
# m.t are the elements

# read the uv map and generate the confmap

vertices, faces = read_obj('../data/bumpcap.obj')
cm = BFF(vertices, faces)
image = cm.layout()
write_obj('bumpcap_with_uv.obj', cm.vertices,
          cm.faces, image.vertices, image.faces)


# read the svg of the mesh edges and generate the svg of the mesh edges with the confmap

mesh = trimesh.load('your_file.PLY', force='mesh')
lines = mesh.vertices[mesh.edges_unique]
sharp = mesh.face_adjacency_angles > np.radians(40)
edges = mesh.face_adjacency_edges[sharp]
lines = mesh.vertices[edges]

edges = mesh.edges
edges_unique = mesh.edges_unique
edges_sorted = np.sort(edges_unique, axis=1)
edges_sorted = np.unique(edges_sorted, axis=0)

lines = mesh.vertices[edges_sorted]

vertices, faces = read_obj('bumpcap_with_uv.obj')


dwg = svgwrite.Drawing('bumpcap_with_confmap.svg', profile='tiny')
for line in lines:
    dwg.add(dwg.line(line[0][:2], line[1][:2], stroke='black'))
dwg.save()


# import the svg of parts to be packed and nest them in svg of Confmap

files = {
    'part1.svg': 1,
    'part2.svg': 2
}
// nest in a 600x300 mm plate, saving to combined.svg

nest('combined.svg', files, 600, 300)


# map the svg of the packed parts to the uv map of the mesh

# read the svg of the packed parts

dwg = svgwrite.Drawing('combined.svg', profile='tiny')
for line in lines:
    dwg.add(dwg.line(line[0][:2], line[1][:2], stroke='black'))
dwg.save()

__file__ = 'bumpcap_with_confmap.svg'

# get the transforms of the packed parts in relation to the uv map



