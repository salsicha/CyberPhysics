import matplotlib.pyplot as plt

import pyvista as pv


plotter = pv.Plotter(off_screen=True)

#actor = plotter.add_mesh(pv.Sphere())
actor = plotter.add_mesh(pv.read('plug.stl'))

plotter.store_image = True

print(plotter.camera)
plotter.show()

zval = plotter.get_image_depth()

plt.figure()
plt.imshow(zval)
plt.colorbar(label='Distance to Camera')
plt.title('Depth image')
plt.xlabel('X Pixel')
plt.ylabel('Y Pixel')
plt.show()
