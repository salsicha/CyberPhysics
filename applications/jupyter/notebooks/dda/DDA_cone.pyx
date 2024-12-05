from libc.math cimport abs
from libc.math cimport round
cimport cython
import numpy as np
cimport numpy as np
from libc.stdio cimport printf

def DDA_cone(float[:, :, ::1] arr,
         int[::1] cam_position,
         float[::1] cam_offset,
         float[::1] cam_direction,
         int x_max,
         int y_max,
         int z_max,
         int image_width,
         int image_height,
         float cone_opening,
         float[:, ::1] dx_arr,
         float[:, ::1] dy_arr,
         float[:, ::1] dz_arr,
         float[:, ::1] dx_p,
         float[:, ::1] dy_p,
         float[:, ::1] dz_p,
         float[:, ::1] dx_open_arr,
         float[:, ::1] dy_open_arr,
         float[:, ::1] dz_open_arr,
         float[:, ::1] output_image):

    cdef int N = 40

    # TODO: make output image 40x32!!!
    # cdef int image_width = 40
    # cdef int image_height = 32

    cdef int i, j, m, n, m_, n_
    cdef int x, y, z
    cdef float x_offset, y_offset, z_offset
    cdef float delta1, delta2, delta1_open, delta2_open
    cdef float dx, dy, dz, dx_open, dy_open, dz_open
    cdef float p
    cdef float max_val = 0.0, new_val = 0.0
    cdef int open_1_int, open_2_int

    # cdef double[:] Y = np.zeros(N)

    # cdef int x_1 = x_max
    # cdef int y_1 = y_max
    # cdef int z_1 = z_max

    x_offset = cam_offset[0]
    y_offset = cam_offset[1]
    z_offset = cam_offset[2]

    # for i in range(N):
    #    for j in range(N):

    for i in range(image_width):
       for j in range(image_height):
            dx = dx_arr[i, j] * dx_arr[i, j]
            dy = dy_arr[i, j] * dy_arr[i, j]
            dz = dz_arr[i, j] * dz_arr[i, j]

            # x_offset = cam_offset[0]
            # y_offset = cam_offset[1]
            # z_offset = cam_offset[2]

            delta1 = 0.0
            delta2 = 0.0

            delta1_open = 0.0
            delta2_open = 0.0

            if dx >= dy and dx >= dz:
                p = dx_p[i, j]

                delta1 = y_offset
                delta2 = z_offset
                delta1_open = y_offset
                delta2_open = z_offset

            elif dy >= dx and dy >= dz:
                p = dy_p[i, j]

                delta1 = x_offset
                delta2 = z_offset
                delta1_open = x_offset
                delta2_open = z_offset

            else:
                p = dz_p[i, j]

                delta1 = x_offset
                delta2 = y_offset
                delta1_open = x_offset
                delta2_open = y_offset

            dx = p * dx_arr[i, j]
            dy = p * dy_arr[i, j]
            dz = p * dz_arr[i, j]

            dx_open = p * dx_open_arr[i, j]
            dy_open = p * dy_open_arr[i, j]
            dz_open = p * dz_open_arr[i, j]

            threshold = 1.0

            x = cam_position[0]
            y = cam_position[1]
            z = cam_position[2]

            # printf("%i\n", x)
            # printf("%f\n\n", dx)

            max_val = 0.0

            while True:
                if dx * dx >= dy * dy and dx * dx >= dz * dz:

                    # printf("%f\n", dx)

                    x += int(round(dx))

                    # TODO: x value is completely wrong... same for y and z no doubt...

                    # printf("%i\n", x)

                    delta1 += dy
                    if delta1 > threshold:
                        y += 1
                        delta1 -= threshold
                    elif delta1 < -threshold:
                        y -= 1
                        delta1 += threshold

                    delta2 += dz
                    if delta2 > threshold:
                        z += 1
                        delta2 -= threshold
                    elif delta2 < -threshold:
                        z -= 1
                        delta2 += threshold

                    if x <= 0 or y <= 0 or z <= 0 or x >= x_max or y >= y_max or z >= z_max:
                        break

                    delta1_open += dy_open
                    delta2_open += dz_open

                    m_ = 0
                    n_ = 0

                    open_1_int = abs(int(delta1_open))
                    open_2_int = abs(int(delta2_open))

                    for m in range(open_1_int):
                        m_ += 1
                        for n in range(open_2_int):
                            n_ += 1
                            if x < x_max and y + m_ < y_max and z + n_ < z_max:
                                new_val = arr[x, y + m_, z + n_]
                                if new_val > max_val:
                                    output_image[i, j] = new_val
                                    max_val = new_val

                    new_val = arr[x, y, z]
                    if new_val > max_val:
                        output_image[i, j] = new_val
                        max_val = new_val

                elif dy * dy >= dx * dx and dy * dy >= dz * dz:
                    y += int(round(dy))

                    delta1 += dx
                    if delta1 > threshold:
                        x += 1
                        delta1 -= threshold
                    if delta1 < -threshold:
                        x -= 1
                        delta1 += threshold

                    delta2 += dz
                    if delta2 > threshold:
                        z += 1
                        delta2 -= threshold
                    if delta2 < -threshold:
                        z -= 1
                        delta2 += threshold

                    if x <= 0 or y <= 0 or z <= 0 or x >= x_max or y >= y_max or z >= z_max:
                        break

                    delta1_open += dx_open
                    delta2_open += dz_open

                    m_ = 0
                    n_ = 0

                    open_1_int = abs(int(delta1_open))
                    open_2_int = abs(int(delta2_open))

                    for m in range(open_1_int):
                        m_ += 1
                        for n in range(open_2_int):
                            n_ += 1
                            if x + m_ < x_max and y < y_max and z + n_ < z_max:
                                new_val = arr[x + m_, y, z + n_]
                                if new_val > max_val:
                                    output_image[i, j] = new_val
                                    max_val = new_val

                    new_val = arr[x, y, z]
                    if new_val > max_val:
                        output_image[i, j] = new_val
                        max_val = new_val

                else:
                    z += int(round(dz))

                    delta1 += dx
                    if delta1 > threshold:
                        x += 1
                        delta1 -= threshold
                    if delta1 < -threshold:
                        x -= 1
                        delta1 += threshold

                    delta2 += dy
                    if delta2 > threshold:
                        y += 1
                        delta2 -= threshold
                    if delta2 < -threshold:
                        y -= 1
                        delta2 += threshold

                    if x <= 0 or y <= 0 or z <= 0 or x >= x_max or y >= y_max or z >= z_max:
                        break

                    delta1_open += dx_open
                    delta2_open += dy_open

                    m_ = 0
                    n_ = 0

                    open_1_int = abs(int(delta1_open))
                    open_2_int = abs(int(delta2_open))

                    for m in range(open_1_int):
                        m_ += 1
                        for n in range(open_2_int):
                            n_ += 1
                            if x + m_ < x_max and y + n_ < y_max and z < z_max:
                                new_val = arr[x + m_, y + n_, z]
                                if new_val > max_val:
                                    output_image[i, j] = new_val
                                    max_val = new_val

                    new_val = arr[x, y, z]
                    if new_val > max_val:
                        output_image[i, j] = new_val
                        max_val = new_val

    return output_image
