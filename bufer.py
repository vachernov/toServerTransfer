def intersectMaps(mask): # make intersection of view areas after given dX, dY, dTheta
    global dX
    global dY
    global dTheta


    x_lt = distance2pixels(-dX)
    y_lt = distance2pixels(-dY)

    x_rt = np.cos(-dTheta) * 799 + distance2pixels(-dX)
    y_rt = -np.sin(-dTheta) * 799 + distance2pixels(-dY)

    x_lb = np.sin(-dTheta) * 799 + distance2pixels(-dX)
    y_lb = np.cos(-dTheta) * 799 + distance2pixels(-dY)

    src_points = np.float32([[0, 0], [799, 0], [0, 799]])
    dst_points = np.float32([[x_lt, y_lt], [x_rt, y_rt], [x_lb, y_lb]])

    affine_matrix = cv2.getAffineTransform(src_points, dst_points)
    transformed_image = cv2.warpAffine(mask, affine_matrix, (WIDTH, HEIGHT))
    return transformed_image
