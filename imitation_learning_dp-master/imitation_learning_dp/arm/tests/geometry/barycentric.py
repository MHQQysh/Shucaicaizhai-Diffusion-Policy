import numpy as np

vertices = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]])


def volume(vertices):
    tetras = vertices[:, [1, 2, 3]] - vertices[:, [0, 0, 0]]
    return np.abs(np.einsum('ijk,ijl->i', tetras[:, :, [1, 2, 0]], tetras[:, :, [2, 0, 1]])) / 6


V = volume(vertices)


def centroid(vertices):
    tetras = vertices[:, [1, 2, 3]] - vertices[:, [0, 0, 0]]
    T = np.einsum('ijk,ijl->il', tetras[:, :, [1, 2, 0]], tetras[:, :, [2, 0, 1]])
    C = np.einsum('i,ij->j', V, (vertices[:, 0] + vertices[:, 1] + vertices[:, 2] + vertices[:, 3]) / 4)
    return C / T.sum(axis=1)


C = centroid(vertices)


# 将点坐标转换到质心坐标系
def to_barycentric(vertices, C, p):
    tetras = vertices[:, [1, 2, 3]] - vertices[:, [0, 0, 0]]
    T = np.einsum('ijk,ijl->il', tetras[:, :, [1, 2, 0]], tetras[:, :, [2, 0, 1]])
    v = p - C
    b = np.einsum('ij,ij->i', np.cross(tetras[:, :, 1], tetras[:, :, 2]), v)
    c = np.einsum('ij,ij->i', np.cross(tetras[:, :, 2], tetras[:, :, 0]), v)
    d = np.einsum('ij,ij->i', np.cross(tetras[:, :, 0], tetras[:, :, 1]), v)
    return np.column_stack((b / T.sum(axis=1), c / T.sum(axis=1), d / T.sum(axis=1)))


# 将质心坐标系中的坐标转换回原坐标系
def from_barycentric(vertices, barycentric):
    return np.einsum('ij,i->ij', vertices, barycentric).sum(axis=1)


# 测试用例
p = np.array([0.25, 0.25, 0.25])
barycentric = to_barycentric(vertices, C, p)
p2 = from_barycentric(vertices, barycentric)
print("原坐标系中的坐标:", p)
print("质心坐标系中的坐标:", barycentric)
print("转换回原坐标系中的坐标:", p2)
