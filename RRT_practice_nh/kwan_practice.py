#In[]
import numpy as np
from sklearn.neighbors import KDTree
import numpy.linalg as LA


rng = np.random.RandomState(0)
X = rng.random_sample((10, 3))  # 10 points in 3 dimensions
tree = KDTree(X, leaf_size=2)

print(X[:1])  
print(tree.query_radius(X[:1], r=0.3, count_only=True))

diff = np.zeros(len(X))
norm = []
for i in range(len(X)):
    diff[i] = X[i] - X[:1]
    print(diff[i])
    # norm[i] = np.linalg.norm(diff[i], 2) 
    
# print(norm)
# %%
