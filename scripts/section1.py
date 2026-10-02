#!/usr/bin/env python3
#add import and helper functions her
import numpy as np
np.random.seed(42)
A = np.random.normal(size=(4,4))
B = np.random.normal(size=(4,2))
D = A@B
# print(D)
np.random.seed(42)
x = np.random.normal(size=(4,10))
print("\n")
x1 = x[:, None, :]
x2 = x[None, :, :]
diff = x1 - x2

b = np.square(diff)
a = np.sum(b, axis=2)

print(a)









if __name__ == "__main__":
    print("hello world")
