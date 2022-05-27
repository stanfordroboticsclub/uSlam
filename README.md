# uSlam

Pose graph based slam using ICP for scan matching



# install 
# for cvxopt seee https://github.com/cvxopt/cvxopt/issues/78

```
wget http://faculty.cse.tamu.edu/davis/SuiteSparse/SuiteSparse-4.5.3.tar.gz
tar -xf SuiteSparse-4.5.3.tar.gz
export CVXOPT_SUITESPARSE_SRC_DIR=$(pwd)/SuiteSparse
```

# notes

using a lot of tools here will require you to be on the 10.0.0.X subnet as they use [UDPComms](https://github.com/stanfordroboticsclub/UDPComms) to communitate with the robot and each other
